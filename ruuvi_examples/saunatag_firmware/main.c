/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/*
 * Saunatag firmware for the RuuviTag B, optimized for logging data from a Sauna.
 * Transmission interval is reduced to 100ms when sauna is in use and 
 * increased to 10s at other times to save battery.
 * 
 * B button toggles led-indicators (enabled after startup):
 * - Both leds blink when significant increase in humidity is detected
 * - Green led blinks when temperature is increasing more than 0.01c per 100ms
 * - Red led blinks when temperature is decreasing more than 0.01c per 100ms
 */

// STDLIB
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

// Nordic SDK
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "app_scheduler.h"
#include "app_timer_appsh.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

#define NRF_LOG_MODULE_NAME "MAIN"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

// BSP
#include "bsp.h"

// Drivers
#include "lis2dh12.h"
#include "lis2dh12_registers.h"
#include "bme280.h"
#include "battery.h"
#include "bluetooth_core.h"
#include "pin_interrupt.h"
#include "rtc.h"

// Libraries
#include "sensortag.h"

// Init
#include "init.h"

// Configuration
#include "bluetooth_config.h"

// Constants
#define DEAD_BEEF               0xDEADBEEF    //!< Value used as error code on stack dump, can be used to identify stack location on stack unwind.

// ID for main loop timer.
APP_TIMER_DEF(main_timer_id); // Creates timer id for our program.

#define INTERVAL_NORMAL APPLICATION_ADV_INTERVAL
#define INTERVAL_FAST 100u // fastest allowed
#define DEBOUNCE_THRESHOLD 250u

// tunables, temperatures are in millicelcius
#define TEMPERATURE_TRESHOLD 3000u // hotter than this is considered warm
#define TEMPERATURE_TRESHOLD_HYSTERESIS 200u // this much below the treshold is considered cold
#define SESSION_PEAK_TEMPERATURE_DELTA 2000u // a temperature this much below the session peak temperature means the sauna is cooling down
#define CYCLE_PEAK_TEMPERATURE_DELTA 1000u // a temperature this much below the cycle peak temperature (=when the temperature was increasing for the last time) means the sauna is cooling down
#define TEMPERATURE_DELTA_ERROR_MARGIN 2 // ignore temperature deltas smaller than this
#define HUMIDITY_INCREASE_TRESHOLD 512 // this big increase in humidity is considered an increase (1024 = 1%)
#define HUMIDITY_BLINK_COUNTER_INCREMENT 2 // each sufficiently increased humidity measurement adds this many "humidity blinks"

static uint8_t data_buffer[24] = {0};
static uint64_t debounce = 0;
static uint16_t acceleration_events = 0;

static bme280_data_t last_environmental; // previous measurement
static bool enable_leds = true; // use flashing leds to indicate stuff
static bool red_led = false; // turn on red led for the next wakeup
static bool green_led = false; // turn on green led for the next wakeup
static bool is_warm = false; // used for transitions to/from greater/less than TEMPERATURE_TRESHOLD
static bool is_cooling_down = false; // true if sauna is still warm but cooling down after use
static int32_t session_peak_temperature = -27315; // peak temperature for this sauna session
static int32_t cycle_peak_temperature = -27315; // peak temperature for this cycle (=last time temperature was going up instead of down)
static uint32_t humidity_blink_counter = 0; // used for blinking the leds when a sudden increase in humidity is detected


static ruuvi_sensor_t data;

static void main_timer_handler(void * p_context);

/**@brief Handler for button press.
 * Called in scheduler, out of interrupt context.
 */
void change_mode(void* data, uint16_t length) {
    // Avoid double presses
    if ((millis() - debounce) < DEBOUNCE_THRESHOLD) {
        return;
    }
    debounce = millis();
    enable_leds = !enable_leds;
}

/**@brief Function for handling button events.
 * Schedulers call to handler.
 */
ret_code_t button_press_handler(const ruuvi_standard_message_t message) {
    NRF_LOG_INFO("Button\r\n");
    app_sched_event_put(NULL, 0, change_mode);
    return ENDPOINT_SUCCESS;
}

/**@brief Function for doing power management.
 */
static void power_manage(void) {
    // Clear both leds before sleep.
    nrf_gpio_pin_set(LED_GREEN);
    nrf_gpio_pin_set(LED_RED);

    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);

    if (green_led) {
        green_led = false;
        nrf_gpio_pin_clear(LED_GREEN);
    }
    if (red_led) {
        red_led = false;
        nrf_gpio_pin_clear(LED_RED);
    }
}

static void updateAdvertisement(void) {
    ret_code_t err_code = NRF_SUCCESS;
    err_code |= bluetooth_set_manufacturer_data(data_buffer, sizeof (data_buffer));
    NRF_LOG_DEBUG("Updating data, data status %d\r\n", err_code);
}

static void checkAndUpdateInterval(bme280_data_t environmental) {
    if (environmental.temperature < TEMPERATURE_TRESHOLD - (is_warm ? TEMPERATURE_TRESHOLD_HYSTERESIS : 0)) {
        if (is_warm) {
            is_warm = false;
            if (!is_cooling_down) { // "cooling down" logic already changes the interval, so do it here only if "cooling down" was not detected before the temperature got low enough
                bme280_set_mode(BME280_MODE_SLEEP); // writes to BME280 config register may be ignored if the sensor is not sleeping
                bme280_set_interval(BME280_STANDBY_1000_MS);
                bme280_set_mode(BME280_MODE_NORMAL);
                app_timer_stop(main_timer_id);
                app_timer_start(main_timer_id, APP_TIMER_TICKS(INTERVAL_NORMAL, RUUVITAG_APP_TIMER_PRESCALER), NULL);
                bluetooth_configure_advertising_interval(INTERVAL_NORMAL);
                bluetooth_apply_configuration();
            }
            is_cooling_down = false; // already cool at this point
            NRF_LOG_INFO("Tag is no longer warm\r\n");
        }
    } else if (!is_cooling_down) { // warm and not cooling down
        if (!is_warm) { // first time "warm" during this session
            NRF_LOG_INFO("Tag is warm, decreasing interval\r\n");
            session_peak_temperature = environmental.temperature;
            cycle_peak_temperature = environmental.temperature;
            is_warm = true;
            bme280_set_mode(BME280_MODE_SLEEP); // writes to BME280 config register may be ignored if the sensor is not sleeping
            bme280_set_interval(BME280_STANDBY_62_5_MS);
            bme280_set_mode(BME280_MODE_NORMAL);
            app_timer_stop(main_timer_id);
            app_timer_start(main_timer_id, APP_TIMER_TICKS(INTERVAL_FAST, RUUVITAG_APP_TIMER_PRESCALER), NULL);
            bluetooth_configure_advertising_interval(INTERVAL_FAST);
            bluetooth_apply_configuration();
            last_environmental = environmental;
        }

        // check if new session peak temperature
        if (session_peak_temperature < environmental.temperature) {
            NRF_LOG_DEBUG("New session peak temperature: %d\r\n", environmental.temperature);
            session_peak_temperature = environmental.temperature;
        }

        // check if new cycle peak temperature
        int32_t tDelta = environmental.temperature - last_environmental.temperature;
        NRF_LOG_DEBUG("temperature: %d -> %d diff: %d\r\n", last_environmental.temperature, environmental.temperature, tDelta);
        if (tDelta > 0) {
            cycle_peak_temperature = environmental.temperature;
        }

        // check if sauna session has ended and the sauna is cooling down
        if (environmental.temperature + SESSION_PEAK_TEMPERATURE_DELTA < session_peak_temperature
                || environmental.temperature + CYCLE_PEAK_TEMPERATURE_DELTA < cycle_peak_temperature) {
            is_cooling_down = true;
            bme280_set_mode(BME280_MODE_SLEEP); // writes to BME280 config register may be ignored if the sensor is not sleeping
            bme280_set_interval(BME280_STANDBY_1000_MS);
            bme280_set_mode(BME280_MODE_NORMAL);
            app_timer_stop(main_timer_id);
            app_timer_start(main_timer_id, APP_TIMER_TICKS(INTERVAL_NORMAL, RUUVITAG_APP_TIMER_PRESCALER), NULL);
            bluetooth_configure_advertising_interval(INTERVAL_NORMAL);
            bluetooth_apply_configuration();
            NRF_LOG_INFO("Cooling down, increasing interval\r\n");
            return;
        }

        // if leds are enabled, show some geeky nerd status indicators and what not
        if (enable_leds) {
            int32_t hDelta = environmental.humidity - last_environmental.humidity;
            if (hDelta > HUMIDITY_INCREASE_TRESHOLD) {
                humidity_blink_counter += HUMIDITY_BLINK_COUNTER_INCREMENT;
            }
            if (humidity_blink_counter > 0) { // indicate stuff related to humidity
                humidity_blink_counter--;
                green_led = true;
                red_led = true;
            } else { // indicate stuff related to temperature
                if (tDelta >= TEMPERATURE_DELTA_ERROR_MARGIN) {
                    green_led = true;
                } else if (-tDelta >= TEMPERATURE_DELTA_ERROR_MARGIN) {
                    red_led = true;
                }
            }
        }
        last_environmental = environmental;
    }
}

void main_timer_handler(void * p_context) {
    static int32_t raw_t = 0;
    static uint32_t raw_p = 0;
    static uint32_t raw_h = 0;
    static lis2dh12_sensor_buffer_t buffer;
    static int32_t acc[3] = {0};
    static uint16_t vbat = 0;

    // Get raw environmental data.
    bme280_read_measurements();
    raw_t = bme280_get_temperature();
    raw_p = bme280_get_pressure();
    raw_h = bme280_get_humidity();

    // Get accelerometer data.
    lis2dh12_read_samples(&buffer, 1);
    acc[0] = buffer.sensor.x;
    acc[1] = buffer.sensor.y;
    acc[2] = buffer.sensor.z;

    // Get battery voltage 
    vbat = getBattery();

    // Embed data into structure for parsing.
    parseSensorData(&data, raw_t, raw_p, raw_h, vbat, acc);
    NRF_LOG_DEBUG("temperature: %d, pressure: %d, humidity: %d x: %d y: %d z: %d\r\n", raw_t, raw_p, raw_h, acc[0], acc[1], acc[2]);
    NRF_LOG_DEBUG("VBAT: %d send %d \r\n", vbat, data.vbat);
    // Prepare bytearray to broadcast.
    bme280_data_t environmental;
    environmental.temperature = raw_t;
    environmental.humidity = raw_h;
    environmental.pressure = raw_p;
    encodeToRawFormat5(data_buffer, &environmental, &buffer.sensor, acceleration_events, vbat, BLE_TX_POWER);

    updateAdvertisement();
    checkAndUpdateInterval(environmental);
    watchdog_feed();
}

ret_code_t lis2dh12_int2_handler(const ruuvi_standard_message_t message) {
    NRF_LOG_DEBUG("Accelerometer interrupt to pin 2\r\n");
    acceleration_events++;
    return NRF_SUCCESS;
}

int main(void) {
    ret_code_t err_code = 0;

    // Initialize log.
    err_code |= init_log();

    // Setup leds. LEDs are active low, so setting high them turns leds off.
    err_code |= init_leds(); // INIT leds first and turn RED on.
    nrf_gpio_pin_clear(LED_RED); // If INIT fails at later stage, RED will stay lit.

    // Initialize BLE Stack. Required in all applications for timer operation.
    err_code |= init_ble();
    bluetooth_configure_advertisement_type(BLE_GAP_ADV_TYPE_ADV_NONCONN_IND);
    bluetooth_tx_power_set(BLE_TX_POWER);
    bluetooth_configure_advertising_interval(INTERVAL_NORMAL);

    // Initialize the application timer module.
    err_code |= init_timer(main_timer_id, INTERVAL_NORMAL, main_timer_handler);
    // Initialize RTC.
    err_code |= init_rtc();
    // Start interrupts.
    err_code |= pin_interrupt_init();
    // Initialize button.
    err_code |= pin_interrupt_enable(BSP_BUTTON_0, NRF_GPIOTE_POLARITY_HITOLO, button_press_handler);
    // Initialize sensors
    err_code |= init_sensors();
    // Clear memory.
    lis2dh12_reset();
    // Wait for reboot.
    nrf_delay_ms(10);
    // Enable XYZ axes.
    lis2dh12_enable();
    lis2dh12_set_scale(LIS2DH12_SCALE2G);
    // Sample rate 10 for activity detection.
    lis2dh12_set_sample_rate(LIS2DH12_RATE_10);
    lis2dh12_set_resolution(LIS2DH12_RES10BIT);
    // Enable high-pass for Interrupt function 2.
    uint8_t ctrl[1];
    ctrl[0] = LIS2DH12_HPIS2_MASK;
    lis2dh12_write_register(LIS2DH12_CTRL_REG2, ctrl, 1);
    // Enable interrupt 2 on X-Y-Z HI/LO.
    ctrl[0] = 0x7F;
    lis2dh12_write_register(LIS2DH12_INT2_CFG, ctrl, 1);
    // Interrupt on 64 mg+ (highpassed, +/-).
    // 4 LSB = 64 mg @2G scale
    ctrl[0] = 0x04;
    lis2dh12_write_register(LIS2DH12_INT2_THS, ctrl, 1);

    // Enable LOTOHI interrupt on nRF52.
    err_code |= pin_interrupt_enable(INT_ACC2_PIN, NRF_GPIOTE_POLARITY_LOTOHI, lis2dh12_int2_handler);

    // Enable Interrupt function 2 on LIS interrupt pin 2 (stays high for 1/ODR).
    lis2dh12_set_interrupts(LIS2DH12_I2C_INT2_MASK, 2);

    // Setup BME280 - oversampling must be set for each used sensor.
    bme280_set_oversampling_hum(BME280_OVERSAMPLING_1);
    bme280_set_oversampling_temp(BME280_OVERSAMPLING_1);
    bme280_set_oversampling_press(BME280_OVERSAMPLING_1);
    bme280_set_iir(BME280_IIR_OFF);
    bme280_set_interval(BME280_STANDBY_1000_MS);
    bme280_set_mode(BME280_MODE_NORMAL);
    NRF_LOG_DEBUG("BME280 configuration done\r\n");

    // Visually display init status. Hangs if there was an error.
    init_blink_status(err_code);

    // Init ok, start watchdog with default wdt event handler (reset).
    init_watchdog(NULL);
    bluetooth_advertising_start();

    // Enter main loop.
    for (;;) {
        app_sched_execute();
        power_manage();
    }
}
