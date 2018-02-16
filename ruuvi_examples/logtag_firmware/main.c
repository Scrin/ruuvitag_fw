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
 * Logtag firmware for the RuuviTag B, optimized for logging sensor data
 * Two modes: normal and fast, switchable by pressing B button
 * 
 * Normal mode: (red led blinks) Useful for long-term logging of environmental data with long battery life
 * Measures & Transmits every 5 seconds, 16x IIR, 10hz accelerometer sampling rate for motion detection, long battery life
 * 
 * Fast mode: (green led blinks rapidly) Useful for logging rapidly changing conditions with raw data
 * Measures &Transmits every 0.1 seconds, no IIR, 50hz accelerometer sampling rate for motion detection, short battery life
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

static uint8_t data_buffer[24] = {0};
static bool fastMode = false;
static uint64_t debounce = false;
static uint16_t acceleration_events = 0;

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
    fastMode = !fastMode;

    app_timer_stop(main_timer_id);
    bme280_set_mode(BME280_MODE_SLEEP); // writes to BME280 config register may be ignored if the sensor is not sleeping

    if (fastMode) {
        lis2dh12_set_sample_rate(LIS2DH12_RATE_50); // 11uA power consumption
        bme280_set_interval(BME280_STANDBY_62_5_MS);
        bme280_set_iir(BME280_IIR_OFF);
        app_timer_start(main_timer_id, APP_TIMER_TICKS(INTERVAL_FAST, RUUVITAG_APP_TIMER_PRESCALER), NULL);
        bluetooth_configure_advertising_interval(INTERVAL_FAST);
    } else {
        lis2dh12_set_sample_rate(LIS2DH12_RATE_10); // 4uA power consumption
        bme280_set_interval(BME280_STANDBY_1000_MS);
        bme280_set_iir(BME280_IIR_16);
        app_timer_start(main_timer_id, APP_TIMER_TICKS(INTERVAL_NORMAL, RUUVITAG_APP_TIMER_PRESCALER), NULL);
        bluetooth_configure_advertising_interval(INTERVAL_NORMAL);
    }

    bme280_set_mode(BME280_MODE_NORMAL);
    bluetooth_advertising_start(); // "restart" with the new interval
    main_timer_handler(NULL);
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

    if (fastMode) nrf_gpio_pin_clear(LED_GREEN);
    else nrf_gpio_pin_clear(LED_RED);
}

static void updateAdvertisement(void) {
    ret_code_t err_code = NRF_SUCCESS;
    err_code |= bluetooth_set_manufacturer_data(data_buffer, sizeof (data_buffer));
    NRF_LOG_DEBUG("Updating data, data status %d\r\n", err_code);
}

void main_timer_handler(void * p_context) {
    static int32_t raw_t = 0;
    static uint32_t raw_p = 0;
    static uint32_t raw_h = 0;
    static lis2dh12_sensor_buffer_t buffer;
    static int32_t acc[3] = {0};

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
    uint16_t vbat = 0;
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
    watchdog_feed();
}

ret_code_t lis2dh12_int2_handler(const ruuvi_standard_message_t message) {
    NRF_LOG_DEBUG("Accelerometer interrupt to pin 2\r\n");
    acceleration_events++;
    return NRF_SUCCESS;
}

int main(void) {
    ret_code_t err_code = 0; // counter, gets incremented by each failed init. It is 0 in the end if init was ok.

    // Initialize log.
    err_code |= init_log();

    // Setup leds. LEDs are active low, so setting high them turns leds off.
    err_code |= init_leds(); // INIT leds first and turn RED on.
    nrf_gpio_pin_clear(LED_RED); // If INIT fails at later stage, RED will stay lit.

    // Initialize BLE Stack. Required in all applications for timer operation.
    err_code |= init_ble();
    bluetooth_advertising_stop();
    bluetooth_tx_power_set(BLE_TX_POWER);
    err_code |= bluetooth_configure_advertisement_type(BLE_GAP_ADV_TYPE_ADV_NONCONN_IND);

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
    //INT2_CFG = 0x7F
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
    bme280_set_iir(BME280_IIR_16);
    bme280_set_interval(BME280_STANDBY_1000_MS);
    bme280_set_mode(BME280_MODE_NORMAL);
    NRF_LOG_DEBUG("BME280 configuration done\r\n");

    // Visually display init status. Hangs if there was an error, waits 3 seconds on success.
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
