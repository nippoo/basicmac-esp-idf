/*******************************************************************************
 * Copyright (c) 2015 Matthijs Kooijman
 * Copyright (c) 2019 Tobias Schramm
 * Copyright (c) 2023 Max Hunter
 *
 * --- Revised 3-Clause BSD License ---
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright notice,
 *       this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright notice,
 *       this list of conditions and the following disclaimer in the documentation
 *       and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its contributors
 *       may be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SEMTECH BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This the HAL to run LMIC on top of the Arduino environment.
 *******************************************************************************/

#define _GNU_SOURCE 1 // For fopencookie
// Must be first, otherwise it might have already been included without _GNU_SOURCE
#include <stdio.h>
#undef _GNU_SOURCE
// Prevent warning on samd where samd21g18a.h from CMSIS defines this
#undef LITTLE_ENDIAN
#include "../basicmac.h"
#include "hal.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/timer.h"
#include "esp_log.h"

#define TAG "lmic"

extern const lmic_pinmap lmic_pins;

// Datasheet defins typical times until busy goes low. Most are < 200us,
// except when waking up from sleep, which typically takes 3500us. Since
// we cannot know here if we are in sleep, we'll have to assume we are.
// Since 3500 is typical, not maximum, wait a bit more than that.
static unsigned long MAX_BUSY_TIME = 5000;

// -----------------------------------------------------------------------------
// I/O

static void hal_io_init () {
    uint8_t i;
    // Checks below assume that all special pin values are >= LMIC_UNUSED_PIN, so check that.
    #if defined(BRD_sx1261_radio) || defined(BRD_sx1262_radio)
    LMIC_STATIC_ASSERT(LMIC_UNUSED_PIN < LMIC_CONTROLLED_BY_DIO3, "Unexpected constant values");
    LMIC_STATIC_ASSERT(LMIC_UNUSED_PIN < LMIC_CONTROLLED_BY_DIO2, "Unexpected constant values");
    #endif // defined(BRD_sx1261_radio) || defined(BRD_sx1262_radio)

    ASSERT(lmic_pins.nss < LMIC_UNUSED_PIN);
    ASSERT(lmic_pins.rst <= LMIC_UNUSED_PIN);
    ASSERT(lmic_pins.rx <= LMIC_UNUSED_PIN);

#if defined(BRD_sx1272_radio) || defined(BRD_sx1276_radio)
    //DIO0 is required, DIO1 is required for LoRa, DIO2 for FSK
    ASSERT(lmic_pins.dio[0] < LMIC_UNUSED_PIN);
    ASSERT(lmic_pins.dio[1] < LMIC_UNUSED_PIN || lmic_pins.dio[2] < LMIC_UNUSED_PIN);

    ASSERT(lmic_pins.busy == LMIC_UNUSED_PIN);
    ASSERT(lmic_pins.tcxo == LMIC_UNUSED_PIN);
    ASSERT(lmic_pins.tx <= LMIC_UNUSED_PIN);
#elif defined(BRD_sx1261_radio) || defined(BRD_sx1262_radio)
    // Only DIO1 should be specified
    ASSERT(lmic_pins.dio[0] == LMIC_UNUSED_PIN);
    ASSERT(lmic_pins.dio[1] < LMIC_UNUSED_PIN);
    ASSERT(lmic_pins.dio[2] == LMIC_UNUSED_PIN);

    ASSERT(lmic_pins.busy <= LMIC_UNUSED_PIN);
    ASSERT(lmic_pins.tcxo == LMIC_UNUSED_PIN || lmic_pins.tcxo == LMIC_CONTROLLED_BY_DIO3);
    ASSERT(lmic_pins.tx <= LMIC_UNUSED_PIN || lmic_pins.tx == LMIC_CONTROLLED_BY_DIO2);
#else
    #error "Unknown radio type?"
#endif

    // Write HIGH to deselect (NSS is active low). Do this before
    // setting output, to prevent a moment of OUTPUT LOW on e.g. AVR.
    gpio_set_level(lmic_pins.nss, 1);
    gpio_set_direction(lmic_pins.nss, GPIO_MODE_OUTPUT);
    // Write HIGH again after setting output, for architectures that
    // reset to LOW when setting OUTPUT (e.g. arduino-STM32L4).
    gpio_set_level(lmic_pins.nss, 1);

    if (lmic_pins.tx < LMIC_UNUSED_PIN)
        gpio_set_direction(lmic_pins.tx, GPIO_MODE_OUTPUT);
    if (lmic_pins.rx < LMIC_UNUSED_PIN)
        gpio_set_direction(lmic_pins.rx, GPIO_MODE_OUTPUT);
    if (lmic_pins.rst < LMIC_UNUSED_PIN)
        gpio_set_direction(lmic_pins.rst, GPIO_MODE_OUTPUT);
    if (lmic_pins.busy < LMIC_UNUSED_PIN)
        gpio_set_direction(lmic_pins.busy, GPIO_MODE_OUTPUT);
    if (lmic_pins.tcxo < LMIC_UNUSED_PIN)
        gpio_set_direction(lmic_pins.busy, GPIO_MODE_OUTPUT);

    for (i = 0; i < NUM_DIO; ++i) {
        if (lmic_pins.dio[i] < LMIC_UNUSED_PIN)
        gpio_set_direction(lmic_pins.dio[i], GPIO_MODE_OUTPUT);
    }
}

// rx = 0, tx = 1, off = -1
void hal_ant_switch (u1_t val) {
    // TODO: Support separate pin for TX2 (PA_BOOST output)
    if (lmic_pins.tx < LMIC_UNUSED_PIN)
        gpio_set_level(lmic_pins.tx, val == HAL_ANTSW_TX || val == HAL_ANTSW_TX2);
    if (lmic_pins.rx < LMIC_UNUSED_PIN)
        gpio_set_level(lmic_pins.rx, val == HAL_ANTSW_RX);
}

// set radio RST pin to given value (or keep floating!)
bool hal_pin_rst (u1_t val) {
    if (lmic_pins.rst >= LMIC_UNUSED_PIN)
        return false;

    if(val == 0 || val == 1) { // drive pin
        gpio_set_direction(lmic_pins.rst, GPIO_MODE_OUTPUT);
        gpio_set_level(lmic_pins.rst, val);
    } else { // keep pin floating
        gpio_set_direction(lmic_pins.rst, GPIO_MODE_INPUT);
    }
    return true;
}

void hal_irqmask_set (int /* mask */) {
    // Not implemented
}

static bool dio_states[NUM_DIO] = {0};

static void hal_io_check() {
    uint8_t i;
    for (i = 0; i < NUM_DIO; ++i) {
        if (lmic_pins.dio[i] >= LMIC_UNUSED_PIN)
            continue;

        if (dio_states[i] != gpio_get_level(lmic_pins.dio[i])) {
            dio_states[i] = !dio_states[i];
            if (dio_states[i])
                radio_irq_handler(i, hal_ticks());
        }
    }
}

#if defined(BRD_sx1272_radio) || defined(BRD_sx1276_radio)
bool hal_pin_tcxo (u1_t val) {
    if (lmic_pins.tcxo >= LMIC_UNUSED_PIN)
        return false;

    gpio_set_level(lmic_pins.tcxo, val);
    return true;
}
#endif // defined(BRD_sx1272_radio) || defined(BRD_sx1276_radio)

void hal_pin_busy_wait (void) {
    if (lmic_pins.busy >= LMIC_UNUSED_PIN) {
        // TODO: We could probably keep some state so we know the chip
        // is in sleep, since otherwise the delay can be much shorter.
        // Also, all delays after commands (rather than waking up from
        // sleep) are measured from the *end* of the previous SPI
        // transaction, so we could wait shorter if we remember when
        // that was.
        delayMicroseconds(MAX_BUSY_TIME);
    } else {
        unsigned long start = micros();

        while((micros() - start) < MAX_BUSY_TIME && gpio_get_level(lmic_pins.busy)) /* wait */;
    }
}

#if defined(BRD_sx1261_radio) || defined(BRD_sx1262_radio)
bool hal_dio3_controls_tcxo (void) {
    return lmic_pins.tcxo == LMIC_CONTROLLED_BY_DIO3;
}
bool hal_dio2_controls_rxtx (void) {
    return lmic_pins.tx == LMIC_CONTROLLED_BY_DIO2;
}
#endif // defined(BRD_sx1261_radio) || defined(BRD_sx1262_radio)

// -----------------------------------------------------------------------------
// SPI

spi_device_handle_t spi_handle;

static void hal_spi_init () {
    ESP_LOGI(TAG, "Starting SPI initialization");
    esp_err_t ret;

    // init master
    spi_bus_config_t buscfg = {
        .miso_io_num = lmic_pins.spi[0],
        .mosi_io_num = lmic_pins.spi[1],
        .sclk_io_num = lmic_pins.spi[2],
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

  // init device
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 100000,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 7,
    };

    ret = spi_bus_initialize(LMIC_SPI, &buscfg, 1);
    assert(ret == ESP_OK);

    ret = spi_bus_add_device(LMIC_SPI, &devcfg, &spi_handle);
    assert(ret == ESP_OK);

    ESP_LOGI(TAG, "Finished SPI initialization");
}

// perform SPI transaction with radio
u1_t hal_spi (u1_t data) {
    uint8_t rxData = 0;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.rxlength = 8;
    t.tx_buffer = &data;
    t.rx_buffer = &rxData;
    esp_err_t ret = spi_device_transmit(spi_handle, &t);
    assert(ret == ESP_OK);

    return (u1_t) rxData;
}

// -----------------------------------------------------------------------------
// TIME

static void hal_time_init () {
  ESP_LOGI(TAG, "Starting initialisation of timer");
  int timer_group = TIMER_GROUP_0;
  int timer_idx = TIMER_1;
  timer_config_t config;
  config.alarm_en = 0;
  config.auto_reload = 0;
  config.counter_dir = TIMER_COUNT_UP;
  config.divider = 1600;
  config.intr_type = 0;
  config.counter_en = TIMER_PAUSE;
  /*Configure timer*/
  timer_init(timer_group, timer_idx, &config);
  /*Stop timer counter*/
  timer_pause(timer_group, timer_idx);
  /*Load counter value */
  timer_set_counter_value(timer_group, timer_idx, 0x0);
  /*Start timer counter*/
  timer_start(timer_group, timer_idx);

  ESP_LOGI(TAG, "Finished initalisation of timer");
}

u4_t hal_ticks () {
  uint64_t val;
  timer_get_counter_value(TIMER_GROUP_0, TIMER_1, &val);
  return (u4_t)val;
}

u64_t hal_xticks (void) {
    // TODO
    return hal_ticks();
}

// Returns the number of ticks until time. Negative values indicate that
// time has already passed.
static s4_t delta_time(u4_t time) {
    return (s4_t)(time - hal_ticks());
}

void hal_waitUntil (u4_t time) {

    ESP_LOGI(TAG, "Wait until");
    s4_t delta = delta_time(time);

    while( delta > 2000){
        vTaskDelay(1 / portTICK_PERIOD_MS);
        delta -= 1000;
    } if(delta > 0){
        vTaskDelay(delta / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "Done waiting until");
}

// check and rewind for target time
u1_t hal_checkTimer (u4_t time) {
  return delta_time(time) <= 0;
}

static uint8_t irqlevel = 0;

int x_irq_level = 0;

void hal_disableIRQs () {
  //ESP_LOGD(TAG, "Disabling interrupts");
  if(x_irq_level < 1){
      //taskDISABLE_INTERRUPTS();
  }
  x_irq_level++;
}

void hal_enableIRQs () {
  //ESP_LOGD(TAG, "Enable interrupts");
  if(--x_irq_level == 0){
      //taskENABLE_INTERRUPTS();
      hal_io_check();
  }
}

u1_t hal_sleep (u1_t type, u4_t targettime) {
    // Actual sleeping not implemented, but jobs are only run when this
    // function returns 0, so make sure we only do that when the
    // targettime is close. When asked to sleep forever (until woken up
    // by an interrupt), just return immediately to keep polling.
    if (type == HAL_SLEEP_FOREVER)
        return 0;

    // TODO: What value should we use for "close"?
    return delta_time(targettime) < 10 ? 0 : 1;
}

void hal_watchcount (int /* cnt */) {
    // Not implemented
}

// -----------------------------------------------------------------------------
// DEBUG

#ifdef CFG_DEBUG
static void hal_debug_init() {
    #ifdef LED_BUILTIN
    pinMode(LED_BUILTIN, OUTPUT);
    #endif
}

#if !defined(CFG_DEBUG_STREAM)
#error "CFG_DEBUG needs CFG_DEBUG_STREAM defined in target-config.h"
#endif

void hal_debug_str (const char* str) {
    CFG_DEBUG_STREAM.print(str);
}

void hal_debug_led (int val) {
    #ifdef LED_BUILTIN
    gpio_set_level(LED_BUILTIN, val);
    #endif
}
#endif // CFG_DEBUG

// -----------------------------------------------------------------------------

#if defined(LMIC_PRINTF_TO)
static int uart_putchar (char c, FILE *)
{
    putc(c);
//  printf("%c", c);
    return 0 ;
}

void hal_printf_init() {
    // nop, init done
}
#endif // defined(LMIC_PRINTF_TO)

void hal_init() {
    // configure radio I/O and interrupt handler
    hal_io_init();
    // configure radio SPI
    hal_spi_init();
    // configure timer and interrupt handler
    hal_time_init();
}

void hal_failed(const char *file, u2_t line) {
    // HALT...
    ESP_LOGE(TAG, "LMIC HAL failed (%s:%u)", file, line);
    hal_disableIRQs();
    hal_sleep();
    while(1);
}

void hal_reboot (void) {
    // TODO
    hal_failed();
}

u1_t hal_getBattLevel (void) {
    // Not implemented
    return 0;
}

void hal_setBattLevel (u1_t /* level */) {
    // Not implemented
}

void hal_fwinfo (hal_fwi* /* fwi */) {
    // Not implemented
}

u1_t* hal_joineui (void) {
    return nullptr;
}

u1_t* hal_deveui (void) {
    return nullptr;
}

u1_t* hal_nwkkey (void) {
    return nullptr;
}

u1_t* hal_appkey (void) {
    return nullptr;
}

u1_t* hal_serial (void) {
    return nullptr;
}

u4_t  hal_region (void) {
    return 0;
}

u4_t  hal_hwid (void) {
    return 0;
}

u4_t  hal_unique (void) {
    return 0;
}

u4_t hal_dnonce_next (void) {
    return os_getRndU2();
}