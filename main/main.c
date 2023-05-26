/*******************************************************************************
 * 
 * ttn-esp32 - The Things Network device library for ESP-IDF / SX127x
 * 
 * Copyright (c) 2021 Manuel Bleichenbacher
 * 
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 *
 * Sample program for C showing how to send and receive messages.
 *******************************************************************************/

#include "freertos/FreeRTOS.h"
#include <lmic.h>
#include <hal.h>
#include "driver/spi_master.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/task.h"


static const char* TAG = "ESP";

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getJoinEui (u1_t* buf) { memcpy(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
// static const u1_t PROGMEM DEVEUI[8]={ 0xa9, 0x3f, 0x6b, 0xa3, 0xb2, 0xc8, 0xf5, 0xfa };
static const u1_t DEVEUI[8]={ 0xfa, 0xf5, 0xc8, 0xb2, 0xa3, 0x6b, 0x3f, 0xa9 };
void os_getDevEui (u1_t* buf) { memcpy(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t APPKEY[16] = { 0x35, 0xc9, 0x55, 0x3e, 0xff, 0xba, 0x47, 0x9d, 0xa3, 0xaf, 0xff, 0x3b, 0x41, 0x67, 0x8a, 0xe6 };
void os_getNwkKey (u1_t* buf) {  memcpy(buf, APPKEY, 16);}

// The region to use, this just uses the first one (can be changed if
// multiple regions are enabled).
u1_t os_getRegion (void) { return LMIC_regionCode(0); }

// Schedule TX every this many milliseconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60000;

// Timestamp of last packet sent
uint32_t last_packet = 0;

const lmic_pinmap lmic_pins = {
    // NSS input pin for SPI communication (required)
    .nss = 36,
    // If needed, these pins control the RX/TX antenna switch (active
    // high outputs). When you have both, the antenna switch can
    // powerdown when unused. If you just have a RXTX pin it should
    // usually be assigned to .tx, reverting to RX mode when idle).
    //
    // The SX127x has an RXTX pin that can automatically control the
    // antenna switch (if internally connected on the transceiver
    // board). This pin is always active, so no configuration is needed
    // for that here.
    // On SX126x, the DIO2 can be used for the same thing, but this is
    // disabled by default. To enable this, set .tx to
    // LMIC_CONTROLLED_BY_DIO2 below (this seems to be common and
    // enabling it when not needed is probably harmless, unless DIO2 is
    // connected to GND or VCC directly inside the transceiver board).
    .tx = LMIC_CONTROLLED_BY_DIO2,
    // .tx = LMIC_UNUSED_PIN,
    .rx = LMIC_UNUSED_PIN,
    // Radio reset output pin (active high for SX1276, active low for
    // others). When omitted, reset is skipped which might cause problems.
    .rst = 37,
    // DIO input pins.
    //   For SX127x, LoRa needs DIO0 and DIO1, FSK needs DIO0, DIO1 and DIO2
    //   For SX126x, Only DIO1 is needed (so leave DIO0 and DIO2 as LMIC_UNUSED_PIN)
    .dio = {/* DIO0 */ LMIC_UNUSED_PIN, /* DIO1 */ 14, /* DIO2 */ LMIC_UNUSED_PIN},
    // Busy input pin (SX126x only). When omitted, a delay is used which might
    // cause problems.
    .busy = 13,
    // TCXO oscillator enable output pin (active high).
    //
    // For SX127x this should be an I/O pin that controls the TCXO, or
    // LMIC_UNUSED_PIN when a crystal is used instead of a TCXO.
    //
    // For SX126x this should be LMIC_CONTROLLED_BY_DIO3 when a TCXO is
    // directly connected to the transceiver DIO3 to let the transceiver
    // start and stop the TCXO, or LMIC_UNUSED_PIN when a crystal is
    // used instead of a TCXO. Controlling the TCXO from the MCU is not
    // supported.
    .tcxo = LMIC_UNUSED_PIN,
    .spi = {47, 48, 35}
};

void onLmicEvent (ev_t ev) {
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            ESP_LOGI(TAG, "EV_SCAN_TIMEOUT");
            break;
        case EV_BEACON_FOUND:
            ESP_LOGI(TAG, "EV_BEACON_FOUND");
            break;
        case EV_BEACON_MISSED:
            ESP_LOGI(TAG, "EV_BEACON_MISSED");
            break;
        case EV_BEACON_TRACKED:
            ESP_LOGI(TAG, "EV_BEACON_TRACKED");
            break;
        case EV_JOINING:
            ESP_LOGI(TAG, "EV_JOINING");
            break;
        case EV_JOINED:
            ESP_LOGI(TAG, "EV_JOINED");

            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            break;
        case EV_RFU1:
            ESP_LOGI(TAG, "EV_RFU1");
            break;
        case EV_JOIN_FAILED:
            ESP_LOGI(TAG, "EV_JOIN_FAILED");
            break;
        case EV_REJOIN_FAILED:
            ESP_LOGI(TAG, "EV_REJOIN_FAILED");
            break;
            break;
        case EV_TXCOMPLETE:
            ESP_LOGI(TAG, "EV_TXCOMPLETE (includes waiting for RX windows)");
            if (LMIC.txrxFlags & TXRX_ACK)
              ESP_LOGI(TAG, "Received ack");
            if (LMIC.dataLen) {
              ESP_LOGI(TAG, "Received %i bytes of payload", LMIC.dataLen);
            }
            break;
        case EV_LOST_TSYNC:
            ESP_LOGI(TAG, "EV_LOST_TSYNC");
            break;
        case EV_RESET:
            ESP_LOGI(TAG, "EV_RESET");
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            ESP_LOGI(TAG, "EV_RXCOMPLETE");
            break;
        case EV_LINK_DEAD:
            ESP_LOGI(TAG, "EV_LINK_DEAD");
            break;
        case EV_LINK_ALIVE:
            ESP_LOGI(TAG, "EV_LINK_ALIVE");
            break;
        case EV_SCAN_FOUND:
            ESP_LOGI(TAG, "EV_SCAN_FOUND");
            break;
        case EV_TXSTART:
            ESP_LOGI(TAG, "EV_TXSTART");
            break;
        case EV_TXDONE:
            ESP_LOGI(TAG, "EV_TXDONE");
            break;
        case EV_DATARATE:
            ESP_LOGI(TAG, "EV_DATARATE");
            break;
        case EV_START_SCAN:
            ESP_LOGI(TAG, "EV_START_SCAN");
            break;
        case EV_ADR_BACKOFF:
            ESP_LOGI(TAG, "EV_ADR_BACKOFF");
            break;

         default:
            ESP_LOGI(TAG, "Unknown event: %s", ev);
            break;
    }
}

void send_packet(){
    // Prepare upstream data transmission at the next possible time.
    uint8_t mydata[] = "Hello, world!";
    LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
    ESP_LOGI(TAG, "Packet queued");

    last_packet = esp_timer_get_time()/1000;
}

void main_task(void *args) {
    // Let LMIC handle background tasks
    os_runstep();

    // If TX_INTERVAL passed, *and* our previous packet is not still
    // pending (which can happen due to duty cycle limitations), send
    // the next packet.
    if (esp_timer_get_time()/1000 - last_packet > TX_INTERVAL && !(LMIC.opmode & (OP_JOINING|OP_TXRXPEND)))
        send_packet();
}

void app_main() {
    ESP_LOGI(TAG, "Starting");

    // LMIC init
    os_init(NULL);
    LMIC_reset();

    // Enable this to increase the receive window size, to compensate
    // for an inaccurate clock.  // This compensate for +/- 10% clock
    // error, a lower value will likely be more appropriate.
    //LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);

    // Start join
    LMIC_startJoining();

    // Make sure the first packet is scheduled ASAP after join completes
    last_packet = esp_timer_get_time()/1000 - TX_INTERVAL;

    // Optionally wait for join to complete (uncomment this is you want
    // to run the loop while joining).
    while ((LMIC.opmode & (OP_JOINING)))
        os_runstep();

    xTaskCreate(main_task, "main_task", 4096, NULL, 6, NULL);

}