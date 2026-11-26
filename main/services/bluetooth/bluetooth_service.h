/*
 * Bluetooth service (ESP32 backend).
 *
 * Owns the ESP-IDF Bluetooth stack bring-up (controller + Bluedroid host) and
 * the generic HID-host machinery (discovery/scan + the single HID-host event
 * callback). Higher-level consumers (e.g. the DualSense transport) register one
 * listener and open devices through the wrappers below.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

#include "esp_err.h"
#include "esp_bt_defs.h"
#include "esp_gap_bt_api.h"
#include "esp_hid_common.h"
#include "esp_hidh.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Controller modes. Values match ESP_BT_MODE_*. */
typedef enum {
    BLUETOOTH_MODE_BT   = 0x02,   /* classic only */
    BLUETOOTH_MODE_BLE  = 0x01,   /* BLE only */
    BLUETOOTH_MODE_BTDM = 0x03,   /* classic + BLE (dual mode) */
} bluetooth_mode_t;

/* A discovered HID device. */
typedef struct bluetooth_device {
    struct bluetooth_device *next;
    uint8_t bda[6];
    const char *name;
    int8_t rssi;
    esp_hid_transport_t transport;   /* BT or BLE */
    esp_bt_cod_t cod;                /* valid when transport == BT */
    uint8_t ble_addr_type;           /* valid when transport == BLE */
    uint16_t ble_appearance;         /* valid when transport == BLE */
} bluetooth_device_t;

typedef enum {
    BLUETOOTH_HID_OPEN = 0,
    BLUETOOTH_HID_CLOSE,
    BLUETOOTH_HID_INPUT,
    BLUETOOTH_HID_FEATURE,
    BLUETOOTH_HID_BATTERY,
} bluetooth_hid_event_t;

typedef struct {
    bluetooth_hid_event_t event;
    esp_hidh_dev_t *dev;
    uint8_t report_id;          /* INPUT/FEATURE */
    uint16_t length;            /* INPUT/FEATURE payload length */
    const uint8_t *data;        /* INPUT/FEATURE payload (valid for the call only) */
    uint8_t battery_level;      /* BATTERY */
    esp_err_t status;           /* OPEN/CLOSE/BATTERY/FEATURE */
    int reason;                 /* CLOSE (BLE) */
} bluetooth_hid_event_data_t;

typedef void (*bluetooth_hid_listener_t)(const bluetooth_hid_event_data_t *evt, void *arg);

/* Bring up the controller + Bluedroid host + HID-host stack. */
esp_err_t bluetooth_service_init(bluetooth_mode_t mode);

/* Register the single HID-host event listener (one consumer for now). */
esp_err_t bluetooth_service_register_hid_listener(bluetooth_hid_listener_t listener, void *arg);

/* Blocking scan for HID devices (classic inquiry + BLE scan). Caller owns the
 * returned list and must free it with bluetooth_service_scan_results_free(). */
esp_err_t bluetooth_service_scan(uint32_t seconds, size_t *num_results, bluetooth_device_t **results);
void bluetooth_service_scan_results_free(bluetooth_device_t *results);

/* HID device operations (thin wrappers around esp_hid). */
esp_hidh_dev_t *bluetooth_service_hid_open(const uint8_t bda[6], esp_hid_transport_t transport,
                                           uint8_t ble_addr_type);
esp_err_t bluetooth_service_hid_close(esp_hidh_dev_t *dev);
void bluetooth_service_hid_free(esp_hidh_dev_t *dev);
void bluetooth_service_hid_free_force(esp_hidh_dev_t *dev);
esp_err_t bluetooth_service_hid_send_output(esp_hidh_dev_t *dev, size_t map_index, size_t report_id,
                                            const uint8_t *data, size_t len);
esp_err_t bluetooth_service_hid_set_protocol(esp_hidh_dev_t *dev, uint8_t protocol_mode);
esp_err_t bluetooth_service_hid_set_idle(esp_hidh_dev_t *dev, uint8_t idle_time);

#ifdef __cplusplus
}
#endif
