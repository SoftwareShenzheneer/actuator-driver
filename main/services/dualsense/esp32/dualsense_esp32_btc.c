/*
 * DualSense transport backend for ESP32 Bluetooth Classic (BTC).
 *
 * Implements dualsense_transport_ops_t on top of the bluetooth_service
 * (esp_hid host). Discovery -> open -> report streaming all happen here; the
 * portable DualSense core above this file does not see any ESP-IDF types.
 */
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "dualsense_esp32_btc.h"
#include "dualsense_types.h"
#include "bluetooth_service.h"

#define DUALSENSE_PEER_NAME      CONFIG_DUALSENSE_PEER_NAME
#define SCAN_DURATION_SECONDS    5

static const char *TAG = "dualsense_btc";

typedef struct {
    dualsense_transport_t *transport;   /* back-pointer to the transport */
    const dualsense_sink_t *sink;       /* set by open() */
    esp_hidh_dev_t *dev;
    bool connected;
    char name[32];                      /* scan-time name (SDP name comes back empty) */
} dualsense_esp32_btc_t;

static dualsense_esp32_btc_t s_self;
static dualsense_transport_t s_transport;

/* HID host events are delivered here from the bluetooth_service. */
static void on_hid_event(const bluetooth_hid_event_data_t *evt, void *arg)
{
    dualsense_esp32_btc_t *self = (dualsense_esp32_btc_t *)arg;

    switch (evt->event) {
    case BLUETOOTH_HID_OPEN:
        if (evt->status == ESP_OK) {
            self->connected = true;

            /* esp_hid does not auto-negotiate the HID protocol mode; tell the
             * controller to use report mode and stream continuously. */
            bluetooth_service_hid_set_protocol(evt->dev, ESP_HID_PROTOCOL_MODE_REPORT);
            bluetooth_service_hid_set_idle(evt->dev, 0);

            const char *name = self->name[0] ? self->name :
                               (esp_hidh_dev_name_get(evt->dev) ? esp_hidh_dev_name_get(evt->dev) : "?");
            ESP_LOGI(TAG, "connected: %s (VID 0x%04x, PID 0x%04x, transport %d)",
                     name,
                     esp_hidh_dev_vendor_id_get(evt->dev),
                     esp_hidh_dev_product_id_get(evt->dev),
                     esp_hidh_dev_transport_get(evt->dev));
            if (self->sink != NULL && self->sink->on_conn != NULL) {
                self->sink->on_conn(self->sink->ctx, DUALSENSE_CONN_CONNECTED);
            }
        } else {
            ESP_LOGE(TAG, "open failed: %d", evt->status);
            if (self->sink != NULL && self->sink->on_conn != NULL) {
                self->sink->on_conn(self->sink->ctx, DUALSENSE_CONN_ERROR);
            }
        }
        break;

    case BLUETOOTH_HID_INPUT:
        /* Raw report at INFO so the byte layout can be validated on hardware. */
        ESP_LOGI(TAG, "input: report_id=0x%02x len=%u", evt->report_id, evt->length);
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, evt->data, evt->length, ESP_LOG_INFO);
        if (self->sink != NULL && self->sink->on_input != NULL) {
            self->sink->on_input(self->sink->ctx, evt->report_id, evt->data, evt->length);
        }
        break;

    case BLUETOOTH_HID_CLOSE:
        if (self->connected) {
            self->connected = false;
            if (self->sink != NULL && self->sink->on_conn != NULL) {
                self->sink->on_conn(self->sink->ctx, DUALSENSE_CONN_DISCONNECTED);
            }
        }
        if (evt->dev != NULL) {
            bluetooth_service_hid_free(evt->dev);
        }
        self->dev = NULL;
        break;

    default:
        break;
    }
}

static void scan_and_connect_task(void *arg)
{
    dualsense_esp32_btc_t *self = (dualsense_esp32_btc_t *)arg;
    size_t num = 0;
    bluetooth_device_t *results = NULL;
    bluetooth_device_t *found = NULL;

    ESP_LOGI(TAG, "scanning for '%s'...", DUALSENSE_PEER_NAME);

    if (bluetooth_service_scan(SCAN_DURATION_SECONDS, &num, &results) != ESP_OK) {
        ESP_LOGE(TAG, "scan failed");
        if (self->sink != NULL && self->sink->on_conn != NULL) {
            self->sink->on_conn(self->sink->ctx, DUALSENSE_CONN_ERROR);
        }
        vTaskDelete(NULL);
        return;
    }

    for (bluetooth_device_t *r = results; r != NULL; r = r->next) {
        if (r->transport == ESP_HID_TRANSPORT_BT && r->name != NULL &&
            strncmp(r->name, DUALSENSE_PEER_NAME, strlen(DUALSENSE_PEER_NAME)) == 0) {
            found = r;
            break;
        }
    }

    if (found == NULL) {
        ESP_LOGW(TAG, "'%s' not found (%u devices seen)", DUALSENSE_PEER_NAME, (unsigned)num);
        if (self->sink != NULL && self->sink->on_conn != NULL) {
            self->sink->on_conn(self->sink->ctx, DUALSENSE_CONN_ERROR);
        }
    } else {
        if (found->name != NULL) {
            strncpy(self->name, found->name, sizeof(self->name) - 1);
            self->name[sizeof(self->name) - 1] = '\0';
        }
        ESP_LOGI(TAG, "opening " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(found->bda));
        self->dev = bluetooth_service_hid_open(found->bda, ESP_HID_TRANSPORT_BT, 0);
        if (self->dev == NULL) {
            ESP_LOGE(TAG, "failed to open device");
            if (self->sink != NULL && self->sink->on_conn != NULL) {
                self->sink->on_conn(self->sink->ctx, DUALSENSE_CONN_ERROR);
            }
        }
        /* OPEN/ERROR is reported asynchronously through on_hid_event(). */
    }

    bluetooth_service_scan_results_free(results);
    vTaskDelete(NULL);
}

static int esp32_btc_open(dualsense_transport_t *t, const dualsense_sink_t *sink)
{
    dualsense_esp32_btc_t *self = (dualsense_esp32_btc_t *)t->ctx;
    self->sink = sink;
    self->connected = false;

    if (xTaskCreate(scan_and_connect_task, "ds_btc", 4096, self, 5, NULL) != pdPASS) {
        return -1;
    }
    return 0;
}

static int esp32_btc_close(dualsense_transport_t *t)
{
    dualsense_esp32_btc_t *self = (dualsense_esp32_btc_t *)t->ctx;
    if (self->dev != NULL) {
        bluetooth_service_hid_close(self->dev);
        self->dev = NULL;
    }
    self->connected = false;
    return 0;
}

static int esp32_btc_send_report(dualsense_transport_t *t, const uint8_t *data, size_t len)
{
    dualsense_esp32_btc_t *self = (dualsense_esp32_btc_t *)t->ctx;
    if (self->dev == NULL || !self->connected) {
        return -1;
    }

    /* map_index 0 (single report map), report id 0x31 for the DualSense output. */
    esp_err_t err = bluetooth_service_hid_send_output(self->dev, 0, DUALSENSE_OUTPUT_REPORT_ID,
                                                      data, len);
    return (err == ESP_OK) ? 0 : -1;
}

static const dualsense_transport_ops_t s_ops = {
    .open        = esp32_btc_open,
    .close       = esp32_btc_close,
    .send_report = esp32_btc_send_report,
};

dualsense_transport_t *dualsense_esp32_btc_get_default(void)
{
    static bool initialized = false;

    if (!initialized) {
        s_transport.ops = &s_ops;
        s_transport.ctx = &s_self;
        s_self.transport = &s_transport;
        bluetooth_service_register_hid_listener(on_hid_event, &s_self);
        initialized = true;
    }

    return &s_transport;
}
