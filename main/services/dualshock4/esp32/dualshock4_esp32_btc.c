/*
 * DualShock 4 transport backend for ESP32 Bluetooth Classic (BTC).
 *
 * Implements dualshock4_transport_ops_t on top of the bluetooth_service
 * (esp_hid host). Discovery -> open -> report streaming all happen here; the
 * portable DualShock 4 core above this file does not see any ESP-IDF types.
 */
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "nvs.h"

#include "dualshock4_esp32_btc.h"
#include "dualshock4_types.h"
#include "bluetooth_service.h"

#define DUALSHOCK4_PEER_NAME   CONFIG_DUALSHOCK4_PEER_NAME
#define SCAN_DURATION_SECONDS  5

/* After a reset, reconnect to the last-known controller for this long before
 * falling back to a fresh discovery scan. */
#define RECONNECT_TIMEOUT_MS   10000

/* A DS4/DualSense in pairing mode alternates inquiry/page scan, so a single page
 * can time out even when the controller is connectable. Retry the open. */
#define OPEN_MAX_ATTEMPTS      3
#define OPEN_TIMEOUT_MS        8000    /* > the ~5.1 s classic page timeout */

/* NVS location of the last-known controller address (6-byte BD_ADDR blob). */
#define NVS_NAMESPACE          "dualshock4"
#define NVS_KEY_BDA            "bda"

/* Event-group bits used to signal the result of a reconnect attempt from the
 * HID-host event callback back to the connect task. */
#define RECONN_BIT_CONNECTED   BIT0
#define RECONN_BIT_FAILED      BIT1

static const char *TAG = "dualshock4_btc";

typedef struct {
    dualshock4_transport_t *transport;   /* back-pointer to the transport */
    const dualshock4_sink_t *sink;       /* set by open() */
    esp_hidh_dev_t *dev;
    bool connected;
    char name[32];                       /* scan-time name (SDP name comes back empty) */
} dualshock4_esp32_btc_t;

static dualshock4_esp32_btc_t s_self;
static dualshock4_transport_t s_transport;
static EventGroupHandle_t s_reconn_evt;   /* signals a reconnect attempt's outcome */

/* ------------------------------------------------------------------------- */
/* NVS persistence of the last-known controller address                      */
/* ------------------------------------------------------------------------- */

static esp_err_t nvs_save_bda(const uint8_t bda[6])
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_blob(handle, NVS_KEY_BDA, bda, 6);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

static bool nvs_load_bda(uint8_t bda[6])
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }

    size_t len = 6;
    esp_err_t err = nvs_get_blob(handle, NVS_KEY_BDA, bda, &len);
    nvs_close(handle);
    return (err == ESP_OK && len == 6);
}

static void signal_reconn(EventBits_t bits)
{
    if (s_reconn_evt != NULL) {
        xEventGroupSetBits(s_reconn_evt, bits);
    }
}

/* HID host events are delivered here from the bluetooth_service. */
static void on_hid_event(const bluetooth_hid_event_data_t *evt, void *arg)
{
    dualshock4_esp32_btc_t *self = (dualshock4_esp32_btc_t *)arg;

    switch (evt->event) {
    case BLUETOOTH_HID_OPEN:
        if (evt->status == ESP_OK) {
            self->connected = true;
            signal_reconn(RECONN_BIT_CONNECTED);

            /* Remember the address so the next boot can reconnect directly. */
            const uint8_t *bda = esp_hidh_dev_bda_get(evt->dev);
            if (bda != NULL && nvs_save_bda(bda) != ESP_OK) {
                ESP_LOGW(TAG, "failed to persist controller address");
            }

            const char *name = self->name[0] ? self->name :
                               (esp_hidh_dev_name_get(evt->dev) ? esp_hidh_dev_name_get(evt->dev) : "?");
            ESP_LOGI(TAG, "connected: %s (VID 0x%04x, PID 0x%04x, transport %d)",
                     name,
                     esp_hidh_dev_vendor_id_get(evt->dev),
                     esp_hidh_dev_product_id_get(evt->dev),
                     esp_hidh_dev_transport_get(evt->dev));
            if (self->sink != NULL && self->sink->on_conn != NULL) {
                self->sink->on_conn(self->sink->ctx, DUALSHOCK4_CONN_CONNECTED);
            }
        } else {
            ESP_LOGE(TAG, "open failed: %d", evt->status);
            self->connected = false;
            self->dev = NULL;   /* esp_hid auto-frees a failed-open device */
            signal_reconn(RECONN_BIT_FAILED);
            if (self->sink != NULL && self->sink->on_conn != NULL) {
                self->sink->on_conn(self->sink->ctx, DUALSHOCK4_CONN_ERROR);
            }
        }
        break;

    case BLUETOOTH_HID_INPUT: {
        /* Log raw reports only every ~5 s. A full 0x11 stream at 250 Hz would
         * otherwise flood the UART and starve the BT stack (link drop). */
        static TickType_t last_log = 0;
        TickType_t now = xTaskGetTickCount();
        if ((now - last_log) >= pdMS_TO_TICKS(5000)) {
            last_log = now;
            ESP_LOGI(TAG, "input: report_id=0x%02x len=%u", evt->report_id, evt->length);
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, evt->data, evt->length, ESP_LOG_INFO);
        }
        if (self->sink != NULL && self->sink->on_input != NULL) {
            self->sink->on_input(self->sink->ctx, evt->report_id, evt->data, evt->length);
        }
        break;
    }

    case BLUETOOTH_HID_CLOSE:
        signal_reconn(RECONN_BIT_FAILED);
        if (self->connected) {
            self->connected = false;
            if (self->sink != NULL && self->sink->on_conn != NULL) {
                self->sink->on_conn(self->sink->ctx, DUALSHOCK4_CONN_DISCONNECTED);
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

typedef enum {
    OPEN_CONNECTED = 0,
    OPEN_FAILED,       /* OPEN-error / CLOSE event reported a failure */
    OPEN_TIMEOUT,      /* no event before the timeout elapsed */
} open_result_t;

/* Issue one esp_hidh_dev_open and wait for the asynchronous OPEN/CLOSE result. */
static open_result_t open_once(dualshock4_esp32_btc_t *self, const uint8_t bda[6],
                               uint32_t timeout_ms)
{
    xEventGroupClearBits(s_reconn_evt, RECONN_BIT_CONNECTED | RECONN_BIT_FAILED);

    self->dev = bluetooth_service_hid_open(bda, ESP_HID_TRANSPORT_BT, 0);
    if (self->dev == NULL) {
        return OPEN_FAILED;
    }

    EventBits_t bits = xEventGroupWaitBits(s_reconn_evt,
                                           RECONN_BIT_CONNECTED | RECONN_BIT_FAILED,
                                           pdTRUE,          /* clear bits on exit */
                                           pdFALSE,         /* wait for any bit */
                                           pdMS_TO_TICKS(timeout_ms));

    if (bits & RECONN_BIT_CONNECTED) {
        return OPEN_CONNECTED;
    }

    if (bits & RECONN_BIT_FAILED) {
        /* The OPEN-error / CLOSE handler already freed the device. */
        return OPEN_FAILED;
    }

    /* Timed out before any event: abort the in-flight connection and force-free
     * the device entry. esp_hid keeps the entry after a failed page, which would
     * otherwise make the next open fail with "Already Connected". */
    if (self->dev != NULL) {
        bluetooth_service_hid_close(self->dev);
        bluetooth_service_hid_free_force(self->dev);
        self->dev = NULL;
    }
    return OPEN_TIMEOUT;
}

static void scan_and_connect(dualshock4_esp32_btc_t *self)
{
    size_t num = 0;
    bluetooth_device_t *results = NULL;
    bluetooth_device_t *found = NULL;
    uint8_t bda[6];

    ESP_LOGI(TAG, "scanning for '%s'...", DUALSHOCK4_PEER_NAME);

    if (bluetooth_service_scan(SCAN_DURATION_SECONDS, &num, &results) != ESP_OK) {
        ESP_LOGW(TAG, "scan failed");
        return;
    }

    for (bluetooth_device_t *r = results; r != NULL; r = r->next) {
        if (r->transport == ESP_HID_TRANSPORT_BT && r->name != NULL &&
            strncmp(r->name, DUALSHOCK4_PEER_NAME, strlen(DUALSHOCK4_PEER_NAME)) == 0) {
            found = r;
            break;
        }
    }

    if (found == NULL) {
        ESP_LOGW(TAG, "'%s' not found (%u devices seen)", DUALSHOCK4_PEER_NAME, (unsigned)num);
        bluetooth_service_scan_results_free(results);
        return;
    }

    if (found->name != NULL) {
        strncpy(self->name, found->name, sizeof(self->name) - 1);
        self->name[sizeof(self->name) - 1] = '\0';
    }
    memcpy(bda, found->bda, sizeof(bda));
    bluetooth_service_scan_results_free(results);

    ESP_LOGI(TAG, "opening " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(bda));

    /* A DS4 in pairing mode alternates inquiry/page scan, so a single page can
     * time out even when it is connectable. Retry the open a few times. */
    for (int attempt = 0; attempt < OPEN_MAX_ATTEMPTS; attempt++) {
        open_result_t r = open_once(self, bda, OPEN_TIMEOUT_MS);
        if (r == OPEN_CONNECTED) {
            break;
        }
        ESP_LOGW(TAG, "open attempt %d/%d failed (%s)", attempt + 1, OPEN_MAX_ATTEMPTS,
                 (r == OPEN_TIMEOUT) ? "timeout" : "sdp/page error");
        if (attempt + 1 < OPEN_MAX_ATTEMPTS) {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

/* Attempt a direct reconnect to `bda`. Blocks up to RECONNECT_TIMEOUT_MS for the
 * asynchronous OPEN/CLOSE result. Returns true if the controller connected. */
static bool try_reconnect(dualshock4_esp32_btc_t *self, const uint8_t bda[6])
{
    return open_once(self, bda, RECONNECT_TIMEOUT_MS) == OPEN_CONNECTED;
}

static void connect_task(void *arg)
{
    dualshock4_esp32_btc_t *self = (dualshock4_esp32_btc_t *)arg;
    uint8_t bda[6];

    /* Prefer reconnecting to the controller we used last time; only scan when
     * that address is unknown or the controller did not answer in time. */
    if (nvs_load_bda(bda)) {
        ESP_LOGI(TAG, "reconnecting to " ESP_BD_ADDR_STR " (up to %u ms)",
                 ESP_BD_ADDR_HEX(bda), (unsigned)RECONNECT_TIMEOUT_MS);
        if (try_reconnect(self, bda)) {
            ESP_LOGI(TAG, "reconnected");
            vTaskDelete(NULL);
            return;
        }
        ESP_LOGW(TAG, "reconnect failed; falling back to scan");
    }

    /* Keep scanning until the controller is put in pairing mode and connects. */
    while (!self->connected) {
        scan_and_connect(self);
        if (!self->connected) {
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }

    vTaskDelete(NULL);
}

static int esp32_btc_open(dualshock4_transport_t *t, const dualshock4_sink_t *sink)
{
    dualshock4_esp32_btc_t *self = (dualshock4_esp32_btc_t *)t->ctx;
    self->sink = sink;
    self->connected = false;

    if (xTaskCreate(connect_task, "ds4_btc", 4096, self, 5, NULL) != pdPASS) {
        return -1;
    }
    return 0;
}

static int esp32_btc_close(dualshock4_transport_t *t)
{
    dualshock4_esp32_btc_t *self = (dualshock4_esp32_btc_t *)t->ctx;
    if (self->dev != NULL) {
        bluetooth_service_hid_close(self->dev);
        self->dev = NULL;
    }
    self->connected = false;
    return 0;
}

static int esp32_btc_send_report(dualshock4_transport_t *t, const uint8_t *data, size_t len)
{
    dualshock4_esp32_btc_t *self = (dualshock4_esp32_btc_t *)t->ctx;
    if (self->dev == NULL || !self->connected) {
        return -1;
    }

    /* map_index 0 (single report map), report id 0x11 for the DualShock 4 output. */
    esp_err_t err = bluetooth_service_hid_send_output(self->dev, 0, DUALSHOCK4_OUTPUT_REPORT_ID,
                                                      data, len);
    return (err == ESP_OK) ? 0 : -1;
}

static const dualshock4_transport_ops_t s_ops = {
    .open        = esp32_btc_open,
    .close       = esp32_btc_close,
    .send_report = esp32_btc_send_report,
};

dualshock4_transport_t *dualshock4_esp32_btc_get_default(void)
{
    static bool initialized = false;

    if (!initialized) {
        s_transport.ops = &s_ops;
        s_transport.ctx = &s_self;
        s_self.transport = &s_transport;
        s_reconn_evt = xEventGroupCreate();
        bluetooth_service_register_hid_listener(on_hid_event, &s_self);
        initialized = true;
    }

    return &s_transport;
}
