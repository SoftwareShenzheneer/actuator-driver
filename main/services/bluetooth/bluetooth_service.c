/*
 * Bluetooth service (ESP32 backend): controller + Bluedroid + HID-host bring-up.
 *
 * Adapted from the esp_hid_host example (examples/bluetooth/esp_hid_host) so the
 * discovery/scan glue is self-contained rather than living in the application.
 */
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"

#if CONFIG_BT_BLE_ENABLED
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#endif

#include "bluetooth_service.h"

static const char *TAG = "bluetooth";

static SemaphoreHandle_t s_bt_sem;    /* signaled when BT discovery completes */
static SemaphoreHandle_t s_ble_sem;   /* signaled when BLE scan completes */

static bluetooth_device_t *s_bt_results;
static size_t s_bt_count;
static bluetooth_device_t *s_ble_results;
static size_t s_ble_count;

static bluetooth_hid_listener_t s_hid_listener;
static void *s_hid_listener_arg;

static bluetooth_mode_t s_mode;
static bool s_initialized;

/* ------------------------------------------------------------------------- */
/* Scan result bookkeeping                                                   */
/* ------------------------------------------------------------------------- */

#if CONFIG_BT_HID_HOST_ENABLED
static bluetooth_device_t *find_bt_result(const uint8_t *bda)
{
    for (bluetooth_device_t *r = s_bt_results; r != NULL; r = r->next) {
        if (memcmp(bda, r->bda, sizeof(esp_bd_addr_t)) == 0) {
            return r;
        }
    }
    return NULL;
}

static void add_bt_result(const uint8_t *bda, const esp_bt_cod_t *cod,
                          uint8_t *name, uint8_t name_len, int8_t rssi)
{
    bluetooth_device_t *r = find_bt_result(bda);
    if (r != NULL) {
        /* Some properties can arrive after the initial discovery result. */
        if (r->name == NULL && name != NULL && name_len > 0) {
            char *name_s = (char *)malloc(name_len + 1);
            if (name_s != NULL) {
                memcpy(name_s, name, name_len);
                name_s[name_len] = 0;
                r->name = name_s;
            }
        }
        if (rssi != 0) {
            r->rssi = rssi;
        }
        return;
    }

    r = (bluetooth_device_t *)calloc(1, sizeof(*r));
    if (r == NULL) {
        return;
    }

    r->transport = ESP_HID_TRANSPORT_BT;
    memcpy(r->bda, bda, sizeof(r->bda));
    r->cod = *cod;
    r->rssi = rssi;

    if (name != NULL && name_len > 0) {
        char *name_s = (char *)malloc(name_len + 1);
        if (name_s == NULL) {
            free(r);
            return;
        }
        memcpy(name_s, name, name_len);
        name_s[name_len] = 0;
        r->name = name_s;
    }

    r->next = s_bt_results;
    s_bt_results = r;
    s_bt_count++;
}
#endif

#if CONFIG_BT_BLE_ENABLED
static void add_ble_result(const uint8_t *bda, uint8_t addr_type, uint16_t appearance,
                           uint8_t *name, uint8_t name_len, int8_t rssi)
{
    bluetooth_device_t *r = (bluetooth_device_t *)calloc(1, sizeof(*r));
    if (r == NULL) {
        return;
    }

    r->transport = ESP_HID_TRANSPORT_BLE;
    memcpy(r->bda, bda, sizeof(r->bda));
    r->ble_addr_type = addr_type;
    r->ble_appearance = appearance;
    r->rssi = rssi;

    if (name != NULL && name_len > 0) {
        char *name_s = (char *)malloc(name_len + 1);
        if (name_s == NULL) {
            free(r);
            return;
        }
        memcpy(name_s, name, name_len);
        name_s[name_len] = 0;
        r->name = name_s;
    }

    r->next = s_ble_results;
    s_ble_results = r;
    s_ble_count++;
}
#endif

/* ------------------------------------------------------------------------- */
/* Classic BT GAP                                                            */
/* ------------------------------------------------------------------------- */

#if CONFIG_BT_HID_HOST_ENABLED
static void handle_bt_device_result(const esp_bt_gap_cb_param_t *disc_res)
{
    uint32_t codv = 0;
    uint8_t *name = NULL;
    uint8_t name_len = 0;
    int8_t rssi = 0;
    esp_bt_cod_t *cod = (esp_bt_cod_t *)&codv;

    for (int i = 0; i < disc_res->disc_res.num_prop; i++) {
        const esp_bt_gap_dev_prop_t *prop = &disc_res->disc_res.prop[i];

        if (prop->type == ESP_BT_GAP_DEV_PROP_BDNAME) {
            name = (uint8_t *)prop->val;
            name_len = strlen((const char *)name);
        } else if (prop->type == ESP_BT_GAP_DEV_PROP_RSSI) {
            rssi = *((int8_t *)prop->val);
        } else if (prop->type == ESP_BT_GAP_DEV_PROP_COD) {
            memcpy(&codv, prop->val, sizeof(uint32_t));
        } else if (prop->type == ESP_BT_GAP_DEV_PROP_EIR && name == NULL) {
            uint8_t len = 0;
            uint8_t *data = esp_bt_gap_resolve_eir_data((uint8_t *)prop->val,
                                                        ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &len);
            if (data == NULL) {
                data = esp_bt_gap_resolve_eir_data((uint8_t *)prop->val,
                                                   ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &len);
            }
            if (data != NULL && len > 0) {
                name = data;
                name_len = len;
            }
        }
    }

    if (cod->major == ESP_BT_COD_MAJOR_DEV_PERIPHERAL || find_bt_result(disc_res->disc_res.bda) != NULL) {
        add_bt_result(disc_res->disc_res.bda, cod, name, name_len, rssi);
    }
}

static void bt_gap_event_handler(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
            xSemaphoreGive(s_bt_sem);
        }
        break;
    case ESP_BT_GAP_DISC_RES_EVT:
        handle_bt_device_result(param);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(TAG, "passkey: %06" PRIu32, param->key_notif.passkey);
        break;
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(TAG, "confirm numeric value: %06" PRIu32, param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_PIN_REQ_EVT:
        ESP_LOGI(TAG, "pin request (min_16_digit=%d)", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            esp_bt_pin_code_t pin = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin);
        } else {
            esp_bt_pin_code_t pin = {'1', '2', '3', '4'};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin);
        }
        break;
    default:
        break;
    }
}

static esp_err_t init_bt_gap(void)
{
    esp_err_t ret;

    /* Secure Simple Pairing with numeric comparison (auto-accepted). */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

    /* Legacy pairing fallback. */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    ret = esp_bt_gap_register_callback(bt_gap_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_bt_gap_register_callback failed: %d", ret);
        return ret;
    }

    ret = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_bt_gap_set_scan_mode failed: %d", ret);
        return ret;
    }

    return ESP_OK;
}

static esp_err_t start_bt_scan(uint32_t seconds)
{
    esp_err_t ret = esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY,
                                               (int)(seconds / 1.28), 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_bt_gap_start_discovery failed: %d", ret);
    }
    return ret;
}
#endif /* CONFIG_BT_HID_HOST_ENABLED */

/* ------------------------------------------------------------------------- */
/* BLE GAP (scan only; host role)                                            */
/* ------------------------------------------------------------------------- */

#if CONFIG_BT_BLE_ENABLED
static void ble_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        xSemaphoreGive(s_ble_sem);
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        switch (param->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT: {
            uint16_t appearance = 0;
            uint8_t name[64] = {0};
            uint8_t name_len = 0;

            uint8_t app_len = 0;
            uint8_t *app = esp_ble_resolve_adv_data_by_type(param->scan_rst.ble_adv,
                                                            param->scan_rst.adv_data_len + param->scan_rst.scan_rsp_len,
                                                            ESP_BLE_AD_TYPE_APPEARANCE, &app_len);
            if (app != NULL && app_len >= 2) {
                appearance = app[0] | (app[1] << 8);
            }

            uint8_t *adv_name = esp_ble_resolve_adv_data_by_type(param->scan_rst.ble_adv,
                                                                 param->scan_rst.adv_data_len + param->scan_rst.scan_rsp_len,
                                                                 ESP_BLE_AD_TYPE_NAME_CMPL, &name_len);
            if (adv_name == NULL) {
                adv_name = esp_ble_resolve_adv_data_by_type(param->scan_rst.ble_adv,
                                                            param->scan_rst.adv_data_len + param->scan_rst.scan_rsp_len,
                                                            ESP_BLE_AD_TYPE_NAME_SHORT, &name_len);
            }
            if (adv_name != NULL && name_len > 0 && name_len < sizeof(name)) {
                memcpy(name, adv_name, name_len);
                name[name_len] = 0;
            }

            /* HID service UUID (0x1812) must be advertised. */
            uint8_t uuid_len = 0;
            uint8_t *uuid = esp_ble_resolve_adv_data_by_type(param->scan_rst.ble_adv,
                                                             param->scan_rst.adv_data_len + param->scan_rst.scan_rsp_len,
                                                             ESP_BLE_AD_TYPE_16SRV_CMPL, &uuid_len);
            uint16_t svc = 0;
            if (uuid != NULL && uuid_len >= 2) {
                svc = uuid[0] | (uuid[1] << 8);
            }

            if (svc == ESP_GATT_UUID_HID_SVC) {
                add_ble_result(param->scan_rst.bda, param->scan_rst.ble_addr_type, appearance,
                               name_len > 0 ? name : NULL, name_len, param->scan_rst.rssi);
            }
            break;
        }
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            xSemaphoreGive(s_ble_sem);
            break;
        default:
            break;
        }
        break;
    }
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        if (!param->ble_security.auth_cmpl.success) {
            ESP_LOGE(TAG, "BLE auth failed: 0x%x", param->ble_security.auth_cmpl.fail_reason);
        }
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    default:
        break;
    }
}

static esp_ble_scan_params_t s_ble_scan_params = {
    .scan_type          = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval      = 0x50,
    .scan_window        = 0x30,
    .scan_duplicate     = BLE_SCAN_DUPLICATE_ENABLE,
};

static esp_err_t init_ble_gap(void)
{
    esp_err_t ret = esp_ble_gap_register_callback(ble_gap_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_ble_gap_register_callback failed: %d", ret);
    }
    return ret;
}

static esp_err_t start_ble_scan(uint32_t seconds)
{
    esp_err_t ret = esp_ble_gap_set_scan_params(&s_ble_scan_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_ble_gap_set_scan_params failed: %d", ret);
        return ret;
    }
    xSemaphoreTake(s_ble_sem, portMAX_DELAY);

    ret = esp_ble_gap_start_scanning(seconds);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_ble_gap_start_scanning failed: %d", ret);
    }
    return ret;
}
#endif /* CONFIG_BT_BLE_ENABLED */

/* ------------------------------------------------------------------------- */
/* HID host event dispatch                                                   */
/* ------------------------------------------------------------------------- */

static void hidh_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    (void)handler_args;
    (void)base;

    esp_hidh_event_t event = (esp_hidh_event_t)id;
    esp_hidh_event_data_t *p = (esp_hidh_event_data_t *)event_data;

    if (s_hid_listener == NULL) {
        return;
    }

    bluetooth_hid_event_data_t e;
    memset(&e, 0, sizeof(e));

    switch (event) {
    case ESP_HIDH_OPEN_EVENT:
        e.event = BLUETOOTH_HID_OPEN;
        e.dev = p->open.dev;
        e.status = p->open.status;
        break;
    case ESP_HIDH_CLOSE_EVENT:
        e.event = BLUETOOTH_HID_CLOSE;
        e.dev = p->close.dev;
        e.status = p->close.status;
        e.reason = p->close.reason;
        break;
    case ESP_HIDH_INPUT_EVENT:
        e.event = BLUETOOTH_HID_INPUT;
        e.dev = p->input.dev;
        e.report_id = p->input.report_id;
        e.length = p->input.length;
        e.data = p->input.data;
        break;
    case ESP_HIDH_FEATURE_EVENT:
        e.event = BLUETOOTH_HID_FEATURE;
        e.dev = p->feature.dev;
        e.report_id = p->feature.report_id;
        e.length = p->feature.length;
        e.data = p->feature.data;
        e.status = p->feature.status;
        break;
    case ESP_HIDH_BATTERY_EVENT:
        e.event = BLUETOOTH_HID_BATTERY;
        e.dev = p->battery.dev;
        e.battery_level = p->battery.level;
        e.status = p->battery.status;
        break;
    default:
        return;   /* START/STOP/CONN_REQUEST are not forwarded */
    }

    s_hid_listener(&e, s_hid_listener_arg);
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */

esp_err_t bluetooth_service_init(bluetooth_mode_t mode)
{
    if (s_initialized) {
        return ESP_OK;
    }

    esp_err_t ret;
    ESP_LOGI(TAG, "init: start mode=%d free_heap=%u", mode, (unsigned)esp_get_free_heap_size());

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
#if CONFIG_IDF_TARGET_ESP32
    bt_cfg.mode = mode;
#endif

    if (!(mode & ESP_BT_MODE_CLASSIC_BT)) {
        ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "esp_bt_controller_mem_release failed: %d", ret);
            return ret;
        }
    }

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_bt_controller_init failed: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "init: controller_init ok free_heap=%u", (unsigned)esp_get_free_heap_size());

    ret = esp_bt_controller_enable(mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_bt_controller_enable failed: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "init: controller_enable ok free_heap=%u", (unsigned)esp_get_free_heap_size());

    /* Boost classic-BT TX power. Some modules (e.g. the DengFOC board's ESP32)
     * have a weaker antenna: discovery (inquiry) succeeds at the default +3 dBm,
     * but the tighter bidirectional page can time out. */
    if (esp_bredr_tx_power_set(ESP_PWR_LVL_P9, ESP_PWR_LVL_P9) == ESP_OK) {
        ESP_LOGI(TAG, "init: br_edr tx power set to +9 dBm");
    } else {
        ESP_LOGW(TAG, "init: failed to set br_edr tx power");
    }

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_bluedroid_init failed: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "init: bluedroid_init ok free_heap=%u", (unsigned)esp_get_free_heap_size());

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_bluedroid_enable failed: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "init: bluedroid_enable ok free_heap=%u", (unsigned)esp_get_free_heap_size());

    s_bt_sem = xSemaphoreCreateBinary();
    s_ble_sem = xSemaphoreCreateBinary();
    if (s_bt_sem == NULL || s_ble_sem == NULL) {
        ESP_LOGE(TAG, "failed to create semaphores");
        return ESP_ERR_NO_MEM;
    }

#if CONFIG_BT_HID_HOST_ENABLED
    if (mode & ESP_BT_MODE_CLASSIC_BT) {
        ret = init_bt_gap();
        if (ret != ESP_OK) {
            return ret;
        }
        ESP_LOGI(TAG, "init: bt_gap ok");
    }
#endif

#if CONFIG_BT_BLE_ENABLED
    if (mode & ESP_BT_MODE_BLE) {
        ret = init_ble_gap();
        if (ret != ESP_OK) {
            return ret;
        }
        ESP_LOGI(TAG, "init: ble_gap ok");

        ret = esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "esp_ble_gattc_register_callback failed: %d", ret);
            return ret;
        }
        ESP_LOGI(TAG, "init: gattc ok");
    }
#endif

    esp_hidh_config_t config = {
        .callback = hidh_event_handler,
        .event_stack_size = 4096,
        .callback_arg = NULL,
    };
    ret = esp_hidh_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_hidh_init failed: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "init: hidh ok");

    s_mode = mode;
    s_initialized = true;
    ESP_LOGI(TAG, "initialized (mode=%d), own addr: " ESP_BD_ADDR_STR,
             mode, ESP_BD_ADDR_HEX(esp_bt_dev_get_address()));
    return ESP_OK;
}

esp_err_t bluetooth_service_register_hid_listener(bluetooth_hid_listener_t listener, void *arg)
{
    if (listener == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    s_hid_listener = listener;
    s_hid_listener_arg = arg;
    return ESP_OK;
}

esp_err_t bluetooth_service_scan(uint32_t seconds, size_t *num_results, bluetooth_device_t **results)
{
    if (num_results == NULL || results == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_bt_results != NULL || s_ble_results != NULL) {
        ESP_LOGE(TAG, "stale scan results; free them first");
        return ESP_FAIL;
    }

#if CONFIG_BT_BLE_ENABLED
    if (s_mode & ESP_BT_MODE_BLE) {
        esp_err_t ret = start_ble_scan(seconds);
        if (ret != ESP_OK) {
            return ret;
        }
        xSemaphoreTake(s_ble_sem, portMAX_DELAY);
    }
#endif

#if CONFIG_BT_HID_HOST_ENABLED
    if (s_mode & ESP_BT_MODE_CLASSIC_BT) {
        esp_err_t ret = start_bt_scan(seconds);
        if (ret != ESP_OK) {
            return ret;
        }
        xSemaphoreTake(s_bt_sem, portMAX_DELAY);
    }
#endif

    *num_results = s_bt_count + s_ble_count;
    *results = s_bt_results;

    if (s_bt_results != NULL) {
        bluetooth_device_t *tail = s_bt_results;
        while (tail->next != NULL) {
            tail = tail->next;
        }
        tail->next = s_ble_results;
    } else {
        *results = s_ble_results;
    }

    s_bt_results = NULL;
    s_bt_count = 0;
    s_ble_results = NULL;
    s_ble_count = 0;
    return ESP_OK;
}

void bluetooth_service_scan_results_free(bluetooth_device_t *results)
{
    while (results != NULL) {
        bluetooth_device_t *r = results;
        results = results->next;
        if (r->name != NULL) {
            free((char *)r->name);
        }
        free(r);
    }
}

esp_hidh_dev_t *bluetooth_service_hid_open(const uint8_t bda[6], esp_hid_transport_t transport,
                                           uint8_t ble_addr_type)
{
    esp_bd_addr_t addr;
    memcpy(addr, bda, sizeof(addr));
    return esp_hidh_dev_open(addr, transport, ble_addr_type);
}

esp_err_t bluetooth_service_hid_close(esp_hidh_dev_t *dev)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return esp_hidh_dev_close(dev);
}

void bluetooth_service_hid_free(esp_hidh_dev_t *dev)
{
    if (dev != NULL) {
        esp_hidh_dev_free(dev);
    }
}

/* esp_hidh internal: actually removes a device from the host device list. The
 * public esp_hidh_dev_free() is a no-op, so expose this to clear a lingering
 * entry left behind after a failed page (which otherwise causes "Already
 * Connected" on the next open). */
extern esp_err_t esp_hidh_dev_free_inner(esp_hidh_dev_t *dev);

void bluetooth_service_hid_free_force(esp_hidh_dev_t *dev)
{
    if (dev != NULL) {
        esp_hidh_dev_free_inner(dev);
    }
}

esp_err_t bluetooth_service_hid_send_output(esp_hidh_dev_t *dev, size_t map_index, size_t report_id,
                                            const uint8_t *data, size_t len)
{
    if (dev == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return esp_hidh_dev_output_set(dev, map_index, report_id, (uint8_t *)data, len);
}

esp_err_t bluetooth_service_hid_set_protocol(esp_hidh_dev_t *dev, uint8_t protocol_mode)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return esp_hidh_dev_set_protocol(dev, protocol_mode);
}

esp_err_t bluetooth_service_hid_set_idle(esp_hidh_dev_t *dev, uint8_t idle_time)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return esp_hidh_dev_set_idle(dev, idle_time);
}
