/*
 * Local low-side current sense for the ESP32 (MCPWM + ADC1), differential shunt.
 *
 * GPIO 34 = Vp (positive end of the shunt), GPIO 35 = Vn (negative end).
 * Current = (Vp - Vn) / (shunt_resistor * amp_gain). Sampling is triggered by an
 * MCPWM comparator at ~70% of the PWM period (the correct low-side conduction
 * instant), instead of the esp_simplefoc on_full event which reads ~0 A.
 */
#include "local_lowside_cs.h"

#include "esp_hal_bldc_6pwm.h"   /* EspMcpwmDriverParams */
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "local_lowside_cs";

static bool IRAM_ATTR cs_on_reach(mcpwm_cmpr_handle_t cmpr,
                                  const mcpwm_compare_event_data_t *edata,
                                  void *user)
{
    (void)cmpr;
    (void)edata;
    LocalLowSideCurrentSense *cs = (LocalLowSideCurrentSense *)user;
    cs->event_count++;
    return false;
}

LocalLowSideCurrentSense::LocalLowSideCurrentSense(float shunt, float gain,
                                                   int a, int b, int c)
{
    pinA = a;   /* Vp */
    pinB = b;   /* Vn */
    pinC = c;
    volts_to_amps_ = 1.0f / shunt / gain;
    gain_a = volts_to_amps_;
    gain_b = volts_to_amps_;
    gain_c = volts_to_amps_;
}

int LocalLowSideCurrentSense::init()
{
    if (driver == nullptr || driver_type != DriverType::BLDC) {
        ESP_LOGE(TAG, "BLDC driver not linked");
        return 0;
    }

    /* --- ADC1 setup (2 channels: Vp, Vn) ----------------------------------- */
    adc_oneshot_unit_init_cfg_t adc_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    if (adc_oneshot_new_unit(&adc_cfg, &adc1_) != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_new_unit failed");
        return 0;
    }

    int pins[2] = { pinA, pinB };
    for (int i = 0; i < 2; i++) {
        adc_unit_t unit;
        adc_channel_t ch;
        if (adc_oneshot_io_to_channel(pins[i], &unit, &ch) != ESP_OK || unit != ADC_UNIT_1) {
            ESP_LOGE(TAG, "GPIO%d is not an ADC1 pin", pins[i]);
            return 0;
        }

        adc_oneshot_chan_cfg_t ch_cfg = {
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_12,
        };
        if (adc_oneshot_config_channel(adc1_, ch, &ch_cfg) != ESP_OK) {
            ESP_LOGE(TAG, "adc_oneshot_config_channel failed for GPIO%d", pins[i]);
            return 0;
        }

        adc_cali_line_fitting_config_t cali_cfg = {
            .unit_id = ADC_UNIT_1,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_12,
        };
        adc_cali_create_scheme_line_fitting(&cali_cfg, &cali_[i]);

        chan_[i] = ch;
    }
    n_chan_ = 2;

    /* --- MCPWM comparator at the low-side conduction point ----------------- */
    EspMcpwmDriverParams *dp = (EspMcpwmDriverParams *)driver->params;
    if (dp == nullptr || dp->oper[0] == nullptr || dp->period_ticks == 0) {
        ESP_LOGE(TAG, "driver params missing oper/timer/period");
        return 0;
    }

    mcpwm_comparator_config_t cmp_cfg = {};
    if (mcpwm_new_comparator(dp->oper[0], &cmp_cfg, &cmpr_) != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_new_comparator failed");
        return 0;
    }

    uint32_t cmp_ticks = (dp->period_ticks * 70u) / 100u;
    if (mcpwm_comparator_set_compare_value(cmpr_, cmp_ticks) != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_comparator_set_compare_value failed");
        return 0;
    }

    mcpwm_comparator_event_callbacks_t cbs = { .on_reach = cs_on_reach };
    if (mcpwm_comparator_register_event_callbacks(cmpr_, &cbs, this) != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_comparator_register_event_callbacks failed");
        return 0;
    }

    /* --- zero-current differential offset ---------------------------------- */
    float sum = 0.0f;
    for (int k = 0; k < 200; k++) {
        int raw_vp = 0, raw_vn = 0;
        adc_oneshot_read(adc1_, chan_[0], &raw_vp);
        adc_oneshot_read(adc1_, chan_[1], &raw_vn);
        int mv_vp = 0, mv_vn = 0;
        if (cali_[0] != nullptr) adc_cali_raw_to_voltage(cali_[0], raw_vp, &mv_vp);
        if (cali_[1] != nullptr) adc_cali_raw_to_voltage(cali_[1], raw_vn, &mv_vn);
        sum += (mv_vp - mv_vn) / 1000.0f;
    }
    offset_v_[0] = sum / 200.0f;

    initialized = true;
    ESP_LOGI(TAG, "init ok: differential Vp(GPIO%d)-Vn(GPIO%d), cmp=%u/%u ticks",
             pinA, pinB, (unsigned)cmp_ticks, (unsigned)dp->period_ticks);
    return 1;
}

PhaseCurrent_s LocalLowSideCurrentSense::getPhaseCurrents()
{
    PhaseCurrent_s c = { 0.0f, 0.0f, 0.0f };
    if (!initialized) {
        return c;
    }

    /* Busy-wait for the comparator event, then read both ADC channels. */
    uint32_t start = event_count;
    int64_t t0 = esp_timer_get_time();
    while (event_count == start) {
        if (esp_timer_get_time() - t0 > 200) {
            break;
        }
    }

    int raw_vp = 0, raw_vn = 0;
    adc_oneshot_read(adc1_, chan_[0], &raw_vp);
    adc_oneshot_read(adc1_, chan_[1], &raw_vn);

    int mv_vp = 0, mv_vn = 0;
    if (cali_[0] != nullptr) adc_cali_raw_to_voltage(cali_[0], raw_vp, &mv_vp);
    if (cali_[1] != nullptr) adc_cali_raw_to_voltage(cali_[1], raw_vn, &mv_vn);

    float v_diff = (mv_vp - mv_vn) / 1000.0f - offset_v_[0];
    c.a = v_diff * volts_to_amps_;
    return c;
}
