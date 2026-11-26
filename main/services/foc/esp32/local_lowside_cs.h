/*
 * Local low-side current sense for the ESP32 (MCPWM + ADC1).
 *
 * The esp_simplefoc LowsideCurrentSense samples the ADC on the MCPWM "on_full"
 * event, which is the wrong instant for a low-side shunt (current is ~0 there).
 * This local implementation instead creates an MCPWM comparator and samples at
 * ~70% of the PWM period, the correct low-side conduction point, so the phase
 * current reads correctly.
 */
#pragma once

#include "common/base_classes/CurrentSense.h"
#include "driver/mcpwm_prelude.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

class LocalLowSideCurrentSense : public CurrentSense {
public:
    LocalLowSideCurrentSense(float shunt_resistor, float gain, int pinA, int pinB, int pinC = NOT_SET);
    int init() override;
    PhaseCurrent_s getPhaseCurrents() override;

    volatile uint32_t event_count = 0;  /* incremented by the comparator ISR */

private:
    float volts_to_amps_ = 1.0f;

    adc_oneshot_unit_handle_t adc1_ = nullptr;
    adc_cali_handle_t cali_[3] = {};
    adc_channel_t chan_[3] = {};
    int n_chan_ = 0;
    float offset_v_[3] = {0.0f, 0.0f, 0.0f};

    mcpwm_cmpr_handle_t cmpr_ = nullptr;
};
