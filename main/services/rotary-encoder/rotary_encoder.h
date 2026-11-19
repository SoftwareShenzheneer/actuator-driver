/*
 * AS5047P magnetic rotary encoder (portable core).
 *
 * Everything under services/rotary-encoder/ except the esp32/ subdirectory must
 * NOT include any ESP-IDF or RTOS headers, so it can be reused unchanged on a
 * different MCU. The only platform dependency is `rotary_encoder_transport_t`:
 * one function that exchanges a single 16-bit SPI frame. A platform backend
 * (e.g. esp32/) provides that function and nothing else.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* AS5047P register addresses (14-bit, see datasheet "Volatile Register Table"). */
#define ROTARY_ENCODER_REG_NOP        0x0000u
#define ROTARY_ENCODER_REG_ERRFL      0x0001u   /* reading clears the error flags */
#define ROTARY_ENCODER_REG_PROG       0x0003u
#define ROTARY_ENCODER_REG_ZPOSM      0x0016u
#define ROTARY_ENCODER_REG_ZPOSL      0x0017u
#define ROTARY_ENCODER_REG_SETTINGS1  0x0018u
#define ROTARY_ENCODER_REG_SETTINGS2  0x0019u
#define ROTARY_ENCODER_REG_DIAAGC     0x3FFCu
#define ROTARY_ENCODER_REG_MAG        0x3FFDu
#define ROTARY_ENCODER_REG_ANGLEUNC   0x3FFEu
#define ROTARY_ENCODER_REG_ANGLECOM   0x3FFFu

#define ROTARY_ENCODER_COUNTS_PER_REV (1u << 14)

/*
 * Minimal platform interface: exchange one 16-bit SPI frame, MSB-first.
 *
 * Returns 0 on success, non-zero on error. The platform backend owns CS timing
 * and (on a shared bus) bus ownership around the transfer.
 */
typedef int (*rotary_encoder_xfer_fn)(void *ctx, uint16_t tx_word, uint16_t *rx_word);

typedef struct {
    rotary_encoder_xfer_fn xfer;
    void *ctx;
} rotary_encoder_transport_t;

/* A decoded 16-bit data frame from the sensor. */
typedef struct {
    uint16_t counts;         /* bits 13:0 */
    bool parity_ok;          /* even parity over the frame matched */
    bool command_error;      /* EF: sensor rejected the previous command frame */
} rotary_encoder_sample_t;

/* DIAAGC + MAG field diagnostics. */
typedef struct {
    uint16_t agc;             /* DIAAGC 7:0 */
    uint16_t magnitude;       /* MAG 13:0 */
    bool field_too_weak;      /* DIAAGC.MAGL: magnetic field too low */
    bool field_too_strong;    /* DIAAGC.MAGH: magnetic field too high */
    bool cordic_overflow;     /* DIAAGC.COF */
    bool offset_loop_ready;   /* DIAAGC.LF */
} rotary_encoder_diag_t;

/*
 * Encoder instance. Allocate it anywhere (stack/static) and bind it once with
 * rotary_encoder_init(). `last_cmd_addr` is internal pipelined-reply state.
 */
typedef struct rotary_encoder {
    rotary_encoder_transport_t io;
    uint16_t last_cmd_addr;
} rotary_encoder_t;

/* Bind the encoder to a platform transport. */
void rotary_encoder_init(rotary_encoder_t *enc, const rotary_encoder_transport_t *transport);

/*
 * Read ANGLECOM. Uses the previous frame's reply when the last command was also
 * an angle read (one frame of latency); otherwise re-primes first. Returns 0 on
 * success.
 */
int rotary_encoder_read_angle(rotary_encoder_t *enc, rotary_encoder_sample_t *out);

/*
 * Full register read: sends the read command, then a NOP to collect the
 * pipelined reply (two 16-bit frames). Returns 0 on success.
 */
int rotary_encoder_read_reg(rotary_encoder_t *enc, uint16_t addr, rotary_encoder_sample_t *out);

/* Read DIAAGC + MAG and decode the field diagnostics. Returns 0 on success. */
int rotary_encoder_read_diag(rotary_encoder_t *enc, rotary_encoder_diag_t *out);

/* Raw 14-bit counts -> angle. */
float rotary_encoder_to_deg(uint16_t counts);
float rotary_encoder_to_rad(uint16_t counts);

#ifdef __cplusplus
}
#endif
