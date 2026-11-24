/*
 * AS5047P magnetic rotary encoder (portable core).
 *
 * Pure protocol logic: command/data frame construction and decoding, and the
 * pipelined-reply state machine. No ESP-IDF or RTOS headers are used; all SPI
 * I/O goes through the single-frame exchange in `rotary_encoder_transport_t`.
 */
#include "rotary_encoder.h"

#include <stddef.h>

#define ADDR_UNKNOWN 0xFFFFu

/* Number of set bits modulo 2 (1 = odd). */
static uint16_t parity_odd(uint16_t v)
{
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return v & 1u;
}

/* Command frame: bit15 = even parity, bit14 = R/W (1 = read), bits13:0 = addr. */
static uint16_t cmd_read(uint16_t addr)
{
    uint16_t cmd = 0x4000u | (addr & 0x3FFFu);
    return cmd | (parity_odd(cmd) << 15);
}

static int exchange(rotary_encoder_t *enc, uint16_t tx_word, uint16_t *rx_word)
{
    if (enc->io.xfer == NULL) {
        return -1;
    }
    return enc->io.xfer(enc->io.ctx, tx_word, rx_word);
}

/* Data frame: bit15 = even parity, bit14 = EF, bits13:0 = data. */
static void decode(uint16_t word, rotary_encoder_sample_t *out)
{
    out->counts = word & 0x3FFFu;
    out->command_error = (word >> 14) & 1u;
    out->parity_ok = (parity_odd(word) == 0);
}

void rotary_encoder_init(rotary_encoder_t *enc, const rotary_encoder_transport_t *transport)
{
    if (enc == NULL || transport == NULL) {
        return;
    }

    enc->io = *transport;
    enc->last_cmd_addr = ADDR_UNKNOWN;
}

int rotary_encoder_read_reg(rotary_encoder_t *enc, uint16_t addr, rotary_encoder_sample_t *out)
{
    if (enc == NULL || out == NULL) {
        return -1;
    }

    uint16_t ignored;
    uint16_t word;
    addr &= 0x3FFFu;

    if (exchange(enc, cmd_read(addr), &ignored) != 0) {
        enc->last_cmd_addr = ADDR_UNKNOWN;
        return -1;
    }
    enc->last_cmd_addr = addr;

    if (exchange(enc, cmd_read(ROTARY_ENCODER_REG_NOP), &word) != 0) {
        enc->last_cmd_addr = ADDR_UNKNOWN;
        return -1;
    }
    enc->last_cmd_addr = ROTARY_ENCODER_REG_NOP;

    decode(word, out);
    return 0;
}

int rotary_encoder_read_angle(rotary_encoder_t *enc, rotary_encoder_sample_t *out)
{
    if (enc == NULL || out == NULL) {
        return -1;
    }

    uint16_t word;
    bool primed = (enc->last_cmd_addr == ROTARY_ENCODER_REG_ANGLECOM);

    if (exchange(enc, cmd_read(ROTARY_ENCODER_REG_ANGLECOM), &word) != 0) {
        enc->last_cmd_addr = ADDR_UNKNOWN;
        return -1;
    }
    if (!primed) {
        /* This reply belongs to the previous (non-angle) command; repeat so the
         * next reply is a real angle sample. */
        if (exchange(enc, cmd_read(ROTARY_ENCODER_REG_ANGLECOM), &word) != 0) {
            enc->last_cmd_addr = ADDR_UNKNOWN;
            return -1;
        }
    }
    enc->last_cmd_addr = ROTARY_ENCODER_REG_ANGLECOM;

    decode(word, out);
    return 0;
}

int rotary_encoder_read_diag(rotary_encoder_t *enc, rotary_encoder_diag_t *out)
{
    if (enc == NULL || out == NULL) {
        return -1;
    }

    rotary_encoder_sample_t dia;
    rotary_encoder_sample_t mag;

    if (rotary_encoder_read_reg(enc, ROTARY_ENCODER_REG_DIAAGC, &dia) != 0 ||
        rotary_encoder_read_reg(enc, ROTARY_ENCODER_REG_MAG, &mag) != 0) {
        return -1;
    }

    uint16_t d = dia.counts;
    out->agc = d & 0xFFu;
    out->offset_loop_ready = (d >> 8) & 1u;
    out->cordic_overflow  = (d >> 9) & 1u;
    out->field_too_weak   = (d >> 10) & 1u;  /* MAGL */
    out->field_too_strong = (d >> 11) & 1u;  /* MAGH */
    out->magnitude = mag.counts & 0x3FFFu;
    return 0;
}

float rotary_encoder_to_deg(uint16_t counts)
{
    return (float)counts * (360.0f / (float)ROTARY_ENCODER_COUNTS_PER_REV);
}

float rotary_encoder_to_rad(uint16_t counts)
{
    return (float)counts * (6.283185307179586f / (float)ROTARY_ENCODER_COUNTS_PER_REV);
}
