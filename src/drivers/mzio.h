#ifndef SD2CMT2_MZIO_H
#define SD2CMT2_MZIO_H

#include <stdbool.h>
#include <stdint.h>
#include <avr/io.h>

/*
    SD2CMT2 Reborn - Arduino Mega 2560 CMT mapping

    WRITE  -> D15, input
    MOTOR  -> D16, input
    READ   -> D2,  output
    SENSE  -> D18, output
    LED    -> D19, output

    The *_from_isr functions use direct AVR port access and are intended for
    the fixed-rate sample clock only. They do not call digitalRead/digitalWrite.
*/
#define MZIO_PIN_WRITE 15U
#define MZIO_PIN_MOTOR 16U
#define MZIO_PIN_READ  2U
#define MZIO_PIN_SENSE 18U
#define MZIO_PIN_LED   19U

void mzio_init(void);

void mz_read_set(bool level);
void mz_sense_set(bool level);
void mz_led_set(bool level);

bool mz_read_get(void);
bool mz_sense_get(void);
bool mz_led_get(void);

bool mz_motor_get(void);
bool mz_write_get(void);

/*
    Low-jitter sample-clock primitives.
    READ is PE4 / Arduino D2, WRITE is PJ0 / Arduino D15 on ATmega2560.
*/
/*
    Fast output primitive for the 44.1/48 kHz Timer1 ISR.
    It is intentionally header-inline: no function call and no status-variable
    write are added to the sample clock critical path.
*/
static inline __attribute__((always_inline))
void mz_read_set_fast_from_isr(uint8_t level)
{
    if (level & 1U)
    {
        PORTE |= _BV(PE4);
    }
    else
    {
        PORTE &= (uint8_t)~_BV(PE4);
    }
}

/* Backward-compatible non-inline helper, not used by the WAV sample ISR. */
void mz_read_set_from_isr(uint8_t level);
uint8_t mz_write_sample_from_isr(void);

uint8_t mzio_read_pin(void);
uint8_t mzio_write_pin(void);

#endif
