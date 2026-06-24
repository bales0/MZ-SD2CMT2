#include "mzio.h"

#include <Arduino.h>
#include <avr/io.h>

/*
    SD2CMT2 Reborn - Arduino Mega 2560 pin mapping

    MZCMT WRITE  -> Arduino pin 15  input  = PJ0
    MZCMT MOTOR  -> Arduino pin 16  input

    MZCMT READ   <- Arduino pin 2   output = PE4
    MZCMT SENSE  <- Arduino pin 18  output
    MZCMT LED    <- Arduino pin 19  output
*/

static volatile uint8_t mz_read_level = 0;
static volatile uint8_t mz_sense_level = 1;
static volatile uint8_t mz_led_level = 0;

void mzio_init(void)
{
    pinMode(MZIO_PIN_WRITE, INPUT);
    pinMode(MZIO_PIN_MOTOR, INPUT);

    pinMode(MZIO_PIN_READ, OUTPUT);
    pinMode(MZIO_PIN_SENSE, OUTPUT);
    pinMode(MZIO_PIN_LED, OUTPUT);

    /*
        Safe idle state:
        READ  = LOW
        SENSE = HIGH
        LED   = LOW
    */
    mz_read_level = 0;
    mz_sense_level = 1;
    mz_led_level = 0;

    digitalWrite(MZIO_PIN_READ, LOW);
    digitalWrite(MZIO_PIN_SENSE, HIGH);
    digitalWrite(MZIO_PIN_LED, LOW);
}

void mz_read_set(bool level)
{
    mz_read_level = level ? 1U : 0U;
    digitalWrite(MZIO_PIN_READ, level ? HIGH : LOW);
}

void mz_sense_set(bool level)
{
    mz_sense_level = level ? 1U : 0U;
    digitalWrite(MZIO_PIN_SENSE, level ? HIGH : LOW);
}

void mz_led_set(bool level)
{
    mz_led_level = level ? 1U : 0U;
    digitalWrite(MZIO_PIN_LED, level ? HIGH : LOW);
}

bool mz_read_get(void)
{
    return mz_read_level != 0;
}

bool mz_sense_get(void)
{
    return mz_sense_level != 0;
}

bool mz_led_get(void)
{
    return mz_led_level != 0;
}

bool mz_motor_get(void)
{
    return digitalRead(MZIO_PIN_MOTOR) == HIGH;
}

bool mz_write_get(void)
{
    return digitalRead(MZIO_PIN_WRITE) == HIGH;
}

void mz_read_set_from_isr(uint8_t level)
{
    /*
        D2 is PE4. On ATmega2560 this compiles to an SBI/CBI-style direct
        port update, preserving all other PORTE pins. It avoids digitalWrite()
        in the sample ISR.
    */
    if (level & 1U)
    {
        PORTE |= _BV(PE4);
        mz_read_level = 1U;
    }
    else
    {
        PORTE &= (uint8_t)~_BV(PE4);
        mz_read_level = 0U;
    }
}

uint8_t mz_write_sample_from_isr(void)
{
    /*
        D15 is PJ0. Record code will call this once per fixed sample period.
    */
    return (PINJ & _BV(PJ0)) ? 1U : 0U;
}

uint8_t mzio_read_pin(void)
{
    return MZIO_PIN_READ;
}

uint8_t mzio_write_pin(void)
{
    return MZIO_PIN_WRITE;
}
