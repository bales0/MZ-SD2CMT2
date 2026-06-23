#include "mzio.h"

#include <Arduino.h>

/*
    SD2CMT2 Reborn - Arduino Mega 2560 pin mapping

    MZCMT WRITE  -> Arduino pin 15  input
    MZCMT MOTOR  -> Arduino pin 16  input

    MZCMT READ   <- Arduino pin 2   output
    MZCMT SENSE  <- Arduino pin 18  output
    MZCMT LED    <- Arduino pin 19  output
*/

#define MZ_PIN_WRITE 15
#define MZ_PIN_MOTOR 16

#define MZ_PIN_READ  2
#define MZ_PIN_SENSE 18
#define MZ_PIN_LED   19

static bool mz_read_level = false;
static bool mz_sense_level = true;
static bool mz_led_level = false;

void mzio_init(void)
{
    pinMode(MZ_PIN_WRITE, INPUT);
    pinMode(MZ_PIN_MOTOR, INPUT);

    pinMode(MZ_PIN_READ, OUTPUT);
    pinMode(MZ_PIN_SENSE, OUTPUT);
    pinMode(MZ_PIN_LED, OUTPUT);

    /*
        Bezpečný idle stav:

        READ  = LOW
        SENSE = HIGH
        LED   = LOW
    */
    mz_read_level = false;
    mz_sense_level = true;
    mz_led_level = false;

    digitalWrite(MZ_PIN_READ, LOW);
    digitalWrite(MZ_PIN_SENSE, HIGH);
    digitalWrite(MZ_PIN_LED, LOW);
}

void mz_read_set(bool level)
{
    mz_read_level = level;

    digitalWrite(MZ_PIN_READ, level ? HIGH : LOW);
}

void mz_sense_set(bool level)
{
    mz_sense_level = level;

    digitalWrite(MZ_PIN_SENSE, level ? HIGH : LOW);
}

void mz_led_set(bool level)
{
    mz_led_level = level;

    digitalWrite(MZ_PIN_LED, level ? HIGH : LOW);
}

bool mz_read_get(void)
{
    return mz_read_level;
}

bool mz_sense_get(void)
{
    return mz_sense_level;
}

bool mz_led_get(void)
{
    return mz_led_level;
}

bool mz_motor_get(void)
{
    return digitalRead(MZ_PIN_MOTOR) == HIGH;
}

bool mz_write_get(void)
{
    return digitalRead(MZ_PIN_WRITE) == HIGH;
}