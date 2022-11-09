#include "Arduino.h"
#include <EEPROM.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#define LIGHT_ON_DURATION_SEC 20
#if defined(__AVR_ATmega328P__)
#define LOAD_PIN 9
#define ADC_PIN A0
#define INT_PIN PD7 // also button pin
#elif defined(__AVR_ATtiny13__)
#define LOAD_PIN PB4
#define ADC_PIN A3
#define INT_PIN PB2 // also button pin
#endif
#define SLEEP_FOREVER 10

static uint16_t lightThresholdDark = 700;
static uint16_t lightThresholdBright = 350;
static enum state_t {
    OFF,
    ON,
    WAIT_FOR_RESET
} STATE;

static int32_t timer_sec = 0;

template <class T>
int EEPROM_writeAnything(int address, const T &value)
{
    const byte *p = (const byte *)(const void *)&value;
    uint8_t i;
    for (i = 0; i < sizeof(value); i++)
        EEPROM.write(address++, *p++);
    return i;
}

template <class T>
int EEPROM_readAnything(int address, T &value)
{
    byte *p = (byte *)(void *)&value;
    uint16_t i;
    for (i = 0; i < sizeof(value); i++)
        *p++ = EEPROM.read(address++);
    return i;
}

void powerDown(uint8_t sleepTime);

#if defined(__AVR_ATmega328P__)
EMPTY_INTERRUPT(PCINT2_vect);
#elif defined(__AVR_ATtiny13__)
EMPTY_INTERRUPT(PCINT0_vect);
#endif

void readButton()
{
    if (digitalRead(INT_PIN) == LOW)
    {
        if (STATE == ON)
        {
            digitalWrite(LOAD_PIN, LOW);
            STATE = WAIT_FOR_RESET;
            delay(2000); // debounce + rising edge
        }
        else if (STATE == OFF || STATE == WAIT_FOR_RESET)
        {
            STATE = ON;
            digitalWrite(LOAD_PIN, HIGH);
            timer_sec = LIGHT_ON_DURATION_SEC;

            // set new lightThresholdDark
            lightThresholdDark = analogRead(ADC_PIN);

            EEPROM_writeAnything(0, lightThresholdDark);
            delay(2000); // debounce + rising edge
        }
    }
}

ISR(WDT_vect)
{
    // WDIE & WDIF is cleared in hardware upon entering this ISR
    wdt_disable();
}

void powerDown(uint8_t sleepTime)
{
    ADCSRA &= ~(1 << ADEN); // adc OFF
    if (sleepTime == SLEEP_FOREVER)
    {
        wdt_reset();
        wdt_disable();
    }
    else
    {
        wdt_enable(sleepTime); // watchdog
        // WDTCSR |= (1 << WDIE);
    }

    do
    {
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        cli(); // Disable interrupts
        sleep_enable();
        sleep_bod_disable();
        sei(); // Enable interrupts
        sleep_cpu();

        sleep_disable();
        sei();
    } while (0);

    ADCSRA |= (1 << ADEN); // adc ON
}

void setup()
{
#if defined(__AVR_ATmega328P__)
    Serial.begin(9600);
    while (!Serial)
        delay(10);

    pinMode(LOAD_PIN, OUTPUT);      // mosfet gate
    pinMode(INT_PIN, INPUT_PULLUP); // button pin

    cli();
    PCICR |= _BV(PCIE2);    //  turn on port d
    PCMSK2 |= _BV(PCINT23); // turn on pin PD7, which is PCINT23, physical pin 7
    sei();
#endif

#if defined(__AVR_ATtiny13__)
    pinMode(PB0, OUTPUT);
    pinMode(PB1, OUTPUT);
    pinMode(PB2, INPUT_PULLUP);
    pinMode(PB3, INPUT); // ADC
    pinMode(PB4, OUTPUT);
#endif

    EEPROM_readAnything(0, lightThresholdDark);
    if (lightThresholdDark < 0 || lightThresholdDark > 1024)
    {
        lightThresholdDark = 700;
        EEPROM_writeAnything(0, lightThresholdDark);
    }
}

void loop()
{
    readButton();

    uint16_t lightLevel = analogRead(ADC_PIN);

    if (lightLevel <= lightThresholdBright && STATE == WAIT_FOR_RESET)
    {
        STATE = OFF;
    }

    if (lightLevel >= lightThresholdDark && STATE == OFF)
    {
        digitalWrite(LOAD_PIN, HIGH);
        timer_sec = LIGHT_ON_DURATION_SEC;
        STATE = ON;
    }
    else if (STATE == ON && timer_sec <= 0)
    {
        digitalWrite(LOAD_PIN, LOW);
        STATE = WAIT_FOR_RESET;
    }
    else if (STATE == ON && lightLevel < lightThresholdDark)
    {
        digitalWrite(LOAD_PIN, LOW);
        STATE = OFF;
    }
    powerDown(WDTO_8S);

    if (timer_sec > 0)
    {
        timer_sec -= 8;
    }
}