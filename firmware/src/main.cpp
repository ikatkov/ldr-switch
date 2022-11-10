#include "Arduino.h"
#include <EEPROM.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

static uint8_t lighOnDurationHours = 2;

#define LOAD_PIN PB0      // pin5 Attiny13
#define ADC_POWER_PIN PB3 //  pin2 on Attiny13
#define ADC_PIN A2        // PB4, pin3 Attiny13
#define INT_PIN PB2       // also button pin, pin7 Attiny13

static uint16_t lightThresholdDark = 700;
#define lightThresholdBright 350
#define LIGHT_THRESHOLD_HYSTERESIS 10

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
void blink(uint8_t times, uint16_t delayTime = 200);
void configureButtonInterrupt(bool enable);

EMPTY_INTERRUPT(PCINT0_vect);
EMPTY_INTERRUPT(WDT_vect)

void setlighOnDurationHours()
{
    uint8_t counter = 0;
    while (digitalRead(INT_PIN) == LOW)
    {
        counter++;
        if (counter > 8)
            counter = 1;
        blink(counter);
        delay(2000);
    }
    if (counter > 0 && counter != lighOnDurationHours)
    {
        lighOnDurationHours = counter;
        EEPROM_writeAnything(2, lighOnDurationHours);
    }
}

void readButton(uint16_t lightLevel)
{
    if (digitalRead(INT_PIN) == LOW)
    {
        if (STATE == ON)
        {
            digitalWrite(LOAD_PIN, LOW);
            STATE = WAIT_FOR_RESET;
            delay(2000); // debounce
        }
        else if (STATE == OFF || STATE == WAIT_FOR_RESET)
        {
            STATE = ON;
            blink(lighOnDurationHours);
            delay(1000); // blink delay + debounce
            digitalWrite(LOAD_PIN, HIGH);
            timer_sec = lighOnDurationHours * 60 * 60;

            // set new lightThresholdDark
            lightThresholdDark = lightLevel;

            EEPROM_writeAnything(0, lightThresholdDark);
        }
        setlighOnDurationHours();
    }
}

void powerDown(uint8_t sleepTime)
{
    ADCSRA &= ~(1 << ADEN); // adc OFF
    wdt_enable(sleepTime);  // watchdog
    WDTCR |= _BV(WDTIE);

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

void blink(uint8_t times, uint16_t delayTime)
{
    for (int i = times; i > 0; i--)
    {
        digitalWrite(LOAD_PIN, HIGH);
        delay(delayTime);
        digitalWrite(LOAD_PIN, LOW);
        delay(delayTime);
        wdt_reset();
    }
}

void setup()
{
    pinMode(ADC_POWER_PIN, OUTPUT);
    pinMode(ADC_PIN, INPUT);        // ADC
    pinMode(INT_PIN, INPUT_PULLUP); // button
    pinMode(LOAD_PIN, OUTPUT);      // gate

    analogReference(DEFAULT);

    cli();
    GIMSK |= _BV(PCIE);   //  Enable PCINT interrupt in the general interrupt mask
    PCMSK |= _BV(PCINT2); // Enable interrupt handler (ISR) for our chosen interrupt pin (PCINT2/PB2/pin 7)
    sei();

    EEPROM_readAnything(0, lightThresholdDark);
    if (lightThresholdDark < 0 || lightThresholdDark > 1024)
    {
        lightThresholdDark = 700;
        EEPROM_writeAnything(0, lightThresholdDark);
    }
    EEPROM_readAnything(2, lighOnDurationHours);
    if (lighOnDurationHours < 1 || lighOnDurationHours > 8)
    {
        lighOnDurationHours = 2;
    }

    blink(lighOnDurationHours);
}

void loop()
{
    digitalWrite(ADC_POWER_PIN, HIGH);
    uint16_t lightLevel = analogRead(ADC_PIN);
    digitalWrite(ADC_POWER_PIN, LOW);
    readButton(lightLevel);

    if (STATE == WAIT_FOR_RESET && lightLevel <= lightThresholdBright)
    {
        STATE = OFF;
    }

    if (STATE == OFF && lightLevel >= lightThresholdDark)
    {
        digitalWrite(LOAD_PIN, HIGH);
        timer_sec = lighOnDurationHours * 60 * 60;
        STATE = ON;
    }
    else if (STATE == ON && timer_sec <= 0)
    {
        digitalWrite(LOAD_PIN, LOW);
        STATE = WAIT_FOR_RESET;
    }
    else if (STATE == ON && lightLevel < lightThresholdDark - LIGHT_THRESHOLD_HYSTERESIS)
    {
        digitalWrite(LOAD_PIN, LOW);
        STATE = OFF;
    }

    powerDown(WDTO_4S);
    if (timer_sec > 0)
    {
        timer_sec -= 4;
    }
}