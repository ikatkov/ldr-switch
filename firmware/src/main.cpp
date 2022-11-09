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
#define LOAD_PIN PB2
#define ADC_PIN A3
#define INT_PIN PB4 // also button pin
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

EMPTY_INTERRUPT(PCINT2_vect);

void readButton()
{
    Serial.println("readButton");
    if (digitalRead(INT_PIN) == LOW)
    {
        if (STATE == ON)
        {
            Serial.println("STATE == ON");
            digitalWrite(LOAD_PIN, LOW);
            delay(2000); // debounce + rising edge
            Serial.println("WAIT_FOR_RESET");
            STATE = WAIT_FOR_RESET;
        }
        else if (STATE == OFF || STATE == WAIT_FOR_RESET)
        {
            STATE = ON;
            digitalWrite(LOAD_PIN, HIGH);
            timer_sec = LIGHT_ON_DURATION_SEC;

            //set new lightThresholdDark
            uint16_t lightLevel = analogRead(ADC_PIN);
            lightThresholdDark = lightLevel;

            Serial.print("STATE=");
            Serial.println(STATE);
            Serial.print("lightLevel=");
            Serial.println(lightLevel);

            // EEPROM_writeAnything(0, lightLevel);
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
    Serial.print("powerDown = ");
    Serial.println(sleepTime);
    delay(100);

    ADCSRA &= ~(1 << ADEN); // adc OFF
    if (sleepTime == SLEEP_FOREVER)
    {
        wdt_reset();
        wdt_disable();
    }
    else
    {
        wdt_enable(sleepTime); // watchdog
        WDTCSR |= (1 << WDIE);
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

    Serial.println("power UP");
    ADCSRA |= (1 << ADEN); // adc ON
}

void setup()
{
    Serial.println("setup");

    Serial.begin(9600);
    while (!Serial)
        delay(10);

    pinMode(LOAD_PIN, OUTPUT);
    pinMode(INT_PIN, INPUT_PULLUP);

    cli();
    PCICR |= _BV(PCIE2);    //  turn on port d
    PCMSK2 |= _BV(PCINT23); // turn on pin PD7, which is PCINT23, physical pin 7
    sei();

#if defined(__AVR_ATtiny13__)

    pinMode(PB0, OUTPUT);
    pinMode(PB1, OUTPUT);
    pinMode(PB2, OUTPUT);
    pinMode(PB3, INPUT);
    pinMode(PB4, INPUT_PULLUP);
#endif

    EEPROM_readAnything(0, lightThresholdDark);
    Serial.print("setupEEPROM = ");
    Serial.println(lightThresholdDark);
    if (lightThresholdDark < 0 || lightThresholdDark > 1024)
    {
        lightThresholdDark = 700;
        EEPROM_writeAnything(0, lightThresholdDark);
    }
    Serial.print("lightThresholdDark = ");
    Serial.println(lightThresholdDark);
}

void loop()
{
    Serial.println("---");
    readButton();

    uint16_t lightLevel = analogRead(ADC_PIN);
    Serial.print("lightLevel = ");
    Serial.println(lightLevel);
    Serial.print("lightThresholdDark = ");
    Serial.println(lightThresholdDark);
    Serial.print("STATE = ");
    Serial.println(STATE);
    delay(100);

    if (lightLevel <= lightThresholdBright && STATE == WAIT_FOR_RESET)
    {
        STATE = OFF;
    }

    if (lightLevel >= lightThresholdDark && STATE == OFF)
    {
        Serial.println("it's dark, turn the lights ON");
        delay(100);
        digitalWrite(LOAD_PIN, HIGH);
        STATE = ON;
        timer_sec = LIGHT_ON_DURATION_SEC;
    }
    else if (STATE == ON && timer_sec <= 0)
    {
        Serial.println("timer is up");
        delay(100);
        digitalWrite(LOAD_PIN, LOW);
        STATE = WAIT_FOR_RESET;
    }
    else if (STATE == ON && lightLevel < lightThresholdDark)
    {
        Serial.println("it's bright");
        delay(100);
        digitalWrite(LOAD_PIN, LOW);
        STATE = OFF;
    }
    Serial.print("timer_sec = ");
    Serial.println(timer_sec);
    powerDown(WDTO_8S);

    if (timer_sec > 0)
    {
        timer_sec -= 8;
    }
}