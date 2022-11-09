#include "Arduino.h"
#include <EEPROM.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#define LIGHT_ON_DELAY_MS 60 * 1000
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

static uint16_t lightThreshold;
static uint32_t lightOnTimestamp;
static enum state_t {
    ON,
    OFF,
    SLEEP
} STATE;

#define PRESSED LOW
#define NOT_PRESSED HIGH
#define SHORT_PRESS_MS 100
#define LONG_PRESS_MS 500
#define DEBOUNCE_MS 10

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

void buttonCheck()
{
    uint8_t value = digitalRead(INT_PIN);

    Serial.print("buttonCheck = ");
    Serial.println(value);
    if (value == LOW)
    {
        Serial.println("digitalRead(INT_PIN) == LOW");
        if (STATE == ON)
        {
            Serial.println("STATE == ON");
            digitalWrite(LOAD_PIN, LOW);
            lightOnTimestamp = 0;
            delay(4000);
            Serial.println("SLEEP_FOREVER");
            STATE = SLEEP;
        }
        else if (STATE == OFF)
        {
            digitalWrite(LOAD_PIN, HIGH);
            STATE = ON;
            lightOnTimestamp = 0;
            uint16_t lightLevel = analogRead(ADC_PIN);

            Serial.print("STATE == OFF lightLevel = ");
            Serial.println(lightLevel);

            // EEPROM_writeAnything(0, lightLevel);
            delay(2000);
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
    if (sleepTime == SLEEP_FOREVER){}
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

    EEPROM_readAnything(0, lightThreshold);
    Serial.print("setupEEPROM = ");
    Serial.println(lightThreshold);
    if (lightThreshold < 0 || lightThreshold > 1024)
    {
        lightThreshold = 700;
        EEPROM_writeAnything(0, lightThreshold);
    }
    Serial.print("lightThreshold = ");
    Serial.println(lightThreshold);
}

void loop()
{
    buttonCheck();
    Serial.println("---");

    if (STATE == SLEEP)
    {
        powerDown(SLEEP_FOREVER);
    }

    uint16_t lightLevel = analogRead(ADC_PIN);
    Serial.print("lightLevel = ");
    Serial.println(lightLevel);
    Serial.print("lightThreshold = ");
    Serial.println(lightThreshold);

    delay(100);
    if (lightLevel >= lightThreshold && lightOnTimestamp == 0)
    {
        Serial.println("it's dark");
        delay(100);
        digitalWrite(LOAD_PIN, HIGH);
        STATE = ON;
        lightOnTimestamp = millis();
    }
    else if (lightLevel < lightThreshold || millis() > lightOnTimestamp + LIGHT_ON_DELAY_MS)
    {
        Serial.println("it's bright or the time is up");
        delay(100);
        digitalWrite(LOAD_PIN, LOW);
        STATE = OFF;
        lightOnTimestamp = 0;
        powerDown(WDTO_8S);
    }
    else
    {
        powerDown(WDTO_8S);
    }
}