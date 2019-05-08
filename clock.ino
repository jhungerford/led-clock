#include "clock.h"
#include <SPI.h>
#include <SparkFunDS3234RTC.h>
#include <Adafruit_NeoPixel.h>

// Time.  Dead on real time clock (RTC)
#define RTC_CS_PIN        18  // Chip-select pin (SS)
#define RTC_INTERRUPT_PIN 3   // SQW / interrupt pin
#define BLINK_MS          250 // MS the hour, minute, or second LED will be on or off in the set modes.

uint16_t hourLed;
uint16_t minuteLed;
uint16_t secondLed;

boolean ledOn = true;
unsigned long nextBlink = 0;

// LEDs.
// NEO_KHZ800 - 800KHz bitstream for WS2812 LEDs
// NEO_GRB - Green, Red, Blue bitstream
#define LED_PIN     6  // Pin the LEDs are connected to
#define NUM_LEDS    30 // Number of LEDs in the strip
#define LED_OFFSET  14 // Index of the 12-o'clock LED

Adafruit_NeoPixel leds = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Modes.  0 = color, 1 = hour, 2 = minute, 3 = second.
#define NUM_MODES   4
#define COLOR_MODE  0
#define HOUR_MODE   1
#define MINUTE_MODE 2
#define SECOND_MODE 3

uint8_t mode = COLOR_MODE;

// Buttons.  Mode and Increment.
#define MODE_PIN        4
#define INCREMENT_PIN   5

#define INITIAL_DEBOUNCE_MS 100 // MS a button must be pressed for a keypress to register
#define FIRST_REPEAT_MS     750 // MS the repeat button will wait before firing repeatedly
#define REPEAT_MS           250 // MS between repeats while the repeat button is held

Button modeButton = newSingleButton(MODE_PIN);
Button incrementButton = newRepeatButton(INCREMENT_PIN);

// ------------------------------------------------------------------------------------------ Setup

void setup() {
  pinMode(RTC_INTERRUPT_PIN, INPUT_PULLUP);
  pinMode(MODE_PIN, INPUT);
  pinMode(INCREMENT_PIN, INPUT);
  
  Serial.begin(9600); // Serial connection, for debugging.
  
  leds.begin();
  leds.show(); // Turn off all LEDs.

  for (int i = 0; i < NUM_LEDS; i ++) {
    leds.setPixelColor(i, leds.Color(0, 255, 255));
  }

//  rtc.begin(RTC_CS_PIN);
//  rtc.autoTime(); // Set the RTC's time to the sketch compilation time
//  rtc.enableAlarmInterrupt(); // Enable the SQW pin as an interrupt.
//  rtc.setAlarm1(); // By default, alarm 1 triggers every second
}

// ------------------------------------------------------------------------------------------ Loop

void loop() {
  for (int i = 0; i < NUM_LEDS; i ++) {
    leds.setPixelColor(i, leds.Color(0, 255, 255));
  }
}

/*
void loop() {
  unsigned int now = millis();
  boolean updateLeds = false;

  // Toggle the LED for the set modes.
  if (now > nextBlink) {
    ledOn = !ledOn;
    nextBlink = now + BLINK_MS;

    if (mode != COLOR_MODE) {
      updateLeds = true;
    }
  }
  
  // Clock update.  Interrupt pin is active-low, so it's low when an alarm is triggered.  Alarm 1 triggers every second.
  if (!digitalRead(RTC_INTERRUPT_PIN) && rtc.alarm1()) {
    rtc.update();
    updateLeds = true;
  }

  // Mode button.  Colors, hour, minute, second.
  if (readButton(&modeButton, now)) {
    mode = (mode + 1) % NUM_MODES;
    updateLeds = true;
    ledOn = false;
    nextBlink = now + BLINK_MS;
  }
  

  // + button.  Press to increment once.  Hold to continuously increment.
  if (readButton(&incrementButton, now)) {
    updateLeds = true;
    
    switch (mode) {
      case HOUR_MODE:
        rtc.setHour((rtc.getHour() + 1) % 24);
        break;
      case MINUTE_MODE:
        rtc.setMinute((rtc.getMinute() + 1) % 60);
        break;
      case SECOND_MODE:
        rtc.setSecond((rtc.getSecond() + 1) % 60);
        break;
    }
  }

  if (updateLeds) {
    renderTime(rtc.hour(), rtc.minute(), rtc.second());
  }
}
*/

// ------------------------------------------------------------------------------------------ Buttons

// Returns a new button that will fire once when it's pressed - it must be depressed before it will fire again.
Button newSingleButton(int pin) {
  return { pin, false, LOW, false, 0 };
}

// Returns a new button that will fire repeatedly when it's held.
Button newRepeatButton(int pin) {
  return { pin, true, LOW, false, 0 };
}

// Reads the button state, storing the current state in the button.
// Returns true if the button fired during this reading.
boolean readButton(Button* button, unsigned long now) {
  int reading = digitalRead(button->pin);

  // If the button state changed due to noise or pressing, reset the debounce timer.
  if (reading != button->lastReading) {
    Serial.print("Reading: ");
    Serial.println(reading);
    button->lastReading = reading;
    button->nextFireTime = now + INITIAL_DEBOUNCE_MS;
    button->fired = false;
  }

  // If the button should fire, fire.
  if (now > button->nextFireTime && reading == HIGH) {
    if (button->repeat) {
      // Repeat button - fire and adjust the next fire time based on whether this is the first time the button has fired.
      Serial.println("Repeat Button.");
      button->nextFireTime = now + (button->fired ? REPEAT_MS : FIRST_REPEAT_MS);
      button->fired = true;
      return true;
      
    } else if (!button->fired) {
      // Single press button that hasn't fired yet - fire!
      Serial.println("Single Press Button.");
      button->fired = true;
      return true;
    }
  }

  return false;
}

// ------------------------------------------------------------------------------------------ Rendering

// Renders the given hour, minute, and second to the LED strip.
// Hour uses 3 LEDs, minute uses 2, and second uses 1.
// The arduino's 5V data pin supplies ~50ma to the 5V pin, and
// each LED draws up to 60ma at full brightness.  Only ~8 pixels 
// can be on at a time at full power.
void renderTime(uint8_t hour, uint8_t minute, uint8_t second) {
  // Clear the previous LEDs.
  leds.setPixelColor(hourLed, leds.Color(0, 0, 0));
  leds.setPixelColor(minuteLed, leds.Color(0, 0, 0));
  leds.setPixelColor(secondLed, leds.Color(0, 0, 0));

  // Turn on the new LEDs.
  if (mode != HOUR_MODE || ledOn) {
    hourLed = renderTimeUnit(hour, 12, leds.Color(255, 0, 0));
  }

  if (mode != MINUTE_MODE || ledOn) {
    minuteLed = renderTimeUnit(minute, 60, leds.Color(0, 255, 0));
  }

  if (mode != SECOND_MODE || ledOn) {
    secondLed = renderTimeUnit(second, 60, leds.Color(0, 0, 255));
  }

  // Push the instructions.
  leds.show();
}


uint16_t renderTimeUnit(uint8_t value, uint8_t num, uint32_t color) {
  uint16_t newLed = (value * NUM_LEDS / num + LED_OFFSET) % NUM_LEDS;
  leds.setPixelColor(newLed, color);
  return newLed;
}

