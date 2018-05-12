#include <SPI.h>
#include <SparkFunDS3234RTC.h>
#include <Adafruit_NeoPixel.h>

// LED configuration
#define LED_PIN     13 // Pin the LEDs are connected to
#define NUM_LEDS    30 // Number of LEDs in the strip
#define LED_OFFSET  14 // Index of the 12-o'clock LED

// Dead on real time clock (RTC) configuration
#define RTC_CS_PIN        10 // Chip-select pin (SS)
#define RTC_INTERRUPT_PIN 2  // SQW / interrupt pin

// NEO_KHZ800 - 800KHz bitstream for WS2812 LEDs
// NEO_GRB - Green, Red, Blue bitstream
Adafruit_NeoPixel leds = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
uint16_t hourLed = 0;
uint16_t minuteLed = 0;
uint16_t secondLed = 0;

void setup() {
  pinMode(RTC_INTERRUPT_PIN, INPUT_PULLUP);
  
  Serial.begin(9600); // Serial connection, for debugging.
  
  leds.begin();
  leds.show(); // Turn off all LEDs.

  rtc.begin(RTC_CS_PIN);
  rtc.autoTime(); // Set the RTC's time to the sketch compilation time
  rtc.setAlarm1(); // By default, alarm 1 triggers every second
}

void loop() {
  if (rtc.alarm1()) {
    rtc.update();
    renderTime(rtc.hour(), rtc.minute(), rtc.second());
  }
}

// Renders the given hour, minute, and second to the LED strip.
// Hour uses 3 LEDs, minute uses 2, and second uses 1.
// The arduino's 5V data pin supplies ~50ma to the 5V pin, and
// each LED draws up to 60ma at full brightness.  Only ~8 pixels 
// can be on at a time.
void renderTime(uint8_t hour, uint8_t minute, uint8_t second) {
  // Clear the previous LEDs.
  leds.setPixelColor(hourLed, leds.Color(0, 0, 0));
  leds.setPixelColor(minuteLed, leds.Color(0, 0, 0));
  leds.setPixelColor(secondLed, leds.Color(0, 0, 0));

  // Turn on the new LEDs.
  hourLed = renderTimeUnit(hour, 12, leds.Color(255, 0, 0));
  minuteLed = renderTimeUnit(minute, 60, leds.Color(0, 255, 0));
  secondLed = renderTimeUnit(second, 60, leds.Color(0, 0, 255));

  // Push the instructions.
  leds.show();
}


uint16_t renderTimeUnit(uint8_t value, uint8_t num, uint32_t color) {
  uint16_t newLed = (value * NUM_LEDS / num + LED_OFFSET) % NUM_LEDS;
  leds.setPixelColor(newLed, color);
  return newLed;
}

