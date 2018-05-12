#include <Adafruit_NeoPixel.h>

// LED configuration
#define LED_PIN     13 // Pin the LEDs are connected to
#define NUM_LEDS    30 // Number of LEDs in the strip
#define LED_OFFSET  14 // Index of the 12-o'clock LED

// NEO_KHZ800 - 800KHz bitstream for WS2812 LEDs
// NEO_GRB - Green, Red, Blue bitstream
Adafruit_NeoPixel leds = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(9600); // Serial connection, for debugging.
  
  leds.begin();
  leds.show(); // Turn off all LEDs.

  renderTime(0, 6, 53);
}

void loop() {}

// Renders the given hour, minute, and second to the LED strip.
// Hour uses 3 LEDs, minute uses 2, and second uses 1.
// The arduino's 5V data pin supplies ~50ma to the 5V pin, and
// each LED draws up to 60ma at full brightness.  Only ~8 pixels 
// can be on at a time.
void renderTime(uint8_t hour, uint8_t minute, uint8_t second) {
  leds.setPixelColor((hour * NUM_LEDS / 12 + LED_OFFSET) % NUM_LEDS, 255, 0, 0);
  leds.setPixelColor((minute * NUM_LEDS / 60 + LED_OFFSET) % NUM_LEDS, 0, 255, 0);
  leds.setPixelColor((second * NUM_LEDS / 60 + LED_OFFSET) % NUM_LEDS, 0, 0, 255);
  leds.show();
}

