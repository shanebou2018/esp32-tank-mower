// WS2812 blink test — ESP32-DevKitC-32E
// LED on GPIO 16, GRB color order
// Cycles Red → Green → Blue → White → Off, 1 second each

#include <FastLED.h>

#define LED_PIN   16
#define NUM_LEDS  1

CRGB leds[NUM_LEDS];

void setup() {
  Serial.begin(115200);
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(128);
  Serial.println("WS2812 blink test starting...");
}

void loop() {
  const struct { CRGB color; const char* name; } steps[] = {
    { CRGB::Red,   "RED"   },
    { CRGB::Green, "GREEN" },
    { CRGB::Blue,  "BLUE"  },
    { CRGB::White, "WHITE" },
    { CRGB::Black, "OFF"   },
  };

  for (auto& s : steps) {
    Serial.println(s.name);
    leds[1] = s.color;
    FastLED.show();
    delay(1000);
  }
}
