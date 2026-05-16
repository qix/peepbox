#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// Configuration
#define DATA_PIN    32
#define NUM_RINGS   5
#define DELAY_MS    100  // Sped it up slightly since there are 104 LEDs now!

// Define the sizes of your rings
int ringSizes[NUM_RINGS] = {12, 16, 20, 24, 32};

// Define colors for each ring (Red, Green, Blue, Yellow, Purple)
uint32_t ringColors[NUM_RINGS];

// Total LEDs = 12 + 16 + 20 + 24 + 32 = 104
#define TOTAL_LEDS 104

Adafruit_NeoPixel strip(TOTAL_LEDS, DATA_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200);
  delay(1000);

  strip.begin();
  strip.show();
  strip.setBrightness(50);

  // Initialize the color array using the strip.Color helper
  ringColors[0] = strip.Color(255, 0, 0);   // Ring 1: Red
  ringColors[1] = strip.Color(0, 255, 0);   // Ring 2: Green
  ringColors[2] = strip.Color(0, 0, 255);   // Ring 3: Blue
  ringColors[3] = strip.Color(255, 255, 0); // Ring 4: Yellow
  ringColors[4] = strip.Color(255, 0, 255); // Ring 5: Purple

  Serial.println("--- Multi-Ring Controller Ready ---");
}

void loop() {
  int currentGlobalLED = 0; // Tracks the absolute position in the chain

  // Loop through each ring
  for (int r = 0; r < NUM_RINGS; r++) {
    Serial.print(">>> Starting Ring #");
    Serial.print(r + 1);
    Serial.print(" (Size: ");
    Serial.print(ringSizes[r]);
    Serial.println(")");

    if (r != 2) {
      for (int l = 0; l < ringSizes[r]; l++) {
        currentGlobalLED++;
      }
      continue;
    }

    // Loop through each LED within the current ring
    for (int l = 0; l < ringSizes[r]; l++) {

      // Light up the LED using the color assigned to this ring
      strip.setPixelColor(currentGlobalLED, ringColors[r]);
      strip.setPixelColor(currentGlobalLED, strip.Color(255, 255, 255));
      strip.show();

      delay(DELAY_MS);

      // Turn it off
      strip.setPixelColor(currentGlobalLED, strip.Color(0, 0, 0));

      currentGlobalLED++; // Move to the next physical LED in the chain
    }
  }
}
