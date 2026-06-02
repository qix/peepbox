#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Stepper.h>

// Configuration
#define SWITCH_PIN 23
#define LED_PIN    32
#define NUM_RINGS   5
#define DELAY_MS    100  // Sped it up slightly since there are 104 LEDs now!

const int STEPS_PER_REV = 2048;
const int BIG_WHEEL_STEPS = 30720;

Stepper motor(STEPS_PER_REV, 25, 27, 26, 14);

// Define the sizes of your rings
int ringSizes[NUM_RINGS] = {12, 16, 20, 24, 32};
int ringStart[NUM_RINGS] = {0, 12, 28, 48, 72};

// Define colors for each ring (Red, Green, Blue, Yellow, Purple)
uint32_t ringColors[NUM_RINGS];

// Total LEDs = 12 + 16 + 20 + 24 + 32 = 104
#define TOTAL_LEDS 104

Adafruit_NeoPixel strip(TOTAL_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200);
  motor.setSpeed(15);

  pinMode(SWITCH_PIN, INPUT_PULLUP);

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
  static long stepsSinceSwitch = 0;
  static long stepsPerRev = BIG_WHEEL_STEPS;
  static unsigned long lastSwitchMs = millis();
  static unsigned long lastReportMs = millis();
  static int lastSwitchState = HIGH;

  strip.clear();

  unsigned long now = millis();
  if (1 || now % 15000 < 10000) {
      int ring = 0;
      int ringPixel = 0;
      for (int i = 0; i < TOTAL_LEDS; i++) {
          ringPixel += 1;
          if (ringPixel >= ringSizes[ring]) {
              ringPixel = 0;
              ring += 1;
          }

          strip.setPixelColor(i, strip.Color(
              255 * (ring % 2),
              255 * ((ring + 1) % 2),
              255 * (ring % 3 == 0)
          ));
      }
  } else if (now % 15000 < 5000) {
      for (int i = 0; i < TOTAL_LEDS; i++) {
          strip.setPixelColor(i, strip.Color(128, (now / 10 + i) % 128, 128));
      }
  } else {
    //strip.setPixelColor(0, strip.Color(255, 255, 255));
    //strip.setPixelColor(ringStart[0], strip.Color(255, 0, 0));
    //strip.setPixelColor(ringStart[1], strip.Color(255, 0, 255));
    //strip.setPixelColor(ringStart[2], strip.Color(255, 255, 0));
    //strip.setPixelColor(ringStart[3], strip.Color(0, 255, 0));
    //strip.setPixelColor(ringStart[4], strip.Color(0, 0, 255));

    float revFraction = (float)stepsSinceSwitch / stepsPerRev;
    for (int r = 0; r < NUM_RINGS; r++) {
      float offsetFloat = (revFraction * ringSizes[r]);
      if (offsetFloat - (int)offsetFloat < 0.9) {
          for (int e = 0; e < 8; e++) {
              int offset = ((int)offsetFloat) % ringSizes[r];
              if (offset < 0) offset += ringSizes[r];

              offset = (offset + e * (ringSizes[r] / 8)) % ringSizes[r];
              strip.setPixelColor(ringStart[r] + offset, strip.Color(255, 255, 255));
              Serial.println(offset);
          }
      }
    }
  }


  strip.show();

  motor.step(-10);
  stepsSinceSwitch += 10;

  int switchState = digitalRead(SWITCH_PIN);
  if (switchState == LOW && lastSwitchState == HIGH) {
    unsigned long now = millis();
    Serial.print("Revolution: ");
    Serial.print(now - lastSwitchMs);
    Serial.print(" ms, steps: ");
    Serial.println(stepsSinceSwitch);
    stepsPerRev = stepsSinceSwitch;
    lastSwitchMs = now;
    stepsSinceSwitch = 0;
  }
  lastSwitchState = switchState;


  if (now - lastReportMs >= 500) {
    float degrees = (stepsSinceSwitch * 360.0f) / stepsPerRev;
    Serial.print("Rotation: ");
    Serial.print(degrees);
    Serial.println(" deg");
    lastReportMs = now;
  }
}
