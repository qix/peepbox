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

const float SWITCH_ROT = 225.25;

int motorSpeed = 10;
bool buttonPushed = false;

Stepper motor(STEPS_PER_REV, 25, 27, 26, 14);

// Define the sizes of your rings

int ringSizes[NUM_RINGS] = {12, 16, 20, 24, 32};
int ringStart[NUM_RINGS] = {0, 12, 28, 48, 72};
float firstHoleDeg[NUM_RINGS] = {
  SWITCH_ROT,
  SWITCH_ROT + 180.00 / ringSizes[1],
  SWITCH_ROT,
  SWITCH_ROT,
  SWITCH_ROT,
};
int pixelCounts[NUM_RINGS] = {20, 30, 40, 40, 50};

// Define colors for each ring (Red, Green, Blue, Yellow, Purple)
uint32_t ringColors[NUM_RINGS];

// Total LEDs = 12 + 16 + 20 + 24 + 32 = 104
#define TOTAL_LEDS 104

Adafruit_NeoPixel strip(TOTAL_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
uint32_t WHITE = strip.Color(255, 255, 255);
uint32_t RED = strip.Color(255, 0, 0);
uint32_t GREEN = strip.Color(0, 255, 0);
uint32_t BLUE = strip.Color(0, 0, 255);


uint32_t getRainbowColor(float pos) {
    pos -= (int) pos;
    // 1. Constrain 'pos' to strictly stay within the 0.0 - 1.0 bounds
    if (pos < 0.0) pos = 0.0;
    if (pos > 1.0) pos = 1.0;

    // 2. Map the 0.0-1.0 float to a 0-255 integer
    byte wheelPos = pos * 255.0;

    // 3. Divide the color wheel into three equal sectors
    if (wheelPos < 85) {
        // Sector 1: Red fading to Green
        return strip.Color(255 - wheelPos * 3, wheelPos * 3, 0);
    } else if (wheelPos < 170) {
        // Sector 2: Green fading to Blue
        wheelPos -= 85;
        return strip.Color(0, 255 - wheelPos * 3, wheelPos * 3);
    } else {
        // Sector 3: Blue fading to Red
        wheelPos -= 170;
        return strip.Color(wheelPos * 3, 0, 255 - wheelPos * 3);
    }
}

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
  if (1) {
    for (int r = 0; r < 5; r++) {
      strip.setPixelColor(ringStart[r], RED);
      strip.setPixelColor(ringStart[r] + ringSizes[r] / 2, GREEN);
    }
  }else if (1 || now % 15000 < 10000) {
      int ring = 0;
      int ringPixel = 0;
      for (int i = 0; i < TOTAL_LEDS; i++) {
          ringPixel += 1;
          if (ringPixel >= ringSizes[ring]) {
              ringPixel = 0;
              ring += 1;
          }

          if (ring == 2) {
              strip.setPixelColor(i,
                  getRainbowColor(
                      ((
                          (ringPixel + 0.0) / ringSizes[ring]
                       ) + now / 1000.0)
                  )
              );
              if ((ringPixel + now / 15) % ringSizes[ring] != 0) {
                  strip.setPixelColor(i, 0);
              }
          } else {
            strip.setPixelColor(i, strip.Color(
                255 * (ring % 2),
                255 * ((ring + 1) % 2),
                0 * (ring % 3 == 0)
            ));
          }
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

  motor.step(-motorSpeed);
  stepsSinceSwitch += motorSpeed;

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
    buttonPushed = true;
  }
  lastSwitchState = switchState;


  float degrees = (stepsSinceSwitch * 360.0f) / stepsPerRev;
  if (now - lastReportMs >= 50) {
    Serial.print("Rotation: ");
    Serial.print(degrees);
    Serial.println(" deg");
    lastReportMs = now;

  }
  if (buttonPushed && degrees > 225.25) {
  //if (degrees > 0.10) {
    motorSpeed = 0;
  }
}
