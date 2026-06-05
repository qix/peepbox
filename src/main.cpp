#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Stepper.h>

// OTA / WiFi is compiled in only when ENABLE_OTA is truthy. Set the env var and
// build (e.g. `ENABLE_OTA=1 pio run`) to include it; when unset or 0 none of the
// WiFi/ArduinoOTA libraries or code are pulled in. See platformio.ini.
#if defined(ENABLE_OTA) && (ENABLE_OTA + 0)
  #define OTA_ENABLED 1
#else
  #define OTA_ENABLED 0
#endif

#if OTA_ENABLED
#include <WiFi.h>
#include <ArduinoOTA.h>

// WiFi credentials come from build flags (see platformio.ini + secrets.ini).
// Fallbacks here just let it compile if you haven't filled secrets.ini yet.
#ifndef WIFI_SSID
#define WIFI_SSID "your-ssid"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS "your-pass"
#endif
#endif // OTA_ENABLED

// Configuration
#define LED_PIN    32
#define NUM_RINGS   5
#define DELAY_MS    100  // Sped it up slightly since there are 104 LEDs now!

const int STEPS_PER_REV = 2048;
const int BIG_WHEEL_STEPS = 30720;

const float SWITCH_ROT = 225.25;

// Light detector mounted over the outermost ring (ring idx 4). Reads HIGH when an
// LED shines through an aligned hole onto the sensor. Adjust the pin for your wiring.
#define DETECTOR_PIN 23

// Physical angle of that detector around the wheel, in degrees: the wheel rotation
// at which ring-4 LED #0 would sit directly under the sensor. Measure this for your
// build. (Defaults to the mechanical-switch angle as a placeholder.)
const float DETECTOR_DEG = SWITCH_ROT;

int motorSpeed = 10;
bool buttonPushed = false;
#if OTA_ENABLED
bool otaReady = false;
#endif

Stepper motor(STEPS_PER_REV, 25, 27, 26, 14);

// Define the sizes of your rings

int ringSizes[NUM_RINGS] = {12, 16, 20, 24, 32};
int ringStart[NUM_RINGS] = {0, 12, 28, 48, 72};
float firstHoleDeg[NUM_RINGS] = {
  SWITCH_ROT,
  (float) (SWITCH_ROT + 180.00 / ringSizes[1]),
  SWITCH_ROT,
  SWITCH_ROT,
  SWITCH_ROT,
};
int holeCounts[NUM_RINGS] = {20, 30, 40, 40, 50};

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

// ---------------------------------------------------------------------------
// Initial position detection (startup calibration)
//
// Establishes where 0 deg is by finding which ring-4 LED sits under the light
// detector, then centring on it. See SPEC.md "Initial position detection".
// ---------------------------------------------------------------------------

long wheelSteps = 0;       // total motor steps since boot; never reset
long zeroStepOffset = 0;   // value of wheelSteps that corresponds to 0 deg
bool calibrated = false;

// How long the detector line must hold a value before calibration trusts it.
#define DETECTOR_DEBOUNCE_MS 5

// Single raw sample of the detector line. True when light is reaching the sensor
// through a hole. Flip the comparison if your sensor is active-low; if it's
// analog, swap for an analogRead() threshold.
bool detectorRaw() {
  return digitalRead(DETECTOR_PIN) == LOW;
}

// Debounced read used during calibration: returns a value only once the line has
// held it steady for DETECTOR_DEBOUNCE_MS, so a brief glitch at a hole edge isn't
// counted. The stability window restarts whenever a sample differs; an overall
// cap stops it spinning forever if the line never settles.
bool detectorReads() {
  bool value = detectorRaw();
  unsigned long stableSince = millis();
  unsigned long start = stableSince;
  while (millis() - stableSince < DETECTOR_DEBOUNCE_MS) {
    bool sample = detectorRaw();
    if (sample != value) {
      value = sample;
      stableSince = millis();   // changed: restart the 5 ms window
    }
    if (millis() - start > 50) break;  // never settled; take the latest sample
  }
  Serial.printf("Read %d\n", value);
  return value;
}

// Step the wheel by `steps` in the running direction, keeping wheelSteps in sync.
void stepWheel(long steps) {
  motor.step(-steps);
  wheelSteps += steps;
}

void setAllLeds(uint32_t color) {
  for (int i = 0; i < TOTAL_LEDS; i++) strip.setPixelColor(i, color);
  strip.show();
}

void setOneLed(int idx, uint32_t color) {
  strip.clear();
  strip.setPixelColor(idx, color);
  strip.show();
}

// Wait (wheel stationary) for the detector to reach `target`, returning how long
// that took in ms. Bounded so a missing/stuck sensor can't hang startup.
unsigned long waitDetector(bool target) {
  unsigned long start = millis();
  while (detectorReads() != target && millis() - start < 1000) {
    if (detectorReads() == target) {
      return millis() - start;
    }
    /* spin */
  }
  Serial.printf("Detector read timeout waiting for %d\n", target);
  return 1000;
}

// Current wheel rotation in [0,360); meaningful only once calibrated.
float currentRotationDeg() {
  float deg = fmodf((wheelSteps - zeroStepOffset) * 360.0f / BIG_WHEEL_STEPS, 360.0f);
  if (deg < 0) deg += 360.0f;
  return deg;
}

void calibrate() {
  Serial.println("Calibration: searching for zero...");

  // 1. Light everything and rotate until a hole lines an LED up with the detector.
  setAllLeds(WHITE);
  long startSteps = wheelSteps;
  while (!detectorReads() && (wheelSteps - startSteps) < BIG_WHEEL_STEPS) {
    stepWheel(1);
  }
  if (!detectorReads()) {
    Serial.println("Calibration FAILED: no hole/detector alignment found.");
    return;
  }

  // 2. Measure the detector's OFF switching time (light removed -> reads 0).
  setAllLeds(0);
  delay(100);
  unsigned long offTime = waitDetector(false);
  Serial.printf("2. Wait until off (%dms)\n", offTime);

  // 3. Measure the ON switching time (light restored -> reads 1).
  setAllLeds(WHITE);
  unsigned long onTime = waitDetector(true);
  Serial.printf("3. Wait until back on (%dms)\n", onTime);

  // Settle interval for the per-LED sweep: double the slower switch time.
  unsigned long settle = 2 * max(offTime, onTime);
  if (settle < 5) settle = 5;
  Serial.printf("Switch times: off=%lums on=%lums -> settle=%lums\n",
                offTime, onTime, settle);

  // 4. Light one ring-4 LED at a time; the one that reaches the detector is ours.
  int chosen = -1;
  for (int p = 0; p < ringSizes[4]; p++) {
    setOneLed(ringStart[4] + p, WHITE);
    delay(settle);
    if (detectorReads()) { chosen = p; break; }
  }
  if (chosen < 0) {
    Serial.println("Calibration FAILED: no single LED lit the detector.");
    return;
  }
  Serial.printf("Chosen ring-4 LED: pixel %d (strip index %d)\n",
                chosen, ringStart[4] + chosen);

  // 5. Centre on the chosen LED: leave the current pulse, then capture the next
  //    rising (0->1) and falling (1->0) edges and take their midpoint.
  setOneLed(ringStart[4] + chosen, WHITE);
  while (detectorReads()) stepWheel(1);    // off the current pulse
  while (!detectorReads()) stepWheel(1);   // rising edge
  long rising = wheelSteps;
  while (detectorReads()) stepWheel(1);    // falling edge
  long falling = wheelSteps;
  long centerSteps = (rising + falling) / 2;

  // 6. The chosen LED is now known to be centred under the detector, so the wheel
  //    rotation there is DETECTOR_DEG minus the LED's own angle within its ring.
  float ledAngle = chosen * 360.0f / ringSizes[4];
  float rotationAtCenter = DETECTOR_DEG - ledAngle;
  zeroStepOffset = centerSteps - lroundf(rotationAtCenter * BIG_WHEEL_STEPS / 360.0f);
  calibrated = true;

  Serial.printf("Calibration done: centerSteps=%ld, rotation there=%.2f deg, "
                "zeroStepOffset=%ld\n", centerSteps, rotationAtCenter, zeroStepOffset);
}

#if OTA_ENABLED
void setupOTA() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // Bounded wait — don't hang the wheel forever if the network is down.
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    delay(250);
    Serial.print('.');
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi failed; running without OTA.");
    return;
  }

  ArduinoOTA.setHostname("led-wheel");     // reachable as led-wheel.local
  // ArduinoOTA.setPassword("changeme");   // uncomment to require a password

  ArduinoOTA.onStart([]() {
    // Stop the motor and blank the LEDs so the flash write runs cleanly.
    motorSpeed = 0;
    strip.clear();
    strip.show();
    Serial.println("OTA update starting...");
  });
  ArduinoOTA.onProgress([](unsigned int done, unsigned int total) {
    Serial.printf("OTA %u%%\r", (done * 100) / total);
  });
  ArduinoOTA.onError([](ota_error_t err) {
    Serial.printf("\nOTA error %u\n", err);
  });
  ArduinoOTA.onEnd([]() { Serial.println("\nOTA done, rebooting."); });

  ArduinoOTA.begin();
  otaReady = true;
  Serial.print("OTA ready at ");
  Serial.println(WiFi.localIP());
}
#endif // OTA_ENABLED

void setup() {
  Serial.begin(115200);
  motor.setSpeed(15);

  pinMode(DETECTOR_PIN, INPUT);

  strip.begin();
  strip.show();
  strip.setBrightness(50);

  // Initialize the color array using the strip.Color helper
  ringColors[0] = strip.Color(255, 0, 0);   // Ring 1: Red
  ringColors[1] = strip.Color(0, 255, 0);   // Ring 2: Green
  ringColors[2] = strip.Color(0, 0, 255);   // Ring 3: Blue
  ringColors[3] = strip.Color(255, 255, 0); // Ring 4: Yellow
  ringColors[4] = strip.Color(255, 0, 255); // Ring 5: Purple

#if OTA_ENABLED
  setupOTA();
#endif

  calibrate();

  Serial.println("--- Multi-Ring Controller Ready ---");
}

void loop() {
#if OTA_ENABLED
  if (otaReady) ArduinoOTA.handle();
#endif

  static long stepsSinceSwitch = 0;
  static long stepsPerRev = BIG_WHEEL_STEPS;
  static unsigned long lastSwitchMs = millis();
  static unsigned long lastReportMs = millis();
  static int lastSwitchState = HIGH;

  strip.clear();

  unsigned long now = millis();

  // Current rotation of the wheel since the last switch trigger.
  float degrees = calibrated ? currentRotationDeg()
                             : (stepsSinceSwitch * 360.0f) / stepsPerRev;

  if (1) {
    // Walk every pixel in the chain, tracking which ring it belongs to and
    // its index within that ring.
    int ring = 0;
    int ringPixel = 0;
    for (int i = 0; i < TOTAL_LEDS; i++) {
      // Where this LED currently sits, in degrees, as the wheel turns. The
      // LEDs are evenly spaced around the ring, and the whole ring is rotated
      // by `degrees`.
      float pixelDeg = (ringPixel * 360.0f) / ringSizes[ring] + degrees;

      // Holes are evenly spaced too, with the first one at firstHoleDeg[ring].
      // Reduce the gap from that first hole into a single hole spacing, then
      // centre it so it measures the distance to the *nearest* hole.
      float holeSpacing = 360.0f / holeCounts[ring];
      float distanceToClosestHole = fmodf(pixelDeg - firstHoleDeg[ring], holeSpacing);
      if (distanceToClosestHole >  holeSpacing / 2) distanceToClosestHole -= holeSpacing;
      if (distanceToClosestHole < -holeSpacing / 2) distanceToClosestHole += holeSpacing;
      // distanceToClosestHole < 0  -> the hole is still coming up
      // distanceToClosestHole > 0  -> the hole has just passed

      // Light the pixel brightest when it lines up with a hole, fading out as
      // it moves away. (Tweak this to taste.)
      float closeness = 1.0f - fabsf(distanceToClosestHole) / (holeSpacing / 2);
      if (closeness < 0) closeness = 0;
      byte b = (byte)(closeness * 255);

      if (distanceToClosestHole < 0) {
	strip.setPixelColor(i, strip.Color(b, 0, 0));
      } else {
	strip.setPixelColor(i, strip.Color(0, b, 0));
      }

      // Advance to the next LED, rolling over into the next ring.
      ringPixel += 1;
      if (ringPixel >= ringSizes[ring]) {
        ringPixel = 0;
        ring += 1;
      }
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

  stepWheel(motorSpeed);
  stepsSinceSwitch += motorSpeed;

  int switchState = digitalRead(DETECTOR_PIN);
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


  if (now - lastReportMs >= 50) {
    Serial.print("Rotation: ");
    Serial.print(degrees);
    Serial.println(" deg");
    lastReportMs = now;
  }
  /**
   * Used for centering on the wheel
	  if (buttonPushed && degrees > 225.25) {
	  //if (degrees > 0.10) {
	    motorSpeed = 0;
	  }
  */
}
