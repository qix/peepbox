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

const int MOTOR_STEPS = 2048;
const int BIG_WHEEL_STEPS = 30720;
const int REV_STEPS = 30720;

// Amount of steps to ensure the gears engage
const int BUFFER_STEPS = 256;

const float SWITCH_ROT = 225.25;

// Light detector mounted over the outermost ring (ring idx 4). Reads HIGH when an
// LED shines through an aligned hole onto the sensor. Adjust the pin for your wiring.
#define DETECTOR_PIN 13

// Physical angle of that detector around the wheel, in degrees: the wheel rotation
// at which ring-4 LED #0 would sit directly under the sensor. Measure this for your
// build. (Defaults to the mechanical-switch angle as a placeholder.)
const float DETECTOR_DEG = 180;

int motorSpeed = 10;
bool buttonPushed = false;
#if OTA_ENABLED
bool otaReady = false;
bool otaRunning = false;
#endif

Stepper motor(MOTOR_STEPS, 14, 26, 27, 25);//25, 27, 26, 14);

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

// Total LEDs = 12 + 16 + 20 + 24 + 32 = 104
#define TOTAL_LEDS 104

Adafruit_NeoPixel strip(TOTAL_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
uint32_t WHITE = strip.Color(255, 255, 255);
uint32_t RED = strip.Color(255, 0, 0);
uint32_t GREEN = strip.Color(0, 255, 0);
uint32_t BLUE = strip.Color(0, 0, 255);


uint32_t getRainbowColor(float pos, float brightness) {
    // 1. Constrain 'pos' to strictly stay within the 0.0 - 1.0 bounds
    pos -= (int) pos;
    if (pos < 0.0) pos = 0.0;
    if (pos > 1.0) pos = 1.0;

    // 2. Constrain 'brightness' to strictly stay within 0.0 - 1.0 bounds
    if (brightness < 0.0) brightness = 0.0;
    if (brightness > 1.0) brightness = 1.0;

    // 3. Map the 0.0-1.0 float to a 0-255 integer
    byte wheelPos = pos * 255.0;

    // Variables to hold the raw color before brightness is applied
    byte r, g, b;

    // 4. Divide the color wheel into three equal sectors
    if (wheelPos < 85) {
        // Sector 1: Red fading to Green
        r = 255 - wheelPos * 3;
        g = wheelPos * 3;
        b = 0;
    } else if (wheelPos < 170) {
        // Sector 2: Green fading to Blue
        wheelPos -= 85;
        r = 0;
        g = 255 - wheelPos * 3;
        b = wheelPos * 3;
    } else {
        // Sector 3: Blue fading to Red
        wheelPos -= 170;
        r = wheelPos * 3;
        g = 0;
        b = 255 - wheelPos * 3;
    }

    // 5. Apply the brightness multiplier and return the final color
    return strip.Color(r * brightness, g * brightness, b * brightness);
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
#define DETECTOR_DEBOUNCE_MS 50

// How long a flipped reading must persist before an edge sweep believes it.
#define DETECTOR_EDGE_DEBOUNCE_MS 5

// Single raw sample of the detector line. True when light is reaching the sensor
// through a hole. Flip the comparison if your sensor is active-low; if it's
// analog, swap for an analogRead() threshold.
int lastDetectorRead = -1;
bool detectorRaw() {
  int read = digitalRead(DETECTOR_PIN);
  if (read != lastDetectorRead) {
    lastDetectorRead = read;
    Serial.printf(
        "%03d] Detector %s [steps=%d]\n",
        millis(), read == LOW ? "on" : "off", wheelSteps
      );
  }
  return read == LOW;
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
  return value;
}

// Detector read tuned for finding an edge while the wheel is stepping. We tell it
// the value we expect to still be seeing; as long as the line agrees it returns
// immediately, so sweeping across a long ON (or OFF) run costs nothing. Only when a
// sample flips away from `expected` do we pause, holding it for
// DETECTOR_EDGE_DEBOUNCE_MS to reject a hole-edge glitch before trusting the change.
// Unlike detectorReads(), there is no fixed per-call wait.
bool detectorHolds(bool expected) {
  if (detectorRaw() == expected) return true;
  unsigned long changedAt = millis();
  while (millis() - changedAt < DETECTOR_EDGE_DEBOUNCE_MS) {
    if (detectorRaw() == expected) return true;  // glitch; snapped back
  }
  return false;  // the flip held: a real edge
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

void setRingLeds(int ring, uint32_t color) {
  strip.clear();
  for (int i = ringStart[ring]; i < ringStart[ring] + ringSizes[ring]; i++) {
    strip.setPixelColor(i, color);
  }
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
  delay(25);
  while (millis() - start < 1000) {
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

  Serial.printf("% 4dms] Step 0. Ensure detector is not broken\n", millis());
  setAllLeds(0);
  if (detectorReads()) {
    Serial.println("Calibration FAILED: no detector found.");
    return;
  }

  // 1. Light everything and rotate until a hole lines an LED up with the detector.
  Serial.printf("% 4dms] Step 1. Find initial position\n", millis());
  setRingLeds(4, WHITE);
  long startSteps = wheelSteps;
  while (!detectorHolds(true) && (wheelSteps - startSteps) < BIG_WHEEL_STEPS) {
    stepWheel(3);
  }
  if (!detectorReads()) {
    Serial.println("Calibration FAILED: no hole/detector alignment found.");
    return;
  }

  // 2. Measure the detector's OFF switching time (light removed -> reads 0).
  setAllLeds(0);
  delay(100);
  unsigned long offTime = waitDetector(false);
  Serial.printf("% 4dms] Step 2. Wait until off (%dms)\n", millis(), offTime);

  // 3. Measure the ON switching time (light restored -> reads 1).
  setRingLeds(4, WHITE);
  unsigned long onTime = waitDetector(true);
  Serial.printf("% 4dms] Step 3. Wait until back on (%dms)\n", millis(), onTime);

  // Settle interval for the per-LED sweep: double the slower switch time.
  unsigned long settle = 2 * max(offTime, onTime);
  if (settle < 5) settle = 5;
  Serial.printf("% 4dms] Switch times: off=%lums on=%lums -> settle=%lums\n",
                millis(), offTime, onTime, settle);

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
  Serial.printf("% 4dms] Chosen ring-4 LED: pixel %d (strip index %d)\n",
                millis(), chosen, ringStart[4] + chosen);

  // 5. Centre on the chosen LED by measuring the span over which it lights the
  //    detector. The wheel is sitting inside that ON pulse now, so rotate both ways
  //    to find each edge of *this* pulse and take the midpoint — no need to leave it
  //    and chase the next one. We use detectorHolds() so stepping across the span
  //    costs nothing; we only wait when a reading flips, to confirm a real edge.
  setOneLed(ringStart[4] + chosen, WHITE);

  Serial.printf("% 4dms] Step 5a. Reverse until detector is off\n", millis());
  while (detectorHolds(true)) stepWheel(-1);

  // Make sure we overshoot a little, so that the gear has a chance to engage in the
  // forward pass, since there's a bit of give in each direction.
  stepWheel(-BUFFER_STEPS);

  Serial.printf("% 4dms] Step 5b. Step until detector back on\n", millis());
  while (detectorHolds(false)) stepWheel(1);  // back into the ON span
  long highEdge = wheelSteps;

  Serial.printf("% 4dms] Step 5c. Step until detector is off again\n", millis());
  while (detectorHolds(true)) stepWheel(1);   // Went off
  long lowEdge = wheelSteps;

  long centerSteps = (highEdge + lowEdge) / 2;
  Serial.printf("% 4dms] Result: %d steps\n", millis(), centerSteps);

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

  ArduinoOTA.setHostname("led-wheel");
  ArduinoOTA.setPassword("wFemjxN28");

  ArduinoOTA.onStart([]() {
    // Stop the motor and blank the LEDs so the flash write runs cleanly.
    motorSpeed = 0;
    otaRunning = true;
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

void setDisco(bool red, bool green, bool blue) {
  digitalWrite(19, !red);
  digitalWrite(21, !green);
  digitalWrite(18, !blue);
}
void setup() {
  Serial.begin(115200);
  motor.setSpeed(15);

  pinMode(DETECTOR_PIN, INPUT);

  pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);
  pinMode(21, OUTPUT);
  setDisco(true, true, true);

  strip.begin();
  strip.show();
  strip.setBrightness(50);

#if OTA_ENABLED
  setupOTA();
#endif

  calibrate();

  Serial.println("--- Multi-Ring Controller Ready ---");
}

float ringDistance(float ring, float point) {
  /** Calculate the distance between rings */
  return min(fabsf(ring - point), fabsf(NUM_RINGS + ring - point));
}

uint32_t pixelColor(
    float wheelDegrees,

    int i,
    int ring,
    int ringPixel
) {
  // Where this LED currently sits, in degrees, as the wheel turns. The
  // LEDs are evenly spaced around the ring, and the whole ring is rotated
  // by `degrees`.
  float pixelDeg = fmod(
      ((ringPixel * 360.0f) / ringSizes[ring] - wheelDegrees) + 270, 360.0);
  if (pixelDeg < 0) pixelDeg += 360.0;

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


  float ringP = fmodf(millis() / (5000.0f), NUM_RINGS);
  float ringP2 = fmodf(millis() / (5000.0f) + 2500, NUM_RINGS);

  float brightness = 1 - min(ringDistance(ring, ringP), ringDistance(ring, ringP));
  if (brightness < 0) brightness = 0;

  byte b = (byte) (brightness * 255);

  // First/fourth ring bright white (split light)
  if (ring == 0 || ring == 3) return strip.Color(b, b, b);

  // Second ring in rainbow mode
  if (ring == 2) {
    return getRainbowColor(pixelDeg / 360.f + millis() / 3000.0f, brightness);
  }

  // Fifth ring is fast rainbow
  if (ring == 4) {
    return getRainbowColor(pixelDeg / 360.f + millis() / 1000.0f, brightness);
  }

  /*** Light pixels up by their distance to a hole ***/
  if (distanceToClosestHole < 0) {
    return strip.Color((byte) (closeness * brightness * 255), 0, 0);
  } else {
    return strip.Color(0, (byte) (closeness * brightness * 255), 0);
  }

  /** Light up just the top, 340-360deg in green, 0-20 deg in red **/
  if (pixelDeg > 0 && pixelDeg < 20) {
    return strip.Color(255, 0, 0);
  } else if (pixelDeg > 340) {
    return strip.Color(0, 255, 0);
  }

  return 0; //getRainbowColor(pixelDeg / 360.0f);

  if (pixelDeg < 90) {
    return strip.Color(255, 0, 0);
  } else if (pixelDeg < 180) {
    return strip.Color(0, 255, 0);
  } else if (pixelDeg < 270) {
    return strip.Color(0, 0, 255);
  }
  return strip.Color(0, 0, 0);
}

void loop() {
#if OTA_ENABLED
  if (otaReady) ArduinoOTA.handle();
  if (otaRunning) {
    return;
  }
#endif

  static unsigned long lastReportMs = millis();

  strip.clear();
  unsigned long now = millis();

  // Enable strobe effect
  bool on = true;
  setDisco(on, on, on);

  // Current rotation of the wheel since the last switch trigger.
  // @todo: note if not calibrated
  float degrees = currentRotationDeg();

  // Walk every pixel in the chain, tracking which ring it belongs to and
  // its index within that ring.
  int ring = 0;
  int ringPixel = 0;
  for (int i = 0; i < TOTAL_LEDS; i++) {
    if (on) {
      strip.setPixelColor(i, pixelColor(degrees, i, ring, ringPixel));
    }

    ringPixel += 1;
    if (ringPixel >= ringSizes[ring]) {
      ringPixel = 0;
      ring += 1;
    }
  }

  strip.show();
  stepWheel(motorSpeed);

  if (now - lastReportMs >= 50) {
    if (!calibrated) {
      Serial.print("UNCALIBRATED ");
    }
    Serial.print("Rotation: ");
    Serial.print(degrees);
    Serial.println(" deg");
    lastReportMs = now;
  }
}
