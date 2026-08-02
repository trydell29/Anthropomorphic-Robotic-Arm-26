// ============================================================================
// ARA - Anthropomorphic Robotic Arm
// On-arm ESP32 firmware.  Implements docs/PROTOCOL.md v0.1.
//
// Responsibilities, in order of importance:
//   1. Own the authoritative 13-axis target array.  Nothing else commands an
//      actuator.
//   2. Arbitrate between two writers: the UDP vision stream and the portal.
//   3. Clamp everything, always, as the last step before output.
//
// Everything marked TBD must be set from bench testing before running CV.
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <AsyncUDP.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_PWMServoDriver.h>
#include <FastAccelStepper.h>

// ============================================================================
// CONFIG
// ============================================================================

// --- Network ---
static const char *AP_SSID = "ARA-HAND";
static const char *AP_PASS = "";          // open network; see README before changing
static const uint16_t UDP_PORT = 4210;

// --- I2C / PCA9685 ---
static const uint8_t PIN_SDA = 21;        // TBD confirm
static const uint8_t PIN_SCL = 22;        // TBD confirm
static const uint8_t PCA_ADDR = 0x40;
static const float   PCA_FREQ_HZ = 50.0f;

// --- Stepper pins ---  TBD confirm all four
static const uint8_t PIN_ROLL_STEP = 26;
static const uint8_t PIN_ROLL_DIR  = 25;
static const uint8_t PIN_EXT_STEP  = 33;
static const uint8_t PIN_EXT_DIR   = 32;
// EN is tied to GND on both drivers -- no enable pin in software.

// --- Stepper geometry ---
static const float STEPS_PER_REV = 200.0f;   // 1.8 deg NEMA 17
static const float MICROSTEPS    = 8.0f;     // TBD confirm MS1/MS2 straps
static const float GEAR_ROLL     = 4.0f;     // pron/sup planetary
static const float GEAR_EXT      = 64.0f;    // elbow planetary

// Motion limits.  The 64:1 axis is slow by construction -- do not chase it
// with more acceleration, a printed planetary will skip and lose the zero.
static const uint32_t ROLL_SPEED_HZ = 4000;
static const uint32_t ROLL_ACCEL    = 8000;
static const uint32_t EXT_SPEED_HZ  = 6000;
static const uint32_t EXT_ACCEL     = 6000;

// --- Timing ---
static const uint32_t CONTROL_PERIOD_MS = 20;    // 50 Hz output tick
static const uint32_t WATCHDOG_MS       = 400;   // PROTOCOL section 3.2
static const uint32_t RAMP_MS           = 300;   // CV entry ramp

// --- Servo pulse range (microseconds) ---
// MG996R and SG90 both tolerate 500-2500.  Narrow per-servo if a joint
// buzzes at an endpoint.
static const uint16_t SERVO_MIN_US = 500;
static const uint16_t SERVO_MAX_US = 2500;

// ============================================================================
// AXIS TABLE  -- see PROTOCOL.md section 2.  Never reorder.
// ============================================================================

enum AxisIdx : uint8_t {
  AX_THUMB_FLEX = 0, AX_INDEX_FLEX, AX_MIDDLE_FLEX, AX_RING_FLEX, AX_PINKY_FLEX,
  AX_WRIST      = 5,
  AX_INDEX_SPLAY = 6, AX_MIDDLE_SPLAY, AX_RING_SPLAY, AX_PINKY_SPLAY,
  AX_THUMB_OPP  = 10,
  AX_ELBOW_ROLL = 11,   // stepper, signed degrees
  AX_ELBOW_EXT  = 12,   // stepper, signed degrees
  AXIS_COUNT    = 13
};

static const uint8_t CV_AXIS_COUNT  = 12;  // UDP carries 0..11
static const uint8_t SERVO_COUNT    = 11;  // PCA channels 0..10

// Axis index == PCA channel for 0..10.  Deliberate.  If a servo moves to a
// different channel, move it in the axis table too -- do not add a lookup.

static const char *AXIS_NAME[AXIS_COUNT] = {
  "thumb_flex", "index_flex", "middle_flex", "ring_flex", "pinky_flex",
  "wrist",
  "index_splay", "middle_splay", "ring_splay", "pinky_splay",
  "thumb_opp",
  "elbow_roll", "elbow_ext"
};

// Limits.  Applied last, after every other stage.  No source may exceed them.
// TBD: everything except the roll pair, which is a wiring-twist limit.
static const float AXIS_MIN[AXIS_COUNT] = {
    0,   0,   0,   0,   0,     // finger flexion
   40,                         // wrist        TBD
   60,  60,  60,  60,          // splay        TBD
   40,                         // thumb opp    TBD
  -80,                         // elbow roll   service-loop limit
  -20                          // elbow ext    TBD from hard stop
};
static const float AXIS_MAX[AXIS_COUNT] = {
  180, 180, 180, 180, 180,
  140,
  120, 120, 120, 120,
  140,
  +80,
  +110
};

static const float AXIS_NEUTRAL[AXIS_COUNT] = {
  180, 180, 180, 180, 180,     // fingers open
   90,                         // wrist neutral
   90,  90,  90,  90,          // splay neutral
   90,                         // thumb opp neutral
    0,                         // roll zero
    0                          // ext zero
};

// ============================================================================
// PRESETS  -- 13 values each, PROTOCOL.md section 4.
// grasp_id refers to Feix et al. (2016); -1 where none applies.
// All values below are PLACEHOLDERS.  Re-author from the bench.
// ============================================================================

struct Preset {
  const char *name;
  int8_t grasp_id;
  float v[AXIS_COUNT];
};

static const Preset PRESETS[] = {
  // name              id    thF  idF  mdF  rgF  pkF   wr   idS  mdS  rgS  pkS  thO  roll  ext
  { "open",            -1, { 180, 180, 180, 180, 180,  90,   90,  90,  90,  90,  90,    0,   0 } },
  { "fist",            -1, {   0,   0,   0,   0,   0,  90,   90,  90,  90,  90,  90,    0,   0 } },
  { "point",           -1, {   0, 180,   0,   0,   0,  90,   90,  90,  90,  90,  90,    0,   0 } },
  { "pinch",            9, {  40, 40,  180, 180, 180,  90,   90,  90,  90,  90, 130,    0,   0 } },
  { "tripod",          10, {  40, 40,   40, 180, 180,  90,   90,  90,  90,  90, 130,    0,   0 } },
  { "lateral",         16, {  60,   0,   0,   0,   0,  90,   90,  90,  90,  90, 110,    0,   0 } },
  { "prismatic wrap",  12, {  20,  20,  20,  25,  30,  95,   90,  90,  90,  90, 140,   15,  40 } },
  { "spread",          -1, { 180, 180, 180, 180, 180,  90,  120, 100,  80,  60,  90,    0,   0 } },
};
static const uint8_t PRESET_COUNT = sizeof(PRESETS) / sizeof(PRESETS[0]);

// ============================================================================
// STATE
// ============================================================================

enum Mode : uint8_t { MODE_MANUAL = 0, MODE_CV = 1 };

static volatile Mode mode = MODE_MANUAL;

static float target[AXIS_COUNT];       // authoritative, post-clamp
static float rampFrom[AXIS_COUNT];     // snapshot for the CV entry ramp
static bool  rampActive = false;
static uint32_t rampStartMs = 0;

// CV stream health
static volatile uint32_t lastPacketMs = 0;
static volatile uint16_t lastSeq = 0;
static volatile bool     seqValid = false;
static volatile uint32_t rxCount = 0;
static volatile uint32_t malformedCount = 0;
static volatile uint32_t staleDropCount = 0;

// Values the UDP callback writes; the control loop consumes them.
static volatile float cvPending[CV_AXIS_COUNT];
static volatile bool  cvHasNew = false;

static bool switching = false;   // manual->CV, waiting on elbow return

Adafruit_PWMServoDriver pca(PCA_ADDR);
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepRoll = nullptr;
FastAccelStepper *stepExt  = nullptr;
AsyncWebServer server(80);
AsyncUDP udp;

// ============================================================================
// HELPERS
// ============================================================================

static inline float clampAxis(uint8_t i, float v) {
  if (v < AXIS_MIN[i]) return AXIS_MIN[i];
  if (v > AXIS_MAX[i]) return AXIS_MAX[i];
  return v;
}

static inline float smoothstep(float t) {
  if (t <= 0.0f) return 0.0f;
  if (t >= 1.0f) return 1.0f;
  return t * t * (3.0f - 2.0f * t);
}

static inline int32_t degToSteps(float deg, float gear) {
  return (int32_t)lroundf(deg * (STEPS_PER_REV * MICROSTEPS * gear) / 360.0f);
}

static inline float stepsToDeg(int32_t steps, float gear) {
  return (float)steps * 360.0f / (STEPS_PER_REV * MICROSTEPS * gear);
}

// Wrap-safe sequence comparison.  PROTOCOL.md section 3.1.
static inline bool seqIsNewer(uint16_t s, uint16_t last) {
  return (uint16_t)(s - last) < 32768u;
}

static void writeServo(uint8_t axis, float deg) {
  float t = deg / 180.0f;
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  uint16_t us = (uint16_t)(SERVO_MIN_US + t * (SERVO_MAX_US - SERVO_MIN_US));
  pca.writeMicroseconds(axis, us);   // axis index == PCA channel
}

// ============================================================================
// UDP  (PROTOCOL.md section 3)
// Format: ARA,<seq>,<a0>..<a11>\n
// Partial application is forbidden -- parse fully into a scratch buffer, then
// commit, or drop the whole packet.
// ============================================================================

static void handlePacket(AsyncUDPPacket &pkt) {
  static char buf[192];
  size_t len = pkt.length();
  if (len == 0 || len >= sizeof(buf)) { malformedCount++; return; }
  memcpy(buf, pkt.data(), len);
  buf[len] = '\0';

  char *save = nullptr;
  char *tok = strtok_r(buf, ",", &save);
  if (!tok || strncmp(tok, "ARA", 3) != 0) { malformedCount++; return; }

  tok = strtok_r(nullptr, ",", &save);
  if (!tok) { malformedCount++; return; }
  uint16_t seq = (uint16_t)strtoul(tok, nullptr, 10);

  float scratch[CV_AXIS_COUNT];
  for (uint8_t i = 0; i < CV_AXIS_COUNT; i++) {
    tok = strtok_r(nullptr, ",\r\n", &save);
    if (!tok) { malformedCount++; return; }
    char *end = nullptr;
    float v = strtof(tok, &end);
    if (end == tok) { malformedCount++; return; }   // non-numeric
    scratch[i] = v;
  }

  if (seqValid && !seqIsNewer(seq, lastSeq)) { staleDropCount++; return; }

  lastSeq = seq;
  seqValid = true;
  lastPacketMs = millis();
  rxCount++;

  for (uint8_t i = 0; i < CV_AXIS_COUNT; i++) cvPending[i] = scratch[i];
  cvHasNew = true;
}

// ============================================================================
// MODE TRANSITIONS  (PROTOCOL.md section 5)
// ============================================================================

static void enterCV() {
  if (mode == MODE_CV) return;
  // Axis 12 is not carried by the stream -- return it to boot zero and park.
  target[AX_ELBOW_EXT] = 0.0f;
  if (stepExt) stepExt->moveTo(0);
  switching = true;

  // Axes 0..11 hold until the first packet, then ramp.
  for (uint8_t i = 0; i < CV_AXIS_COUNT; i++) rampFrom[i] = target[i];
  rampActive = false;      // armed on first packet, not now
  seqValid = false;        // a new session restarts sequence tracking
  cvHasNew = false;
  mode = MODE_CV;
}

static void enterManual() {
  if (mode == MODE_MANUAL) return;
  // Targets are inherited from where the arm actually is.  The portal
  // repopulates its sliders from /state; it must not push its own values.
  rampActive = false;
  mode = MODE_MANUAL;
}

static void applyPreset(const Preset &p) {
  for (uint8_t i = 0; i < AXIS_COUNT; i++) target[i] = clampAxis(i, p.v[i]);
  rampActive = false;   // preset recall retargets, it does not queue
}

// ============================================================================
// CONTROL TICK
// ============================================================================

static void controlTick() {
  uint32_t now = millis();

  if (mode == MODE_CV) {
    if (cvHasNew) {
      cvHasNew = false;
      if (!rampActive && rxCount <= 1) {
        // First packet of this CV session -- arm the entry ramp.
        for (uint8_t i = 0; i < CV_AXIS_COUNT; i++) rampFrom[i] = target[i];
        rampActive = true;
        rampStartMs = now;
      }
      for (uint8_t i = 0; i < CV_AXIS_COUNT; i++) {
        target[i] = clampAxis(i, cvPending[i]);
      }
    }
    // Watchdog: hold last values.  Never zero, never open.  A hand full of
    // MG996Rs snapping to a limit is how tendons and printed parts break.
    // Nothing to do here -- holding IS doing nothing.  This branch exists so
    // the behavior is visible in the source rather than implied by absence.
  }

  // Compose the output frame, applying the entry ramp if active.
  float out[AXIS_COUNT];
  float k = 1.0f;
  if (rampActive) {
    k = smoothstep((float)(now - rampStartMs) / (float)RAMP_MS);
    if (k >= 1.0f) rampActive = false;
  }
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    if (rampActive && i < CV_AXIS_COUNT) {
      out[i] = rampFrom[i] + (target[i] - rampFrom[i]) * k;
    } else {
      out[i] = target[i];
    }
    out[i] = clampAxis(i, out[i]);
  }

  // Servos 0..10
  for (uint8_t i = 0; i < SERVO_COUNT; i++) writeServo(i, out[i]);

  // Steppers.  moveTo is idempotent, so calling every tick is fine.
  if (stepRoll) stepRoll->moveTo(degToSteps(out[AX_ELBOW_ROLL], GEAR_ROLL));
  if (stepExt)  stepExt->moveTo(degToSteps(out[AX_ELBOW_EXT],  GEAR_EXT));

  if (switching && stepExt && !stepExt->isRunning()) switching = false;
}

// ============================================================================
// PORTAL  (PROTOCOL.md section 8)
// NOTE: this uses query parameters rather than JSON bodies -- simpler on the
// firmware side and trivially testable with curl.  PROTOCOL.md section 8 must
// be updated to match.
// ============================================================================

static String stateJson() {
  uint32_t now = millis();
  bool stale = (mode == MODE_CV) && (now - lastPacketMs > WATCHDOG_MS);

  String s = "{\"mode\":\"";
  s += (mode == MODE_CV ? "CV" : "MANUAL");
  s += "\",\"stale\":";       s += stale ? "true" : "false";
  s += ",\"switching\":";     s += switching ? "true" : "false";
  s += ",\"ramping\":";       s += rampActive ? "true" : "false";
  s += ",\"rx\":";            s += rxCount;
  s += ",\"malformed\":";     s += malformedCount;
  s += ",\"stale_drops\":";   s += staleDropCount;
  s += ",\"roll_running\":";  s += (stepRoll && stepRoll->isRunning()) ? "true" : "false";
  s += ",\"ext_running\":";   s += (stepExt  && stepExt->isRunning())  ? "true" : "false";
  s += ",\"axes\":[";
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    if (i) s += ",";
    s += String(target[i], 1);
  }
  s += "],\"names\":[";
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    if (i) s += ",";
    s += "\""; s += AXIS_NAME[i]; s += "\"";
  }
  s += "],\"presets\":[";
  for (uint8_t i = 0; i < PRESET_COUNT; i++) {
    if (i) s += ",";
    s += "\""; s += PRESETS[i].name; s += "\"";
  }
  s += "]}";
  return s;
}

static void setupRoutes() {
  server.on("/state", HTTP_GET, [](AsyncWebServerRequest *r) {
    r->send(200, "application/json", stateJson());
  });

  // POST /axis?idx=5&value=95      (manual mode only)
  server.on("/axis", HTTP_POST, [](AsyncWebServerRequest *r) {
    if (mode != MODE_MANUAL) { r->send(409, "text/plain", "not in MANUAL"); return; }
    if (!r->hasParam("idx", true) || !r->hasParam("value", true)) {
      r->send(400, "text/plain", "need idx and value"); return;
    }
    int idx = r->getParam("idx", true)->value().toInt();
    float v = r->getParam("value", true)->value().toFloat();
    if (idx < 0 || idx >= AXIS_COUNT) { r->send(400, "text/plain", "bad idx"); return; }
    target[idx] = clampAxis((uint8_t)idx, v);
    r->send(200, "application/json", stateJson());
  });

  // POST /preset?name=fist
  server.on("/preset", HTTP_POST, [](AsyncWebServerRequest *r) {
    if (mode != MODE_MANUAL) { r->send(409, "text/plain", "not in MANUAL"); return; }
    if (!r->hasParam("name", true)) { r->send(400, "text/plain", "need name"); return; }
    String n = r->getParam("name", true)->value();
    for (uint8_t i = 0; i < PRESET_COUNT; i++) {
      if (n == PRESETS[i].name) {
        applyPreset(PRESETS[i]);
        r->send(200, "application/json", stateJson());
        return;
      }
    }
    r->send(404, "text/plain", "no such preset");
  });

  // POST /mode?mode=CV
  server.on("/mode", HTTP_POST, [](AsyncWebServerRequest *r) {
    if (!r->hasParam("mode", true)) { r->send(400, "text/plain", "need mode"); return; }
    String m = r->getParam("mode", true)->value();
    if      (m == "CV")     enterCV();
    else if (m == "MANUAL") enterManual();
    else { r->send(400, "text/plain", "bad mode"); return; }
    r->send(200, "application/json", stateJson());
  });

  // POST /zero?axis=11   -- declare current physical position to be zero.
  // Drift on an open-loop stepper is silent and cumulative; this is the
  // recovery path and it will get used constantly.
  server.on("/zero", HTTP_POST, [](AsyncWebServerRequest *r) {
    if (!r->hasParam("axis", true)) { r->send(400, "text/plain", "need axis"); return; }
    int a = r->getParam("axis", true)->value().toInt();
    if (a == AX_ELBOW_ROLL && stepRoll) {
      stepRoll->forceStopAndNewPosition(0); target[AX_ELBOW_ROLL] = 0.0f;
    } else if (a == AX_ELBOW_EXT && stepExt) {
      stepExt->forceStopAndNewPosition(0);  target[AX_ELBOW_EXT] = 0.0f;
    } else {
      r->send(400, "text/plain", "axis is not a stepper"); return;
    }
    r->send(200, "application/json", stateJson());
  });

  server.onNotFound([](AsyncWebServerRequest *r) {
    r->send(200, "text/plain", "ARA. See /state");
  });
}

// ============================================================================
// SETUP / LOOP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nARA firmware, PROTOCOL v0.1");

  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    target[i]   = AXIS_NEUTRAL[i];
    rampFrom[i] = AXIS_NEUTRAL[i];
  }

  Wire.begin(PIN_SDA, PIN_SCL);
  pca.begin();
  pca.setOscillatorFrequency(27000000);
  pca.setPWMFreq(PCA_FREQ_HZ);

  engine.init();
  stepRoll = engine.stepperConnectToPin(PIN_ROLL_STEP);
  if (stepRoll) {
    stepRoll->setDirectionPin(PIN_ROLL_DIR);
    stepRoll->setSpeedInHz(ROLL_SPEED_HZ);
    stepRoll->setAcceleration(ROLL_ACCEL);
    stepRoll->setCurrentPosition(0);      // boot assumption: mechanical neutral
  } else Serial.println("WARN: roll stepper failed to attach");

  stepExt = engine.stepperConnectToPin(PIN_EXT_STEP);
  if (stepExt) {
    stepExt->setDirectionPin(PIN_EXT_DIR);
    stepExt->setSpeedInHz(EXT_SPEED_HZ);
    stepExt->setAcceleration(EXT_ACCEL);
    stepExt->setCurrentPosition(0);
  } else Serial.println("WARN: ext stepper failed to attach");

  WiFi.mode(WIFI_AP);
  if (strlen(AP_PASS) == 0) WiFi.softAP(AP_SSID);
  else                      WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP up at "); Serial.println(WiFi.softAPIP());

  if (udp.listen(UDP_PORT)) {
    udp.onPacket(handlePacket);
    Serial.printf("UDP listening on %u\n", UDP_PORT);
  } else Serial.println("WARN: UDP listen failed");

  setupRoutes();
  server.begin();
  Serial.println("HTTP up on :80");
}

void loop() {
  static uint32_t nextTick = 0;
  uint32_t now = millis();
  if ((int32_t)(now - nextTick) >= 0) {
    nextTick = now + CONTROL_PERIOD_MS;
    controlTick();
  }
}