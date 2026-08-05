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
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>

// Optional station credentials for development, so you can reach the board
// without leaving your normal network. Create firmware/include/wifi_secrets.h
// from wifi_secrets.example.h -- it is gitignored and never committed.
#if __has_include("wifi_secrets.h")
  #include "wifi_secrets.h"
#endif
#ifndef STA_SSID
  #define STA_SSID ""
  #define STA_PASS ""
#endif

#include "portal_html.h"
#include <Adafruit_PWMServoDriver.h>
#include <FastAccelStepper.h>

// ============================================================================
// CONFIG
// ============================================================================

// --- Network ---
static const char *AP_SSID = "ARA-HAND";
static const char *AP_PASS = "";          // open network; see README before changing
static const uint16_t UDP_PORT = 4210;

// Dual mode. The softAP is always up -- that is what the arm ships with and what
// the Pi joins in the field. If STA credentials exist, the board ALSO joins that
// network, so during development your laptop keeps its internet and you can
// reach the board at http://ara.local without switching WiFi.
//
// Caveat worth knowing: an ESP32 cannot run AP and STA on different channels.
// Joining your router moves the softAP to the router's channel. Harmless, but it
// means the AP briefly drops when STA connects.
static const char *MDNS_HOST = "ara";
static const uint32_t STA_TIMEOUT_MS = 8000;

// --- I2C / PCA9685 ---
static const uint8_t PIN_SDA = 21;        // TBD confirm
static const uint8_t PIN_SCL = 22;        // TBD confirm
static const uint8_t PCA_ADDR = 0x40;
static const float   PCA_FREQ_HZ = 50.0f;

// --- Stepper pins -- SUPERSEDED 2026-08-04 ---
// Repo originally assumed two identical TMC2209s sharing one pin/geometry
// config and EN tied to GND. As-built, the two axes use different driver
// ICs on different pins, each with its own active-low EN line. Kept for
// history; not read by any code below. See StepperAxisConfig / EXT_CFG /
// ROLL_CFG for the values actually in effect.
static const uint8_t PIN_ROLL_STEP = 26;
static const uint8_t PIN_ROLL_DIR  = 25;
static const uint8_t PIN_EXT_STEP  = 33;
static const uint8_t PIN_EXT_DIR   = 32;
// EN is tied to GND on both drivers -- no enable pin in software.

// --- Stepper geometry -- SUPERSEDED 2026-08-04 ---
// MICROSTEPS was a shared TBD guess (8) applied to both axes via one
// gear-ratio parameter. As-built, both drivers turned out to be strapped
// for 1/16 (a coincidence, not a shared config), but they're on different
// pins with different EN lines, so each axis now carries its own full
// config rather than sharing these globals. Kept for history; not read by
// any code below.
static const float STEPS_PER_REV = 200.0f;   // 1.8 deg NEMA 17
static const float MICROSTEPS    = 8.0f;     // TBD confirm MS1/MS2 straps
static const float GEAR_ROLL     = 4.0f;     // pron/sup planetary
static const float GEAR_EXT      = 64.0f;    // elbow planetary

// --- Motion limits -- SUPERSEDED 2026-08-04 ---
// Folded into speedHz/accel in EXT_CFG/ROLL_CFG below (same values carried
// over). Kept for history; not read by any code below.
// The 64:1 axis is slow by construction -- do not chase it with more
// acceleration, a printed planetary will skip and lose the zero.
static const uint32_t ROLL_SPEED_HZ = 4000;
static const uint32_t ROLL_ACCEL    = 8000;
static const uint32_t EXT_SPEED_HZ  = 6000;
static const uint32_t EXT_ACCEL     = 6000;

// --- Stepper driver config (as-built 2026-08-04) ---
// Two DIFFERENT driver ICs -- not two of the same board:
//   ext  (elbow extension/retraction, 64:1 planetary) -- Adafruit TMC2209
//        breakout (PID 6121), standalone/no UART, MS1+MS2 tied to VDD =>
//        1/16 microstepping. PDN/UART left floating. VM = 12 V (board max
//        29 V).
//   roll (elbow pronation/supination, 4:1 planetary) -- Adafruit A4988
//        breakout (PID 6109), MS1/MS2/MS3 left open => 1/16 microstepping
//        (this board's default). RESET jumpered to SLEEP. VMOT = 12 V
//        (board max 35 V).
// Both motors: StepperOnline 17HS19-2004S1 NEMA 17, 1.8 deg/step (200 full
// steps/rev), 2.0 A/phase rated -- but both drivers' onboard trimpots are
// set to ~1.2 A, the driver IC's thermal limit, not the motor's. Both EN
// lines are active-low. 3200 microsteps/rev on both axes.
struct StepperAxisConfig {
  uint8_t  stepPin;
  uint8_t  dirPin;
  uint8_t  enPin;             // active low on both drivers
  bool     invertDir;         // flip if the motor spins the wrong way
  float    fullStepsPerRev;   // motor full steps/rev (1.8 deg -> 200)
  float    microsteps;        // driver's microstep divisor
  float    gearRatio;         // planetary reduction
  uint32_t speedHz;
  uint32_t accel;
};

static constexpr StepperAxisConfig EXT_CFG = {
  26, 25, 27,                 // step, dir, en -- unchanged from original pinout
  false,                      // invertDir
  200.0f, 16.0f, 64.0f,       // fullStepsPerRev, microsteps, gearRatio
  6000, 6000,                 // speedHz, accel
};

static constexpr StepperAxisConfig ROLL_CFG = {
  32, 33, 14,                 // step, dir, en
  false,                      // invertDir
  200.0f, 16.0f, 4.0f,        // fullStepsPerRev, microsteps, gearRatio
  4000, 8000,                 // speedHz, accel
};

static constexpr float axisStepsPerDeg(const StepperAxisConfig &c) {
  return (c.fullStepsPerRev * c.microsteps * c.gearRatio) / 360.0f;
}
// ext:  (200 * 16 * 64) / 360 = 568.888... steps/deg
// roll: (200 * 16 *  4) / 360 =  35.555... steps/deg
static constexpr float EXT_STEPS_PER_DEG  = axisStepsPerDeg(EXT_CFG);
static constexpr float ROLL_STEPS_PER_DEG = axisStepsPerDeg(ROLL_CFG);

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
static const uint8_t SERVO_COUNT    = 11;  // 11 servos, spread over PCA ch 0..15

// Axis index -> PCA9685 channel. Not 1:1: the finger+wrist bank sits on the
// LAST 6 channels (10-15) and the splay+opposition bank on the FIRST 5
// (0-4), leaving ch 5-9 as a spare gap in the middle. If a servo moves to a
// different channel, update this table -- it is the only place that knows
// the mapping.
static const uint8_t AXIS_TO_PCA[SERVO_COUNT] = {
  10, 11, 12, 13, 14, // thumb/index/middle/ring/pinky flex
  15,                 // wrist
  0, 1, 2, 3,         // index/middle/ring/pinky splay
  4                   // thumb opposition
};

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

// Bench jog: continuous run at an operator-chosen speed/direction, for
// verifying wiring/microstepping/gear direction before trusting normal
// position control. While active for an axis, controlTick() skips that
// axis's moveTo() entirely -- see JOG_SPEED_MIN_HZ/MAX_HZ below. There is
// NO position limit while jogging, only the operator's stop button.
static bool jogRollActive = false;
static bool jogExtActive  = false;
static const uint32_t JOG_SPEED_MIN_HZ = 100;
static const uint32_t JOG_SPEED_MAX_HZ = 8000;

Adafruit_PWMServoDriver pca(PCA_ADDR);
static bool pcaPresent = false;   // probed at boot; see setup()
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

// SUPERSEDED 2026-08-04: took a shared gear ratio and read the shared
// STEPS_PER_REV/MICROSTEPS globals above, which assumed both axes ran the
// same microstepping. Kept for history; not called below. See
// degToStepsAxis(), which takes a precomputed per-axis steps-per-degree
// instead (EXT_STEPS_PER_DEG / ROLL_STEPS_PER_DEG).
static inline int32_t degToSteps(float deg, float gear) {
  return (int32_t)lroundf(deg * (STEPS_PER_REV * MICROSTEPS * gear) / 360.0f);
}

static inline float stepsToDeg(int32_t steps, float gear) {
  return (float)steps * 360.0f / (STEPS_PER_REV * MICROSTEPS * gear);
}

static inline int32_t degToStepsAxis(float deg, float stepsPerDeg) {
  return (int32_t)lroundf(deg * stepsPerDeg);
}

static inline float stepsToDegAxis(int32_t steps, float stepsPerDeg) {
  return (float)steps / stepsPerDeg;
}

// Attaches one stepper axis from its StepperAxisConfig. Both drivers are
// wired EN-active-low; enableOutputs() is called once here and left on,
// matching the old EN-tied-to-GND behavior rather than switching to
// FastAccelStepper's auto-enable/disable.
static FastAccelStepper *attachAxis(const StepperAxisConfig &cfg, const char *label) {
  FastAccelStepper *s = engine.stepperConnectToPin(cfg.stepPin);
  if (!s) {
    Serial.printf("WARN: %s stepper failed to attach\n", label);
    return nullptr;
  }
  s->setDirectionPin(cfg.dirPin, !cfg.invertDir);
  s->setEnablePin(cfg.enPin, true);   // true = low_active_enables_stepper
  s->setAutoEnable(false);
  s->enableOutputs();
  s->setSpeedInHz(cfg.speedHz);
  s->setAcceleration(cfg.accel);
  s->setCurrentPosition(0);           // boot assumption: mechanical neutral
  return s;
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
  pca.writeMicroseconds(AXIS_TO_PCA[axis], us);
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
  // A bench jog fighting the CV stream's control of roll (or the
  // boot-neutral return on ext, below) is not a state worth supporting --
  // abort outright.
  if (jogRollActive && stepRoll) { stepRoll->forceStop(); jogRollActive = false; }
  if (jogExtActive  && stepExt)  { stepExt->forceStop();  jogExtActive  = false; }
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
  if (pcaPresent) {
    for (uint8_t i = 0; i < SERVO_COUNT; i++) writeServo(i, out[i]);
  }

  // Steppers.  moveTo is idempotent, so calling every tick is fine.
  // While a bench jog is active for an axis, skip its moveTo() entirely --
  // otherwise this would fight runForward()/runBackward() every 20ms. Once
  // a stopped jog actually settles (isRunning() goes false), resync
  // target[] to wherever it landed (clamped, same invariant as every other
  // write to target[]) so normal control resumes from there -- if the jog
  // went past the normal range, this schedules an ordinary accelerated
  // moveTo() back inside it on the next tick, not a snap. See /jog.
  if (stepRoll) {
    if (jogRollActive) {
      if (!stepRoll->isRunning()) {
        target[AX_ELBOW_ROLL] = clampAxis(AX_ELBOW_ROLL,
            stepsToDegAxis(stepRoll->getCurrentPosition(), ROLL_STEPS_PER_DEG));
        jogRollActive = false;
      }
    } else {
      stepRoll->moveTo(degToStepsAxis(out[AX_ELBOW_ROLL], ROLL_STEPS_PER_DEG));
    }
  }
  if (stepExt) {
    if (jogExtActive) {
      if (!stepExt->isRunning()) {
        target[AX_ELBOW_EXT] = clampAxis(AX_ELBOW_EXT,
            stepsToDegAxis(stepExt->getCurrentPosition(), EXT_STEPS_PER_DEG));
        jogExtActive = false;
      }
    } else {
      stepExt->moveTo(degToStepsAxis(out[AX_ELBOW_EXT], EXT_STEPS_PER_DEG));
    }
  }

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
  s += ",\"pca\":";           s += pcaPresent ? "true" : "false";
  s += ",\"sta_ip\":\"";
  s += (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : String("");
  s += "\"";
  s += ",\"ramping\":";       s += rampActive ? "true" : "false";
  s += ",\"rx\":";            s += rxCount;
  s += ",\"malformed\":";     s += malformedCount;
  s += ",\"stale_drops\":";   s += staleDropCount;
  s += ",\"roll_running\":";  s += (stepRoll && stepRoll->isRunning()) ? "true" : "false";
  s += ",\"ext_running\":";   s += (stepExt  && stepExt->isRunning())  ? "true" : "false";
  s += ",\"roll_jogging\":";  s += jogRollActive ? "true" : "false";
  s += ",\"ext_jogging\":";   s += jogExtActive  ? "true" : "false";
  s += ",\"axes\":[";
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    if (i) s += ",";
    s += String(target[i], 1);
  }
  s += "],\"min\":[";
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    if (i) s += ",";
    s += String(AXIS_MIN[i], 0);
  }
  s += "],\"max\":[";
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    if (i) s += ",";
    s += String(AXIS_MAX[i], 0);
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
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r) {
    r->send_P(200, "text/html", PORTAL_HTML);
  });

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

  // POST /jog?axis=11&dir=1&speed=2000  -- bench-only: spin continuously.
  // POST /jog?axis=11&stop=1            -- decelerate; controlTick() hands
  //                                         the axis back to normal control
  //                                         once it actually stops.
  // MANUAL mode only, stepper axes only. Bypasses AXIS_MIN/AXIS_MAX -- this
  // is deliberately a raw hardware test, not a position command. Speed is
  // clamped to JOG_SPEED_MIN_HZ..JOG_SPEED_MAX_HZ regardless of what's sent.
  server.on("/jog", HTTP_POST, [](AsyncWebServerRequest *r) {
    if (mode != MODE_MANUAL) { r->send(409, "text/plain", "not in MANUAL"); return; }
    if (!r->hasParam("axis", true)) { r->send(400, "text/plain", "need axis"); return; }
    int a = r->getParam("axis", true)->value().toInt();

    FastAccelStepper *s = nullptr;
    bool *active = nullptr;
    if (a == AX_ELBOW_ROLL)      { s = stepRoll; active = &jogRollActive; }
    else if (a == AX_ELBOW_EXT)  { s = stepExt;  active = &jogExtActive;  }
    if (!s || !active) { r->send(400, "text/plain", "axis is not a stepper"); return; }

    if (r->hasParam("stop", true)) {
      s->stopMove();   // normal deceleration -- see controlTick() for resync
      r->send(200, "application/json", stateJson());
      return;
    }

    if (!r->hasParam("dir", true) || !r->hasParam("speed", true)) {
      r->send(400, "text/plain", "need dir and speed"); return;
    }
    int dir = r->getParam("dir", true)->value().toInt();
    uint32_t speed = (uint32_t)r->getParam("speed", true)->value().toInt();
    if (speed < JOG_SPEED_MIN_HZ) speed = JOG_SPEED_MIN_HZ;
    if (speed > JOG_SPEED_MAX_HZ) speed = JOG_SPEED_MAX_HZ;

    s->setSpeedInHz(speed);
    *active = true;
    if (dir >= 0) s->runForward(); else s->runBackward();
    r->send(200, "application/json", stateJson());
  });

  server.onNotFound([](AsyncWebServerRequest *r) {
    r->redirect("/");
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
  // Probe before trusting the board is there. Without this, writeServo() fails
  // silently 550 times a second while the firmware reports itself healthy --
  // and the I2C error spam buries every other line on the console.
  Wire.beginTransmission(PCA_ADDR);
  pcaPresent = (Wire.endTransmission() == 0);
  if (pcaPresent) {
    pca.begin();
    pca.setOscillatorFrequency(27000000);
    pca.setPWMFreq(PCA_FREQ_HZ);
    Serial.println("PCA9685 found at 0x40");
  } else {
    Serial.println("WARN: no PCA9685 at 0x40 -- servo output disabled");
  }

  engine.init();
  stepRoll = attachAxis(ROLL_CFG, "roll");
  stepExt  = attachAxis(EXT_CFG,  "ext");

  const bool wantSta = (strlen(STA_SSID) > 0);
  WiFi.mode(wantSta ? WIFI_AP_STA : WIFI_AP);

  if (strlen(AP_PASS) == 0) WiFi.softAP(AP_SSID);
  else                      WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP up at "); Serial.println(WiFi.softAPIP());

  if (wantSta) {
    Serial.printf("joining %s ", STA_SSID);
    WiFi.begin(STA_SSID, STA_PASS);
    uint32_t t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < STA_TIMEOUT_MS) {
      delay(250);
      Serial.print(".");
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("STA up at "); Serial.println(WiFi.localIP());
    } else {
      // Not fatal. The arm runs on its own AP; STA is a convenience.
      Serial.println("STA failed -- continuing on softAP only");
    }
  }

  if (MDNS.begin(MDNS_HOST)) {
    MDNS.addService("http", "tcp", 80);
    Serial.printf("mDNS: http://%s.local\n", MDNS_HOST);
  }

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