// ============================================================================
// ARA - Anthropomorphic Robotic Arm
// On-arm ESP32 firmware. Implements docs/PROTOCOL.md.
//
// Standalone manual controller: hosts a captive-portal web UI with one slider
// per axis and a per-stepper zero (calibration) button. Nothing else writes
// the target array.
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
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

// Dual mode. The softAP is always up -- that is what the arm ships with and what
// the operator joins in the field. If STA credentials exist, the board ALSO joins
// that network, so during development your laptop keeps its internet and you can
// reach the board at http://ara.local without switching WiFi.
//
// Caveat worth knowing: an ESP32 cannot run AP and STA on different channels.
// Joining your router moves the softAP to the router's channel. Harmless, but it
// means the AP briefly drops when STA connects.
static const char *MDNS_HOST = "ara";
static const uint32_t STA_TIMEOUT_MS = 8000;

// --- I2C / PCA9685 ---
static const uint8_t PIN_SDA = 21;        // confirmed 2026-08-09 -- board runs, PCA9685 responds
static const uint8_t PIN_SCL = 22;        // confirmed 2026-08-09 -- board runs, PCA9685 responds
static const uint8_t PCA_ADDR = 0x40;
static const float   PCA_FREQ_HZ = 50.0f;

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

static const uint8_t SERVO_COUNT = 11;  // 11 servos, spread over PCA ch 0..15

// Axis index -> PCA9685 channel. Wiring order on the board, ch 0-10:
// thumb opp, index/middle/ring/pinky splay, wrist,
// thumb/index/middle/ring/pinky flex. Bench-confirmed 2026-08-09 -- each
// axis moves only its own servo. If a servo moves to a different channel,
// update this table -- it is the only place that knows the mapping.
static const uint8_t AXIS_TO_PCA[SERVO_COUNT] = {
  6, 7, 8, 9, 10,     // thumb/index/middle/ring/pinky flex
  5,                  // wrist
  1, 2, 3, 4,         // index/middle/ring/pinky splay
  0                   // thumb opposition
};

// Per-servo output slew rate, deg/s. writeServo() otherwise jumps straight
// to target[] every tick -- fine for the fingers, but the wrist's mass/lever
// arm turns an instant commanded step into a visible snap. 0 = unlimited
// (servo's own speed is the only limit). Tune per-axis here if others need
// it too.
static const float SERVO_MAX_DEG_PER_S[SERVO_COUNT] = {
  0, 0, 0, 0, 0,      // finger flex   -- unlimited
  90,                 // wrist         -- capped, ~2s for a full 180 sweep
  0, 0, 0, 0,         // splay         -- unlimited
  0                   // thumb opp     -- unlimited
};

// Physical output direction per servo. true flips the pulse so that target[]'s
// 0=closed / 180=open convention (see AXIS_MIN/MAX comments above and
// AXIS_NEUTRAL's "fingers open" at 180) still holds at the horn even when the
// servo itself moves the opposite way. Confirmed on the bench 2026-08-11: all
// five flex servos now open at 0 and close at 180 -- reversed from every
// other assumption in this file (AXIS_NEUTRAL, PROTOCOL.md) -- so invert
// here, once, rather than flip the sign convention everywhere else.
static const bool SERVO_INVERT[SERVO_COUNT] = {
  true, true, true, true, true,    // thumb/index/middle/ring/pinky flex -- reversed
  false,                           // wrist
  false, false, false, false,      // splay
  false                            // thumb opp
};

static const char *AXIS_NAME[AXIS_COUNT] = {
  "thumb_flex", "index_flex", "middle_flex", "ring_flex", "pinky_flex",
  "wrist",
  "index_splay", "middle_splay", "ring_splay", "pinky_splay",
  "thumb_opp",
  "elbow_roll", "elbow_ext"
};

// Limits.  Applied last, after every other stage.  No source may exceed them.
// All bench-confirmed 2026-08-09 except fingers (0-4, full sweep by choice --
// tendon over-pull at full close not yet verified) and roll, which is a
// wiring-twist limit rather than a bench measurement. Splay and thumb opp
// (6-9, 10) were opened to full 0-180 on 2026-08-10 for jog testing, then
// narrowed to these bench-tested ranges the same day.
static const float AXIS_MIN[AXIS_COUNT] = {
    0,   0,   0,   0,   0,     // finger flexion
   45,                         // wrist        90 +/- 45
  160, 160,   0,   0,          // splay        index/middle 160-180, ring/pinky 0-20
    0,                         // thumb opp    0-160
  -80,                         // elbow roll   service-loop limit
  -90                          // elbow ext    bench-confirmed 2026-08-09, +/-90 from zero to hard stop
};
static const float AXIS_MAX[AXIS_COUNT] = {
  180, 180, 180, 180, 180,
  135,
  180, 180,  20,  20,
  160,
  +80,
  +90
};

static const float AXIS_NEUTRAL[AXIS_COUNT] = {
  180, 180, 180, 180, 180,     // fingers open
   90,                         // wrist neutral
  180, 170,   0,   0,          // splay start: index 180, middle 170, ring/pinky 0
    0,                         // thumb opp start: 0
    0,                         // roll zero
    0                          // ext zero
};

// ============================================================================
// STATE
// ============================================================================

static float target[AXIS_COUNT];       // authoritative, post-clamp

// Servos have no position feedback -- PCA9685 is pure output, MG996R/SG90/
// AGFRC have no telemetry line. Firmware cannot know where a servo actually
// is, so it doesn't guess: a channel is only driven once something
// explicitly commands that specific axis (a portal slider). Until then it
// gets no PWM at all, rather than snapping to AXIS_NEUTRAL at boot. Set
// wherever target[] is explicitly written for a servo axis; see /axis.
static bool axisTouched[SERVO_COUNT] = {false};

// Slew-limited output state -- see SERVO_MAX_DEG_PER_S. servoPos[i] is the
// last position actually written to the PCA; servoSlewInit[i] tracks whether
// it's been seeded yet, so the very first write for an axis still jumps
// straight to target (no prior physical position to slew from) and only
// later moves get capped.
static float servoPos[SERVO_COUNT];
static bool  servoSlewInit[SERVO_COUNT] = {false};

Adafruit_PWMServoDriver pca(PCA_ADDR);
static bool pcaPresent = false;   // probed at boot; see setup()
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepRoll = nullptr;
FastAccelStepper *stepExt  = nullptr;
AsyncWebServer server(80);

// ============================================================================
// HELPERS
// ============================================================================

static inline float clampAxis(uint8_t i, float v) {
  if (v < AXIS_MIN[i]) return AXIS_MIN[i];
  if (v > AXIS_MAX[i]) return AXIS_MAX[i];
  return v;
}

static inline int32_t degToStepsAxis(float deg, float stepsPerDeg) {
  return (int32_t)lroundf(deg * stepsPerDeg);
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

static void writeServo(uint8_t axis, float deg) {
  if (SERVO_INVERT[axis]) deg = 180.0f - deg;
  float t = deg / 180.0f;
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  uint16_t us = (uint16_t)(SERVO_MIN_US + t * (SERVO_MAX_US - SERVO_MIN_US));
  pca.writeMicroseconds(AXIS_TO_PCA[axis], us);
}

// ============================================================================
// CONTROL TICK
// ============================================================================

static void controlTick() {
  float out[AXIS_COUNT];
  for (uint8_t i = 0; i < AXIS_COUNT; i++) out[i] = clampAxis(i, target[i]);

  // Servos 0..10. Only channels that have been explicitly commanded at
  // least once are driven -- see axisTouched above.
  if (pcaPresent) {
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
      if (!axisTouched[i]) continue;
      float cmd = out[i];
      if (!servoSlewInit[i]) {
        servoPos[i] = cmd;          // first write for this axis: jump, don't slew
        servoSlewInit[i] = true;
      } else if (SERVO_MAX_DEG_PER_S[i] > 0.0f) {
        float maxStep = SERVO_MAX_DEG_PER_S[i] * (CONTROL_PERIOD_MS / 1000.0f);
        float delta = cmd - servoPos[i];
        if (delta > maxStep) cmd = servoPos[i] + maxStep;
        else if (delta < -maxStep) cmd = servoPos[i] - maxStep;
        servoPos[i] = cmd;
      } else {
        servoPos[i] = cmd;
      }
      writeServo(i, servoPos[i]);
    }
  }

  // Steppers.  moveTo is idempotent, so calling every tick is fine.
  if (stepRoll) stepRoll->moveTo(degToStepsAxis(out[AX_ELBOW_ROLL], ROLL_STEPS_PER_DEG));
  if (stepExt)  stepExt->moveTo(degToStepsAxis(out[AX_ELBOW_EXT],  EXT_STEPS_PER_DEG));
}

// ============================================================================
// PORTAL  (PROTOCOL.md section 8)
// NOTE: this uses query parameters rather than JSON bodies -- simpler on the
// firmware side and trivially testable with curl.
// ============================================================================

static String stateJson() {
  String s = "{\"pca\":";        s += pcaPresent ? "true" : "false";
  s += ",\"sta_ip\":\"";
  s += (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : String("");
  s += "\"";
  s += ",\"roll_running\":"; s += (stepRoll && stepRoll->isRunning()) ? "true" : "false";
  s += ",\"ext_running\":";  s += (stepExt  && stepExt->isRunning())  ? "true" : "false";
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
  // Physical PCA9685 channel each servo axis is wired to -- see AXIS_TO_PCA.
  // -1 for the two stepper axes (roll/ext), which aren't on the PCA at all.
  s += "],\"pca_ch\":[";
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    if (i) s += ",";
    s += (i < SERVO_COUNT) ? String(AXIS_TO_PCA[i]) : String(-1);
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

  // POST /axis?idx=5&value=95
  server.on("/axis", HTTP_POST, [](AsyncWebServerRequest *r) {
    if (!r->hasParam("idx", true) || !r->hasParam("value", true)) {
      r->send(400, "text/plain", "need idx and value"); return;
    }
    int idx = r->getParam("idx", true)->value().toInt();
    float v = r->getParam("value", true)->value().toFloat();
    if (idx < 0 || idx >= AXIS_COUNT) { r->send(400, "text/plain", "bad idx"); return; }
    // raw=1: bench-test escape hatch for servo axes only. Ignores this
    // axis's assumed AXIS_MIN/AXIS_MAX (e.g. wrist's narrower 40-140
    // range) and just constrains to the servo's physical 0-180 sweep --
    // for sorting out which physical servo landed on which logical axis
    // without fighting per-axis limits that may not even apply to what's
    // actually wired there right now.
    bool raw = r->hasParam("raw", true) && idx < SERVO_COUNT;
    target[idx] = raw ? constrain(v, 0.0f, 180.0f) : clampAxis((uint8_t)idx, v);
    if (idx < SERVO_COUNT) axisTouched[idx] = true;
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
    r->redirect("/");
  });
}

// ============================================================================
// SETUP / LOOP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nARA firmware");

  for (uint8_t i = 0; i < AXIS_COUNT; i++) target[i] = AXIS_NEUTRAL[i];

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
