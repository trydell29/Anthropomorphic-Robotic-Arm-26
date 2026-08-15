# ARA Control Protocol

**Version:** 0.2
**Scope:** the ESP32 firmware's portal — the only interface to the arm.

This document is the single source of truth for anything the firmware and portal
must agree on. If `main.cpp`/`portal_html.h` and this file disagree, this file
decides which one is wrong.

---

## 1. System overview

```
  browser ──── HTTP ────► ESP32 ──► PCA9685 ──► 11 servos
                            │
                            ├──► A4988   ──► elbow roll stepper (4:1)
                            └──► TMC2209 ──► elbow extension stepper (64:1)

  captive portal, softAP "ARA-HAND", ESP32 at 192.168.4.1
```

The ESP32 hosts a standalone manual controller: a captive-portal web UI with one
slider per axis and a per-stepper zero (calibration) button. It owns the
authoritative target array at all times. Nothing but the portal ever commands
an actuator.

---

## 2. Axis table

There is one canonical axis index used everywhere — in the portal and in
firmware. Never reorder this table.

| Idx | Axis | Actuator | Output | Unit | Neutral | Convention |
|----|------|----------|--------|------|---------|------------|
| 0 | thumb flexion | MG996R | PCA ch 6 | 0–180 | 180 | 180 = open, 0 = closed |
| 1 | index flexion | MG996R | PCA ch 7 | 0–180 | 180 | 180 = open, 0 = closed |
| 2 | middle flexion | MG996R | PCA ch 8 | 0–180 | 180 | 180 = open, 0 = closed |
| 3 | ring flexion | MG996R | PCA ch 9 | 0–180 | 180 | 180 = open, 0 = closed |
| 4 | pinky flexion | MG996R | PCA ch 10 | 0–180 | 180 | 180 = open, 0 = closed |
| 5 | wrist flex/ext | AGFRC 15.5kg | PCA ch 5 | 0–180 | 90 | >90 = flexion (palm-ward), <90 = extension |
| 6 | index splay | SG90 | PCA ch 1 | 70–110 | 90 | >90 = abduction (away from middle) |
| 7 | middle splay | SG90 | PCA ch 2 | 70–110 | 90 | >90 = abduction |
| 8 | ring splay | SG90 | PCA ch 3 | 70–110 | 90 | >90 = abduction |
| 9 | pinky splay | SG90 | PCA ch 4 | 70–110 | 90 | >90 = abduction |
| 10 | thumb opposition | SG90 | PCA ch 0 | 0–180 | 90 | >90 = toward pinky |
| 11 | elbow roll (pron/sup) | NEMA 17, 4:1 | A4988 | signed deg | 0 | + = pronation, − = supination |
| 12 | elbow extension | NEMA 17, 64:1 | TMC2209 | signed deg | 0 | + = extension, − = retraction |

**Axis index does not equal PCA channel.** Wiring order on the board, ch 0–10:
thumb opposition, index/middle/ring/pinky splay, wrist, thumb/index/middle/
ring/pinky flex. Ch 11–15 are unused. Bench-confirmed 2026-08-09 -- see
`AXIS_TO_PCA` in `firmware/src/main.cpp`, the one place that knows the
axis-index-to-channel mapping. If a servo moves to a different channel,
update that table.

### 2.1 Power architecture (as-built, confirmed 2026-08-09)

A single 4S LiPo (14.8 V nominal) powers the whole arm. It is **not** wired
directly to the servos or logic:

- **Steppers** run off the 4S rail directly -- both driver boards are rated
  well above it (29 V / 35 V max, see 2.3).
- **Servos** are fed through a buck converter to a terminal block, stepped
  down to the servos' actual rated voltage, before reaching the PCA9685's V+
  pins. The PCA9685's signal pins carry PWM only, no power.
- **ESP32 logic** gets its own properly regulated supply, separate from the
  above.

### 2.2 Per-axis limits

Clamped in firmware, applied last, after every other stage. No source can
exceed these.

| Idx | Min | Max | Notes |
|----|-----|-----|-------|
| 0–4 | 0 | 180 | full sweep by choice 2026-08-09; tendon over-pull risk at the closed end not yet bench-verified -- watch the first slow close |
| 5 | 45 | 135 | bench-confirmed 2026-08-09 |
| 6–9 | see axis table | see axis table | 90 ± 20, locked around neutral |
| 10 | 0 | 160 | bench-confirmed 2026-08-09 |
| 11 | −80 | +80 | wiring twist limit — do not raise without a service-loop check |
| 12 | −90 | +90 | bench-confirmed 2026-08-09, symmetric around zero to hard stop |

The bench-test escape hatch (`/axis` with `raw=1`) constrains a servo axis to
its physical 0–180° sweep instead of the table above — see §3.

### 2.3 Stepper hardware (as-built)

The two stepper axes are driven by **different driver ICs**, not two of the
same board. Both motors are StepperOnline 17HS19-2004S1 NEMA 17 (1.8°/step,
200 full steps/rev, 2.0 A/phase rated, coils A+ black / A− green / B+ red /
B− blue). Both drivers are current-limited to **~1.2 A** via their onboard
trimpots — the driver IC's thermal limit, not the motor's — and both run at
**1/16 microstepping** (3200 microsteps/rev), so it is a coincidence that the
two numbers match, not a shared config. Both `VM`/`VMOT` rails are 12 V.

| Axis | Driver | STEP | DIR | EN | Microstep config | Board max V |
|------|--------|------|-----|----|--------------------|-------------|
| elbow extension (idx 12) | Adafruit TMC2209 (PID 6121), standalone, PDN/UART floating | GPIO 26 | GPIO 25 | GPIO 27 | MS1+MS2 → VDD ⇒ 1/16 | 29 V |
| elbow roll (idx 11) | Adafruit A4988 (PID 6109), RESET→SLEEP jumpered | GPIO 32 | GPIO 33 | GPIO 14 | MS1/MS2/MS3 open ⇒ 1/16 (default) | 35 V |

Both EN lines are **active-low**; firmware drives them low at boot and leaves
them enabled, matching the earlier EN-tied-to-GND assumption. See
`StepperAxisConfig` / `EXT_CFG` / `ROLL_CFG` in `firmware/src/main.cpp` for
the single source of truth — including a per-axis `invertDir` flag for the
case where a motor spins the wrong way (the two breakouts number their coil
outputs differently).

---

## 3. Portal endpoints

`/state` is what the portal polls (every 600 ms) to repopulate sliders.

| Method | Path | Purpose |
|--------|------|---------|
| GET | `/state` | Current target array, servo-board presence, stepper motion status |
| POST | `/axis` | Set one axis. Form body: `idx=5&value=95`. Add `raw=1` on a servo axis to bypass its `AXIS_MIN`/`AXIS_MAX` and constrain to 0–180 instead. |
| POST | `/zero` | `axis=11` — declare current physical position as zero |

`/state` response shape:

```json
{
  "pca": true,
  "sta_ip": "",
  "roll_running": false,
  "ext_running": false,
  "axes":  [180, 180, ..., 0, 0],
  "min":   [0, 0, ..., -80, -90],
  "max":   [180, 180, ..., 80, 90],
  "names": ["thumb_flex", "index_flex", ..., "elbow_roll", "elbow_ext"],
  "pca_ch": [6, 7, ..., -1, -1]
}
```

Query parameters, not JSON bodies -- simpler on the firmware side and
trivially testable with curl:

```
curl -X POST 'http://ara.local/axis' -d 'idx=5&value=95'
curl -X POST 'http://ara.local/zero' -d 'axis=11'
```

---

## 4. Stepper position reference

Both steppers are open-loop with no encoder. The ESP32 tracks **commanded**
step count from boot, which is not the same as actual position.

**Boot assumption:** both steppers are at mechanical neutral at power-on. The
operator is responsible for this.

**Drift is silent and cumulative.** One skipped step offsets every absolute
move afterward, with no indication anywhere in the system.

Required mitigations:

- **Re-zero button, per stepper, on the portal.** Declares the current
  physical position to be zero and resets the counter.
- **Re-zero before every measurement session.** Not optional for GRASP or
  payload trials; without it, "the same posture" is not the same posture
  across sessions.
- The 64:1 axis should get a hard-stop homing routine when time allows. At
  that reduction, output error is too large a step count to recover by eye.

### 4.1 Servo position reference

Servos (MG996R, SG90, AGFRC) have **no position feedback at all** — no
encoder, no telemetry line — and the PCA9685 is pure PWM output. Firmware
cannot know where a servo physically is, at boot or ever, so it does not
guess.

**A servo channel is only driven once its axis is explicitly commanded** — a
portal slider move. Until then, no PWM is sent to that channel at all; it
holds no position because nothing is telling it to. This means, unlike the
steppers, there is no boot-neutral assumption for servos: `AXIS_NEUTRAL` is
what a slider *shows* on first load, not what gets sent.

---

## 5. Open items

- [x] All axis limits, from bench testing -- see §2.2
- [x] Confirm the PCA channel map -- bench-confirmed 2026-08-09
- [ ] Hard-stop homing routine for axis 12

---

## Revision history

| Version | Date | Change |
|---------|------|--------|
| 0.1 | 2026-08-02 | Initial draft |
| 0.1 | 2026-08-04 | Stepper hardware corrected: elbow roll is an A4988 (was assumed TMC2209), elbow extension is a TMC2209 on unchanged pins; both confirmed 1/16 microstepping. |
| 0.1 | 2026-08-04 | PCA channel map reworked: finger+wrist bank moved to the last 6 channels (10–15), splay+opposition bank to the first 5 (0–4), leaving ch 5–9 as a spare gap. |
| 0.1 | 2026-08-05 | Reverted the 2026-08-04 bench-wiring `AXIS_TO_PCA`/`AXIS_WIRED` override (back to the planned last-6/first-5 layout). |
| 0.1 | 2026-08-05 | Servos are no longer driven to `AXIS_NEUTRAL` at boot (§4.1) — a channel gets no PWM until explicitly commanded. Added `/axis` `raw=1` to bypass a servo axis's assumed limits for full 0–180 bench sweeps. |
| 0.2 | 2026-08-14 | Removed the vision/CV pipeline entirely. The arm is now a standalone ESP32 portal: no UDP stream, no CV mode, no presets, no bench-jog, no limit-capture UI. Portal is sliders + per-stepper zero only. Dropped the Pi/MediaPipe vision host, its UDP protocol (old §3), grasp presets (old §4), the MANUAL/CV mode machinery (old §5), and the vision-host requirements section (old §7) from this document. |
