# ARA Control Protocol

**Version:** 0.1 (draft — not yet implemented)
**Scope:** the interface between the Raspberry Pi vision host and the on-arm ESP32.

This document is the single source of truth for anything both sides must agree on.
If the firmware and the Python disagree, this file decides which one is wrong.

Values marked **TBD** must be filled in from bench testing before either side is
written against them.

---

## 1. System overview

```
  camera ──► Raspberry Pi 5 ──── UDP ────► ESP32 ──► PCA9685 ──► 11 servos
             MediaPipe Pose        (WiFi)    │
             MediaPipe Hand                  ├──► A4988   ──► elbow roll stepper (4:1)
             12 axis values                  ├──► TMC2209 ──► elbow extension stepper (64:1)
                                             │
             browser ──── HTTP ──────────────┘  captive portal (softAP "ARA-HAND")
```

The Pi is a **client** on the ESP32's softAP. The ESP32 is at `192.168.4.1`.

The ESP32 owns the authoritative target array at all times. The Pi is one possible
writer to that array, not a controller of servos. Nothing outside the ESP32 ever
commands a servo directly.

---

## 2. Axis table

There is one canonical axis index used everywhere — in the UDP packet, in presets,
in the portal, and in firmware. Never reorder this table.

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
  well above it (29 V / 35 V max, see 2.2).
- **Servos** are fed through a buck converter to a terminal block, stepped
  down to the servos' actual rated voltage, before reaching the PCA9685's V+
  pins. The PCA9685's signal pins carry PWM only, no power.
- **ESP32 logic** gets its own properly regulated supply, separate from the
  above.

Resolves the earlier BOM note about a 3S/4S conflict -- the pack is 4S; the
servo and logic rails are regulated down from it, not run raw.

### 2.2 Stepper hardware (as-built)

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

PCA channel assignment is **provisional**. Confirm by manual bench testing before
running CV.

### 2.1 Per-axis limits

Clamped in firmware, applied last, after every other stage. No source can exceed these.

| Idx | Min | Max | Notes |
|----|-----|-----|-------|
| 0–4 | 0 | 180 | full sweep by choice 2026-08-09; tendon over-pull risk at the closed end not yet bench-verified -- watch the first slow close |
| 5 | 40 | 140 | bench-confirmed 2026-08-09 |
| 6–9 | 70 | 110 | 90 ± 20, locked around neutral -- see §2 axis table |
| 10 | 40 | 140 | bench-confirmed 2026-08-09 |
| 11 | −80 | +80 | wiring twist limit — do not raise without a service-loop check |
| 12 | −90 | +90 | bench-confirmed 2026-08-09, symmetric around zero to hard stop |

---

## 3. UDP stream (Pi → ESP32)

**Endpoint:** `192.168.4.1:4210`, UDP, fire-and-forget.
**Rate:** 40 Hz nominal. The ESP32 must tolerate any rate from 0 to 100 Hz.
**Encoding:** ASCII, one datagram per frame, newline-terminated.

```
ARA,<seq>,<a0>,<a1>,<a2>,<a3>,<a4>,<a5>,<a6>,<a7>,<a8>,<a9>,<a10>,<a11>\n
```

- `ARA` — literal magic string. Reject anything else.
- `<seq>` — unsigned integer, 0–65535, increments per sent packet, wraps.
- `<a0>`…`<a11>` — axis values 0 through 11, integers, per the axis table.

Axis 12 (elbow extension) is **not** in the CV stream. See §5.

ASCII rather than binary is intentional: you can watch the stream with
`nc -ul 4210` on any machine and read it. At this rate the bandwidth cost is
irrelevant.

### 3.1 Receiver rules

| Condition | Action |
|-----------|--------|
| Magic string missing or wrong field count | Drop. Increment `malformed` counter. Do not partially apply. |
| Any value non-numeric | Drop the whole packet, not the field. |
| Packet older than the last accepted | Drop. See wrap rule below. |
| Value outside axis limits | Accept the packet, clamp the value. |
| Received while in MANUAL mode | Parse and discard. Do not write targets. |

**Wrap-safe ordering.** A packet is newer if
`(seq - last_seq) mod 65536 < 32768`. Never compare `seq` directly.

**Partial application is forbidden.** Either all 12 axes update or none do. A
half-applied frame is a pose the hand was never asked to make.

### 3.2 Watchdog

If no valid packet arrives for **400 ms** while in CV mode:

1. Hold all targets at their last accepted values. **Do not zero, do not open, do
   not return to neutral.** A hand full of MG996Rs snapping to a limit is how
   tendons and printed parts break.
2. Set the portal status to `CV STALE`.
3. Remain in CV mode. Recovery is automatic on the next valid packet, via the
   entry ramp in §5.2.

---

## 4. Presets

A preset is a named array of **13** values, axis 0 through 12 inclusive — the full
axis table, including both steppers.

```json
{
  "name": "prismatic wrap",
  "grasp_id": 12,
  "values": [0, 20, 20, 25, 30, 95, 90, 90, 90, 90, 140, 15, 40]
}
```

`grasp_id` refers to the Feix et al. (2016) GRASP taxonomy index where the preset
corresponds to one, otherwise null. This is what makes the preset library double as
the dexterity test battery.

**Don't-care axes.** Most hand-shape presets have no opinion about the elbow.
`NAN` in an axis slot means "leave this axis wherever it currently is" --
firmware skips it entirely rather than commanding a value (`main.cpp`, see `DC`
in `PRESETS[]` and the check in `applyPreset()`). Only presets that need a
specific elbow pose (e.g. `prismatic wrap`) set axes 11/12 to real numbers;
every other preset marks them `NAN` so recalling a hand shape never yanks the
elbow back to 0/0.

Preset recall is a **motion, not an assignment.** Servos arrive in tens of
milliseconds; a 90° move on the 64:1 elbow is roughly 3,200 full steps and takes
seconds. Therefore:

- Recall must be non-blocking. The main loop keeps running.
- Recall must be interruptible. A second preset pressed mid-move retargets rather
  than queuing.
- The portal shows `IN TRANSIT` until every axis is within tolerance of target,
  then `HELD`. Do not report arrival on command dispatch.

---

## 5. Modes

Two modes, one global enum. There are no per-axis ownership flags.

| Mode | Writes the target array |
|------|-------------------------|
| `MANUAL` | Portal sliders and preset recall |
| `CV` | UDP packet parser (axes 0–11) |

### 5.1 Axis 12 in CV mode

The CV stream does not carry elbow extension. On entering CV mode, axis 12 returns
to its boot-neutral zero (`moveTo(0)`) and holds there for the duration.

Consequence, stated plainly: **in CV mode the elbow is always at neutral.** Live
teleop of a grasp with the elbow raised is not possible. Any demonstration
requiring a positioned elbow must be a preset.

### 5.2 Transitions

**MANUAL → CV**

1. Axis 12 begins its return to zero. This is a multi-second motion; the mode
   switch is not instantaneous and the portal must show `SWITCHING`.
2. Axes 0–11 hold current position until the first valid packet arrives.
3. On that first packet, ramp from current position to the commanded values over
   **300 ms** using smoothstep. Do not jump — the first CV frame may be far from
   where a preset left the arm.

**CV → MANUAL**

1. Targets are inherited from the arm's current position, not from the sliders'
   last values.
2. The portal must repopulate every slider from the ESP32's current target array
   on switch and on page load. Otherwise the first slider touched snaps its joint.

**Safety note.** Switching to CV lowers the elbow. If the hand is holding
something, it lowers while holding. The control must be a deliberate press with a
confirm, not a widget adjacent to the sliders.

---

## 6. Stepper position reference

Both steppers are open-loop with no encoder. The ESP32 tracks **commanded** step
count from boot, which is not the same as actual position.

**Boot assumption:** both steppers are at mechanical neutral at power-on. The
operator is responsible for this.

**Drift is silent and cumulative.** One skipped step offsets every absolute preset
afterward, with no indication anywhere in the system.

Required mitigations:

- **Re-zero button, per stepper, on the portal.** Declares the current physical
  position to be zero and resets the counter. Build this early — it will be used
  constantly.
- **Re-zero before every measurement session.** Not optional for GRASP or payload
  trials; without it, "the same preset" is not the same posture across sessions.
- The 64:1 axis should get a hard-stop homing routine when time allows. At that
  reduction, output error is too large a step count to recover by eye.

### 6.1 Servo position reference

Servos (MG996R, SG90, AGFRC) have **no position feedback at all** — no encoder, no
telemetry line — and the PCA9685 is pure PWM output. Firmware cannot know where a
servo physically is, at boot or ever, so it does not guess.

**A servo channel is only driven once its axis is explicitly commanded** — a portal
slider move, a preset recall, or a CV packet. Until then, no PWM is sent to that
channel at all; it holds no position because nothing is telling it to. This means,
unlike the steppers, there is no boot-neutral assumption for servos: `AXIS_NEUTRAL`
is what a slider *shows* on first load, not what gets sent.

The bench-test escape hatch (`/axis` with `raw=1`) constrains to the servo's
physical 0–180° sweep instead of that axis's assumed `AXIS_MIN`/`AXIS_MAX` — for
sorting out which physical servo is wired to which logical axis without fighting
limits that may not apply to whatever is actually connected right now.

---

## 7. Vision host requirements

Enforced on the Pi, documented here because the ESP32's behavior depends on it.

- **Calibration is mandatory before transmission.** The Pi sends nothing until the
  operator has captured the neutral reference frame. A stream that starts
  uncalibrated produces plausible-looking garbage, which is worse than no stream.
- **Forearm axis comes from PoseLandmarker** (elbow → wrist), making wrist flexion
  and roll measurements rather than deviations from a held pose.
- **Hand landmarks run on a wrist-anchored ROI crop** of the full-resolution frame,
  so the wide framing Pose requires does not degrade finger and splay precision.
- **Splay is gated on curl.** Splay estimates degrade as a finger closes and its ray
  foreshortens. Above a curl threshold (**TBD**), freeze that finger's splay at its
  last trusted value.
- **Roll freezes near the singularity.** When hand pitch relative to the camera
  exceeds **TBD**, hold roll rather than tracking it.
- **No hand detected** → stop sending. Let the ESP32 watchdog hold position. Do not
  send a neutral pose.

---

## 8. Portal endpoints

| Method | Path | Purpose |
|--------|------|---------|
| GET | `/state` | Current target array, mode, transit status, watchdog state |
| POST | `/axis` | Set one axis. Body: `{"idx": 5, "value": 95}`. Add `"raw": 1` on a servo axis to bypass its `AXIS_MIN`/`AXIS_MAX` and constrain to 0–180 instead — see §6.1. |
| POST | `/preset` | Recall by name |
| POST | `/mode` | `{"mode": "CV"}` or `{"mode": "MANUAL"}` |
| POST | `/zero` | `{"axis": 11}` — declare current position as zero |
| POST | `/jog` | Bench-only continuous spin. `{"axis": 11, "dir": 1, "speed": 2000}` to run; `{"axis": 11, "stop": 1}` to decelerate. MANUAL mode + stepper axes only. |

`/state` is what the portal polls to repopulate sliders. It is the mechanism behind
the inherit-on-switch rule in §5.2.

`/jog` bypasses `AXIS_MIN`/`AXIS_MAX` entirely — it commands the driver directly via
`runForward()`/`runBackward()`, not a position. `target[]` is resynced to the
stepper's actual rest position (clamped) once it settles after `stop`, so normal
control picks up from there rather than snapping back to the pre-jog target.
Switching to CV mode force-stops any active jog.

---

## 9. Open items

- [x] All **TBD** axis limits, from bench testing -- see §2.1, all set 2026-08-09
- [x] Confirm the provisional PCA channel map -- bench-confirmed 2026-08-09
- [ ] Re-author the 8 existing presets as 13-value arrays
- [x] Decide whether presets carry a per-axis "don't care" marker -- yes, `NAN`
      sentinel, see §4
- [x] Resolve the 3S / 4S LiPo conflict between the BOM and the as-built architecture -- see §2.1
- [ ] Hard-stop homing routine for axis 12

---

## Revision history

| Version | Date | Change |
|---------|------|--------|
| 0.1 | 2026-08-02 | Initial draft |
| 0.1 | 2026-08-04 | Stepper hardware corrected: elbow roll is an A4988 (was assumed TMC2209), elbow extension is a TMC2209 on unchanged pins; both confirmed 1/16 microstepping. |
| 0.1 | 2026-08-04 | PCA channel map reworked: finger+wrist bank moved to the last 6 channels (10–15), splay+opposition bank to the first 5 (0–4), leaving ch 5–9 as a spare gap. |
| 0.1 | 2026-08-05 | Reverted the 2026-08-04 bench-wiring `AXIS_TO_PCA`/`AXIS_WIRED` override (back to the planned last-6/first-5 layout). Added `/jog` bench test mode: continuous stepper spin with operator-set direction/speed, independent of `AXIS_MIN`/`AXIS_MAX`. |
| 0.1 | 2026-08-05 | Servos are no longer driven to `AXIS_NEUTRAL` at boot (§6.1) — a channel gets no PWM until explicitly commanded. Added `/axis` `raw=1` to bypass a servo axis's assumed limits for full 0–180 bench sweeps. |