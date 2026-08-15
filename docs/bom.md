# Electronics — parts list

Informal reference for the electronics side of ARA. Mechanical parts are in the
CAD model.

## Servos (11 total, all on the PCA9685)
| Qty | Part | Role | ~$ |
|-----|------|------|----|
| 5 | MG996R metal-gear servo | Finger flexion — thumb, index, middle, ring, pinky | |
| 1 | AGFRC 15.5 kg low-profile servo | Wrist flex/extension | |
| 5 | SG90 micro servo | 4× finger splay + 1× thumb opposition | |

## Control
| Qty | Part | Role | ~$ |
|-----|------|------|----|
| 1 | ESP32 DevKit (WROOM-32, `esp32dev`) | On-arm controller, hosts the portal | |
| 1 | PCA9685 16-ch PWM driver (I²C @ 0x40) | Drives all 11 servos | |

## Elbow steppers + drivers
| Qty | Part | Role | ~$ |
|-----|------|------|----|
| 2 | NEMA 17 — StepperOnline 17HS19-2004S1 (1.8°, 2.0 A/phase) | Elbow extension (64:1) + elbow roll (4:1) | |
| 1 | Adafruit TMC2209 breakout (PID 6121) | Elbow extension driver, 1/16 microstep | |
| 1 | Adafruit A4988 breakout (PID 6109) | Elbow roll driver, 1/16 microstep | |

## Power
| Qty | Part | Role | ~$ |
|-----|------|------|----|
| 1 | 4S LiPo, 14.8 V (~3700 mAh), XT60 | Single pack powers the whole arm | |
| 1 | Buck converter (~30 A) | Steps 4S down to servo voltage for the PCA9685 V+ rail | |
| 1 | Terminal block | Servo power distribution | |
| — | Lever / Wago-style connectors | Battery distribution off the pack | |
| — | Wire: 12–14 AWG trunk, 18 AWG branches, ferrule crimps | Distribution wiring | |
| 1 | Separate regulated 5 V supply for the ESP32 | Clean logic rail, isolated from servo noise | |
