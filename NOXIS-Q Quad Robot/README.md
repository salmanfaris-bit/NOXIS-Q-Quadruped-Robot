# 🐾 NOXIS — Quadruped Robot Dog

<p align="center">
  <img src="https://img.shields.io/badge/Platform-ESP8266-blue?style=flat-square" />
  <img src="https://img.shields.io/badge/Servos-12×%20MG946R-orange?style=flat-square" />
  <img src="https://img.shields.io/badge/Control-WiFi%20%2B%20Serial-green?style=flat-square" />
  <img src="https://img.shields.io/badge/Version-v10.0-red?style=flat-square" />
  <img src="https://img.shields.io/badge/Status-Working-brightgreen?style=flat-square" />
</p>

> A 12-servo, ESP8266-powered quadruped robot dog with custom inverse kinematics, diagonal trot gait, MPU6050 stabilisation, WiFi web controller, and air quality sensing — built and tuned entirely from scratch.

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Hardware](#-hardware)
- [Wiring](#-wiring)
- [Leg & Servo Map](#-leg--servo-map)
- [Inverse Kinematics](#-inverse-kinematics)
- [Gait System](#-gait-system)
- [WiFi Control](#-wifi-control)
- [Servo Calibration](#-servo-calibration)
- [File Structure](#-file-structure)
- [Libraries Required](#-libraries-required)
- [How to Flash](#-how-to-flash)
- [Serial Commands](#-serial-commands)
- [Roadmap](#-roadmap)

---

## 🤖 Overview

NOXIS is a fully functional 4-legged walking robot built on the **Nova SM3 (Spot-Mini Micro)** 3D-printed frame. Every layer of the software — IK solver, gait engine, MPU stabilisation, and web UI — was written from scratch and verified on the real physical robot.

**What it can do:**
- Walk forward / backward
- Turn left / right (diagonal trot with frontal-plane D-arc for coxa)
- Spot turn in place
- Change body height across 5 levels
- Sit and stand with smooth transitions
- Greeting wave gesture
- MPU6050 tilt stabilisation while walking
- Real-time air quality monitoring (MQ-135 AQI + MQ-7 CO alert)
- WiFi web UI with virtual joystick — works from any phone browser

---

## 🛠 Hardware

| Component | Details |
|---|---|
| Microcontroller | ESP8266 NodeMCU 1.0 (ESP-12E) |
| Servo Driver | PCA9685 — I2C address `0x40` |
| Servos | 12× MG946R (180°, **not** 270°) |
| IMU | MPU6050 — I2C address `0x68` |
| Air Quality | MQ-135 (analog AQI) on `A0` |
| CO Sensor | MQ-7 (digital alert) on `D5` / GPIO14 |
| 3D Frame | Nova SM3 — [Thingiverse #4767006](https://www.thingiverse.com/thing:4767006) |

**Measured leg geometry:**

| Segment | Length |
|---|---|
| Coxa (hip bracket) | 30 mm |
| Femur (hip → knee) | 105 mm |
| Tibia (knee → foot) | 130 mm |
| Max reach (F + T) | 235 mm |
| Normal stand height | 205 mm (20.5 cm) |
| Full stretch height | 235 mm (23.5 cm) |

---

## 🔌 Wiring

```
ESP8266 NodeMCU
│
├── D1 (SCL) ──┬── PCA9685 SCL
│              └── MPU6050 SCL
│
├── D2 (SDA) ──┬── PCA9685 SDA
│              └── MPU6050 SDA
│
├── A0 ──────────── MQ-135 AOUT  (analog air quality)
└── D5 (GPIO14) ─── MQ-7  DOUT  (digital CO alert: HIGH=clean, LOW=detected)

PCA9685 channels 0–11 → 12× MG946R servos
```

> Both PCA9685 and MPU6050 share the same I2C bus. `Wire.begin()` with no arguments uses D1/D2 by default on ESP8266.

---

## 🦵 Leg & Servo Map

```
         FRONT
    RF          FL
     \          /
      [  BODY  ]
     /          \
    BR          BL
         BACK
```

| Leg | Joint | PCA9685 Channel | Servo # |
|---|---|---|---|
| RF (Front Right) | Coxa | ch 0 | S1 |
| RF | Femur | ch 1 | S2 |
| RF | Tibia | ch 2 | S3 |
| BR (Back Right) | Coxa | ch 3 | S4 |
| BR | Femur | ch 4 | S5 |
| BR | Tibia | ch 5 | S6 |
| BL (Back Left) | Coxa | ch 6 | S7 |
| BL | Femur | ch 7 | S8 |
| BL | Tibia | ch 8 | S9 |
| FL (Front Left) | Coxa | ch 9 | S10 |
| FL | Femur | ch 10 | S11 |
| FL | Tibia | ch 11 | S12 |

**PWM settings:**
```cpp
SERVO_FREQ = 50     // Hz
SERVO_MIN  = 135    // pulse count → 0°
SERVO_MAX  = 545    // pulse count → 180°
```

---

## 📐 Inverse Kinematics

A 2-DOF sagittal-plane IK solver handles all leg positioning. Only **femur + tibia** are solved; the coxa stays at 90° during all walking and only swings during turns.

**Coordinate system (per leg):**
- Origin = femur pivot point
- `X+` = forward
- `Z+` = downward (positive = foot below pivot)
- At normal stand: `X = 20 mm`, `Z = 205 mm`

**Math:**
```
L    = sqrt(x² + z²)

cosK = (F² + T² - L²) / (2·F·T)    →  knee bend
cosA = (F² + L² - T²) / (2·F·L)
γ    = atan2(z, x)

femur_math = γ − acos(cosA)
tibia_math = 180° − acos(cosK)      →  0 = straight leg
```

**Servo angle mapping** (left legs are physically mirrored on the frame):

| Side | Femur servo | Tibia servo |
|---|---|---|
| Right (RF, BR) | `90 − femur_math` | `50 + tibia_math` |
| Left (BL, FL) | `90 + femur_math` | `130 − tibia_math` |

---

## 🚶 Gait System

**Diagonal Trot** — two diagonal pairs move 180° out of phase:

| Pair | Legs | Starting phase |
|---|---|---|
| A | RF + BL | 0.0 |
| B | BR + FL | 0.5 |

Each leg runs a continuous phase value `0.0 → 1.0`, incremented every 20 ms tick.

| Phase | State | Foot motion |
|---|---|---|
| 0.0 → 0.5 | Ground | Sweeps `+STRIDE → −STRIDE` in X at constant Z |
| 0.5 → 1.0 | Air | Sine arc — lifts foot and swings forward |

**Gait parameters:**

```cpp
LEAN_X     = 20 mm   // foot forward of hip at stand
STRIDE     = 22 mm   // half-stride (foot sweeps ±22 mm)
LIFT_H     = 38 mm   // foot lift height
TURN_SWEEP = 16 deg  // coxa swing per turn cycle
PHASE_INC  = 0.028   // phase per 20ms tick (~714ms full cycle)
```

**Turning** uses a frontal-plane D-arc: coxa sweeps during the ground phase to push the body into a yaw, then resets during the air phase.

---

## 📡 WiFi Control

The robot broadcasts its own WiFi access point — no router needed:

```
SSID     : RobotDog
Password : 12345678
URL      : http://192.168.4.1
```

Open the URL from any phone or laptop browser. The web UI provides a virtual joystick and action buttons (Stand, Sit, Greet, Height+, Height−).

**REST API endpoints:**

| Endpoint | Description |
|---|---|
| `GET /` | Serves the web UI |
| `GET /joy?x=&y=&m=` | Joystick XY + magnitude input |
| `GET /cmd?c=<char>` | Single-char command |
| `GET /sensors` | Returns air quality JSON |

---

## 🔧 Servo Calibration

**Confirmed standing angles (physically measured on robot):**

| Leg | Coxa | Femur | Tibia |
|---|---|---|---|
| RF | 90° | 55° | 110° |
| BR | 90° | 65° | 110° |
| BL | 90° | 125° | 70° |
| FL | 90° | 125° | 70° |

**Tibia zero reference (leg fully straight):**
- Right legs (RF, BR): `50°`
- Left legs (BL, FL): `130°`

**Height levels (body underside → ground):**

| Level | Z (mm) |
|---|---|
| 0 | 165 |
| 1 | 180 |
| 2 (default) | 205 |
| 3 | 215 |
| 4 | 220 |

---

## 📁 File Structure

```
noxis-quadruped/
├── quadruped_final_code.ino   # Main firmware — IK, gait, WiFi, sensors (v10.0)
├── servos_test_code.ino       # Standalone servo calibration utility
└── README.md
```

> Use `servos_test_code.ino` first to verify all 12 servos move correctly before flashing the main code. Send `<servo_number> <angle>` over Serial (e.g. `3 90`).

---

## 📦 Libraries Required

Install via **Arduino Library Manager**:

```
Adafruit PWM Servo Driver Library
```

The following are included with the ESP8266 board package:
```
ESP8266WiFi
ESP8266WebServer
Wire
math.h
```

**Board package URL** (add in Preferences → Additional Boards Manager URLs):
```
http://arduino.esp8266.com/stable/package_esp8266com_index.json
```

---

## ⚡ How to Flash

1. Install [Arduino IDE](https://www.arduino.cc/en/software)
2. Add the ESP8266 board URL in **File → Preferences → Additional Boards Manager URLs**
3. Install **esp8266** via **Tools → Boards Manager**
4. Install **Adafruit PWM Servo Driver** via **Sketch → Include Library → Manage Libraries**
5. Open `quadruped_final_code.ino`
6. Select: Board = **NodeMCU 1.0 (ESP-12E)**, Port = your COM port
7. Upload
8. Open Serial Monitor at **9600 baud** — the robot will print its WiFi IP on boot

---

## 🖥 Serial Commands

Connect at **9600 baud** and send single-character commands:

| Key | Action |
|---|---|
| `f` | Walk forward |
| `b` | Walk backward |
| `l` | Turn left |
| `r` | Turn right |
| `s` | Stop / stand |
| `x` | Sit |
| `u` | Height up |
| `d` | Height down |
| `g` | Greeting wave |
| `h` | Print help |

---

## 🗺 Roadmap

- [x] 12-servo diagonal trot walk
- [x] Turning with frontal-plane D-arc coxa sweep
- [x] Height control (5 levels)
- [x] Sit / stand with smooth transitions
- [x] Greeting wave gesture
- [x] MPU6050 tilt stabilisation
- [x] WiFi web controller with virtual joystick
- [x] Air quality monitoring (MQ-135 AQI + MQ-7 CO)
- [ ] Variable walk speed
- [ ] Side-step / strafe gait
- [ ] Crawl gait (3 feet always on ground)
- [ ] MPU-based auto-level while standing
- [ ] Battery voltage monitor

---

## 👤 Author

**Salman Faris S** · May 2026

3D frame: [Nova SM3 by Nikola Lalic](https://www.thingiverse.com/thing:4767006)
