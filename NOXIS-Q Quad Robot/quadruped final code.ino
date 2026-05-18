/*
 ============================================================
  QUADRUPED ROBOT DOG — IK v10.0
  CHANGES vs v9:
    1. Greeting wave: ONLY femur+tibia move (no coxa/hip) — pure up/down
    2. Height change: STEP_SLOW interpolation, much smoother
    3. Sit → Stand transition: slow smooth rise, leg by leg
    4. Stand → Sit transition: slow smooth lower, leg by leg
  Author  : Salman Faris S
  Hardware: ESP8266 NodeMCU + PCA9685 + MG946R (180°)
            MQ-135 (Air Quality) on A0 | MQ-7 (CO) digital on D5

 ============================================================
  WHAT IS FIXED / NEW IN v9 vs v8:
 ============================================================

  FIX 1 — TURN D-ARC IS NOW FRONTAL-PLANE (viewed from front/back)
  ---------------------------------------------------------------
  Before (v5-v8): coxa servo swung sideways during air phase only.
  That produced a tiny foot shift at the hip level — not a real D.

  Now: During a TURN, the foot traces a D-arc in the FRONTAL plane:
    Ground phase: coxa sweeps from +TURN_AMT back to 0 (or 0 to -TURN_AMT)
                  foot on ground, pushes body to yaw
    Air phase   : femur+tibia lift the foot (same sine arc as forward walk),
                  coxa swings to the start position for the next ground push
  This is exactly symmetrical to how forward/backward uses the sagittal D:
    Fwd/Back sagittal D: viewed from SIDE — femur/tibia draw the D
    Turn frontal D      : viewed from FRONT/BACK — coxa draws the D

  The coxa servo mapping for left/right mirrored legs is handled by
  coxaServo() which has always been correct in v7+.

  FIX 2 — BACKWARD TURN DIRECTION CORRECTED
  ------------------------------------------
  When going backward, a right turn means the robot curves right while
  moving backward — the body's yaw is the same as forward right turn,
  but the leg sweep directions are already reversed by walkDir=-1.
  The coxa turn sign must NOT additionally flip for backward.
  We keep turnAmt sign consistent and let walkDir handle sweep direction.

  FIX 3 — HEIGHT CONTROL: LEGS NO LONGER GO BELOW HIP AXIS
  ----------------------------------------------------------
  Root cause: HEIGHT_Z[4]=230mm is 230/235 = 97.9% of max reach.
  At that extension, femur is nearly horizontal. Any tiny IK error
  pushes it past 90 degrees (below hip axis).

  Fix a: HEIGHT_Z[4] capped at 220mm (93% of reach — safe zone).
  Fix b: In femurServo(), result is clamped to [30,150] — prevents
         femur servo going past horizontal in either direction.
  Fix c: In applyHeight(), we also recalc legX[] to LEAN_X so the
         foot position is fully reset (not left at a walk position).

  FIX 4 — LEAN_X = 20 (foot 20mm forward of hip)
  ------------------------------------------------
  v7/v8 accidentally reset this back to 5mm from v6's 20mm fix.
  Restoring to 20mm so legs are visually in front of hip line.

  FIX 5 — MQ-135 (AIR QUALITY) on A0, MQ-7 (CO) on D5
  -------------------------------------------------------
  MQ-135 AOUT → A0 (analog, direct read)
  MQ-7   DOUT → D5 (GPIO14, digital HIGH=clean / LOW=CO detected)
  No MUX needed. Simple and clean.

  FIX 6 — WEB UI REDESIGN
  ------------------------
  Buttons: SIT | STAND | GREET | HEIGHT+ | HEIGHT-
  Air quality panel shows CO (ppm-raw), AQ index, and status label.
  Joystick unchanged. Status shows current motion.

 ============================================================
  GEOMETRY (verified on real robot):
    Coxa  = 30mm  (horizontal bracket)
    Femur = 105mm, Tibia = 130mm
    Max reach = 235mm
    Normal stand Z = 205mm
    Body length = 275mm

  CALIBRATION:
    RF tibia zero = 50   (straight leg servo angle)  — confirmed
    BR tibia zero = 60   (confirmed real robot)
    BL tibia zero = 130  — confirmed
    FL tibia zero = 130  — confirmed
    Femurs: 90 = vertical
    Coxa:   90 = neutral forward

  CHANNELS:
    RF: coxa=ch0  femur=ch1  tibia=ch2
    BR: coxa=ch3  femur=ch4  tibia=ch5
    BL: coxa=ch6  femur=ch7  tibia=ch8
    FL: coxa=ch9  femur=ch10 tibia=ch11

  CONTROL:
    WiFi AP: SSID=RobotDog  PASS=12345678  IP=192.168.4.1
    Serial commands (baud 9600): f b l r s x u d g h

  MQ SENSOR WIRING:
    MQ-135 AOUT → A0  (analog read, air quality)
    MQ-7   DOUT → D5  (GPIO14, digital: HIGH=clean, LOW=CO detected)
 ============================================================
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <math.h>

// ── WiFi AP ───────────────────────────────────────────────
const char* AP_SSID = "RobotDog";
const char* AP_PASS = "12345678";
ESP8266WebServer server(80);

// ── PWM driver ────────────────────────────────────────────
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
#define SERVO_FREQ  50
#define SERVO_MIN  135
#define SERVO_MAX  545

// ── Leg / joint indices ───────────────────────────────────
#define RF 0
#define BR 1
#define BL 2
#define FL 3
#define CX 0
#define FM 1
#define TB 2

const uint8_t CH[4][3] = {
  { 0,  1,  2},   // RF
  { 3,  4,  5},   // BR
  { 6,  7,  8},   // BL
  { 9, 10, 11}    // FL
};

// Right-side legs have servo directions NOT mirrored
const bool IS_RIGHT[4] = {true, true, false, false};

// ── Leg geometry (mm) ─────────────────────────────────────
#define F_LEN  105.0f
#define T_LEN  130.0f
#define MAX_REACH 234.0f   // F+T-1, hard physical limit

// ── MQ sensor pins ────────────────────────────────────────
// MQ-135: analog air quality sensor on A0
// MQ-7:   CO sensor digital output on D5 (GPIO14)
//         DOUT=HIGH → CO below threshold (clean air)
//         DOUT=LOW  → CO above threshold (detected)
#define MQ135_PIN  A0           // analog
#define MQ7_PIN    14           // D5 = GPIO14, digital

int  mq135_raw  = 0;            // 0-1023 analog reading
bool mq7_alert  = false;        // true = CO detected
unsigned long lastSensorRead = 0;
const unsigned int SENSOR_INTERVAL = 2000;  // read every 2s

// ── Async servo system ────────────────────────────────────
#define LOOP_MS   20
#define DEADBAND   2
#define STEP_WALK  2.5f
#define STEP_SLOW  0.8f
#define STEP_MED   1.5f

float stepDeg = STEP_WALK;
int   sPos[4][3];
int   sTgt[4][3];
bool  sAct[4][3];

// ── Height control ────────────────────────────────────────
// MAX is now 220 (was 230) — prevents femur going past horizontal
const int HEIGHT_Z[5] = {165, 180, 205, 215, 220};
int hLevel = 2;
int standZ = 205;

// ── Gait constants ────────────────────────────────────────
#define LEAN_X       20     // mm foot FORWARD of hip (v9: restored to 20mm)
#define STRIDE       22     // half-stride in sagittal plane (mm)
#define LIFT_H       38     // foot lift height (mm)
#define TURN_SWEEP   16     // coxa degrees swept per full turn cycle in frontal D
#define SPOT_THRESH  0.60f  // joystick magnitude threshold for spot turn

// ── Gait state ────────────────────────────────────────────
// Diagonal trot: RF+BL = pair A (phase 0.0), BR+FL = pair B (phase 0.5)
float legPhase[4] = {0.0f, 0.5f, 0.0f, 0.5f};
bool  gaitRunning = false;

// ── Joystick / command input ──────────────────────────────
float joyX   = 0.0f;
float joyY   = 0.0f;
float joyMag = 0.0f;

// ── Derived gait params ───────────────────────────────────
float phaseInc   = 0.0f;
float walkDir    = 1.0f;
float prevWalkDir= 1.0f;
float turnAmt    = 0.0f;
bool  spotTurn   = false;

// ── Motion state machine ──────────────────────────────────
enum MotionState { ST_STAND, ST_SIT, ST_WALK, ST_GREET, ST_GREET_RETURN };
MotionState motionState = ST_STAND;

// ── Foot tracking ─────────────────────────────────────────
float legX[4];
float legZ[4];

// =============================================================
//  INVERSE KINEMATICS (2-DOF sagittal plane)
// =============================================================
bool solveIK(float x_fwd, float z_down, float &fm, float &tm) {
  float L2 = x_fwd*x_fwd + z_down*z_down;
  float L  = sqrtf(L2);
  if (L > MAX_REACH)       return false;
  if (L < fabsf(F_LEN - T_LEN) + 1.0f) return false;

  float cosK = (F_LEN*F_LEN + T_LEN*T_LEN - L2) / (2.0f*F_LEN*T_LEN);
  cosK = constrain(cosK, -1.0f, 1.0f);
  tm = (M_PI - acosf(cosK)) * (180.0f / M_PI);

  float cosA = (F_LEN*F_LEN + L2 - T_LEN*T_LEN) / (2.0f*F_LEN*L);
  cosA = constrain(cosA, -1.0f, 1.0f);
  float gamma = atan2f(z_down, x_fwd);
  fm = (gamma - acosf(cosA)) * (180.0f / M_PI);
  return true;
}

// femur servo angle from IK result
// Clamped [30,150] to prevent femur going below horizontal
int femurServo(uint8_t leg, float fm) {
  int ang = IS_RIGHT[leg] ? (90 - (int)roundf(fm))
                          : (90 + (int)roundf(fm));
  return constrain(ang, 30, 150);
}

// Tibia zero (straight-down servo angle per leg)
// RF=50, BR=60 confirmed. BL=FL=130 confirmed.
const int TIBIA_ZERO[4] = {50, 60, 130, 130};

int tibiaServo(uint8_t leg, float tm) {
  return constrain(IS_RIGHT[leg] ? (TIBIA_ZERO[leg] + (int)roundf(tm))
                                 : (TIBIA_ZERO[leg] - (int)roundf(tm)), 0, 180);
}

// =============================================================
//  COXA SERVO HELPER
//  rightwardSwing: positive = foot tip moves toward robot's RIGHT
//  Right legs (RF,BR): servo increase → tip right  → ang = 90 + swing
//  Left  legs (BL,FL): servo decrease → tip right  → ang = 90 - swing
// =============================================================
int coxaServo(uint8_t leg, float rightwardSwing) {
  float s = IS_RIGHT[leg] ? rightwardSwing : -rightwardSwing;
  return constrain(90 + (int)roundf(s), 45, 135);
}

// =============================================================
//  SERVO HARDWARE LAYER
// =============================================================
void writeServo(uint8_t leg, uint8_t j) {
  int a = constrain(sPos[leg][j], 0, 180);
  pwm.setPWM(CH[leg][j], 0, map(a, 0, 180, SERVO_MIN, SERVO_MAX));
}

void setTgt(uint8_t leg, uint8_t j, int t) {
  t = constrain(t, 0, 180);
  if (abs(t - sPos[leg][j]) <= DEADBAND) {
    sPos[leg][j] = sTgt[leg][j] = t;
    sAct[leg][j] = false;
    writeServo(leg, j);
  } else {
    sTgt[leg][j] = t;
    sAct[leg][j] = true;
  }
}

void snapS(uint8_t leg, uint8_t j, int v) {
  v = constrain(v, 0, 180);
  sPos[leg][j] = sTgt[leg][j] = v;
  sAct[leg][j] = false;
  writeServo(leg, j);
}

void updateServos() {
  for (int leg = 0; leg < 4; leg++) {
    for (int j = 0; j < 3; j++) {
      if (!sAct[leg][j]) continue;
      int diff = sTgt[leg][j] - sPos[leg][j];
      if (abs(diff) <= DEADBAND) {
        sPos[leg][j] = sTgt[leg][j];
        sAct[leg][j] = false;
      } else {
        int step = max(1, (int)stepDeg);
        sPos[leg][j] += (diff > 0) ? step : -step;
      }
      writeServo(leg, j);
    }
  }
}

bool allDone(uint8_t leg) {
  return !sAct[leg][CX] && !sAct[leg][FM] && !sAct[leg][TB];
}
bool allLegsDone() {
  for (int i = 0; i < 4; i++) if (!allDone(i)) return false;
  return true;
}

// =============================================================
//  FOOT MOVEMENT
// =============================================================
// coxaRight: rightward swing in degrees (positive = tip toward robot's right)
bool moveFoot(uint8_t leg, float x_fwd, float z_down, float coxaRight = 0.0f) {
  float fm, tm;
  if (!solveIK(x_fwd, z_down, fm, tm)) return false;
  setTgt(leg, CX, coxaServo(leg, coxaRight));
  setTgt(leg, FM, femurServo(leg, fm));
  setTgt(leg, TB, tibiaServo(leg, tm));
  return true;
}

bool snapFoot(uint8_t leg, float x_fwd, float z_down, float coxaRight = 0.0f) {
  float fm, tm;
  if (!solveIK(x_fwd, z_down, fm, tm)) return false;
  snapS(leg, CX, coxaServo(leg, coxaRight));
  snapS(leg, FM, femurServo(leg, fm));
  snapS(leg, TB, tibiaServo(leg, tm));
  return true;
}

bool moveFootRawCox(uint8_t leg, float x_fwd, float z_down, int rawCox) {
  float fm, tm;
  if (!solveIK(x_fwd, z_down, fm, tm)) return false;
  setTgt(leg, CX, constrain(rawCox, 0, 180));
  setTgt(leg, FM, femurServo(leg, fm));
  setTgt(leg, TB, tibiaServo(leg, tm));
  return true;
}

void waitMs(unsigned long ms) {
  unsigned long t = millis();
  while (millis() - t < ms) {
    updateServos();
    server.handleClient();
    yield();
    delay(LOOP_MS);
  }
}

// Foot lift arc: smooth sine Z, peaks at standZ - LIFT_H at t=0.5
float liftArc(float t) {
  return standZ - LIFT_H * sinf(t * M_PI);
}

// =============================================================
//  STAND POSE
// =============================================================
void poseStand(bool slow = false) {
  standZ  = HEIGHT_Z[hLevel];

  if (slow && motionState == ST_SIT) {
    // ── STAND FROM SIT ────────────────────────────────────
    // Step 1: Both BACK legs rise to stand simultaneously
    stepDeg = STEP_MED;
    moveFoot(BR, LEAN_X, standZ, 0.0f);
    moveFoot(BL, LEAN_X, standZ, 0.0f);
    waitMs(750);   // wait for back legs to fully rise

    // Step 2: Both FRONT legs pull back to normal stand simultaneously
    moveFoot(RF, LEAN_X, standZ, 0.0f);
    moveFoot(FL, LEAN_X, standZ, 0.0f);
    waitMs(650);   // wait for front legs to settle

  } else if (slow) {
    // Normal slow stand (joystick release, greeting return)
    stepDeg = STEP_MED;
    for (int leg = 0; leg < 4; leg++) moveFoot(leg, LEAN_X, standZ, 0.0f);

  } else {
    // Instant snap (boot or serial 's')
    stepDeg = STEP_WALK;
    for (int leg = 0; leg < 4; leg++) snapFoot(leg, LEAN_X, standZ, 0.0f);
  }

  for (int leg = 0; leg < 4; leg++) {
    legX[leg] = LEAN_X;
    legZ[leg] = standZ;
  }
  motionState = ST_STAND;
  gaitRunning = false;
  Serial.println(F("[STAND]"));
}

// =============================================================
//  SIT POSE
//  — Both back legs fold DOWN together (simultaneously)
//  — Both front legs stretch FORWARD together (simultaneously)
//  — All 4 move at the same time in one smooth motion
// =============================================================
void poseSit() {
  if (gaitRunning) { gaitRunning = false; }
  standZ  = HEIGHT_Z[hLevel];
  stepDeg = STEP_MED;   // medium speed — smooth but not sluggish

  float sitZ      = 92.0f;          // back legs fold to this height (low squat)
  float sitFrontZ = standZ - 8.0f;  // front legs stretch slightly longer (lower nose)
  float sitFrontX = LEAN_X + 18.0f; // front feet reach forward during sit

  // All 4 legs move simultaneously — no waitMs between them
  moveFoot(RF, sitFrontX, sitFrontZ, 0.0f);  // front right — stretch forward
  moveFoot(FL, sitFrontX, sitFrontZ, 0.0f);  // front left  — stretch forward
  moveFoot(BR, LEAN_X + 5.0f, sitZ,  0.0f);  // back right  — fold down
  moveFoot(BL, LEAN_X + 5.0f, sitZ,  0.0f);  // back left   — fold down

  waitMs(900);  // single wait for all 4 to reach position together

  legX[RF] = sitFrontX;      legZ[RF] = sitFrontZ;
  legX[FL] = sitFrontX;      legZ[FL] = sitFrontZ;
  legX[BR] = LEAN_X + 5.0f;  legZ[BR] = sitZ;
  legX[BL] = LEAN_X + 5.0f;  legZ[BL] = sitZ;
  motionState = ST_SIT;
  Serial.println(F("[SIT]"));
}

// =============================================================
//  HEIGHT ADJUST
// =============================================================
void applyHeight() {
  standZ  = HEIGHT_Z[hLevel];
  stepDeg = STEP_SLOW;
  for (int leg = 0; leg < 4; leg++) {
    // Always reset X to LEAN_X on height change — prevents
    // mid-gait X position causing IK overshoot at extreme heights
    legX[leg] = LEAN_X;
    legZ[leg] = standZ;
    moveFoot(leg, LEAN_X, standZ, 0.0f);
  }
  Serial.print(F("[HEIGHT] level=")); Serial.print(hLevel);
  Serial.print(F(" Z=")); Serial.println(standZ);
}

// =============================================================
//  GAIT ENGINE — DIAGONAL TROT
//
//  FORWARD/BACKWARD D-ARC (sagittal, viewed from SIDE):
//    Ground phase (ph 0→0.5):
//      fx = LEAN_X + dir*STRIDE*(1-2t)  sweeps back (propels body fwd)
//      fz = standZ (foot on ground, flat)
//    Air phase (ph 0.5→1.0):
//      fx = LEAN_X + dir*STRIDE*(2t-1)  swings forward to reset
//      fz = standZ - LIFT_H*sin(t*PI)   lifted, D-shape arc
//
//  TURN D-ARC (frontal, viewed from FRONT/BACK):
//    This is a separate mode when spotTurn=true (joystick far sideways).
//    The coxa draws the D in the frontal plane:
//      Ground phase: coxa sweeps from +swing to -swing (foot pushes ground laterally → body yaws)
//      Air phase   : femur+tibia lift foot (same sine arc), coxa resets to +swing
//    Right turn (+turnAmt): right-side legs (RF,BR) sweep coxa backward
//                           left-side legs (BL,FL) sweep coxa forward
//    This gives correct counter-rotation.
//
//  CURVE TURN (while walking fwd/bwd):
//    Differential stride: right legs shorter, left legs longer for right turn.
//    Coxa stays at neutral (90) during curve turns — no frontal D needed.
//
//  BACKWARD WALK:
//    initGait() offsets all phases by 0.5 when walkDir=-1
//    so the footfall sequence is exactly reversed.
//    walkDir=-1 also flips the sweep formula via dir variable.
//    turnAmt sign stays the same — no inversion needed for backward turn.
// =============================================================

void initGait() {
  standZ = HEIGHT_Z[hLevel];
  for (int leg = 0; leg < 4; leg++) {
    legX[leg] = LEAN_X;
    legZ[leg] = standZ;
    snapFoot(leg, LEAN_X, standZ, 0.0f);
  }
  // Phase offset: for backward walk, offset all by 0.5 to reverse footfall sequence
  float bOff = (walkDir < 0.0f) ? 0.5f : 0.0f;
  legPhase[RF] = fmodf(0.0f + bOff, 1.0f);
  legPhase[BL] = fmodf(0.0f + bOff, 1.0f);
  legPhase[BR] = fmodf(0.5f + bOff, 1.0f);
  legPhase[FL] = fmodf(0.5f + bOff, 1.0f);
  gaitRunning = true;
}

void stepGait() {
  stepDeg = STEP_WALK;

  for (int leg = 0; leg < 4; leg++) {
    float ph = legPhase[leg];
    bool isRightLeg = (leg == RF || leg == BR);

    // ── SPOT TURN: FRONTAL D-ARC ─────────────────────────────
    // The coxa draws a D-shape viewed from front/back.
    // Right turn (+turnAmt):
    //   Right legs sweep coxa from +swing to -swing on ground (tip goes right → body yaws right)
    //   Left  legs sweep coxa from -swing to +swing on ground (same net yaw direction)
    //
    // Coxa rightward-swing convention (positive = tip toward robot's right):
    //   At start of ground phase:  swing = +TURN_SWEEP * |turnAmt| for right legs
    //                              swing = -TURN_SWEEP * |turnAmt| for left legs
    //   At end of ground phase:    swing reverses sign
    //   Air phase resets back to start position (preparing for next push)

    if (spotTurn) {
      float sweepAmt = TURN_SWEEP * fabsf(turnAmt);

      // For RIGHT turn (+turnAmt), right legs push their tip rightward, left legs leftward.
      // "rightward sweep direction" for right legs when turnAmt > 0:
      //   ground: starts at +sweepAmt, sweeps to -sweepAmt
      // For left legs (turnAmt > 0):
      //   ground: starts at -sweepAmt, sweeps to +sweepAmt
      // This achieves the same yaw rotation direction because both push against the ground.
      float startSwing = isRightLeg ?  sweepAmt * (turnAmt > 0 ? 1.0f : -1.0f)
                                    : -sweepAmt * (turnAmt > 0 ? 1.0f : -1.0f);
      float endSwing   = -startSwing;

      if (ph < 0.5f) {
        // GROUND PHASE: coxa sweeps start→end (foot pushes ground, body yaws)
        float t  = ph / 0.5f;
        float coxRight = startSwing + (endSwing - startSwing) * t;  // linear sweep
        moveFoot(leg, LEAN_X, standZ, coxRight);
        legX[leg] = LEAN_X;
        legZ[leg] = standZ;
      } else {
        // AIR PHASE: foot lifts (femur+tibia sine arc), coxa swings back to startSwing
        float t  = (ph - 0.5f) / 0.5f;
        float fz = liftArc(t);
        // coxa swings from endSwing back to startSwing during air phase
        float coxRight = endSwing + (startSwing - endSwing) * t;
        moveFoot(leg, LEAN_X, fz, coxRight);
        legX[leg] = LEAN_X;
        legZ[leg] = fz;
      }

    } else {
      // ── NORMAL WALK (forward, backward, curve turn) ──────────
      //
      // Curve turn: differential stride — right legs shorter, left longer for right turn.
      // Coxa stays at 0 (neutral 90°) during curve walk.
      float curveFactor = 1.0f;
      if (fabsf(turnAmt) > 0.05f) {
        float bias = turnAmt * 0.55f;
        curveFactor = isRightLeg ? (1.0f - bias) : (1.0f + bias);
        curveFactor = constrain(curveFactor, 0.10f, 1.90f);
      }
      float effStride = STRIDE * curveFactor;
      float dir = walkDir;  // +1 forward, -1 backward

      if (ph < 0.5f) {
        // GROUND PHASE: foot sweeps back (forward walk) or forward (backward walk)
        float t  = ph / 0.5f;
        float fx = LEAN_X + dir * effStride * (1.0f - 2.0f*t);
        moveFoot(leg, fx, standZ, 0.0f);
        legX[leg] = fx;
        legZ[leg] = standZ;
      } else {
        // AIR PHASE: foot lifts and swings forward (or backward) to reset
        float t  = (ph - 0.5f) / 0.5f;
        float fx = LEAN_X + dir * effStride * (2.0f*t - 1.0f);
        float fz = liftArc(t);
        moveFoot(leg, fx, fz, 0.0f);
        legX[leg] = fx;
        legZ[leg] = fz;
      }
    }

    // Advance phase (same direction for all modes)
    legPhase[leg] += phaseInc;
    if (legPhase[leg] >= 1.0f) legPhase[leg] -= 1.0f;
  }
}

// =============================================================
//  JOYSTICK → GAIT PARAMS
// =============================================================
void updateGaitFromJoystick() {
  if (joyMag < 0.10f) {
    if (gaitRunning) {
      gaitRunning = false;
      poseStand(true);
    }
    return;
  }

  float newDir = (joyY >= 0.0f) ? 1.0f : -1.0f;
  float absX   = fabsf(joyX);
  float absY   = fabsf(joyY);
  bool  newSpot = (joyMag > SPOT_THRESH) && (absX > absY * 1.3f);

  if (!gaitRunning) {
    walkDir = newDir;
    initGait();
    motionState = ST_WALK;
  } else {
    // Re-init if walk direction reversed (for correct phase flip)
    if (!newSpot && !spotTurn && newDir != prevWalkDir) {
      walkDir = newDir;
      initGait();
    } else {
      walkDir = newDir;
    }
  }

  prevWalkDir = walkDir;
  spotTurn    = newSpot;
  turnAmt     = joyX;
  phaseInc    = 0.012f + joyMag * 0.020f;

  if (spotTurn) walkDir = 1.0f;
}

// =============================================================
//  GREETING GESTURE
// =============================================================
void doGreeting() {
  Serial.println(F("[GREET] Starting..."));
  motionState = ST_GREET;
  gaitRunning = false;
  standZ = HEIGHT_Z[hLevel];

  // Phase 1: Bow
  stepDeg = STEP_MED;
  moveFoot(BL, LEAN_X + 5.0f, 115.0f);
  moveFoot(BR, LEAN_X + 5.0f, 115.0f);
  moveFoot(FL, LEAN_X - 5.0f, standZ - 18.0f);
  moveFoot(RF, LEAN_X - 5.0f, standZ - 18.0f);
  waitMs(2200);

  // Phase 2: Wave RF paw 3 times — PURE UP/DOWN, NO coxa/hip movement
  // Coxa stays locked at 90 (neutral) the entire time.
  // Only femur+tibia move to raise and lower the foot.
  // Slow speed for smooth, natural-looking wave.
  stepDeg = STEP_SLOW;

  // First: lift the paw straight up from bow position
  // coxaRight=0 keeps coxa at servo 90 (perfectly neutral, no inward/outward)
  moveFoot(RF, LEAN_X, standZ - 55.0f, 0.0f);   // raise paw high
  waitMs(1100);

  // Wave: 3 repetitions of down → up
  for (int i = 0; i < 3; i++) {
    moveFoot(RF, LEAN_X, standZ - 20.0f, 0.0f);  // lower (but still lifted)
    waitMs(900);
    moveFoot(RF, LEAN_X, standZ - 55.0f, 0.0f);  // raise again
    waitMs(900);
  }

  // Park RF back to front bow position before returning
  moveFoot(RF, LEAN_X - 5.0f, standZ - 18.0f, 0.0f);
  waitMs(800);

  // Phase 3: Slow return to stand
  motionState = ST_GREET_RETURN;
  stepDeg = STEP_SLOW;
  moveFoot(RF, LEAN_X, standZ, 0.0f);  waitMs(950);
  moveFoot(FL, LEAN_X, standZ, 0.0f);  waitMs(950);
  moveFoot(BR, LEAN_X, standZ, 0.0f);  waitMs(1300);
  moveFoot(BL, LEAN_X, standZ, 0.0f);  waitMs(1500);

  stepDeg = STEP_WALK;
  for (int leg = 0; leg < 4; leg++) {
    legX[leg] = LEAN_X;
    legZ[leg] = standZ;
  }
  motionState = ST_STAND;
  Serial.println(F("[GREET] Done."));
}

// =============================================================
//  MQ SENSOR READING
// =============================================================
//
//  MQ-135 (Air Quality) — analog on A0
//  Detects: CO2, NH3, benzene, smoke, alcohol, NOx
//  Raw value 0-1023 from ESP8266 ADC (0-1V mapped internally)
//  Higher raw value = worse air quality / more gas detected
//
//  MQ-7 (Carbon Monoxide) — digital on D5 (GPIO14)
//  DOUT: LOW  = CO detected above threshold (danger)
//        HIGH = CO below threshold (safe)
//  Note: MQ-7 module has onboard potentiometer to set threshold.
//        Adjust the pot until DOUT goes LOW only when CO is present.
//
//  IMPORTANT: Both MQ sensors need 24-48h burn-in time on first use
//  for stable readings. Pre-heat for at least 3 minutes each session.

void readSensors() {
  mq135_raw = analogRead(MQ135_PIN);           // 0-1023
  mq7_alert = (digitalRead(MQ7_PIN) == LOW);   // LOW = CO detected
}

// Converts MQ-135 raw ADC value to an estimated Air Quality Index (AQI-like)
// Maps 0-1023 → 0-500 (higher = worse, like standard AQI scale)
int rawToAQI(int raw) {
  return map(constrain(raw, 0, 1023), 0, 1023, 0, 500);
}

// Human-readable label for MQ-135 reading
// These thresholds are tuned for the MQ-135 at 5V with ESP8266 1V ADC input
const char* airQualityLabel(int raw) {
  if (raw < 150)  return "EXCELLENT";   // very clean indoor air
  if (raw < 280)  return "GOOD";        // normal indoor air
  if (raw < 420)  return "MODERATE";    // some pollutants, open a window
  if (raw < 600)  return "POOR";        // noticeable pollution, ventilate
  if (raw < 800)  return "VERY POOR";   // harmful, leave area
  return           "HAZARDOUS";         // dangerous, evacuate
}

// Short advice message based on air quality
const char* airQualityAdvice(int raw) {
  if (raw < 150)  return "Air is clean.";
  if (raw < 280)  return "Air is good.";
  if (raw < 420)  return "Open a window.";
  if (raw < 600)  return "Ventilate now.";
  if (raw < 800)  return "Leave the room!";
  return           "EVACUATE AREA!";
}

// =============================================================
//  WEB UI
// =============================================================
const char HTML_PAGE[] PROGMEM = R"rawHTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
<title>RobotDog v10</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Orbitron:wght@400;700;900&family=Share+Tech+Mono&display=swap');
  :root {
    --bg:#080810; --panel:#0d0d1a; --border:#1a2040;
    --accent:#00e5ff; --accent2:#ff6b00; --accent3:#7c4dff; --accent4:#00ff88;
    --glow:0 0 18px rgba(0,229,255,0.35);
    --text:#c0d0e8; --dim:#3a5070;
  }
  *{margin:0;padding:0;box-sizing:border-box;-webkit-tap-highlight-color:transparent;touch-action:none;}
  body{background:var(--bg);color:var(--text);font-family:'Share Tech Mono',monospace;
       min-height:100vh;display:flex;flex-direction:column;align-items:center;
       overflow:hidden;user-select:none;padding-bottom:8px;}
  body::before{content:'';position:fixed;inset:0;
    background-image:linear-gradient(rgba(0,229,255,0.025) 1px,transparent 1px),
                     linear-gradient(90deg,rgba(0,229,255,0.025) 1px,transparent 1px);
    background-size:36px 36px;pointer-events:none;z-index:0;}

  .header{position:relative;z-index:1;text-align:center;padding:8px 0 2px;}
  .header h1{font-family:'Orbitron',sans-serif;font-weight:900;font-size:1.25rem;
             color:var(--accent);text-shadow:var(--glow);letter-spacing:3px;}
  .header .sub{font-size:0.55rem;color:var(--dim);letter-spacing:2px;margin-top:2px;}

  #statusEl{position:relative;z-index:1;color:var(--accent);font-family:'Orbitron',sans-serif;
            font-weight:700;font-size:0.85rem;letter-spacing:2px;text-align:center;margin:4px 0;}

  /* Sensor panel */
  .sensor-panel{position:relative;z-index:1;margin:4px 8px;
    width:calc(100% - 16px);max-width:360px;
    background:var(--panel);border:1px solid var(--border);border-radius:10px;
    padding:8px 10px;display:flex;flex-direction:column;gap:6px;}
  .sensor-title{font-size:0.5rem;color:var(--dim);letter-spacing:2px;margin-bottom:2px;}
  .sensor-row{display:flex;align-items:center;gap:8px;}
  .sensor-icon{font-size:1.1rem;width:24px;text-align:center;flex-shrink:0;}
  .sensor-info{flex:1;display:flex;flex-direction:column;gap:1px;}
  .sensor-name{font-size:0.5rem;color:var(--dim);letter-spacing:1px;}
  .sensor-value{font-size:0.85rem;font-weight:bold;letter-spacing:1px;}
  .sensor-sub{font-size:0.55rem;color:var(--dim);}
  .sensor-badge{font-size:0.6rem;font-weight:bold;padding:3px 8px;border-radius:4px;
    letter-spacing:1px;border:1px solid;align-self:center;white-space:nowrap;}
  .divider{height:1px;background:var(--border);margin:0 -2px;}

  /* Control buttons */
  .ctrl-grid{position:relative;z-index:1;display:grid;
             grid-template-columns:1fr 1fr 1fr;
             grid-template-rows:auto auto;
             gap:6px;margin:6px 8px;width:calc(100% - 16px);max-width:360px;}
  .btn{background:var(--panel);border:1px solid var(--border);color:var(--text);
       font-family:'Share Tech Mono',monospace;font-size:0.7rem;padding:9px 4px;
       border-radius:6px;cursor:pointer;letter-spacing:1px;transition:all 0.15s;
       text-align:center;}
  .btn:active{transform:scale(0.93);}
  .btn.c-stand {border-color:var(--accent);  color:var(--accent);  box-shadow:0 0 8px rgba(0,229,255,0.2);}
  .btn.c-sit   {border-color:var(--accent3); color:var(--accent3); box-shadow:0 0 8px rgba(124,77,255,0.2);}
  .btn.c-greet {border-color:var(--accent2); color:var(--accent2); box-shadow:0 0 8px rgba(255,107,0,0.2);}
  .btn.c-hup   {border-color:var(--accent4); color:var(--accent4);}
  .btn.c-hdn   {border-color:#ff4060;        color:#ff4060;}
  #heightEl{position:relative;z-index:1;font-size:0.65rem;color:var(--accent3);
            text-align:center;letter-spacing:1px;margin-bottom:2px;}

  /* Joystick */
  .joy-area{position:relative;z-index:1;display:flex;flex-direction:column;align-items:center;margin-top:4px;}
  .joy-labels{position:relative;width:260px;height:260px;}
  .dir-label{position:absolute;font-size:0.55rem;color:var(--dim);letter-spacing:1px;}
  .dir-label.top   {top:2px;   left:50%;transform:translateX(-50%);}
  .dir-label.bottom{bottom:2px;left:50%;transform:translateX(-50%);}
  .dir-label.left  {left:2px;  top:50%; transform:translateY(-50%);}
  .dir-label.right {right:2px; top:50%; transform:translateY(-50%);}
  #joyBase{position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);
           width:210px;height:210px;border-radius:50%;
           background:radial-gradient(circle,#0d1525 0%,#080810 70%);
           border:2px solid var(--border);
           box-shadow:0 0 30px rgba(0,229,255,0.06),inset 0 0 20px rgba(0,0,0,0.5);
           cursor:pointer;touch-action:none;}
  #joyBase::before{content:'';position:absolute;inset:18px;border-radius:50%;border:1px solid rgba(0,229,255,0.08);}
  #joyBase::after {content:'';position:absolute;inset:40px;border-radius:50%;border:1px solid rgba(0,229,255,0.05);}
  #vector{position:absolute;bottom:50%;left:50%;width:2px;transform-origin:bottom center;
          background:linear-gradient(to top,var(--accent),transparent);
          opacity:0;border-radius:2px;pointer-events:none;margin-left:-1px;}
  #crossH,#crossV{position:absolute;background:rgba(0,229,255,0.07);pointer-events:none;}
  #crossH{width:100%;height:1px;top:50%;left:0;transform:translateY(-50%);}
  #crossV{height:100%;width:1px;left:50%;top:0;transform:translateX(-50%);}
  #joyKnob{position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);
           width:64px;height:64px;border-radius:50%;
           background:radial-gradient(circle at 35% 35%,#1a3060 0%,#0a1428 60%,#060c18 100%);
           border:2px solid var(--accent);
           box-shadow:0 0 16px rgba(0,229,255,0.3),inset 0 2px 6px rgba(255,255,255,0.05);
           pointer-events:none;z-index:2;}
  #joyKnob::after{content:'';position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);
                  width:8px;height:8px;border-radius:50%;background:var(--accent);box-shadow:0 0 8px var(--accent);}
  .version{position:fixed;bottom:4px;right:8px;font-size:0.45rem;color:var(--dim);z-index:1;}
</style>
</head>
<body>
<div class="header">
  <h1>ROBOT DOG</h1>
  <div class="sub">v10.0 · SALMAN FARIS · WiFi CONTROL</div>
</div>

<div id="statusEl">STAND BY</div>

<!-- Sensor Panel -->
<div class="sensor-panel">
  <div class="sensor-title">🌡 ENVIRONMENT MONITOR</div>

  <!-- MQ-135 Air Quality -->
  <div class="sensor-row">
    <div class="sensor-icon">🍃</div>
    <div class="sensor-info">
      <span class="sensor-name">AIR QUALITY (MQ-135)</span>
      <span class="sensor-value" id="aqValue" style="color:#00ff88">---</span>
      <span class="sensor-sub" id="aqAdvice">Warming up sensors...</span>
    </div>
    <div class="sensor-badge" id="aqBadge" style="color:#00ff88;border-color:#00ff88">---</div>
  </div>

  <!-- AQI bar -->
  <div style="background:#0d1525;border-radius:4px;height:6px;overflow:hidden;margin:-2px 0 2px;">
    <div id="aqiBar" style="height:100%;width:0%;border-radius:4px;
         background:linear-gradient(90deg,#00ff88,#ffcc00,#ff6b00,#ff2040);
         transition:width 1s ease;"></div>
  </div>

  <div class="divider"></div>

  <!-- MQ-7 CO -->
  <div class="sensor-row">
    <div class="sensor-icon">💨</div>
    <div class="sensor-info">
      <span class="sensor-name">CARBON MONOXIDE (MQ-7)</span>
      <span class="sensor-value" id="coValue" style="color:#00ff88">READING...</span>
      <span class="sensor-sub" id="coSub">Threshold set by onboard pot</span>
    </div>
    <div class="sensor-badge" id="coBadge" style="color:#00ff88;border-color:#00ff88">SAFE</div>
  </div>
</div>

<!-- Control Buttons: 3 cols × 2 rows -->
<div class="ctrl-grid">
  <button class="btn c-stand" onclick="sendCmd('s')">■ STAND</button>
  <button class="btn c-sit"   onclick="sendCmd('x')">⬇ SIT</button>
  <button class="btn c-greet" onclick="sendCmd('g')">🐾 GREET</button>
  <button class="btn c-hup"   onclick="sendCmd('u')">▲ HEIGHT+</button>
  <div id="heightEl" style="display:flex;align-items:center;justify-content:center;">H:2</div>
  <button class="btn c-hdn"   onclick="sendCmd('d')">▼ HEIGHT-</button>
</div>

<!-- Joystick -->
<div class="joy-area">
  <div class="joy-labels">
    <span class="dir-label top">FORWARD</span>
    <span class="dir-label bottom">BACKWARD</span>
    <span class="dir-label left">◄ LEFT</span>
    <span class="dir-label right">RIGHT ►</span>
    <div id="joyBase">
      <div id="crossH"></div>
      <div id="crossV"></div>
      <div id="vector"></div>
      <div id="joyKnob"></div>
    </div>
  </div>
</div>
<div class="version">RobotDog v10 · ESP8266</div>

<script>
const base=document.getElementById('joyBase'),
      knob=document.getElementById('joyKnob'),
      vector=document.getElementById('vector'),
      statusEl=document.getElementById('statusEl'),
      heightEl=document.getElementById('heightEl');

const RADIUS=82;
let originX=0,originY=0,active=false,jX=0,jY=0,sendTimer=null,heightLevel=2;

function processJoy(cx,cy){
  let dx=cx-originX,dy=cy-originY;
  let dist=Math.sqrt(dx*dx+dy*dy);
  let mag=Math.min(dist/RADIUS,1.0);
  if(dist>RADIUS){dx=dx/dist*RADIUS;dy=dy/dist*RADIUS;}
  knob.style.transform=`translate(calc(-50% + ${dx}px),calc(-50% + ${dy}px))`;
  if(dist>6){
    let angle=Math.atan2(dy,dx)*180/Math.PI+90;
    let len=Math.min(dist,RADIUS)-32;
    vector.style.height=Math.max(0,len)+'px';
    vector.style.transform=`rotate(${angle}deg)`;
    vector.style.opacity='0.7';
  } else { vector.style.opacity='0'; }
  jX=dx/RADIUS; jY=-(dy/RADIUS);
  let aX=Math.abs(jX),aY=Math.abs(jY);
  if(mag<0.10){
    statusEl.textContent='STAND BY';statusEl.style.color='var(--accent)';
  } else {
    let isSpot=mag>0.60&&aX>aY*1.3;
    if(isSpot){statusEl.textContent=jX>0?'↻ TURN R':'↺ TURN L';statusEl.style.color='#ff6b00';}
    else if(aY>aX*0.6){statusEl.textContent=jY>0?'▲ FORWARD':'▼ BACKWARD';statusEl.style.color='var(--accent)';}
    else{statusEl.textContent=jX>0?'↗ CURVE R':'↖ CURVE L';statusEl.style.color='#00ff88';}
  }
}

function resetKnob(){
  knob.style.transition='transform 0.18s cubic-bezier(0.34,1.56,0.64,1)';
  knob.style.transform='translate(-50%,-50%)';
  vector.style.opacity='0';
  setTimeout(()=>{knob.style.transition='none';},200);
  jX=0;jY=0;
  statusEl.textContent='STAND BY';statusEl.style.color='var(--accent)';
  sendJoy(0,0,0);
}

function getOrigin(){let r=base.getBoundingClientRect();originX=r.left+r.width/2;originY=r.top+r.height/2;}
function sendJoy(x,y,m){fetch(`/joy?x=${x.toFixed(3)}&y=${y.toFixed(3)}&m=${m.toFixed(3)}`).catch(()=>{});}
function sendCmd(cmd){
  fetch(`/cmd?c=${cmd}`).then(r=>r.text()).then(()=>{
    if(cmd==='u'){heightLevel=Math.min(4,heightLevel+1);heightEl.textContent=`H:${heightLevel}`;}
    if(cmd==='d'){heightLevel=Math.max(0,heightLevel-1);heightEl.textContent=`H:${heightLevel}`;}
    if(cmd==='s'){statusEl.textContent='STAND BY';statusEl.style.color='var(--accent)';}
    if(cmd==='x'){statusEl.textContent='SIT';statusEl.style.color='var(--accent3)';}
    if(cmd==='g'){statusEl.textContent='🐾 GREETING';statusEl.style.color='#ff6b00';}
  }).catch(()=>{});
}

// Sensor polling every 3s
function pollSensors(){
  fetch('/sensors').then(r=>r.json()).then(d=>{

    // ── MQ-135 Air Quality ─────────────────────────────
    const aqColors = {
      'EXCELLENT':'#00ff88', 'GOOD':'#00ff88',
      'MODERATE':'#ffcc00',  'POOR':'#ff8800',
      'VERY POOR':'#ff4400', 'HAZARDOUS':'#ff2040'
    };
    const col = aqColors[d.status] || '#00ff88';

    // AQI value display (0-500 scale, like standard AQI)
    document.getElementById('aqValue').textContent  = 'AQI  ' + d.aqi;
    document.getElementById('aqValue').style.color  = col;
    document.getElementById('aqAdvice').textContent = d.advice + '  (raw: ' + d.raw + '/1023)';
    document.getElementById('aqBadge').textContent  = d.status;
    document.getElementById('aqBadge').style.color       = col;
    document.getElementById('aqBadge').style.borderColor = col;

    // AQI progress bar (0-500 → 0-100%)
    document.getElementById('aqiBar').style.width = Math.min(d.aqi / 5, 100) + '%';

    // ── MQ-7 Carbon Monoxide ───────────────────────────
    if(d.co_alert){
      document.getElementById('coValue').textContent     = '⚠ CO DETECTED';
      document.getElementById('coValue').style.color     = '#ff2040';
      document.getElementById('coSub').textContent       = 'Above safe threshold — ventilate immediately!';
      document.getElementById('coBadge').textContent     = 'ALERT';
      document.getElementById('coBadge').style.color     = '#ff2040';
      document.getElementById('coBadge').style.borderColor = '#ff2040';
    } else {
      document.getElementById('coValue').textContent     = '✓ CO CLEAR';
      document.getElementById('coValue').style.color     = '#00ff88';
      document.getElementById('coSub').textContent       = 'Carbon monoxide below threshold';
      document.getElementById('coBadge').textContent     = 'SAFE';
      document.getElementById('coBadge').style.color     = '#00ff88';
      document.getElementById('coBadge').style.borderColor = '#00ff88';
    }

  }).catch(()=>{
    document.getElementById('aqAdvice').textContent = 'Sensor read failed — retrying...';
  });
}
setInterval(pollSensors, 3000);
pollSensors();

// Touch events
base.addEventListener('touchstart',e=>{e.preventDefault();active=true;getOrigin();
  clearInterval(sendTimer);
  sendTimer=setInterval(()=>{sendJoy(jX,jY,Math.min(Math.sqrt(jX*jX+jY*jY),1));},80);
},{passive:false});
base.addEventListener('touchmove',e=>{e.preventDefault();if(!active)return;processJoy(e.touches[0].clientX,e.touches[0].clientY);},{passive:false});
base.addEventListener('touchend',e=>{e.preventDefault();active=false;clearInterval(sendTimer);resetKnob();},{passive:false});
// Mouse events (for desktop testing)
base.addEventListener('mousedown',e=>{active=true;getOrigin();clearInterval(sendTimer);
  sendTimer=setInterval(()=>{sendJoy(jX,jY,Math.min(Math.sqrt(jX*jX+jY*jY),1));},80);});
window.addEventListener('mousemove',e=>{if(!active)return;processJoy(e.clientX,e.clientY);});
window.addEventListener('mouseup',()=>{if(!active)return;active=false;clearInterval(sendTimer);resetKnob();});
</script>
</body>
</html>
)rawHTML";

// =============================================================
//  WEB SERVER HANDLERS
// =============================================================
void handleRoot()   { server.send_P(200, "text/html", HTML_PAGE); }

void handleJoy() {
  if (server.hasArg("x") && server.hasArg("y") && server.hasArg("m")) {
    joyX   = constrain(server.arg("x").toFloat(), -1.0f, 1.0f);
    joyY   = constrain(server.arg("y").toFloat(), -1.0f, 1.0f);
    joyMag = constrain(server.arg("m").toFloat(),  0.0f, 1.0f);
  }
  server.send(200, "text/plain", "ok");
}

void handleCmd() {
  if (!server.hasArg("c")) { server.send(400, "text/plain", "no cmd"); return; }
  char cmd = server.arg("c").charAt(0);
  switch (cmd) {
    case 's': joyX=joyY=joyMag=0; gaitRunning=false; poseStand(motionState==ST_SIT); break;
    case 'x': joyX=joyY=joyMag=0; poseSit();   break;
    case 'u': if (hLevel < 4) { hLevel++; applyHeight(); } break;
    case 'd': if (hLevel > 0) { hLevel--; applyHeight(); } break;
    case 'g': joyX=joyY=joyMag=0; doGreeting(); break;
  }
  server.send(200, "text/plain", "ok");
}

void handleSensors() {
  int aqi = rawToAQI(mq135_raw);
  String json = "{";
  json += "\"raw\":";       json += mq135_raw;
  json += ",\"aqi\":";      json += aqi;
  json += ",\"status\":\""; json += airQualityLabel(mq135_raw); json += "\"";
  json += ",\"advice\":\""; json += airQualityAdvice(mq135_raw); json += "\"";
  json += ",\"co_alert\":"; json += mq7_alert ? "true" : "false";
  json += ",\"co_text\":\"";json += mq7_alert ? "CO DETECTED" : "CLEAR"; json += "\"";
  json += "}";
  server.send(200, "application/json", json);
}

// =============================================================
//  SETUP
// =============================================================
void setup() {
  Serial.begin(9600);

  // MQ sensor pins
  pinMode(MQ7_PIN, INPUT);     // D5 digital input for MQ-7 CO alert

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(200);

  for (int leg = 0; leg < 4; leg++) {
    for (int j = 0; j < 3; j++) {
      sPos[leg][j] = 90;
      sTgt[leg][j] = 90;
      sAct[leg][j] = false;
    }
    legX[leg] = LEAN_X;
    legZ[leg] = HEIGHT_Z[2];
  }

  standZ  = HEIGHT_Z[hLevel];
  stepDeg = STEP_WALK;
  poseStand(false);

  // Read sensors once at boot
  readSensors();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print(F("[WiFi] AP IP: "));
  Serial.println(WiFi.softAPIP());

  server.on("/",        handleRoot);
  server.on("/joy",     handleJoy);
  server.on("/cmd",     handleCmd);
  server.on("/sensors", handleSensors);
  server.begin();

  Serial.println(F("=== RobotDog v10.0 Ready ==="));
  Serial.println(F("WiFi: RobotDog / 12345678"));
  Serial.println(F("Browser: http://192.168.4.1"));
  Serial.println(F("Serial cmds: f b l r s x u d g h"));
}

// =============================================================
//  MAIN LOOP
// =============================================================
void loop() {
  static unsigned long lastLoop = 0;
  unsigned long now = millis();

  server.handleClient();
  yield();

  // Sensor polling (non-blocking, every SENSOR_INTERVAL)
  if (now - lastSensorRead > SENSOR_INTERVAL) {
    readSensors();
    lastSensorRead = now;
  }

  if (now - lastLoop < LOOP_MS) return;
  lastLoop = now;

  updateServos();

  if (motionState == ST_STAND || motionState == ST_WALK) {
    updateGaitFromJoystick();
    if (gaitRunning) stepGait();
  }

  // Serial debug commands
  if (Serial.available()) {
    char c = tolower(Serial.read());
    while (Serial.available()) Serial.read();
    switch (c) {
      case 'f': joyX=0;   joyY=1;  joyMag=0.75f; break;
      case 'b': joyX=0;   joyY=-1; joyMag=0.75f; break;
      case 'l': joyX=-1;  joyY=0;  joyMag=0.80f; break;
      case 'r': joyX=1;   joyY=0;  joyMag=0.80f; break;
      case 's': joyX=joyY=joyMag=0; gaitRunning=false; poseStand(motionState==ST_SIT); break;
      case 'x': joyX=joyY=joyMag=0; poseSit();   break;
      case 'u': if (hLevel < 4) { hLevel++; applyHeight(); } break;
      case 'd': if (hLevel > 0) { hLevel--; applyHeight(); } break;
      case 'g': joyX=joyY=joyMag=0; doGreeting(); break;
      case 'h':
        Serial.println(F("f=fwd  b=back  l=turn-L  r=turn-R"));
        Serial.println(F("s=stand  x=sit  u=H+  d=H-  g=greet"));
        break;
    }
  }
}
