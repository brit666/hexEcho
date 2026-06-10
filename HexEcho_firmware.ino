/*
 * ================================================================
 *  HexEcho Hexapod Firmware  —  v2 (with interpolation engine)
 * ================================================================
 * Hardware:
 *   Arduino Nano, 2x PCA9685 (0x40 / 0x41), HC-SR04, MG90S + SG90
 *
 * Gait: Wave gait (1 leg at a time) — most stable
 * Motion: All servo moves go through the interpolation engine.
 *         Servos never snap — they glide from current to target.
 *
 * ----------------------------------------------------------------
 * CHANNEL MAP
 * ----------------------------------------------------------------
 * PCA0 (0x40) — LEFT SIDE  legs 0,1,2  (Front / Mid / Back)
 *   Ch 0,1,2  = Leg0 Coxa, Femur, Tibia
 *   Ch 3,4,5  = Leg1 Coxa, Femur, Tibia
 *   Ch 6,7,8  = Leg2 Coxa, Femur, Tibia
 *
 * PCA1 (0x41) — RIGHT SIDE legs 3,4,5  (Front / Mid / Back)
 *   Ch 0,1,2  = Leg3 Coxa, Femur, Tibia
 *   Ch 3,4,5  = Leg4 Coxa, Femur, Tibia
 *   Ch 6,7,8  = Leg5 Coxa, Femur, Tibia
 *
 * Top-view layout (front = top):
 *      [FRONT]
 *   L0       R3
 *   L1       R4
 *   L2       R5
 *      [BACK]
 *
 * ----------------------------------------------------------------
 * HOME POSITION SUMMARY (what the code commands on boot)
 * ----------------------------------------------------------------
 *   Coxa  = 90°  (straight sideways)
 *   Femur = 60°  (tilted slightly forward-down for a standing pose)
 *   Tibia = 110° (angled to plant foot flat on ground)
 *
 * Mount each servo FIRST to these angles — see the mounting guide PDF.
 * ================================================================
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ================================================================
//  USER-CONFIGURABLE — edit to match your hardware
// ================================================================

// Leg segment lengths in mm (only used for reference / future IK)
#define COXA_LEN_MM    60.0f
#define FEMUR_LEN_MM   50.0f
#define TIBIA_LEN_MM  70.0f

// Home stance angles
#define COXA_HOME_DEG    90
#define FEMUR_HOME_DEG   60
#define TIBIA_HOME_DEG  110

// Gait geometry
#define STEP_COXA_SWING   25    // degrees coxa travels per step
#define LIFT_FEMUR_DEG    20    // degrees femur rises during swing
#define TURN_COXA_OFFSET  22    // extra coxa offset used during rotation

// Interpolation speed — lower = slower / smoother
// INTERP_STEP_DEG: how many degrees to advance per tick
// INTERP_TICK_MS : ms between each tick
// At 1 deg / 15 ms → a 30-degree move takes ~450 ms
#define INTERP_STEP_DEG    1    // degrees per tick (keep at 1 for smoothest)
#define INTERP_TICK_MS    12    // ms per tick

// Pauses between gait phases (on top of interpolation time)
#define PHASE_SETTLE_MS  120    // brief settle after each phase completes
#define STEP_SETTLE_MS   300    // pause after each leg finishes its step
#define CYCLE_SETTLE_MS  700    // pause after a full 6-leg cycle — SAFE POWER-OFF WINDOW

// HC-SR04 pins
#define TRIG_PIN   7
#define ECHO_PIN   8

// Obstacle distance threshold
#define OBSTACLE_CM   20

// PCA9685 pulse range — tune if servos don't reach 0° or 180°
#define PWM_MIN  150   // pulse count at 0°
#define PWM_MAX  600   // pulse count at 180°

// ================================================================
//  PCA9685 INSTANCES
// ================================================================
Adafruit_PWMServoDriver pca0 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pca1 = Adafruit_PWMServoDriver(0x41);

// ================================================================
//  LEG STRUCTURE
//  currentDeg[] = live angles (updated every interpolation tick)
//  targetDeg[]  = where we want to go
//  mirrorCoxa   = true for right-side legs (inverts coxa direction)
// ================================================================
struct Leg {
  uint8_t board;                // 0 = pca0, 1 = pca1
  uint8_t ch[3];                // channel indices: 0=coxa 1=femur 2=tibia
  float   currentDeg[3];        // current servo angle (float for smooth interp)
  int     targetDeg[3];         // target angle
  bool    mirrorCoxa;           // true = invert coxa for right-side legs
};

Leg legs[6];

// ================================================================
//  LOW-LEVEL SERVO WRITE
// ================================================================

uint16_t degToPulse(float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  return (uint16_t)map((long)(deg * 10), 0, 1800, PWM_MIN, PWM_MAX);
}

void writeServoRaw(uint8_t board, uint8_t channel, float deg) {
  uint16_t pulse = degToPulse(deg);
  if (board == 0) pca0.setPWM(channel, 0, pulse);
  else             pca1.setPWM(channel, 0, pulse);
}

// Flush current angles to hardware for one leg
void flushLeg(uint8_t li) {
  Leg& L = legs[li];
  float coxaDeg = L.mirrorCoxa ? (180.0f - L.currentDeg[0]) : L.currentDeg[0];
  writeServoRaw(L.board, L.ch[0], coxaDeg);
  writeServoRaw(L.board, L.ch[1], L.currentDeg[1]);
  writeServoRaw(L.board, L.ch[2], L.currentDeg[2]);
}

// ================================================================
//  INTERPOLATION ENGINE
//  Moves all 6 legs from their currentDeg toward their targetDeg
//  one INTERP_STEP_DEG tick at a time.
//  Returns only when all legs have reached their targets.
// ================================================================
void interpolateAllLegs() {
  bool moving = true;
  while (moving) {
    moving = false;
    for (uint8_t li = 0; li < 6; li++) {
      for (uint8_t j = 0; j < 3; j++) {
        float diff = legs[li].targetDeg[j] - legs[li].currentDeg[j];
        if (abs(diff) > 0.5f) {
          moving = true;
          float step = constrain(diff, -(float)INTERP_STEP_DEG, (float)INTERP_STEP_DEG);
          legs[li].currentDeg[j] += step;
        } else {
          legs[li].currentDeg[j] = (float)legs[li].targetDeg[j];
        }
      }
      flushLeg(li);
    }
    if (moving) delay(INTERP_TICK_MS);
  }
  // One final flush after motion completes — ensures all servos are exactly on target
  for (uint8_t li = 0; li < 6; li++) flushLeg(li);
}

// ================================================================
//  HOLD DELAY
//  Replaces all bare delay() calls during pauses.
//  Re-flushes every servo every HOLD_REFRESH_MS during the wait
//  so the PCA9685 keeps asserting PWM and servos don't go limp.
//  Also catches clone PCA9685 boards that occasionally lose state.
// ================================================================
#define HOLD_REFRESH_MS  20   // re-send PWM every 20ms during holds

void holdDelay(uint32_t ms) {
  uint32_t start = millis();
  while (millis() - start < ms) {
    for (uint8_t li = 0; li < 6; li++) flushLeg(li);
    delay(HOLD_REFRESH_MS);
  }
}

// Convenience: set target for one leg's joint (0=coxa,1=femur,2=tibia)
void setTarget(uint8_t li, uint8_t joint, int deg) {
  legs[li].targetDeg[joint] = constrain(deg, 0, 180);
}

// Set all three joints of one leg at once
void setLegTarget(uint8_t li, int coxaDeg, int femurDeg, int tibiaDeg) {
  legs[li].targetDeg[0] = constrain(coxaDeg,  0, 180);
  legs[li].targetDeg[1] = constrain(femurDeg, 0, 180);
  legs[li].targetDeg[2] = constrain(tibiaDeg, 0, 180);
}

// ================================================================
//  LEG TABLE INIT
// ================================================================
void initLegsTable() {
  // Helper lambda-style inline — sets a leg entry
  // Left legs (0,1,2): mirrorCoxa = false
  // Right legs (3,4,5): mirrorCoxa = true
  struct LegInit { uint8_t board; uint8_t c, f, t; bool mirror; };
  const LegInit cfg[6] = {
    {0,  0, 1, 2,  false},  // Leg0 L-Front
    {0,  3, 4, 5,  false},  // Leg1 L-Mid
    {0,  6, 7, 8,  false},  // Leg2 L-Back
    {1,  0, 1, 2,  true },  // Leg3 R-Front
    {1,  3, 4, 5,  true },  // Leg4 R-Mid
    {1,  6, 7, 8,  true },  // Leg5 R-Back
  };
  for (uint8_t i = 0; i < 6; i++) {
    legs[i].board       = cfg[i].board;
    legs[i].ch[0]       = cfg[i].c;
    legs[i].ch[1]       = cfg[i].f;
    legs[i].ch[2]       = cfg[i].t;
    legs[i].mirrorCoxa  = cfg[i].mirror;
    // Start current and target both at home
    legs[i].currentDeg[0] = COXA_HOME_DEG;
    legs[i].currentDeg[1] = FEMUR_HOME_DEG;
    legs[i].currentDeg[2] = TIBIA_HOME_DEG;
    legs[i].targetDeg[0]  = COXA_HOME_DEG;
    legs[i].targetDeg[1]  = FEMUR_HOME_DEG;
    legs[i].targetDeg[2]  = TIBIA_HOME_DEG;
  }
}

// ================================================================
//  HOME POSITION
//  Moves legs to home one at a time so the bot doesn't lurch.
// ================================================================
void goHome() {
  Serial.println(F("[HOME] Moving to home position..."));
  for (uint8_t i = 0; i < 6; i++) {
    setLegTarget(i, COXA_HOME_DEG, FEMUR_HOME_DEG, TIBIA_HOME_DEG);
    interpolateAllLegs();
    holdDelay(PHASE_SETTLE_MS);
  }
  holdDelay(CYCLE_SETTLE_MS);
  Serial.println(F("[HOME] Done."));
}

// ================================================================
//  SINGLE LEG STEP  (lift → swing → plant)
//  direction : +1 = forward, -1 = backward
//  coxaExtra : extra coxa offset for turning
//
//  While one leg moves, the others stay put (wave gait).
// ================================================================
void stepLeg(uint8_t li, int direction, int coxaExtra) {
  int targetCoxa = COXA_HOME_DEG + (direction * STEP_COXA_SWING) + coxaExtra;
  targetCoxa = constrain(targetCoxa, 45, 135);

  // --- Phase 1: Lift ---
  setLegTarget(li, COXA_HOME_DEG,
                   FEMUR_HOME_DEG - LIFT_FEMUR_DEG,
                   TIBIA_HOME_DEG);
  interpolateAllLegs();
  holdDelay(PHASE_SETTLE_MS);

  // --- Phase 2: Swing ---
  setLegTarget(li, targetCoxa,
                   FEMUR_HOME_DEG - LIFT_FEMUR_DEG,
                   TIBIA_HOME_DEG);
  interpolateAllLegs();
  holdDelay(PHASE_SETTLE_MS);

  // --- Phase 3: Plant ---
  setLegTarget(li, targetCoxa,
                   FEMUR_HOME_DEG,
                   TIBIA_HOME_DEG);
  interpolateAllLegs();
  holdDelay(STEP_SETTLE_MS);  // *** SAFE POWER-OFF POINT — leg planted, stable ***
}

// ================================================================
//  RESET ALL COXAS TO HOME  (after a full gait cycle)
// ================================================================
void resetAllCoxas() {
  for (uint8_t i = 0; i < 6; i++) {
    setTarget(i, 0, COXA_HOME_DEG);
  }
  interpolateAllLegs();
  holdDelay(PHASE_SETTLE_MS);
}

// ================================================================
//  WAVE GAIT — FORWARD / BACKWARD
//  Sequence: L-Front → R-Front → L-Mid → R-Mid → L-Back → R-Back
//  (alternates sides to keep weight balanced)
// ================================================================
const uint8_t WAVE_ORDER[6] = {0, 3, 1, 4, 2, 5};

void waveGait(int direction) {
  for (uint8_t i = 0; i < 6; i++) {
    stepLeg(WAVE_ORDER[i], direction, 0);
  }
  resetAllCoxas();
  holdDelay(CYCLE_SETTLE_MS);  // *** SAFE POWER-OFF WINDOW ***
}

void stepForward()  { Serial.println(F("[GAIT] Forward"));  waveGait(+1); }
void stepBackward() { Serial.println(F("[GAIT] Backward")); waveGait(-1); }

// ================================================================
//  ROTATION  (turn in place)
//  rotDir: +1 = left, -1 = right
//
//  Left turn:  left legs push back, right legs push forward
//  Right turn: left legs push forward, right legs push back
// ================================================================
void rotateStep(int rotDir) {
  for (uint8_t i = 0; i < 6; i++) {
    uint8_t li = WAVE_ORDER[i];
    // Legs 0,1,2 = left → opposite direction to rotDir
    // Legs 3,4,5 = right → same direction as rotDir
    int legDir     = (li < 3) ? (-rotDir) : (rotDir);
    int coxaExtra  = TURN_COXA_OFFSET * legDir;
    stepLeg(li, legDir, coxaExtra);
  }
  resetAllCoxas();
  holdDelay(CYCLE_SETTLE_MS);
}

void turnLeft()  { Serial.println(F("[TURN] Left"));  rotateStep(+1); }
void turnRight() { Serial.println(F("[TURN] Right")); rotateStep(-1); }

// ================================================================
//  HC-SR04 SENSOR
// ================================================================
float getDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(4);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 26000UL); // timeout ~4.5m
  if (dur == 0) return 999.0f;
  return (dur * 0.0343f) / 2.0f;
}

bool obstacleAhead() {
  float d = getDistanceCm();
  Serial.print(F("[SENSOR] "));
  Serial.print(d, 1);
  Serial.println(F(" cm"));
  return (d < OBSTACLE_CM);
}

// ================================================================
//  OBSTACLE AVOIDANCE
//
//  Logic:
//    1. Clear ahead  → step forward
//    2. Blocked      → try LEFT
//    3. Still blocked → turn RIGHT x2 (undo left, then face right)
//    4. Still blocked → turn RIGHT once more (~facing backward)
//    5. Still blocked → go home, halt (safe power-off)
// ================================================================
void obstacleAvoidanceLoop() {
  Serial.println(F("--- Checking ahead ---"));

  if (!obstacleAhead()) {
    stepForward();
    return;
  }

  Serial.println(F("[NAV] Obstacle! Trying LEFT..."));
  turnLeft();
  if (!obstacleAhead()) { stepForward(); return; }

  Serial.println(F("[NAV] Still blocked. Turning RIGHT x2..."));
  turnRight();   // undo left → back to original heading
  turnRight();   // now facing right
  if (!obstacleAhead()) { stepForward(); return; }

  Serial.println(F("[NAV] Still blocked. Turning RIGHT once more..."));
  turnRight();   // now facing ~180° from original
  if (!obstacleAhead()) { stepForward(); return; }

  Serial.println(F("[NAV] ALL DIRECTIONS BLOCKED. Returning home and halting."));
  goHome();
  while (true) holdDelay(500); // keeps servo PWM alive at home position
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(9600);
  Serial.println(F("=== HexEcho booting ==="));

  Wire.begin();

  pca0.begin(); pca0.setPWMFreq(50);
  pca1.begin(); pca1.setPWMFreq(50);
  delay(200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  initLegsTable();

  // Flush initial angles to hardware immediately (no snap — already at home)
  for (uint8_t i = 0; i < 6; i++) flushLeg(i);
  holdDelay(300);

  // Smooth move to home (in case servos powered up in unknown position)
  goHome();

  Serial.println(F("=== Ready ==="));
  holdDelay(1000);
}

// ================================================================
//  MAIN LOOP
// ================================================================
void loop() {
  obstacleAvoidanceLoop();
  holdDelay(CYCLE_SETTLE_MS);
}
