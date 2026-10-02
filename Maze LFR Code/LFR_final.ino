// ============================================================
// WHITE LINE FOLLOWER — FINAL
// Hardware : Arduino Uno/Nano + TB6612FNG + 16-ch IR sensor
//            array via CD74HC4067 MUX
//
// Changelog from v3:
//  F1. rampTo(target, step, stepDelayMs) — handles up AND down,
//      removes the "wrong semantics" bug when called from junction
//  F2. Guard in rampTo: no speed increase when already lower
//      (removed, covered by bidirectional rampTo)
//  F3. Calibration: original 4s single-direction spin (unchanged)
//  F4. Calibration: warns via Serial if spread < 100 (unchanged)
//  F5. Turn entry: rampTo(TURN_SPEED, 3, 1)  → ~30ms
//  F6. Junction entry: rampTo(JUNCTION_SPEED, 2, 1) → ~55ms
//  F7. SENSOR_THRESHOLD #define — easy to adjust
//  F8. Removed redundant setup()/loop() forward declarations
//  F9. DEBUG_SERIAL #define — set 1 to enable Serial Plotter
//      output during PID follow
// ============================================================

// -------- AVR BIT-MANIPULATION HELPERS --------
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// -------- SENSOR PATTERN MASKS --------
#define ALL_LINE  ((uint16_t)0xFFFF)   // all 16 sensors detecting line
#define NO_LINE   ((uint16_t)0x0000)   // no sensors detecting line

// ============================================================
// USER-ADJUSTABLE CONFIGURATION — start here when tuning
// ============================================================

// Set 1 to stream error/PID/lsp/rsp to Serial Plotter
#define DEBUG_SERIAL        0

// Line colour: true = white line on black surface
//              false = black line on white surface
#define FOLLOW_WHITE_LINE   true

// Detection threshold after calibrated map (0–1000 scale)
// Raise to 600 if sensors are noisy / ambient light is high
#define SENSOR_THRESHOLD    500

// -------- PID GAINS --------
// Tune Kp first (Ki=0, Kd=0), then Kd, then tiny Ki if drift remains.
// With ±15 weight scale, max centroid error ≈ ±15.
//   Kp = 1.8  →  max P correction = 27 PWM steps    (gentle)
//   Kd = 8.0  →  max D correction = 120 PWM steps   (strong damp)
//   Ki = 0.003 → integral saturates at 300 → 0.9 PWM (tiny trim)
#define KP   1.8f
#define KI   0.003f
#define KD   8.0f

// -------- SPEED CONSTANTS (PWM 0-255) --------
#define LF_SPEED          180   // cruise speed
#define TURN_SPEED         90   // in-place pivot speed
#define TURN_SETTLE        45   // post-turn slow creep (anti-overshoot)
#define JUNCTION_SPEED     65   // forward creep at junction
#define RAMP_STEP           1   // speed increment per main-loop tick

// -------- RAMP PARAMETERS --------
// rampTo(target, RAMP_TURN_STEP, 1)    → 30 steps × 1ms = ~30ms
// rampTo(target, RAMP_JCT_STEP,  1)    → 57 steps × 1ms = ~57ms
#define RAMP_TURN_STEP      3
#define RAMP_JCT_STEP       2

// -------- INTEGRAL WINDUP CLAMP --------
#define I_MAX   300.0f
#define I_MIN  -300.0f

// ============================================================
// PIN ASSIGNMENTS
// ============================================================

// TB6612FNG motor driver
#define IN1    5
#define IN2    4
#define IN3    7
#define IN4    8
#define enA    3
#define enB    9
#define STBY   6

// Indicators / user input
#define LED    2
#define BUTTON 10

// CD74HC4067 MUX select lines
#define S0   A1
#define S1   A2
#define S2   A3
#define S3   A4

// Shared analog input from MUX
#define SENSOR_PIN  A0
#define NUM_SENSORS 16

// ============================================================
// SENSOR WEIGHTS
// Symmetric ±15 scale.
//   Sensor  0 = far left  → weight +15
//   Sensor 15 = far right → weight -15
// PID error > 0 → line is LEFT of centre → steer left
// PID error < 0 → line is RIGHT of centre → steer right
// ============================================================
const int sensorWeight[NUM_SENSORS] = {
  15, 13, 11,  9,  7,  5,  3,  1,
  -1, -3, -5, -7, -9, -11, -13, -15
};

// ============================================================
// RUNTIME VARIABLES
// ============================================================

// Calibration bounds
int minValues[NUM_SENSORS];
int maxValues[NUM_SENSORS];

// Per-cycle sensor data
int  sensorValue[NUM_SENSORS];   // 0–1000 mapped value
bool sensorArray[NUM_SENSORS];   // true = line detected on this sensor

// PID state
float errorVal  = 0.0f;
float prevError = 0.0f;
float integral  = 0.0f;
float PIDvalue  = 0.0f;

// Speed and control
int      currentSpeed = 30;
bool     onLine       = false;
int      lsp, rsp;
uint16_t pattern      = 0;

// ============================================================
// JUNCTION TYPE
// ============================================================
typedef enum {
  JT_END,      // no exit — dead end or true endpoint
  JT_T_FWD,    // forward path only (side branches irrelevant / PID takes over)
  JT_T_LEFT,   // only a left branch after the bar
  JT_T_RIGHT,  // only a right branch after the bar
  JT_CROSS     // forward + at least one side branch
} JunctionType;

// ============================================================
// FORWARD DECLARATIONS
// (setup/loop excluded — Arduino doesn't need them)
// ============================================================
void          calibrate();
void          readLine();
void          buildPattern();
JunctionType  classifyJunction();
void          handleJunction();
void          handleSharpTurn(bool turnLeft);
void          lineFollow();
void          recoverLine();
void          rampTo(int targetSpeed, int step, int stepDelayMs);
void          resetPID();
void          stopMotors();
void          motor1run(int spd);
void          motor2run(int spd);
int           sensorRead(int ch);
bool          centerOnLine();
bool          leftSideOn();
bool          rightSideOn();
bool          forwardPathOn();

// ============================================================
// SETUP
// ============================================================
void setup() {
  // ADC prescaler = 32 → ~500 kHz ADC clock
  // Best balance of speed and 10-bit accuracy for IR sensors
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  sbi(ADCSRA, ADPS0);

  Serial.begin(115200);

  // Motor driver
  pinMode(IN1,    OUTPUT);
  pinMode(IN2,    OUTPUT);
  pinMode(IN3,    OUTPUT);
  pinMode(IN4,    OUTPUT);
  pinMode(enA,    OUTPUT);
  pinMode(enB,    OUTPUT);
  pinMode(STBY,   OUTPUT);
  digitalWrite(STBY, HIGH);   // enable driver immediately

  // UI
  pinMode(LED,    OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);

  // MUX
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  // ---- BUTTON 1: start calibration ----
  digitalWrite(LED, HIGH);
  Serial.println(F("Press BUTTON to calibrate."));
  while (digitalRead(BUTTON) == HIGH) {}
  delay(60);                   // debounce
  digitalWrite(LED, LOW);

  calibrate();

  // ---- BUTTON 2: start run ----
  digitalWrite(LED, HIGH);
  Serial.println(F("Calibration done. Press BUTTON to start."));
  while (digitalRead(BUTTON) == HIGH) {}
  delay(60);
  digitalWrite(LED, LOW);

  resetPID();
  Serial.println(F("Running."));
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  readLine();
  buildPattern();

  // ----------------------------------------------------------
  // Priority 1: ALL sensors on white → junction or endpoint
  // ----------------------------------------------------------
  if (pattern == ALL_LINE) {
    handleJunction();
    return;
  }

  // ----------------------------------------------------------
  // Priority 2: SHARP TURN detection
  //   Left  mask: sensors 0-4  lit  →  line far left  → turn left
  //   Right mask: sensors 11-15 lit →  line far right → turn right
  //   Both at once → treat as junction (wide crossing bar)
  // ----------------------------------------------------------
  const bool sharpLeft  = (pattern & 0b1111100000000000u)
                          == 0b1111100000000000u;
  const bool sharpRight = (pattern & 0b0000000000011111u)
                          == 0b0000000000011111u;

  if (sharpLeft && sharpRight) {
    handleJunction();
    return;
  }
  if (sharpLeft)  { handleSharpTurn(true);  return; }
  if (sharpRight) { handleSharpTurn(false); return; }

  // ----------------------------------------------------------
  // Priority 3: Normal PID line follow
  // ----------------------------------------------------------
  if (currentSpeed < LF_SPEED) currentSpeed += RAMP_STEP;

  if (onLine) {
    lineFollow();
  } else {
    recoverLine();   // blocking — only reached on genuine line loss
  }
}

// ============================================================
// BUILD 16-BIT PATTERN
// Bit 15 = sensor 0 (leftmost), bit 0 = sensor 15 (rightmost)
// ============================================================
void buildPattern() {
  pattern = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    pattern = (pattern << 1) | (sensorArray[i] ? 1u : 0u);
  }
}

// ============================================================
// SENSOR ZONE HELPERS
// Three non-overlapping zones for junction classification.
// A fourth wider zone (forwardPathOn) spans both centre zones.
//
//   Outer-left  : sensors  0-4  (leftSideOn)
//   Centre      : sensors  6-9  (centerOnLine) ← turn-completion check
//   Fwd band    : sensors  5-10 (forwardPathOn) ← junction classification
//   Outer-right : sensors 11-15 (rightSideOn)
//
// Sensors 5 and 10 are in forwardPathOn but not centerOnLine
// — wider net for classification, stricter check for turn done.
// ============================================================
bool centerOnLine() {
  return sensorArray[6] || sensorArray[7] ||
         sensorArray[8] || sensorArray[9];
}

bool leftSideOn() {
  return sensorArray[0] || sensorArray[1] || sensorArray[2] ||
         sensorArray[3] || sensorArray[4];
}

bool rightSideOn() {
  return sensorArray[11] || sensorArray[12] || sensorArray[13] ||
         sensorArray[14] || sensorArray[15];
}

bool forwardPathOn() {
  return sensorArray[5]  || sensorArray[6]  || sensorArray[7] ||
         sensorArray[8]  || sensorArray[9]  || sensorArray[10];
}

// ============================================================
// JUNCTION CLASSIFICATION
// Called after the robot has crept past the junction bar.
// sensorArray must be fresh (readLine already called).
//
// Decision table:
//   Fwd   Left  Right  → Type
//   ---   ----  -----  -------
//    0     0     0     JT_END
//    1     *     *     JT_T_FWD  (forward exists, PID handles sides)
//    0     1     0     JT_T_LEFT
//    0     0     1     JT_T_RIGHT
//    0     1     1     JT_CROSS  (no fwd, both sides → U-turn or cross)
// ============================================================
JunctionType classifyJunction() {
  const bool fwd   = forwardPathOn();
  const bool left  = leftSideOn();
  const bool right = rightSideOn();

  if (!fwd && !left && !right) return JT_END;
  if ( fwd)                    return JT_T_FWD;   // forward always wins
  if ( left && !right)         return JT_T_LEFT;
  if (!left &&  right)         return JT_T_RIGHT;
  return JT_CROSS;   // both sides, no forward
}

// ============================================================
// JUNCTION / ENDPOINT HANDLER
// ============================================================
void handleJunction() {

  // Step 1: smooth deceleration to junction creep speed
  rampTo(JUNCTION_SPEED, RAMP_JCT_STEP, 1);

  // Step 2: sensor-gated creep past the cross-bar
  // Breaks as soon as pattern is no longer ALL_LINE,
  // or after 200ms max (prevents overshoot on narrow lines).
  motor1run(JUNCTION_SPEED);
  motor2run(JUNCTION_SPEED);

  unsigned long creepStart = millis();
  while (millis() - creepStart < 200UL) {
    readLine();
    buildPattern();
    if (pattern != ALL_LINE) break;
  }
  stopMotors();
  delay(40);

  // Step 3: fresh read for classification
  readLine();
  buildPattern();

  // ---- ENDPOINT: still ALL_LINE after creeping ----
  // Robot did not clear the bar → true endpoint.
  if (pattern == ALL_LINE) {
    stopMotors();
    digitalWrite(LED, HIGH);
    Serial.println(F("[JCT] Endpoint."));

    unsigned long blinkT = millis();
    while (true) {
      readLine();
      // Resume when centre is on line AND outer zones are clear
      if (centerOnLine() && !leftSideOn() && !rightSideOn()) {
        digitalWrite(LED, LOW);
        resetPID();
        Serial.println(F("[JCT] Resuming from endpoint."));
        return;
      }
      stopMotors();
      if (millis() - blinkT > 800UL) {
        blinkT = millis();
        digitalWrite(LED, !digitalRead(LED));
      }
    }
  }

  // ---- Classify what's ahead ----
  JunctionType jt = classifyJunction();

  switch (jt) {

    // --------------------------------------------------------
    case JT_END:
    // No line visible in any direction — dead end or line gap.
    // Try LEFT 180° then RIGHT 180°.
    // --------------------------------------------------------
      Serial.println(F("[JCT] Dead end — searching."));
      stopMotors();
      delay(40);

      // Search LEFT (up to 1500ms ≈ 270° for most robots at 100 PWM)
      motor1run(-100);
      motor2run( 100);
      {
        unsigned long t = millis();
        while (millis() - t < 1500UL) {
          readLine();
          if (centerOnLine()) {
            stopMotors(); delay(30);
            resetPID();
            Serial.println(F("[JCT] Line found (left)."));
            return;
          }
        }
      }
      stopMotors();
      delay(40);

      // Search RIGHT (up to 1500ms)
      motor1run( 100);
      motor2run(-100);
      {
        unsigned long t = millis();
        while (millis() - t < 1500UL) {
          readLine();
          if (centerOnLine()) {
            stopMotors(); delay(30);
            resetPID();
            Serial.println(F("[JCT] Line found (right)."));
            return;
          }
        }
      }

      // Truly lost
      stopMotors();
      Serial.println(F("[JCT] Line not found — halting."));
      digitalWrite(LED, HIGH);
      while (true) {}
      break;

    // --------------------------------------------------------
    case JT_T_FWD:
    // Forward path exists. Hand back to PID immediately.
    // --------------------------------------------------------
      Serial.println(F("[JCT] T-fwd — straight ahead."));
      resetPID();
      break;

    // --------------------------------------------------------
    case JT_T_LEFT:
    // Left branch only. Pivot left to centre on it.
    // handleSharpTurn will rampTo(TURN_SPEED) — since
    // currentSpeed is already at JUNCTION_SPEED (< TURN_SPEED)
    // rampTo sets it directly without over-speeding.
    // --------------------------------------------------------
      Serial.println(F("[JCT] T-left — pivoting left."));
      handleSharpTurn(true);
      break;

    // --------------------------------------------------------
    case JT_T_RIGHT:
    // Right branch only. Pivot right.
    // --------------------------------------------------------
      Serial.println(F("[JCT] T-right — pivoting right."));
      handleSharpTurn(false);
      break;

    // --------------------------------------------------------
    case JT_CROSS:
    // Both side branches, no forward.
    // Left-hand rule: take LEFT by default.
    // --------------------------------------------------------
      Serial.println(F("[JCT] Cross (no fwd) — taking left."));
      handleSharpTurn(true);
      break;

    default:
      resetPID();
      break;
  }
}

// ============================================================
// SHARP TURN HANDLER
// turnLeft = true  → left motor back,  right motor forward
// turnLeft = false → left motor forward, right motor back
//
// Sequence:
//  1. Smooth ramp down to TURN_SPEED
//  2. Spin in-place until centre sensors reacquire line (1s max)
//  3. Slow settle creep at TURN_SETTLE for 35ms (reduce overshoot)
//  4. Hard stop → resetPID
// ============================================================
void handleSharpTurn(bool turnLeft) {
  Serial.print(F("[TURN] "));
  Serial.println(turnLeft ? F("LEFT") : F("RIGHT"));

  // Step 1: ramp to turn speed
  // rampTo handles the case where currentSpeed is already below
  // TURN_SPEED (called from junction handler) — it sets directly.
  rampTo(TURN_SPEED, RAMP_TURN_STEP, 1);

  // Step 2: pivot
  if (turnLeft) {
    motor1run(-TURN_SPEED);
    motor2run( TURN_SPEED);
  } else {
    motor1run( TURN_SPEED);
    motor2run(-TURN_SPEED);
  }

  unsigned long tStart = millis();
  while (millis() - tStart < 1000UL) {
    readLine();
    if (centerOnLine()) break;
  }

  // Step 3: slow settle (momentum bleed-off)
  if (turnLeft) {
    motor1run(-TURN_SETTLE);
    motor2run( TURN_SETTLE);
  } else {
    motor1run( TURN_SETTLE);
    motor2run(-TURN_SETTLE);
  }
  delay(35);

  // Step 4: stop and reset
  stopMotors();
  delay(20);
  resetPID();
}

// ============================================================
// LOST LINE RECOVERY
// Phase 0: straight creep 150ms — bridges small track gaps
// Phase 1: slow spin toward last error direction (800ms)
// Phase 2: faster reverse spin (1000ms)
// Phase 3: hard halt
// ============================================================
void recoverLine() {

  // Phase 0: try bridging a gap by going straight
  motor1run(80);
  motor2run(80);
  {
    unsigned long t = millis();
    while (millis() - t < 150UL) {
      readLine();
      if (onLine) { stopMotors(); delay(20); resetPID(); return; }
    }
  }

  // spinDir: +1 = spin left, -1 = spin right
  // prevError > 0 → line was left → robot drifted right → spin left
  int spinDir = (prevError > 0) ? 1 : -1;

  // Phase 1: slow spin toward last-known line direction
  motor1run(-110 * spinDir);
  motor2run( 110 * spinDir);
  {
    unsigned long t = millis();
    while (millis() - t < 800UL) {
      readLine();
      if (onLine) { stopMotors(); delay(20); resetPID(); return; }
    }
  }

  // Phase 2: faster reverse spin (opposite direction)
  motor1run( 150 * spinDir);
  motor2run(-150 * spinDir);
  {
    unsigned long t = millis();
    while (millis() - t < 1000UL) {
      readLine();
      if (onLine) { stopMotors(); delay(20); resetPID(); return; }
    }
  }

  // Phase 3: truly lost — halt
  stopMotors();
  Serial.println(F("[RECOVER] Line lost — halting."));
  digitalWrite(LED, HIGH);
  while (true) {}
}

// ============================================================
// SMOOTH SPEED RAMP
// Moves currentSpeed toward targetSpeed in increments of `step`
// with `stepDelayMs` ms between each step.
// Works for BOTH ramp-down AND ramp-up.
// Motors are updated at each step so the robot actually follows
// the speed change rather than driving open-loop.
// ============================================================
void rampTo(int targetSpeed, int step, int stepDelayMs) {
  if (step <= 0) step = 1;

  while (currentSpeed != targetSpeed) {
    if (currentSpeed > targetSpeed) {
      currentSpeed -= step;
      if (currentSpeed < targetSpeed) currentSpeed = targetSpeed;
    } else {
      currentSpeed += step;
      if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
    }
    motor1run(currentSpeed);
    motor2run(currentSpeed);
    delay(stepDelayMs);
  }
}

// ============================================================
// PID LINE FOLLOW
// Pure position centroid — sensorValue magnitude not used,
// only sensorArray (threshold-gated binary).
//
// error > 0 → line left of centre → lsp reduced, rsp increased
//           → robot steers left to chase line ✓
// error < 0 → line right of centre → robot steers right ✓
// ============================================================
void lineFollow() {
  float weightedSum = 0.0f;
  int   active      = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorArray[i]) {
      weightedSum += (float)sensorWeight[i];
      active++;
    }
  }

  // If any sensors active: update error.
  // If none (transient gap): hold last error so PID doesn't snap to 0.
  if (active > 0) {
    errorVal = weightedSum / (float)active;
  }

  // PID terms
  const float P = errorVal;
  integral = constrain(integral + errorVal, I_MIN, I_MAX);
  const float D = errorVal - prevError;

  PIDvalue  = (KP * P) + (KI * integral) + (KD * D);
  prevError = errorVal;

  lsp = currentSpeed - (int)PIDvalue;
  rsp = currentSpeed + (int)PIDvalue;

  // Clamp: -200 allows significant inner-wheel reverse on tight bends
  // Upper 255 is the PWM ceiling
  lsp = constrain(lsp, -200, 255);
  rsp = constrain(rsp, -200, 255);

  motor1run(lsp);
  motor2run(rsp);

#if DEBUG_SERIAL
  // Compatible with Arduino Serial Plotter (CSV format)
  Serial.print(errorVal, 2); Serial.print(',');
  Serial.print(PIDvalue, 2); Serial.print(',');
  Serial.print(lsp);         Serial.print(',');
  Serial.println(rsp);
#endif
}

// ============================================================
// CALIBRATION
// Spins robot over the line for 4 seconds to record min/max
// for each sensor. Warns via Serial if spread is too narrow.
// ============================================================
void calibrate() {
  Serial.println(F("Calibrating..."));

  // Seed min/max with the first reading
  for (int i = 0; i < NUM_SENSORS; i++) {
    int v        = sensorRead(i);
    minValues[i] = v;
    maxValues[i] = v;
  }

  // Spin in one direction for 4 seconds, sampling continuously
  unsigned long t = millis();
  while (millis() - t < 4000UL) {
    motor1run(70);
    motor2run(-70);
    for (int i = 0; i < NUM_SENSORS; i++) {
      int v = sensorRead(i);
      if (v < minValues[i]) minValues[i] = v;
      if (v > maxValues[i]) maxValues[i] = v;
    }
  }
  stopMotors();
  delay(200);

  // Warn if any sensor didn't see a meaningful range
  for (int i = 0; i < NUM_SENSORS; i++) {
    int spread = maxValues[i] - minValues[i];
    if (spread < 100) {
      Serial.print(F("WARNING: Sensor "));
      Serial.print(i);
      Serial.print(F(" low spread: "));
      Serial.println(spread);
    }
  }

  Serial.println(F("Calibration complete."));
}

// ============================================================
// READ ALL SENSORS INTO sensorValue[] AND sensorArray[]
// ============================================================
void readLine() {
  onLine = false;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int raw = sensorRead(i);

#if FOLLOW_WHITE_LINE
    // White line = higher reflectance = higher ADC = map → high value
    sensorValue[i] = map(raw, minValues[i], maxValues[i], 0, 1000);
#else
    // Black line = lower reflectance = lower ADC = map → high value
    sensorValue[i] = map(raw, minValues[i], maxValues[i], 1000, 0);
#endif

    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    sensorArray[i] = (sensorValue[i] > SENSOR_THRESHOLD);
    if (sensorArray[i]) onLine = true;
  }
}

// ============================================================
// RESET PID STATE AND SPEED
// Call after every junction, turn, or recovery event.
// ============================================================
void resetPID() {
  errorVal     = 0.0f;
  prevError    = 0.0f;
  integral     = 0.0f;
  PIDvalue     = 0.0f;
  currentSpeed = 30;
}

// ============================================================
// STOP MOTORS — active short-brake (both IN HIGH, full PWM)
// TB6612FNG: IN1=IN2=HIGH with EN=255 → short brake mode
// ============================================================
void stopMotors() {
  motor1run(0);
  motor2run(0);
}

// ============================================================
// MOTOR 1 — LEFT MOTOR
//   spd >  0 → forward
//   spd <  0 → reverse
//   spd == 0 → active brake
// ============================================================
void motor1run(int spd) {
  spd = constrain(spd, -255, 255);
  if (spd > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(enA, spd);
  } else if (spd < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(enA, -spd);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
    analogWrite(enA, 255);   // short brake
  }
}

// ============================================================
// MOTOR 2 — RIGHT MOTOR
// ============================================================
void motor2run(int spd) {
  spd = constrain(spd, -255, 255);
  if (spd > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(enB, spd);
  } else if (spd < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(enB, -spd);
  } else {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, HIGH);
    analogWrite(enB, 255);   // short brake
  }
}

// ============================================================
// MUX SENSOR READ
// Selects channel ch (0-15) on the CD74HC4067 via S0-S3,
// waits 10µs for MUX to settle, then reads the analog value.
// ============================================================
int sensorRead(int ch) {
  digitalWrite(S0, (ch >> 0) & 1);
  digitalWrite(S1, (ch >> 1) & 1);
  digitalWrite(S2, (ch >> 2) & 1);
  digitalWrite(S3, (ch >> 3) & 1);
  delayMicroseconds(10);
  return analogRead(SENSOR_PIN);
}
