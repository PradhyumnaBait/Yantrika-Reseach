// ============================================================
// WHITE LINE FOLLOWER — v3
// Fixes over v2:
//  A. Forward declaration corrected: handleSharpTurn(bool)
//  B. recoverLine() Phase 0: straight creep before spinning
//  C. Sharp turn timeout reduced to 1000ms
//  D. Post-turn slow creep before stop (reduce overshoot)
//  E. Junction entry: gradual speed ramp-down, not instant jump
//  F. Junction max creep time reduced to 200ms
//  G. Junction type classification: END / T-FWD / T-LEFT /
//     T-RIGHT / CROSS — each handled distinctly
//  H. Kd tuned to 8.0 (was 12.0) to reduce jerk on gradual curves
// ============================================================

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// -------- SENSOR PATTERN MASKS --------
#define ALL_LINE  ((uint16_t)0xFFFF)
#define NO_LINE   ((uint16_t)0x0000)

// -------- TB6612FNG MOTOR DRIVER --------
#define IN1    5
#define IN2    4
#define IN3    7
#define IN4    8
#define enA    3
#define enB    9
#define LED    2
#define BUTTON 10
#define STBY   6

// -------- MUX PINS --------
#define S0  A1
#define S1  A2
#define S2  A3
#define S3  A4

// -------- SENSOR --------
#define SENSOR_PIN  A0
#define NUM_SENSORS 16

// -------- LINE CONFIG --------
// true  = follow WHITE line on black surface
// false = follow BLACK line on white surface
#define FOLLOW_WHITE_LINE true

// -------- SENSOR WEIGHTS --------
// Symmetric ±15 scale. Sensor 0 = leftmost (+15), sensor 15 = rightmost (−15)
// Error > 0 → line is left of centre → steer left
// Error < 0 → line is right of centre → steer right
const int sensorWeight[NUM_SENSORS] = {
  15, 13, 11, 9, 7, 5, 3, 1,
  -1, -3, -5, -7, -9, -11, -13, -15
};

int  minValues[NUM_SENSORS], maxValues[NUM_SENSORS];
int  sensorValue[NUM_SENSORS];
bool sensorArray[NUM_SENSORS];   // true = line detected

// -------- PID TUNING --------
float Kp = 1.8f;   // proportional — scales with ±15 weight range
float Ki = 0.003f; // integral     — corrects slow steady drift only
float Kd = 8.0f;   // derivative   — reduced from 12.0 to reduce jerk

#define I_MAX  300.0f
#define I_MIN -300.0f

float errorVal  = 0.0f;
float prevError = 0.0f;
float integral  = 0.0f;
float PIDvalue  = 0.0f;

// -------- SPEED CONSTANTS --------
#define LF_SPEED        180   // cruise speed
#define TURN_SPEED       90   // spin speed for sharp turns
#define TURN_SETTLE      50   // slow creep after turn reacquires line
#define JUNCTION_SPEED   70   // creep speed at junction
#define RAMP_STEP         1   // speed increment per loop tick

int currentSpeed = 30;

// -------- JUNCTION TYPE ENUM --------
typedef enum {
  JT_UNKNOWN = 0,
  JT_END,       // dead end — line stops in all directions
  JT_T_FWD,     // T-junction: forward + (left or right or both) branch
  JT_T_LEFT,    // T-junction: left branch only (no forward, no right)
  JT_T_RIGHT,   // T-junction: right branch only (no forward, no left)
  JT_CROSS      // + junction: all branches present
} JunctionType;

// -------- STATE --------
bool     onLine  = false;
int      lsp, rsp;
uint16_t pattern = 0;

// ============================================================
// FORWARD DECLARATIONS (all correct signatures)
// ============================================================
void          setup();
void          loop();
void          calibrate();
void          readLine();
void          buildPattern();
JunctionType  classifyJunction();
void          handleJunction();
void          handleSharpTurn(bool turnLeft);
void          lineFollow();
void          recoverLine();
void          rampDownTo(int targetSpeed, int stepDelay);
void          resetPID();
void          stopMotors();
void          motor1run(int spd);
void          motor2run(int spd);
int           sensorRead(int ch);
bool          centerOnLine();
bool          leftSideOn();
bool          rightSideOn();

// ============================================================
// SETUP
// ============================================================
void setup() {
  // ADC prescaler = 32 for ~500 kHz ADC clock (accuracy + speed)
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  sbi(ADCSRA, ADPS0);

  Serial.begin(115200);

  pinMode(IN1,    OUTPUT);
  pinMode(IN2,    OUTPUT);
  pinMode(IN3,    OUTPUT);
  pinMode(IN4,    OUTPUT);
  pinMode(enA,    OUTPUT);
  pinMode(enB,    OUTPUT);
  pinMode(LED,    OUTPUT);
  pinMode(STBY,   OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(S0,     OUTPUT);
  pinMode(S1,     OUTPUT);
  pinMode(S2,     OUTPUT);
  pinMode(S3,     OUTPUT);

  digitalWrite(STBY, HIGH);

  // Wait for button → calibrate
  digitalWrite(LED, HIGH);
  Serial.println(F("Press BUTTON to calibrate."));
  while (digitalRead(BUTTON) == HIGH) {}
  delay(60);
  digitalWrite(LED, LOW);

  calibrate();

  // Wait for button → start run
  digitalWrite(LED, HIGH);
  Serial.println(F("Press BUTTON to start."));
  while (digitalRead(BUTTON) == HIGH) {}
  delay(60);
  digitalWrite(LED, LOW);

  resetPID();
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  readLine();
  buildPattern();

  // ----------------------------------------------------------
  // 1. ALL SENSORS ON LINE → classify and handle junction
  // ----------------------------------------------------------
  if (pattern == ALL_LINE) {
    handleJunction();
    return;
  }

  // ----------------------------------------------------------
  // 2. SHARP TURN DETECTION
  //    Uses 5 outermost sensors on one side to trigger a pivot.
  //    Left  mask: sensors 0-4   (0b1111100000000000)
  //    Right mask: sensors 11-15 (0b0000000000011111)
  //    Both on → wide bar → treat as junction
  // ----------------------------------------------------------
  bool sharpLeft  = (pattern & 0b1111100000000000) == 0b1111100000000000;
  bool sharpRight = (pattern & 0b0000000000011111) == 0b0000000000011111;

  if (sharpLeft && sharpRight) {
    handleJunction();
    return;
  }
  if (sharpLeft) {
    handleSharpTurn(true);
    return;
  }
  if (sharpRight) {
    handleSharpTurn(false);
    return;
  }

  // ----------------------------------------------------------
  // 3. NORMAL PID FOLLOW
  // ----------------------------------------------------------
  if (currentSpeed < LF_SPEED) currentSpeed += RAMP_STEP;

  if (onLine) {
    lineFollow();
  } else {
    recoverLine();
  }
}

// ============================================================
// BUILD 16-BIT SENSOR PATTERN
// ============================================================
void buildPattern() {
  pattern = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    pattern = (pattern << 1) | (sensorArray[i] ? 1u : 0u);
  }
}

// ============================================================
// SENSOR POSITION HELPERS
// ============================================================
bool centerOnLine() {
  // Sensors 6-9 = inner four (directly around centre)
  return (sensorArray[6] || sensorArray[7] ||
          sensorArray[8] || sensorArray[9]);
}

bool leftSideOn() {
  // Sensors 0-4 = outer left
  return (sensorArray[0] || sensorArray[1] ||
          sensorArray[2] || sensorArray[3] || sensorArray[4]);
}

bool rightSideOn() {
  // Sensors 11-15 = outer right
  return (sensorArray[11] || sensorArray[12] ||
          sensorArray[13] || sensorArray[14] || sensorArray[15]);
}

bool forwardPathOn() {
  // Sensors 5-10 = middle band (forward path indicator)
  return (sensorArray[5]  || sensorArray[6]  ||
          sensorArray[7]  || sensorArray[8]  ||
          sensorArray[9]  || sensorArray[10]);
}

// ============================================================
// JUNCTION CLASSIFICATION
// Called AFTER the robot has crept past the junction bar and
// sensorArray has been freshly updated.
//
//  Sensor zones after crossing bar:
//    Left  branch → outer-left sensors lit
//    Right branch → outer-right sensors lit
//    Fwd   branch → centre sensors lit
//
//  Returns JunctionType based on which zones are active.
// ============================================================
JunctionType classifyJunction() {
  bool hasFwd   = forwardPathOn();
  bool hasLeft  = leftSideOn();
  bool hasRight = rightSideOn();

  if (!hasFwd && !hasLeft && !hasRight) return JT_END;
  if ( hasFwd && !hasLeft && !hasRight) return JT_T_FWD;   // straight T
  if (!hasFwd &&  hasLeft && !hasRight) return JT_T_LEFT;
  if (!hasFwd && !hasLeft &&  hasRight) return JT_T_RIGHT;
  // Any combination with forward + side(s) or both sides = cross/T-forward
  return JT_CROSS;
}

// ============================================================
// JUNCTION / ENDPOINT HANDLER
// ============================================================
void handleJunction() {
  // ---- Step 1: gradual speed ramp-down to junction creep speed ----
  rampDownTo(JUNCTION_SPEED, 2);   // 2ms per step, smooth deceleration

  Serial.println(F("Junction: creeping..."));

  // ---- Step 2: sensor-gated creep past the junction bar ----
  motor1run(JUNCTION_SPEED);
  motor2run(JUNCTION_SPEED);

  unsigned long creepStart = millis();
  while (millis() - creepStart < 200UL) {   // max 200ms (narrow line safe)
    readLine();
    buildPattern();
    if (pattern != ALL_LINE) break;
  }
  stopMotors();
  delay(40);

  // ---- Step 3: fresh read and classification ----
  readLine();
  buildPattern();

  // Still ALL_LINE → robot didn't clear the bar → endpoint
  if (pattern == ALL_LINE) {
    Serial.println(F("ENDPOINT."));
    stopMotors();
    digitalWrite(LED, HIGH);

    unsigned long waitStart = millis();
    while (true) {
      readLine();
      if (centerOnLine() && !leftSideOn() && !rightSideOn()) {
        digitalWrite(LED, LOW);
        resetPID();
        Serial.println(F("Resuming."));
        return;
      }
      stopMotors();
      if (millis() - waitStart > 1000UL) {
        waitStart = millis();
        digitalWrite(LED, !digitalRead(LED));
      }
    }
  }

  JunctionType jt = classifyJunction();

  switch (jt) {

    // ----------------------------------------------------------
    case JT_END:
    // Dead end — no line visible after bar.
    // Rotate LEFT (default maze convention), then try RIGHT.
    // ----------------------------------------------------------
      Serial.println(F("Dead end — searching."));
      stopMotors(); delay(40);

      // Try LEFT
      motor1run(-100); motor2run(100);
      { unsigned long t = millis();
        while (millis() - t < 2500UL) {
          readLine();
          if (centerOnLine()) {
            stopMotors(); delay(30);
            resetPID();
            Serial.println(F("Found (L)."));
            return;
          }
        }
      }
      stopMotors(); delay(40);

      // Try RIGHT
      motor1run(100); motor2run(-100);
      { unsigned long t = millis();
        while (millis() - t < 2500UL) {
          readLine();
          if (centerOnLine()) {
            stopMotors(); delay(30);
            resetPID();
            Serial.println(F("Found (R)."));
            return;
          }
        }
      }

      // Genuinely lost
      stopMotors();
      Serial.println(F("Lost — halting."));
      digitalWrite(LED, HIGH);
      while (true) {}
      break;

    // ----------------------------------------------------------
    case JT_T_FWD:
    // Straight line continues. Just keep going.
    // ----------------------------------------------------------
      Serial.println(F("T-fwd: straight ahead."));
      resetPID();
      break;

    // ----------------------------------------------------------
    case JT_T_LEFT:
    // Left branch only. Pivot left to acquire it.
    // ----------------------------------------------------------
      Serial.println(F("T-left: pivoting left."));
      handleSharpTurn(true);
      break;

    // ----------------------------------------------------------
    case JT_T_RIGHT:
    // Right branch only. Pivot right.
    // ----------------------------------------------------------
      Serial.println(F("T-right: pivoting right."));
      handleSharpTurn(false);
      break;

    // ----------------------------------------------------------
    case JT_CROSS:
    // + junction or T with multiple branches.
    // Default maze policy: always go forward (straight priority).
    // If forward path is confirmed, just resetPID.
    // If forward is not active, sharp turn left (left-hand rule).
    // ----------------------------------------------------------
      Serial.println(F("Cross junction."));
      if (forwardPathOn()) {
        resetPID();
      } else {
        handleSharpTurn(true);   // left-hand rule fallback
      }
      break;

    default:
      resetPID();
      break;
  }
}

// ============================================================
// SHARP TURN HANDLER
// turnLeft = true  → pivot left (IN motor back, OUT motor fwd)
// turnLeft = false → pivot right
//
// Sequence:
//  1. Ramp down to TURN_SPEED
//  2. Spin in-place until centre reacquires (timeout 1000ms)
//  3. Slow-creep at TURN_SETTLE speed for 30ms to reduce overshoot
//  4. Stop → resetPID
// ============================================================
void handleSharpTurn(bool turnLeft) {
  Serial.print(F("Sharp turn "));
  Serial.println(turnLeft ? F("LEFT") : F("RIGHT"));

  // Step 1: ramp down
  rampDownTo(TURN_SPEED, 2);

  // Step 2: pivot
  if (turnLeft) {
    motor1run(-TURN_SPEED);
    motor2run( TURN_SPEED);
  } else {
    motor1run( TURN_SPEED);
    motor2run(-TURN_SPEED);
  }

  unsigned long tStart = millis();
  while (millis() - tStart < 1000UL) {   // reduced timeout: 1000ms
    readLine();
    if (centerOnLine()) break;
  }

  // Step 3: slow settle creep (anti-overshoot)
  if (turnLeft) {
    motor1run(-TURN_SETTLE);
    motor2run( TURN_SETTLE);
  } else {
    motor1run( TURN_SETTLE);
    motor2run(-TURN_SETTLE);
  }
  delay(30);

  // Step 4: stop and reset
  stopMotors();
  delay(20);
  resetPID();
}

// ============================================================
// LOST LINE RECOVERY
// Phase 0: straight creep (bridges small gaps)
// Phase 1: slow spin toward last error side (800ms)
// Phase 2: faster reverse spin (1000ms)
// Phase 3: halt
// ============================================================
void recoverLine() {
  // Phase 0: creep straight — bridges track gaps / joins
  motor1run(80);
  motor2run(80);
  unsigned long t0 = millis();
  while (millis() - t0 < 150UL) {
    readLine();
    if (onLine) { stopMotors(); delay(20); resetPID(); return; }
  }

  // Phase 1: slow spin toward last known error direction
  int spinDir = (prevError > 0) ? 1 : -1;   // +1=left, -1=right
  motor1run(-110 * spinDir);
  motor2run( 110 * spinDir);

  unsigned long t1 = millis();
  while (millis() - t1 < 800UL) {
    readLine();
    if (onLine) { stopMotors(); delay(20); resetPID(); return; }
  }

  // Phase 2: faster spin opposite direction
  motor1run( 150 * spinDir);
  motor2run(-150 * spinDir);

  unsigned long t2 = millis();
  while (millis() - t2 < 1000UL) {
    readLine();
    if (onLine) { stopMotors(); delay(20); resetPID(); return; }
  }

  // Phase 3: halt
  stopMotors();
  Serial.println(F("Line lost — halting."));
  digitalWrite(LED, HIGH);
  while (true) {}
}

// ============================================================
// SMOOTH SPEED RAMP-DOWN
// Decrements currentSpeed from its current value to targetSpeed
// with stepDelay ms between each step.
// Also writes the new speed to both motors each step.
// ============================================================
void rampDownTo(int targetSpeed, int stepDelay) {
  if (currentSpeed <= targetSpeed) {
    currentSpeed = targetSpeed;
    return;
  }
  while (currentSpeed > targetSpeed) {
    currentSpeed--;
    motor1run(currentSpeed);
    motor2run(currentSpeed);
    delay(stepDelay);
  }
}

// ============================================================
// PID LINE FOLLOW
// Pure position centroid — no sensorValue weighting
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

  if (active > 0) {
    errorVal = weightedSum / (float)active;
  }
  // If active == 0: keep last errorVal so PID doesn't snap to zero

  float P  = errorVal;
  integral = constrain(integral + errorVal, I_MIN, I_MAX);
  float D  = errorVal - prevError;

  PIDvalue  = (Kp * P) + (Ki * integral) + (Kd * D);
  prevError = errorVal;

  lsp = currentSpeed - (int)PIDvalue;
  rsp = currentSpeed + (int)PIDvalue;

  lsp = constrain(lsp, -200, 255);
  rsp = constrain(rsp, -200, 255);

  motor1run(lsp);
  motor2run(rsp);

  // Uncomment for Serial Plotter tuning:
  // Serial.print(errorVal,2); Serial.print(",");
  // Serial.print(PIDvalue,2); Serial.print(",");
  // Serial.print(lsp);        Serial.print(",");
  // Serial.println(rsp);
}

// ============================================================
// CALIBRATION
// ============================================================
void calibrate() {
  Serial.println(F("Calibrating..."));

  for (int i = 0; i < NUM_SENSORS; i++) {
    int v        = sensorRead(i);
    minValues[i] = v;
    maxValues[i] = v;
  }

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

  bool ok = true;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int spread = maxValues[i] - minValues[i];
    if (spread < 100) {
      Serial.print(F("WARN s"));
      Serial.print(i);
      Serial.print(F(" spread="));
      Serial.println(spread);
      ok = false;
    }
  }
  if (ok) {
    Serial.println(F("Cal OK."));
  } else {
    for (int b = 0; b < 5; b++) {
      digitalWrite(LED, HIGH); delay(150);
      digitalWrite(LED, LOW);  delay(150);
    }
  }
}

// ============================================================
// READ ALL SENSORS
// ============================================================
void readLine() {
  onLine = false;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int raw = sensorRead(i);

#if FOLLOW_WHITE_LINE
    sensorValue[i] = map(raw, minValues[i], maxValues[i], 0, 1000);
#else
    sensorValue[i] = map(raw, minValues[i], maxValues[i], 1000, 0);
#endif

    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    sensorArray[i] = (sensorValue[i] > 500);
    if (sensorArray[i]) onLine = true;
  }
}

// ============================================================
// RESET PID STATE
// ============================================================
void resetPID() {
  errorVal     = 0.0f;
  prevError    = 0.0f;
  integral     = 0.0f;
  PIDvalue     = 0.0f;
  currentSpeed = 30;
}

// ============================================================
// STOP MOTORS — active brake
// ============================================================
void stopMotors() {
  motor1run(0);
  motor2run(0);
}

// ============================================================
// MOTOR 1 (left motor)
// ============================================================
void motor1run(int spd) {
  spd = constrain(spd, -255, 255);
  if (spd > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    analogWrite(enA, spd);
  } else if (spd < 0) {
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    analogWrite(enA, -spd);
  } else {
    // Active brake: both IN HIGH, full PWM
    digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH);
    analogWrite(enA, 255);
  }
}

// ============================================================
// MOTOR 2 (right motor)
// ============================================================
void motor2run(int spd) {
  spd = constrain(spd, -255, 255);
  if (spd > 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(enB, spd);
  } else if (spd < 0) {
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
    analogWrite(enB, -spd);
  } else {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
    analogWrite(enB, 255);
  }
}

// ============================================================
// MUX SENSOR READ — correct bit-shift extraction
// ============================================================
int sensorRead(int ch) {
  digitalWrite(S0, (ch >> 0) & 1);
  digitalWrite(S1, (ch >> 1) & 1);
  digitalWrite(S2, (ch >> 2) & 1);
  digitalWrite(S3, (ch >> 3) & 1);
  delayMicroseconds(10);
  return analogRead(SENSOR_PIN);
}
