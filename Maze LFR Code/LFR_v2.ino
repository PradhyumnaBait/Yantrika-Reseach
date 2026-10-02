// ============================================================
// WHITE LINE FOLLOWER — v2
// Fixes over v1:
//  - Sharp turn: blocking loop until center reacquires line
//  - Sharp turn: both-sides-on treated as junction (not ignored)
//  - Junction creep: sensor-gated, not time-gated
//  - Junction rotation: tries LEFT first, then RIGHT if not found
//  - Error calculation: pure position centroid (no sensorValue weighting)
//  - lsp/rsp constrain now symmetric (-200 to 200)
//  - Speed deceleration approaching junction/sharp turn
//  - Lost line recovery: staged search (slow spin, timeout, halt)
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
// Position centroid: left = positive, right = negative
// Weights are symmetric around centre gap between s7 and s8
// Using ×2 scale (–15 to +15) gives more resolution for the centroid
const int sensorWeight[NUM_SENSORS] = {
  15, 13, 11, 9, 7, 5, 3, 1,
  -1, -3, -5, -7, -9, -11, -13, -15
};

int   minValues[NUM_SENSORS], maxValues[NUM_SENSORS];
int   sensorValue[NUM_SENSORS];
bool  sensorArray[NUM_SENSORS];   // true = line detected

// -------- PID TUNING --------
// These are the primary tuning knobs — adjust on hardware
float Kp = 1.8f;    // proportional gain  (was 0.12 × old scale, rescaled for new ±15 weights)
float Ki = 0.003f;  // integral gain      (small — only corrects steady drift)
float Kd = 12.0f;   // derivative gain    (high — damps oscillation aggressively)

// Integral windup clamp
#define I_MAX  300.0f
#define I_MIN -300.0f

float errorVal  = 0.0f;
float prevError = 0.0f;
float integral  = 0.0f;
float PIDvalue  = 0.0f;

// -------- SPEED --------
#define LF_SPEED       180   // max cruise speed (PWM 0-255)
#define TURN_SPEED     120   // reduced speed for sharp turns
#define JUNCTION_SPEED  80   // reduced speed at junction creep
#define RAMP_STEP        1   // how much to increment speed each loop tick

int currentSpeed = 30;

// -------- STATE --------
bool     onLine  = false;
int      lsp, rsp;
uint16_t pattern = 0;

// ============================================================
// FORWARD DECLARATIONS
// ============================================================
void  calibrate();
void  readLine();
void  buildPattern();
void  handleJunction();
void  handleSharpTurn();
void  lineFollow();
void  recoverLine();
void  resetPID();
void  stopMotors();
void  motor1run(int spd);
void  motor2run(int spd);
int   sensorRead(int ch);
bool  centerOnLine();
bool  leftSensorsOn();
bool  rightSensorsOn();

// ============================================================
// SETUP
// ============================================================
void setup() {
  // ADC prescaler = 32 → ~500 kHz ADC clock, good accuracy + speed
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

  // --- WAIT FOR BUTTON: start calibration ---
  digitalWrite(LED, HIGH);
  Serial.println(F("Press BUTTON to calibrate..."));
  while (digitalRead(BUTTON) == HIGH) {}
  delay(60);
  digitalWrite(LED, LOW);

  calibrate();

  // --- WAIT FOR BUTTON: start run ---
  digitalWrite(LED, HIGH);
  Serial.println(F("Press BUTTON to start run..."));
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
  // 1. ALL SENSORS ON LINE → junction or endpoint
  // ----------------------------------------------------------
  if (pattern == ALL_LINE) {
    handleJunction();
    return;
  }

  // ----------------------------------------------------------
  // 2. SHARP TURN DETECTION
  //    Left:  sensors 0-4 all on  (0b1111100000000000)
  //    Right: sensors 11-15 all on (0b0000000000011111)
  //    Both:  treat as junction (wide bar / cross)
  // ----------------------------------------------------------
  bool sharpLeft  = (pattern & 0b1111100000000000) == 0b1111100000000000;
  bool sharpRight = (pattern & 0b0000000000011111) == 0b0000000000011111;

  if (sharpLeft || sharpRight) {
    // Both sides on simultaneously → treat as junction, not a turn
    if (sharpLeft && sharpRight) {
      handleJunction();
      return;
    }
    handleSharpTurn(sharpLeft);
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
// BUILD 16-BIT PATTERN
// ============================================================
void buildPattern() {
  pattern = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    pattern = (pattern << 1) | (sensorArray[i] ? 1u : 0u);
  }
}

// ============================================================
// HELPER: is the robot centred on line?
// ============================================================
bool centerOnLine() {
  return (sensorArray[6] || sensorArray[7] ||
          sensorArray[8] || sensorArray[9]);
}

bool leftSensorsOn() {
  return (sensorArray[0] || sensorArray[1] || sensorArray[2]);
}

bool rightSensorsOn() {
  return (sensorArray[13] || sensorArray[14] || sensorArray[15]);
}

// ============================================================
// JUNCTION / ENDPOINT HANDLER
// ============================================================
void handleJunction() {
  // Decelerate to junction speed
  currentSpeed = JUNCTION_SPEED;

  // Sensor-gated creep: move forward until the junction bar clears
  // i.e. NOT all sensors on anymore, OR a pattern appears ahead
  Serial.println(F("Junction detected — creeping..."));
  motor1run(JUNCTION_SPEED);
  motor2run(JUNCTION_SPEED);

  unsigned long creepStart = millis();
  while (millis() - creepStart < 400UL) {   // max 400ms creep
    readLine();
    buildPattern();
    // Stop creeping as soon as we leave the ALL_LINE state
    if (pattern != ALL_LINE) break;
  }
  stopMotors();
  delay(50);

  readLine();
  buildPattern();

  // --- ENDPOINT: still ALL_LINE after creeping ---
  if (pattern == ALL_LINE) {
    stopMotors();
    digitalWrite(LED, HIGH);
    Serial.println(F("ENDPOINT reached."));

    unsigned long waitStart = millis();
    while (true) {
      readLine();
      // Resume when centre sensors detect line AND both outer sensors are clear
      if (centerOnLine() && !leftSensorsOn() && !rightSensorsOn()) {
        digitalWrite(LED, LOW);
        resetPID();
        Serial.println(F("Resuming."));
        return;
      }
      stopMotors();
      // Blink LED while waiting
      if (millis() - waitStart > 1000UL) {
        waitStart = millis();
        digitalWrite(LED, !digitalRead(LED));
      }
    }
  }

  // --- NO LINE after creeping → search for line ---
  if (pattern == NO_LINE) {
    Serial.println(F("No line after creep — searching..."));
    stopMotors();
    delay(40);

    // Try LEFT first (270° sweep, 2.5 s)
    motor1run(-100);
    motor2run(100);
    unsigned long tLeft = millis();
    while (millis() - tLeft < 2500UL) {
      readLine();
      if (centerOnLine()) {
        stopMotors();
        delay(30);
        resetPID();
        Serial.println(F("Found line (left search)."));
        return;
      }
    }
    stopMotors();
    delay(40);

    // Try RIGHT (270° sweep, 2.5 s)
    motor1run(100);
    motor2run(-100);
    unsigned long tRight = millis();
    while (millis() - tRight < 2500UL) {
      readLine();
      if (centerOnLine()) {
        stopMotors();
        delay(30);
        resetPID();
        Serial.println(F("Found line (right search)."));
        return;
      }
    }

    // Genuinely lost
    stopMotors();
    Serial.println(F("Line not found — halting."));
    digitalWrite(LED, HIGH);
    while (true) {}
  }

  // --- LINE CONTINUES AHEAD ---
  Serial.println(F("Junction crossed — continuing."));
  resetPID();
}

// ============================================================
// SHARP TURN HANDLER
// Blocking: spins until centre sensors reacquire the line
// turnLeft = true  → hard left
// turnLeft = false → hard right
// ============================================================
void handleSharpTurn(bool turnLeft) {
  currentSpeed = TURN_SPEED;
  Serial.print(F("Sharp turn "));
  Serial.println(turnLeft ? F("LEFT") : F("RIGHT"));

  // Brief slowdown before the turn
  motor1run(TURN_SPEED);
  motor2run(TURN_SPEED);
  delay(40);

  // Spin in-place until centre reacquires line
  if (turnLeft) {
    motor1run(-TURN_SPEED);
    motor2run(TURN_SPEED);
  } else {
    motor1run(TURN_SPEED);
    motor2run(-TURN_SPEED);
  }

  // Wait until we see the line again at centre
  unsigned long tStart = millis();
  while (millis() - tStart < 2000UL) {
    readLine();
    if (centerOnLine()) break;
  }

  stopMotors();
  delay(30);
  resetPID();
}

// ============================================================
// LOST LINE RECOVERY
// Staged: slow spin toward last error side, 1.5 s timeout
// ============================================================
void recoverLine() {
  // Phase 1: slow spin (800 ms)
  int spinSpd = 120;
  if (prevError > 0) {
    motor1run(-spinSpd);
    motor2run(spinSpd);
  } else {
    motor1run(spinSpd);
    motor2run(-spinSpd);
  }

  unsigned long tStart = millis();
  while (millis() - tStart < 800UL) {
    readLine();
    if (onLine) {
      stopMotors();
      delay(20);
      resetPID();
      return;
    }
  }

  // Phase 2: faster spin other direction (1 s)
  spinSpd = 160;
  if (prevError > 0) {
    motor1run(spinSpd);
    motor2run(-spinSpd);
  } else {
    motor1run(-spinSpd);
    motor2run(spinSpd);
  }

  tStart = millis();
  while (millis() - tStart < 1000UL) {
    readLine();
    if (onLine) {
      stopMotors();
      delay(20);
      resetPID();
      return;
    }
  }

  // Phase 3: halt — truly lost
  stopMotors();
  Serial.println(F("Line lost — halting."));
  digitalWrite(LED, HIGH);
  while (true) {}
}

// ============================================================
// PID LINE FOLLOW
// Pure position centroid error (no sensorValue weighting)
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

  // Error = average position offset from centre
  // = 0 when balanced, positive = line left of centre, negative = right
  if (active > 0) {
    errorVal = weightedSum / (float)active;
  }
  // If no active sensors: keep last error (don't reset to zero)

  // PID computation
  float P  = errorVal;
  integral = constrain(integral + errorVal, I_MIN, I_MAX);
  float D  = errorVal - prevError;

  PIDvalue  = (Kp * P) + (Ki * integral) + (Kd * D);
  prevError = errorVal;

  lsp = currentSpeed - (int)PIDvalue;
  rsp = currentSpeed + (int)PIDvalue;

  // Symmetric clamp: allows full reverse on inner wheel for tight turns
  lsp = constrain(lsp, -200, 255);
  rsp = constrain(rsp, -200, 255);

  motor1run(lsp);
  motor2run(rsp);

  // Serial debug — uncomment for tuning via Serial Plotter
  // Serial.print(errorVal); Serial.print(",");
  // Serial.print(PIDvalue); Serial.print(",");
  // Serial.print(lsp);      Serial.print(",");
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

  bool calOK = true;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int spread = maxValues[i] - minValues[i];
    if (spread < 100) {
      Serial.print(F("WARNING sensor "));
      Serial.print(i);
      Serial.print(F(" spread="));
      Serial.println(spread);
      calOK = false;
    }
  }

  if (calOK) {
    Serial.println(F("Calibration OK."));
  } else {
    // Blink LED 3x to warn of poor calibration
    for (int b = 0; b < 3; b++) {
      digitalWrite(LED, HIGH); delay(200);
      digitalWrite(LED, LOW);  delay(200);
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
    // White line: higher reflectance → higher ADC → maps to higher value
    sensorValue[i] = map(raw, minValues[i], maxValues[i], 0, 1000);
#else
    // Black line: lower reflectance → lower ADC → maps to higher value
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
// STOP MOTORS (active brake)
// ============================================================
void stopMotors() {
  motor1run(0);
  motor2run(0);
}

// ============================================================
// MOTOR 1
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
    // Active brake: both IN high + full enable
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
    analogWrite(enA, 255);
  }
}

// ============================================================
// MOTOR 2
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
    analogWrite(enB, 255);
  }
}

// ============================================================
// MUX SENSOR READ
// ============================================================
int sensorRead(int ch) {
  digitalWrite(S0, (ch >> 0) & 1);
  digitalWrite(S1, (ch >> 1) & 1);
  digitalWrite(S2, (ch >> 2) & 1);
  digitalWrite(S3, (ch >> 3) & 1);
  delayMicroseconds(10);
  return analogRead(SENSOR_PIN);
}
