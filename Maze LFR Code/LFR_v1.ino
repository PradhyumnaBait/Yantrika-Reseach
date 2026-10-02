// ============================================================
// WHITE LINE FOLLOWER — v1
// Fixes: MUX channel bits, integral windup, type consistency,
//        brake vs coast, button-start, isBlackLine naming
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
#define IN1  5
#define IN2  4
#define IN3  7
#define IN4  8
#define enA  3
#define enB  9
#define LED  2
#define BUTTON 10
#define STBY 6

// -------- MUX PINS --------
#define S0  A1
#define S1  A2
#define S2  A3
#define S3  A4

// -------- SENSOR --------
#define SENSOR_PIN  A0
#define NUM_SENSORS 16

// -------- LINE FOLLOW CONFIG --------
// false = WHITE LINE on black surface
// true  = BLACK LINE on white surface
bool followWhiteLine = true;   // renamed from isBlackLine (inverted naming fixed)

// Sensor weights: left sensors positive, right sensors negative
// Sensor 0 = leftmost, sensor 15 = rightmost
const int sensorWeight[NUM_SENSORS] = {
  7, 6, 5, 4, 3, 2, 1, 0,
  0, -1, -2, -3, -4, -5, -6, -7
};

int minValues[NUM_SENSORS], maxValues[NUM_SENSORS];
int sensorValue[NUM_SENSORS];
bool sensorArray[NUM_SENSORS];  // true = line detected

// -------- PID VARIABLES (all float for precision) --------
float Kp = 0.12f;
float Ki = 0.0002f;
float Kd = 1.0f;

float errorVal     = 0.0f;
float prevError    = 0.0f;
float integral     = 0.0f;
float P, D, PIDvalue;

// Integral windup clamp
#define I_MAX  500.0f
#define I_MIN -500.0f

// -------- SPEED --------
int lfSpeed      = 180;   // cruise speed
int currentSpeed = 30;    // starts low, ramps up

// -------- STATE --------
bool onLine      = false;
int  lsp, rsp;
uint16_t pattern = 0;

// ============================================================
// SETUP
// ============================================================
void setup() {
  // ADC prescaler = 32 → ~500kSPS at 16MHz; good accuracy/speed balance
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  sbi(ADCSRA, ADPS0);   // prescaler 32 (was 16 — too fast, accuracy loss)

  Serial.begin(115200);

  // Motor driver pins
  pinMode(IN1,   OUTPUT);
  pinMode(IN2,   OUTPUT);
  pinMode(IN3,   OUTPUT);
  pinMode(IN4,   OUTPUT);
  pinMode(enA,   OUTPUT);
  pinMode(enB,   OUTPUT);
  pinMode(LED,   OUTPUT);
  pinMode(STBY,  OUTPUT);
  digitalWrite(STBY, HIGH);

  // Button with internal pull-up
  pinMode(BUTTON, INPUT_PULLUP);

  // MUX select pins
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  // -------- CALIBRATION --------
  // Wait for button press before calibrating
  digitalWrite(LED, HIGH);
  Serial.println(F("Press BUTTON to start calibration..."));
  while (digitalRead(BUTTON) == HIGH) { /* wait */ }
  delay(50);  // debounce
  digitalWrite(LED, LOW);

  calibrate();

  // -------- WAIT FOR BUTTON PRESS TO START RUN --------
  digitalWrite(LED, HIGH);
  Serial.println(F("Calibration done. Press BUTTON to start run..."));
  while (digitalRead(BUTTON) == HIGH) { /* wait */ }
  delay(50);
  digitalWrite(LED, LOW);

  // Reset PID state before run
  resetPID();
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  readLine();
  buildPattern();

  // ============================================================
  // 1. ALL SENSORS ON LINE → junction or endpoint
  // ============================================================
  if (pattern == ALL_LINE) {
    handleJunction();
    return;
  }

  // ============================================================
  // 2. SHARP TURN DETECTION (5+ consecutive sensors on one side)
  // ============================================================
  if (handleSharpTurn()) {
    return;
  }

  // ============================================================
  // 3. NORMAL PID FOLLOW
  // ============================================================
  // Speed ramp-up
  if (currentSpeed < lfSpeed) currentSpeed++;

  if (onLine) {
    lineFollow();
  } else {
    // Lost line recovery — spin toward last known direction
    recoverLine();
  }
}

// ============================================================
// BUILD 16-BIT SENSOR PATTERN
// ============================================================
void buildPattern() {
  pattern = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    pattern = (pattern << 1) | (sensorArray[i] ? 1 : 0);
  }
}

// ============================================================
// JUNCTION / ENDPOINT HANDLER
// ============================================================
void handleJunction() {
  // Creep forward a little to move past the junction bar
  motor1run(80);
  motor2run(80);
  delay(120);
  stopMotors();
  delay(60);

  readLine();
  buildPattern();

  // --- ENDPOINT: still ALL_LINE after creeping ---
  if (pattern == ALL_LINE) {
    stopMotors();
    digitalWrite(LED, HIGH);
    Serial.println(F("ENDPOINT reached. Waiting..."));

    // Wait until center sensors see line (robot repositioned manually)
    unsigned long waitStart = millis();
    while (true) {
      readLine();
      // Center sensors (7 and 8) detect line AND outer sensors clear
      bool centerOnLine = (sensorArray[7] || sensorArray[8]);
      bool outersClear  = (!sensorArray[0] && !sensorArray[15]);

      if (centerOnLine && outersClear) {
        digitalWrite(LED, LOW);
        resetPID();
        Serial.println(F("Resuming from endpoint."));
        break;
      }
      stopMotors();
      // Safety: after 30s blink LED to indicate still waiting
      if ((millis() - waitStart) > 30000UL) {
        digitalWrite(LED, (millis() >> 9) & 1);  // slow blink
      }
    }
    return;
  }

  // --- NO LINE after creeping → T-junction or end, rotate to find line ---
  if (pattern == NO_LINE) {
    Serial.println(F("No line after junction creep. Searching..."));
    stopMotors();
    delay(60);

    // Rotate LEFT to find line; timeout after 2s
    motor1run(-100);
    motor2run(100);
    unsigned long tStart = millis();
    bool found = false;
    while (millis() - tStart < 2000UL) {
      readLine();
      if (sensorArray[6] || sensorArray[7] ||
          sensorArray[8] || sensorArray[9]) {
        found = true;
        break;
      }
    }
    stopMotors();
    delay(40);

    if (!found) {
      Serial.println(F("Line not found after rotation! Stopping."));
      digitalWrite(LED, HIGH);
      while (true) { /* halt */ }
    }

    resetPID();
    return;
  }

  // --- LINE CONTINUES (T or cross junction, main path ahead) ---
  Serial.println(F("Junction: line continues ahead."));
  resetPID();
}

// ============================================================
// SHARP TURN HANDLER
// Returns true if a sharp turn was detected and handled
// ============================================================
bool handleSharpTurn() {
  // Left side: sensors 0-5 or more filled, right side mostly clear
  uint16_t leftMask  = 0b1111110000000000;  // sensors 0–5
  uint16_t rightMask = 0b0000000000111111;  // sensors 10–15

  bool sharpLeft  = (pattern & leftMask) == leftMask;
  bool sharpRight = (pattern & rightMask) == rightMask;

  if (sharpLeft && !sharpRight) {
    // Hard left turn
    motor1run(-90);
    motor2run(200);
    return true;
  }

  if (sharpRight && !sharpLeft) {
    // Hard right turn
    motor1run(200);
    motor2run(-90);
    return true;
  }

  return false;
}

// ============================================================
// LOST LINE RECOVERY
// ============================================================
void recoverLine() {
  // Spin toward last known error direction
  if (prevError > 0) {
    motor1run(-180);
    motor2run(180);
  } else {
    motor1run(180);
    motor2run(-180);
  }
}

// ============================================================
// PID LINE FOLLOW
// ============================================================
void lineFollow() {
  errorVal    = 0.0f;
  int active  = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorArray[i]) {
      errorVal += (float)sensorWeight[i] * (float)sensorValue[i];
      active++;
    }
  }

  if (active > 0) errorVal /= (float)active;

  // PID terms — all float
  P         = errorVal;
  integral  = constrain(integral + errorVal, I_MIN, I_MAX);  // clamped integral
  D         = errorVal - prevError;

  PIDvalue  = (Kp * P) + (Ki * integral) + (Kd * D);
  prevError = errorVal;

  lsp = currentSpeed - (int)PIDvalue;
  rsp = currentSpeed + (int)PIDvalue;

  lsp = constrain(lsp, -100, 200);
  rsp = constrain(rsp, -100, 200);

  motor1run(lsp);
  motor2run(rsp);

  // Debug output (comment out for competition)
  // Serial.print(F("err:")); Serial.print(errorVal);
  // Serial.print(F(" PID:")); Serial.println(PIDvalue);
}

// ============================================================
// CALIBRATION
// Spins robot over line/surface for 4 seconds to find min/max
// ============================================================
void calibrate() {
  Serial.println(F("Calibrating..."));

  // Init min/max
  for (int i = 0; i < NUM_SENSORS; i++) {
    int v       = sensorRead(i);
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

  // Validate calibration — warn if spread is too narrow
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
// READ ALL SENSORS
// ============================================================
void readLine() {
  onLine = false;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int raw = sensorRead(i);

    if (followWhiteLine) {
      // White line: high reflectance → high ADC → map to high value
      sensorValue[i] = map(raw, minValues[i], maxValues[i], 0, 1000);
    } else {
      // Black line: low reflectance → high ADC → map to low value
      sensorValue[i] = map(raw, minValues[i], maxValues[i], 1000, 0);
    }

    sensorValue[i] = constrain(sensorValue[i], 0, 1000);

    // Threshold: >500 = line detected
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
  P = D        = 0.0f;
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
    // Active brake: both IN pins HIGH, full PWM
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
    // Active brake
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, HIGH);
    analogWrite(enB, 255);
  }
}

// ============================================================
// MUX SENSOR READ — fixed bit extraction
// ============================================================
int sensorRead(int ch) {
  digitalWrite(S0, (ch >> 0) & 1);
  digitalWrite(S1, (ch >> 1) & 1);
  digitalWrite(S2, (ch >> 2) & 1);
  digitalWrite(S3, (ch >> 3) & 1);
  delayMicroseconds(10);  // slight extra settle time for MUX
  return analogRead(SENSOR_PIN);
}
