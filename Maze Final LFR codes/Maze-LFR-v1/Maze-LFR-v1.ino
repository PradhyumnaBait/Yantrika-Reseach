// ============================================================
// MESHMERIZE - LINE FOLLOWER + OPTIMIZED FINAL RUN
//
// 16 sensors + 16:1 analog MUX + TB6612FNG
// White line on black surface
//
// FIRST / LEARNING RUN:
//   Right-hand rule:
//   RIGHT > STRAIGHT > LEFT
//
//   Path stored as:
//   R = Right
//   L = Left
//   S = Straight
//   U = U-turn
//   E = End
//
// AFTER E:
//   1. Robot stops
//   2. LED D10 turns ON
//   3. Path is optimized
//   4. Robot waits at E
//   5. Press BUTTON2 (D12)
//   6. Robot performs optimized final run
//
// IMPORTANT:
//   After reaching E, manually place the robot back at START.
//   Then press BUTTON2 (D12).
// ============================================================

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define DEBUG_SERIAL 0

// ------------------------------------------------------------
// PATTERNS
// ------------------------------------------------------------
#define ALL_LINE 0xFFFF
#define NO_LINE  0x0000

// ------------------------------------------------------------
// PINS - TB6612FNG
// ------------------------------------------------------------
#define IN1 5
#define IN2 4
#define IN3 7
#define IN4 8
#define enA 3
#define enB 9

#define LED 10

#define BUTTON1 11
#define BUTTON2 12

#define STBY 6

// ------------------------------------------------------------
// PINS - 16:1 ANALOG MUX
// ------------------------------------------------------------
#define S0 A1
#define S1 A2
#define S2 A3
#define S3 A4

#define sensorPin A0

// 0 = white line on black
// 1 = black line on white
bool isBlackLine = false;

// ------------------------------------------------------------
// SENSOR WEIGHTS
// Sensor 0 = far left
// Sensor 15 = far right
// ------------------------------------------------------------
const int sensorWeight[16] = {
   15, 13, 11, 9, 7, 5, 3, 1,
   -1, -3, -5, -7, -9, -11, -13, -15
};

// ------------------------------------------------------------
// TUNABLE PARAMETERS
// ------------------------------------------------------------

// Line following
int lfSpeed = 180;

#define START_SPEED 60

float Kp = 60.0;
float Ki = 0.0;
float Kd = 500.0;

// Junction peek
#define PEEK_TIME         160
#define PEEK_START_SPEED   60
#define PEEK_END_SPEED    120
#define PEEK_RAMP_TIME     40

// Turning
#define TURN_SPEED        150
#define MIN_TURN_TIME     120
#define MAX_TURN_TIME    1500
#define MAX_UTURN_TIME   2200

// Dead end / lost line
#define LOST_CONFIRM_MS       15
#define UTURN_SIDE_THRESHOLD  8.0

// End box confirmation
#define END_CONFIRM_TIME   100
#define END_CONFIRM_SPEED  100

// Locks after decisions
#define JUNCTION_LOCK_MS  150
#define TURN_LOCK_MS       60
#define SIDE_MASK_MS      250

// ------------------------------------------------------------
// STATE
// ------------------------------------------------------------
uint16_t pattern = NO_LINE;

int minValues[16];
int maxValues[16];
int sensorValue[16];

uint8_t sensorArray[16];

float error = 0;
float previousError = 0;

float P = 0;
float I = 0;
float D = 0;

float PIDvalue = 0;

int currentSpeed = START_SPEED;

int lsp = 0;
int rsp = 0;

bool onLine = false;

unsigned long junctionLockUntil = 0;
unsigned long sideMaskUntil = 0;
unsigned long lostSince = 0;

// ------------------------------------------------------------
// FIRST / LEARNING PATH
// ------------------------------------------------------------
char path[100];
uint8_t pathLen = 0;

// ------------------------------------------------------------
// OPTIMIZED PATH
// ------------------------------------------------------------
char optimizedPath[100];
uint8_t optimizedLen = 0;

// ------------------------------------------------------------
// RUN STATE
// ------------------------------------------------------------
bool mazeSolved = false;
bool finalRunDone = false;

// ------------------------------------------------------------
// FUNCTION DECLARATIONS
// ------------------------------------------------------------
void linefollow();

void calibrate();
void waitForStart();

void readLine();
void buildPattern();

int sensorRead(int ch);

void stopMotors();
void motor1run(int spd);
void motor2run(int spd);

bool centerLineDetected();
bool rightBranchDetected();
bool leftBranchDetected();

void peekAndScan(bool &sawL, bool &sawR, bool &sawAll);

bool turnUntilLine(int dir,
                   unsigned long minTime,
                   unsigned long maxTime);

bool confirmEnd();

void reachedEnd();

void handleJunction(bool allW, bool l, bool r);
void handleDeadEnd();

void resetState();
void finishTurn();

void logMove(char c);

// ------------------------------------------------------------
// OPTIMIZATION
// ------------------------------------------------------------
void optimizePath();

bool reduceRightHandSequence(char a,
                             char b,
                             char c,
                             char &replacement);

void printPath(const char *label,
               const char *p,
               uint8_t len);

// ------------------------------------------------------------
// FINAL RUN
// ------------------------------------------------------------
void optimizedRun();

// ============================================================
// SETUP
// ============================================================
void setup() {

  // ----------------------------------------------------------
  // Fast ADC
  // ----------------------------------------------------------
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  Serial.begin(9600);

  // ----------------------------------------------------------
  // Motor pins
  // ----------------------------------------------------------
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  // ----------------------------------------------------------
  // LED
  // ----------------------------------------------------------
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  // ----------------------------------------------------------
  // Buttons
  // ----------------------------------------------------------

  // BUTTON1 = first run
  pinMode(BUTTON1, INPUT_PULLUP);

  // BUTTON2 = final optimized run
  pinMode(BUTTON2, INPUT_PULLUP);

  // ----------------------------------------------------------
  // Standby
  // ----------------------------------------------------------
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // ----------------------------------------------------------
  // MUX select pins
  // ----------------------------------------------------------
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  // ----------------------------------------------------------
  // Initial state
  // ----------------------------------------------------------
  stopMotors();

  calibrate();

  waitForStart();
}

// ============================================================
// WAIT FOR FIRST START BUTTON
// BUTTON1 = D11
// ============================================================
void waitForStart() {

  stopMotors();

  digitalWrite(LED, LOW);

  // ----------------------------------------------------------
  // Wait until BUTTON1 is pressed
  // ----------------------------------------------------------
  while (digitalRead(BUTTON1) == HIGH) {

    stopMotors();
  }

  // ----------------------------------------------------------
  // Debounce
  // ----------------------------------------------------------
  delay(30);

  // ----------------------------------------------------------
  // Wait for button release
  // ----------------------------------------------------------
  while (digitalRead(BUTTON1) == LOW) {
    stopMotors();
  }

  // ----------------------------------------------------------
  // Reset runtime state
  // ----------------------------------------------------------
  resetState();

  junctionLockUntil = 0;
  sideMaskUntil = 0;
  lostSince = 0;

  mazeSolved = false;
  finalRunDone = false;

  pathLen = 0;
  optimizedLen = 0;
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {

  // ==========================================================
  // AFTER E HAS BEEN REACHED
  //
  // Robot stays stopped here until BUTTON2 / D12 is pressed.
  // ==========================================================
  if (mazeSolved && !finalRunDone) {

    stopMotors();

    digitalWrite(LED, HIGH);

    // --------------------------------------------------------
    // Wait for BUTTON2
    // --------------------------------------------------------
    if (digitalRead(BUTTON2) == LOW) {

      // Debounce
      delay(30);

      // Wait for release
      while (digitalRead(BUTTON2) == LOW) {

        stopMotors();
      }

      // ------------------------------------------------------
      // Start final optimized run
      // ------------------------------------------------------
      digitalWrite(LED, LOW);

      optimizedRun();

      return;
    }

    return;
  }

  // ==========================================================
  // NORMAL FIRST / LEARNING RUN
  // ==========================================================

  readLine();

  buildPattern();

  const unsigned long now = millis();

  // ----------------------------------------------------------
  // 1. NO LINE -> DEAD END
  // ----------------------------------------------------------
  if (!onLine) {

    if (lostSince == 0) {

      lostSince = now;
    }

    if (now - lostSince >= LOST_CONFIRM_MS) {

      lostSince = 0;

      handleDeadEnd();
    }

    return;
  }

  lostSince = 0;

  // ----------------------------------------------------------
  // 2. JUNCTION
  // ----------------------------------------------------------
  if (now >= junctionLockUntil) {

    const bool allW = (pattern == ALL_LINE);

    const bool r = rightBranchDetected();

    const bool l = leftBranchDetected();

    if (allW || r || l) {

      handleJunction(allW, l, r);

      return;
    }
  }

  // ----------------------------------------------------------
  // 3. NORMAL PID LINE FOLLOWING
  // ----------------------------------------------------------
  if (currentSpeed < lfSpeed) {

    ++currentSpeed;
  }

  linefollow();
}

// ============================================================
// JUNCTION HANDLER
// ============================================================
void handleJunction(bool allW,
                    bool l,
                    bool r) {

  // ----------------------------------------------------------
  // Remember anything seen at the beginning
  // ----------------------------------------------------------
  bool sawAll = allW;

  bool sawL = l || allW;

  bool sawR = r || allW;

  // ----------------------------------------------------------
  // Move forward and continuously scan
  // ----------------------------------------------------------
  peekAndScan(sawL,
              sawR,
              sawAll);

  delay(20);

  // ----------------------------------------------------------
  // Read final position
  // ----------------------------------------------------------
  readLine();

  buildPattern();

  const bool lineAhead = centerLineDetected();

  // ----------------------------------------------------------
  // ALL 16 sensors active
  // Check whether it is END
  // ----------------------------------------------------------
  if (pattern == ALL_LINE) {

    if (confirmEnd()) {

      reachedEnd();

      return;
    }

    // --------------------------------------------------------
    // Not END -> wide T / crossing
    // Right priority
    // --------------------------------------------------------
    sawR = true;
  }

  // ----------------------------------------------------------
  // RIGHT HAS PRIORITY
  // ----------------------------------------------------------
  if (sawR) {

    logMove('R');

    turnUntilLine(+1,
                  MIN_TURN_TIME,
                  MAX_TURN_TIME);

    finishTurn();

    return;
  }

  // ----------------------------------------------------------
  // LEFT BRANCH
  // ----------------------------------------------------------
  if (sawL) {

    // --------------------------------------------------------
    // Straight exists
    // --------------------------------------------------------
    if (lineAhead) {

      logMove('S');

      resetState();

      junctionLockUntil =
        millis() + JUNCTION_LOCK_MS;

      sideMaskUntil =
        millis() + SIDE_MASK_MS;

      return;
    }

    // --------------------------------------------------------
    // LEFT ONLY
    // --------------------------------------------------------
    logMove('L');

    turnUntilLine(-1,
                  MIN_TURN_TIME,
                  MAX_TURN_TIME);

    finishTurn();

    return;
  }

  // ----------------------------------------------------------
  // FALSE ALARM
  // ----------------------------------------------------------
  resetState();

  junctionLockUntil =
    millis() + TURN_LOCK_MS;
}

// ============================================================
// DEAD END / U-TURN
// ============================================================
void handleDeadEnd() {

  stopMotors();

  delay(30);

  // ----------------------------------------------------------
  // Default U-turn = RIGHT
  // ----------------------------------------------------------
  int dir = +1;

  // ----------------------------------------------------------
  // If previous line was strongly on left,
  // U-turn LEFT
  // ----------------------------------------------------------
  if (previousError > UTURN_SIDE_THRESHOLD) {

    dir = -1;
  }

  logMove('U');

  turnUntilLine(dir,
                0,
                MAX_UTURN_TIME);

  finishTurn();
}

// ============================================================
// TURN UNTIL NEW LINE FOUND
//
// dir = +1 -> RIGHT
// dir = -1 -> LEFT
//
// Phase 1: leave old line
// Phase 2: detect new line
// ============================================================
bool turnUntilLine(int dir,
                   unsigned long minTime,
                   unsigned long maxTime) {

  previousError = 0;

  I = 0;

  // ----------------------------------------------------------
  // Pivot
  // ----------------------------------------------------------
  motor1run(dir * TURN_SPEED);

  motor2run(-dir * TURN_SPEED);

  const unsigned long t0 = millis();

  bool leftOldLine = (minTime == 0);

  // ----------------------------------------------------------
  // Turn loop
  // ----------------------------------------------------------
  while (millis() - t0 < maxTime) {

    readLine();

    const bool center =
      sensorArray[7] || sensorArray[8];

    // --------------------------------------------------------
    // Make sure old line has been left
    // --------------------------------------------------------
    if (!center) {

      leftOldLine = true;
    }

    // --------------------------------------------------------
    // New line found
    // --------------------------------------------------------
    if (leftOldLine &&
        center &&
        (millis() - t0) >= minTime) {

      stopMotors();

      return true;
    }
  }

  stopMotors();

  return false;
}

// ============================================================
// FORWARD PEEK WITH CONTINUOUS SCAN
// ============================================================
void peekAndScan(bool &sawL,
                 bool &sawR,
                 bool &sawAll) {

  const unsigned long start = millis();

  while (millis() - start < PEEK_TIME) {

    const unsigned long elapsed =
      millis() - start;

    int speed;

    // --------------------------------------------------------
    // Ramp UP
    // --------------------------------------------------------
    if (elapsed < PEEK_RAMP_TIME) {

      speed = map(elapsed,
                  0,
                  PEEK_RAMP_TIME,
                  PEEK_START_SPEED,
                  PEEK_END_SPEED);
    }

    // --------------------------------------------------------
    // Full peek speed
    // --------------------------------------------------------
    else if (elapsed <
             (PEEK_TIME - PEEK_RAMP_TIME)) {

      speed = PEEK_END_SPEED;
    }

    // --------------------------------------------------------
    // Ramp DOWN
    // --------------------------------------------------------
    else {

      speed = map(elapsed,
                  PEEK_TIME - PEEK_RAMP_TIME,
                  PEEK_TIME,
                  PEEK_END_SPEED,
                  PEEK_START_SPEED);
    }

    motor1run(speed);

    motor2run(speed);

    readLine();

    buildPattern();

    // --------------------------------------------------------
    // ALL LINE
    // --------------------------------------------------------
    if (pattern == ALL_LINE) {

      sawAll = true;

      sawL = true;

      sawR = true;
    }

    // --------------------------------------------------------
    // Individual branches
    // --------------------------------------------------------
    else {

      if (leftBranchDetected()) {

        sawL = true;
      }

      if (rightBranchDetected()) {

        sawR = true;
      }
    }
  }

  stopMotors();
}

// ============================================================
// END BOX CONFIRMATION
// ============================================================
bool confirmEnd() {

  const unsigned long t0 = millis();

  motor1run(END_CONFIRM_SPEED);

  motor2run(END_CONFIRM_SPEED);

  while (millis() - t0 < END_CONFIRM_TIME) {

    readLine();

    buildPattern();

    // --------------------------------------------------------
    // Any inactive sensor -> not END
    // --------------------------------------------------------
    if (pattern != ALL_LINE) {

      stopMotors();

      return false;
    }
  }

  stopMotors();

  return true;
}

// ============================================================
// REACHED END OF FIRST / LEARNING RUN
//
// Robot stops here.
// Path gets optimized.
// Then waits for BUTTON2.
// ============================================================
void reachedEnd() {

  stopMotors();

  // ----------------------------------------------------------
  // Store END marker
  // ----------------------------------------------------------
  logMove('E');

  // ----------------------------------------------------------
  // Optimize the recorded path
  // ----------------------------------------------------------
  optimizePath();

  // ----------------------------------------------------------
  // Mark maze as solved
  // ----------------------------------------------------------
  mazeSolved = true;

  finalRunDone = false;

  // ----------------------------------------------------------
  // LED ON
  // ----------------------------------------------------------
  digitalWrite(LED, HIGH);

#if DEBUG_SERIAL

  Serial.println();

  Serial.println("==============================");

  Serial.println("E REACHED");

  printPath("RAW PATH",
            path,
            pathLen);

  printPath("OPTIMIZED PATH",
            optimizedPath,
            optimizedLen);

  Serial.println("WAITING FOR BUTTON2 (D12)");

  Serial.println("==============================");

#endif
}

// ============================================================
// STATE HELPERS
// ============================================================
void resetState() {

  previousError = 0;

  I = 0;

  currentSpeed = START_SPEED;

  lostSince = 0;
}

// ============================================================
// FINISH TURN
// ============================================================
void finishTurn() {

  stopMotors();

  delay(40);

  resetState();

  junctionLockUntil =
    millis() + TURN_LOCK_MS;

  sideMaskUntil =
    millis() + SIDE_MASK_MS;
}

// ============================================================
// LOG MOVE
// ============================================================
void logMove(char c) {

  if (pathLen < sizeof(path) - 1) {

    path[pathLen++] = c;

    path[pathLen] = '\0';
  }

#if DEBUG_SERIAL

  Serial.println(c);

#endif
}

// ============================================================
// PATH REDUCTION RULES
//
// RIGHT-HAND RULE MIRRORED FROM THE REFERENCE.
//
// Your rules:
//
// R U L -> U
// R U S -> L
// L U R -> U
// S U R -> L
// S U S -> U
// R U R -> S
// ============================================================
bool reduceRightHandSequence(char a,
                             char b,
                             char c,
                             char &replacement) {

  replacement = 0;

  // ----------------------------------------------------------
  // All reductions require U in the middle
  // ----------------------------------------------------------
  if (b != 'U') {

    return false;
  }

  // ----------------------------------------------------------
  // R U L -> U
  // ----------------------------------------------------------
  if (a == 'R' && c == 'L') {

    replacement = 'U';

    return true;
  }

  // ----------------------------------------------------------
  // R U S -> L
  // ----------------------------------------------------------
  if (a == 'R' && c == 'S') {

    replacement = 'L';

    return true;
  }

  // ----------------------------------------------------------
  // L U R -> U
  // ----------------------------------------------------------
  if (a == 'L' && c == 'R') {

    replacement = 'U';

    return true;
  }

  // ----------------------------------------------------------
  // S U R -> L
  // ----------------------------------------------------------
  if (a == 'S' && c == 'R') {

    replacement = 'L';

    return true;
  }

  // ----------------------------------------------------------
  // S U S -> U
  // ----------------------------------------------------------
  if (a == 'S' && c == 'S') {

    replacement = 'U';

    return true;
  }

  // ----------------------------------------------------------
  // R U R -> S
  // ----------------------------------------------------------
  if (a == 'R' && c == 'R') {

    replacement = 'S';

    return true;
  }

  return false;
}

// ============================================================
// OPTIMIZE PATH
//
// Example:
//
// RAW:
// R U R R U S L E
//
// Reduction:
// R U R -> S
// R U S -> L
//
// Result:
// S L L
//
// The optimizer repeatedly reduces until no more reduction
// is possible.
// ============================================================
void optimizePath() {

  optimizedLen = 0;

  // ----------------------------------------------------------
  // Read complete first-run path
  // ----------------------------------------------------------
  for (uint8_t i = 0; i < pathLen; ++i) {

    char current = path[i];

    // --------------------------------------------------------
    // E marks the end
    // --------------------------------------------------------
    if (current == 'E') {

      break;
    }

    // --------------------------------------------------------
    // Add current move
    // --------------------------------------------------------
    if (optimizedLen <
        sizeof(optimizedPath) - 1) {

      optimizedPath[optimizedLen++] =
        current;

      optimizedPath[optimizedLen] =
        '\0';
    }

    // --------------------------------------------------------
    // Repeatedly reduce the last 3 moves
    // --------------------------------------------------------
    bool reduced = true;

    while (reduced &&
           optimizedLen >= 3) {

      reduced = false;

      char a =
        optimizedPath[optimizedLen - 3];

      char b =
        optimizedPath[optimizedLen - 2];

      char c =
        optimizedPath[optimizedLen - 1];

      char replacement = 0;

      // ------------------------------------------------------
      // Check reduction
      // ------------------------------------------------------
      if (reduceRightHandSequence(a,
                                   b,
                                   c,
                                   replacement)) {

        // ----------------------------------------------------
        // Remove last 3 moves
        // ----------------------------------------------------
        optimizedLen -= 3;

        // ----------------------------------------------------
        // Add replacement
        // ----------------------------------------------------
        optimizedPath[optimizedLen++] =
          replacement;

        optimizedPath[optimizedLen] =
          '\0';

        reduced = true;
      }
    }
  }
}

// ============================================================
// PRINT PATH
// ============================================================
void printPath(const char *label,
               const char *p,
               uint8_t len) {

#if DEBUG_SERIAL

  Serial.print(label);

  Serial.print(": ");

  for (uint8_t i = 0; i < len; ++i) {

    Serial.print(p[i]);

    Serial.print(' ');
  }

  Serial.println();

#endif
}

// ============================================================
// FINAL OPTIMIZED RUN
//
// IMPORTANT:
// Robot must be manually placed back at START.
//
// During this run, it DOES NOT calculate
// RIGHT > STRAIGHT > LEFT.
//
// It follows optimizedPath[].
//
// Example:
//
// optimizedPath = R S L R
//
// Robot executes:
//
// Junction 1 -> RIGHT
// Junction 2 -> STRAIGHT
// Junction 3 -> LEFT
// Junction 4 -> RIGHT
// ============================================================
void optimizedRun() {

  // ----------------------------------------------------------
  // Stop before starting
  // ----------------------------------------------------------
  stopMotors();

  delay(300);

  resetState();

  junctionLockUntil =
    millis() + 250;

  sideMaskUntil = 0;

  lostSince = 0;

  uint8_t step = 0;

#if DEBUG_SERIAL

  Serial.println();

  Serial.println("==============================");

  Serial.println("STARTING FINAL OPTIMIZED RUN");

  printPath("PLAYBACK",
            optimizedPath,
            optimizedLen);

  Serial.println("==============================");

#endif

  // ==========================================================
  // EXECUTE EVERY OPTIMIZED DECISION
  // ==========================================================
  while (step < optimizedLen) {

    const char wantedMove =
      optimizedPath[step];

    bool decisionPointFound = false;

    bool endFound = false;

    lostSince = 0;

    // --------------------------------------------------------
    // FOLLOW LINE UNTIL NEXT JUNCTION
    // --------------------------------------------------------
    while (!decisionPointFound) {

      readLine();

      buildPattern();

      const unsigned long now =
        millis();

      // ------------------------------------------------------
      // END DETECTION
      // ------------------------------------------------------
      if (pattern == ALL_LINE &&
          now >= junctionLockUntil) {

        if (confirmEnd()) {

          endFound = true;

          break;
        }

        // ----------------------------------------------------
        // Wide junction, not END
        // ----------------------------------------------------
        decisionPointFound = true;
      }

      // ------------------------------------------------------
      // Normal junction detection
      // ------------------------------------------------------
      if (!decisionPointFound &&
          now >= junctionLockUntil) {

        const bool r =
          rightBranchDetected();

        const bool l =
          leftBranchDetected();

        if (r || l) {

          decisionPointFound = true;
        }
      }

      // ------------------------------------------------------
      // U-turn fallback
      // ------------------------------------------------------
      if (!decisionPointFound &&
          !onLine &&
          wantedMove == 'U') {

        if (lostSince == 0) {

          lostSince = millis();
        }

        if (millis() - lostSince >=
            LOST_CONFIRM_MS) {

          decisionPointFound = true;
        }
      }
      else if (onLine) {

        lostSince = 0;
      }

      // ------------------------------------------------------
      // Continue normal PID line following
      // ------------------------------------------------------
      if (!decisionPointFound &&
          !endFound) {

        if (currentSpeed < lfSpeed) {

          ++currentSpeed;
        }

        linefollow();
      }
    }

    // ========================================================
    // UNEXPECTED END
    // ========================================================
    if (endFound) {

      stopMotors();

      digitalWrite(LED, HIGH);

      finalRunDone = true;

#if DEBUG_SERIAL

      Serial.println(
        "FINAL RUN: END REACHED");

#endif

      return;
    }

    // ========================================================
    // EXECUTE STORED DECISION
    // ========================================================

#if DEBUG_SERIAL

    Serial.print("FINAL STEP ");

    Serial.print(step);

    Serial.print(": ");

    Serial.println(wantedMove);

#endif

    // --------------------------------------------------------
    // RIGHT
    // --------------------------------------------------------
    if (wantedMove == 'R') {

      turnUntilLine(+1,
                    MIN_TURN_TIME,
                    MAX_TURN_TIME);

      finishTurn();
    }

    // --------------------------------------------------------
    // LEFT
    // --------------------------------------------------------
    else if (wantedMove == 'L') {

      turnUntilLine(-1,
                    MIN_TURN_TIME,
                    MAX_TURN_TIME);

      finishTurn();
    }

    // --------------------------------------------------------
    // STRAIGHT
    // --------------------------------------------------------
    else if (wantedMove == 'S') {

      resetState();

      // Ignore the side branch just passed
      junctionLockUntil =
        millis() + JUNCTION_LOCK_MS;

      sideMaskUntil =
        millis() + SIDE_MASK_MS;
    }

    // --------------------------------------------------------
    // U-TURN
    //
    // Normally most U turns should disappear during
    // optimization, but this is kept as a safety fallback.
    // --------------------------------------------------------
    else if (wantedMove == 'U') {

      int dir = +1;

      if (previousError >
          UTURN_SIDE_THRESHOLD) {

        dir = -1;
      }

      turnUntilLine(dir,
                    0,
                    MAX_UTURN_TIME);

      finishTurn();
    }

    // --------------------------------------------------------
    // Next optimized move
    // --------------------------------------------------------
    ++step;
  }

  // ==========================================================
  // ALL OPTIMIZED DECISIONS COMPLETED
  //
  // Continue straight until END is detected.
  // ==========================================================
  while (true) {

    readLine();

    buildPattern();

    // --------------------------------------------------------
    // Check END
    // --------------------------------------------------------
    if (pattern == ALL_LINE) {

      if (confirmEnd()) {

        stopMotors();

        digitalWrite(LED, HIGH);

        finalRunDone = true;

#if DEBUG_SERIAL

        Serial.println();

        Serial.println(
          "==============================");

        Serial.println(
          "OPTIMIZED FINAL RUN COMPLETE");

        Serial.println(
          "==============================");

#endif

        return;
      }
    }

    // --------------------------------------------------------
    // Lost line
    // --------------------------------------------------------
    if (!onLine) {

      stopMotors();

      return;
    }

    // --------------------------------------------------------
    // Continue PID
    // --------------------------------------------------------
    if (currentSpeed < lfSpeed) {

      ++currentSpeed;
    }

    linefollow();
  }
}

// ============================================================
// PID LINE FOLLOW
//
// Error = weighted centroid.
//
// After straight at a junction,
// sensors 4..11 are temporarily used
// to mask the side branch.
// ============================================================
void linefollow() {

  int lo = 0;

  int hi = 15;

  if (millis() < sideMaskUntil) {

    lo = 4;

    hi = 11;
  }

  float num = 0;

  float den = 0;

  for (int i = lo; i <= hi; ++i) {

    if (sensorArray[i]) {

      num +=
        (float)sensorWeight[i] *
        sensorValue[i];

      den += sensorValue[i];
    }
  }

  if (den > 0) {

    error = num / den;
  }
  else {

    error = previousError;
  }

  P = error;

  I += error;

  I = constrain(I,
                -1000,
                1000);

  D = error - previousError;

  PIDvalue =
    (Kp * P) +
    (Ki * I) +
    (Kd * D);

  previousError = error;

  lsp = (int)constrain(
    currentSpeed - PIDvalue,
    -100,
    200
  );

  rsp = (int)constrain(
    currentSpeed + PIDvalue,
    -100,
    200
  );

  motor1run(lsp);

  motor2run(rsp);
}

// ============================================================
// CALIBRATION
// ============================================================
void calibrate() {

  for (int i = 0; i < 16; ++i) {

    minValues[i] =
      sensorRead(i);

    maxValues[i] =
      minValues[i];
  }

  const unsigned long startTime =
    millis();

  while (millis() - startTime <
         4000UL) {

    motor1run(70);

    motor2run(-70);

    for (int i = 0; i < 16; ++i) {

      const int value =
        sensorRead(i);

      if (value < minValues[i]) {

        minValues[i] = value;
      }

      if (value > maxValues[i]) {

        maxValues[i] = value;
      }
    }
  }

  stopMotors();

  delay(300);
}

// ============================================================
// READ LINE
// ============================================================
void readLine() {

  onLine = false;

  for (int i = 0; i < 16; ++i) {

    const int rawValue =
      sensorRead(i);

    if (!isBlackLine) {

      sensorValue[i] =
        map(rawValue,
            minValues[i],
            maxValues[i],
            1000,
            0);
    }
    else {

      sensorValue[i] =
        map(rawValue,
            minValues[i],
            maxValues[i],
            0,
            1000);
    }

    sensorValue[i] =
      constrain(sensorValue[i],
                0,
                1000);

    sensorArray[i] =
      (sensorValue[i] > 500);

    if (sensorArray[i]) {

      onLine = true;
    }
  }
}

// ============================================================
// BUILD SENSOR PATTERN
//
// Bit 15 = sensor 0
// Bit 0  = sensor 15
// ============================================================
void buildPattern() {

  pattern = NO_LINE;

  for (int i = 0; i < 16; ++i) {

    pattern =
      (pattern << 1) |
      (sensorArray[i] ? 1 : 0);
  }
}

// ============================================================
// BRANCH DETECTION
// ============================================================

// ------------------------------------------------------------
// Straight / ahead
// ------------------------------------------------------------
bool centerLineDetected() {

  return sensorArray[6] ||
         sensorArray[7] ||
         sensorArray[8] ||
         sensorArray[9];
}

// ------------------------------------------------------------
// Right branch
// 2 of 3 sensors 13,14,15 active
// ------------------------------------------------------------
bool rightBranchDetected() {

  return
    (sensorArray[13] +
     sensorArray[14] +
     sensorArray[15]) >= 2;
}

// ------------------------------------------------------------
// Left branch
// 2 of 3 sensors 0,1,2 active
// ------------------------------------------------------------
bool leftBranchDetected() {

  return
    (sensorArray[0] +
     sensorArray[1] +
     sensorArray[2]) >= 2;
}

// ============================================================
// MOTORS
// ============================================================
void stopMotors() {

  motor1run(0);

  motor2run(0);
}

// ============================================================
// MOTOR 1
// ============================================================
void motor1run(int spd) {

  spd = constrain(spd,
                  -255,
                  255);

  if (spd > 0) {

    digitalWrite(IN1, HIGH);

    digitalWrite(IN2, LOW);

    analogWrite(enA, spd);
  }
  else if (spd < 0) {

    digitalWrite(IN1, LOW);

    digitalWrite(IN2, HIGH);

    analogWrite(enA, -spd);
  }
  else {

    digitalWrite(IN1, HIGH);

    digitalWrite(IN2, HIGH);

    analogWrite(enA, 0);
  }
}

// ============================================================
// MOTOR 2
// ============================================================
void motor2run(int spd) {

  spd = constrain(spd,
                  -255,
                  255);

  if (spd > 0) {

    digitalWrite(IN3, HIGH);

    digitalWrite(IN4, LOW);

    analogWrite(enB, spd);
  }
  else if (spd < 0) {

    digitalWrite(IN3, LOW);

    digitalWrite(IN4, HIGH);

    analogWrite(enB, -spd);
  }
  else {

    digitalWrite(IN3, HIGH);

    digitalWrite(IN4, HIGH);

    analogWrite(enB, 0);
  }
}

// ============================================================
// 16:1 ANALOG MUX SENSOR READ
// ============================================================
int sensorRead(int ch) {

  digitalWrite(S0, ch & 1);

  digitalWrite(S1, ch & 2);

  digitalWrite(S2, ch & 4);

  digitalWrite(S3, ch & 8);

  delayMicroseconds(5);

  return analogRead(sensorPin);
}