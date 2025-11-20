/*
  Updated Arduino sketch

  Features for Debugging:
    - Mode 1 distance forward and backward: 4.52 m (452 cm)
    - Mode 2 distance: 2.44 m (244 cm)
    - Start logic starts automatically for Mode 1
    - Set pin 7 HIGH at startup
    - Read pin 5 at startup: if HIGH -> Mode 2, else (LOW) -> Mode 1
    - Wait 3 seconds after determining startup mode before starting the motion
    - Other features retained: interrupt-only tachometer on pin 3, safe servo init,
*/

#include <Servo.h>

// ---------------- Pins ----------------
const int servoPin  = 9;
const int irPin     = 3;    // Tachometer input (interrupt-only)
const int enablePin = 11;   // PWM
const int in1       = 12;
const int in2       = 13;

const int modeSelectPin = 7; // read at startup to choose mode (HIGH -> Mode2, LOW -> Mode1)
const int pin5ToSetHigh = 5; // will be set HIGH at startup

// ---------------- Physical constants ----------------
const float wheelRadius_m = 0.02605;    // 2.605 cm
const int pulsesPerRev = 2;             // two black bands -> two pulses per rotation
const float PI_F = 3.14159265358979323846;
const float wheelCircumference_m = 2.0 * PI_F * wheelRadius_m;

// ---------------- Mode distances (meters) ----------------
const float MODE1_PHASE_DISTANCE = 4.52; // 452 cm forward, then 4.52 m back
const float MODE2_DISTANCE = 2.44;       // 244 cm total travel for Mode2

// ---------------- Runtime (volatile updated in ISR) ----------------
volatile unsigned long pulseCount = 0;
volatile unsigned long lastPulseMicros = 20; // for debounce in ISR
const unsigned long debounceMicros = 25000UL; // 25 ms debounce in ISR

// motor speeds (0-255)
int motorSpeedHigh = 255;
int motorSpeedLow  = 90;

// mode flags
int currentMode = 0; // 0 none, 1 Mode1, 2 Mode2
bool mode1Running = false;
bool mode2Running = false;

// Servo: logical mapping and user tuning
Servo servo;
// logicalZero is the offset (in degrees) we add to userOffset to get physical command:
// physicalAngle = logicalZero + userOffset
int servoLogicalZero = 0;   // logical zero reference (initially 0)
int servoUserOffset = 43;    // adjustment the user makes with + / -
const int servoRotate = 120; // how many degrees Mode2 rotates relative to logical zero

// stats printing interval (used only when a mode is running)
const unsigned long statsInterval = 20; // ms
unsigned long lastStatsMillis = 0;

// for RPM/speed windows
unsigned long lastWindowMillis = 0;
unsigned long lastWindowPulseCount = 0;

// Mode state variables (global so we can reset them when restarting modes)
bool m1_reversed = false;
float m1_previousPhaseDistance = 0.0;

enum { M2_PHASE_START, M2_PHASE_RUNNING, M2_PHASE_DONE };
int m2_phase = M2_PHASE_START;
float m2_phaseStartDist = 0.0;

// ---------------- Forward declarations ---------------- (Noah)
void stopMotor();
void setMotorForward(int pwmVal);
void setMotorBackward(int pwmVal);
float getDistanceMeters();
float getSpeed_m_s(unsigned long windowMillis);
float getRPM(unsigned long windowMillis);
void tachISR();
void handleSerial(const String &sIn);
void safeServoInit();
void servoToZero();
void servoRotateAndReset();
void servoWriteClamped(int deg);

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; } // optional wait for Serial

  // pins
  pinMode(irPin, INPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Mode select pins
  pinMode(modeSelectPin, INPUT);
  pinMode(pin5ToSetHigh, OUTPUT);
  digitalWrite(pin5ToSetHigh, HIGH); // Set pin 7 HIGH at startup

  stopMotor();

  // Safe servo init: attach but DO NOT command an initial position.
  safeServoInit();

  lastWindowMillis = millis();
  lastWindowPulseCount = 0;
  lastStatsMillis = millis();

  // attach interrupt on irPin (pin 3)
  int interruptNum = digitalPinToInterrupt(irPin);
  if (interruptNum == NOT_AN_INTERRUPT) {
    Serial.println(F("ERROR: chosen IR pin is not interrupt-capable. Use a board that supports interrupts on pin 3."));
    while (1) { delay(1000); } // halt to avoid running without tachometer
  }
  attachInterrupt(interruptNum, tachISR, CHANGE);
  Serial.print(F("attachInterrupt() on pin "));
  Serial.println(irPin);

  // Determine startup mode from pin 5
  int pin5state = digitalRead(modeSelectPin);
  if (pin5state == HIGH) {
    currentMode = 2;
    mode2Running = true;
    mode1Running = false;
    // reset tachometer counters for fresh run
    noInterrupts(); pulseCount = 0; interrupts();
    lastWindowMillis = millis();
    lastWindowPulseCount = 0;
    lastStatsMillis = millis();
    // reset Mode2 state
    m2_phase = M2_PHASE_START;
    m2_phaseStartDist = 0.0;
    Serial.println(F("Startup: pin 5 HIGH -> Mode 2 selected."));
  } else {
    currentMode = 1;
    mode1Running = true;
    mode2Running = false;
    // reset tachometer counters for fresh run
    noInterrupts(); pulseCount = 0; interrupts();
    lastWindowMillis = millis();
    lastWindowPulseCount = 0;
    lastStatsMillis = millis();
    // reset Mode1 state
    m1_reversed = false;
    m1_previousPhaseDistance = 0.0;
    Serial.println(F("Startup: pin 5 LOW -> Mode 1 selected."));
  }

  Serial.println(F("Waiting 3 seconds before starting the chosen mode..."));
  delay(3000); // wait 3 seconds after mode is known

  // Start the selected mode immediately (no "Start" button)
  if (currentMode == 2 && mode2Running) {
    // Mode2 start
    m2_phase = M2_PHASE_START;
    // set motor forward - actual motor command happens in loop initial phase
    Serial.println(F("Mode 2 starting now."));
  }
  if (currentMode == 1 && mode1Running) {
    // Mode1 start: begin driving forward immediately
    m1_reversed = false;
    m1_previousPhaseDistance = 0.0;
    setMotorForward(motorSpeedHigh);
    Serial.println(F("Mode 1 starting now."));
  }

  Serial.println(F("Ready. Serial commands still available (Mode 1/2, Stop, SetHigh, SetLow, +, -, SetServo)."));
}

// ---------------- Loop ---------------- (Rayan)
void loop() {
  // Serial command processing (non-blocking)
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) handleSerial(line);
  }

  // Stats printing only when a mode is actively running
  bool printingActive = (mode1Running || mode2Running);
  if (printingActive && (millis() - lastStatsMillis >= statsInterval)) {
    lastStatsMillis = millis();
    float dist = getDistanceMeters();
    float rpm = getRPM(statsInterval);             // wheel RPM (pulses -> RPM -> divide by pulsesPerRev)
    float speed = getSpeed_m_s(statsInterval);
    Serial.print(F("Distance (m): "));
    Serial.print(dist, 3);
    Serial.print(F("  |  Wheel RPM: "));
    Serial.print(rpm, 2);
    Serial.print(F("  |  Speed (m/s): "));
    Serial.println(speed, 3);
  }

  // Mode 1 logic: forward MODE1_PHASE_DISTANCE, then reverse and go another MODE1_PHASE_DISTANCE
  if (mode1Running) {
    float dist = getDistanceMeters();
    if (!m1_reversed) {
      // driving forward phase
      if (dist >= MODE1_PHASE_DISTANCE + 0.25) {
        Serial.println(F("Mode1: reached forward distance. Reversing..."));
        m1_reversed = true;
        m1_previousPhaseDistance = dist;
        // reverse motor direction and run until distance increases by MODE1_PHASE_DISTANCE
        setMotorBackward(motorSpeedHigh);
      }
    } else {
      // reversed phase: stop after additional MODE1_PHASE_DISTANCE
      if (dist >= (m1_previousPhaseDistance + MODE1_PHASE_DISTANCE - 0.0001)) {
        stopMotor();
        Serial.println(F("Mode 1 complete. Final stats:"));
        Serial.print(F("Total distance (m): "));
        Serial.println(getDistanceMeters(), 4);
        mode1Running = false;
        currentMode = 0;
      }
    }
  }

  // Mode 2 logic: drive until MODE2_DISTANCE, stop and trigger servo
  if (mode2Running) {
    float dist = getDistanceMeters();

    if (m2_phase == M2_PHASE_START) {
      m2_phase = M2_PHASE_RUNNING;
      m2_phaseStartDist = dist;
      setMotorForward(motorSpeedHigh);
      Serial.println(F("Mode2: started driving toward target distance."));
    }
    else if (m2_phase == M2_PHASE_RUNNING) {
      // Stop when absolute traveled distance >= MODE2_DISTANCE (since reset at start)
      if (dist >= MODE2_DISTANCE) {
        stopMotor();
        Serial.println(F("Mode2: reached target distance. Activating servo..."));
        servoRotateAndReset();
        Serial.println(F("Mode2: servo action complete."));
        m2_phase = M2_PHASE_DONE;
        mode2Running = false;
        currentMode = 0;
      }
    }
  }

  delay(2); // keep loop responsive
}

// ---------------- Tachometer ISR ---------------- (Noah)
// very short ISR: debounce and increment pulseCount
void tachISR() {
  unsigned long now = micros();
  if (now - lastPulseMicros >= debounceMicros) {
    pulseCount++;
    lastPulseMicros = now;
  }
}

// ---------------- Measurement helpers ---------------- (Rayan)
float getDistanceMeters() {
  unsigned long pulses;
  noInterrupts();
  pulses = pulseCount;
  interrupts();
  float revs = (float)pulses / (float)pulsesPerRev; // number of wheel revolutions
  return revs * wheelCircumference_m;
}

// get speed (m/s) computed over the provided window (ms)
float getSpeed_m_s(unsigned long windowMillis) {
  unsigned long now = millis();
  static unsigned long prevMillis = 0;
  static unsigned long prevPulseCount = 0;

  if (prevMillis == 0) {
    prevMillis = now;
    noInterrupts();
    prevPulseCount = pulseCount;
    interrupts();
    return 0.0;
  }

  if (now - prevMillis >= windowMillis) {
    unsigned long currentPulses;
    noInterrupts();
    currentPulses = pulseCount;
    interrupts();

    unsigned long deltaPulses = currentPulses - prevPulseCount;
    float revs = (float)deltaPulses / (float)pulsesPerRev;
    float meters = revs * wheelCircumference_m;
    float seconds = (now - prevMillis) / 1000.0;
    float speed = (seconds > 0.0) ? (meters / seconds) : 0.0;

    prevMillis = now;
    prevPulseCount = currentPulses;
    return speed;
  } else {
    return 0.0;
  }
}

// get wheel RPM over windowMillis (ms).
// Per instruction: compute from pulses-per-minute then divide by pulsesPerRev (2).
float getRPM(unsigned long windowMillis) {
  unsigned long now = millis();
  static unsigned long prevMillis = 0;
  static unsigned long prevPulseCount = 0;

  if (prevMillis == 0) {
    prevMillis = now;
    noInterrupts();
    prevPulseCount = pulseCount;
    interrupts();
    return 0.0;
  }

  if (now - prevMillis >= windowMillis) {
    unsigned long currentPulses;
    noInterrupts();
    currentPulses = pulseCount;
    interrupts();

    unsigned long deltaPulses = currentPulses - prevPulseCount;
    float minutes = (now - prevMillis) / 60000.0;
    float pulsesPerMinute = (minutes > 0.0) ? ((float)deltaPulses / minutes) : 0.0;
    float wheelRPM = pulsesPerMinute / (float)pulsesPerRev; // divide-by-2 step
    prevMillis = now;
    prevPulseCount = currentPulses;
    return wheelRPM;
  } else {
    return 0.0;
  }
}

// ---------------- Motor helpers ---------------- (Rayan)
void setMotorForward(int pwmVal) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enablePin, constrain(pwmVal, 0, 255));
}

void setMotorBackward(int pwmVal) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enablePin, constrain(pwmVal, 0, 255));
}

void stopMotor() {
  analogWrite(enablePin, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

// ---------------- Serial command handler ---------------- (Rayan)
void handleSerial(const String &sIn) {
  String s = sIn;
  s.trim();

  // Single-character servo adjustments: "+" or "-"
  if (s == "+") {
    // increment user offset by +1 (attempt to move by 1 degree)
    int desired = servoLogicalZero + servoUserOffset + 1;
    desired = constrain(desired, 0, 180);
    servoUserOffset = desired - servoLogicalZero;
    servoWriteClamped(desired);
    Serial.print(F("Servo moved to "));
    Serial.print(desired);
    Serial.println(F(" deg (user offset incremented)."));
    return;
  }
  if (s == "-") {
    int desired = servoLogicalZero + servoUserOffset - 1;
    desired = constrain(desired, 0, 180);
    servoUserOffset = desired - servoLogicalZero;
    servoWriteClamped(desired);
    Serial.print(F("Servo moved to "));
    Serial.print(desired);
    Serial.println(F(" deg (user offset decremented)."));
    return;
  }

  if (s.equalsIgnoreCase("SetServo")) {
    // Make current physical angle the new logical zero.
    // That means logicalZero += userOffset; userOffset = 0;
    servoLogicalZero += servoUserOffset;
    // clamp logical zero to keep physical writes valid later
    servoLogicalZero = constrain(servoLogicalZero, 0, 180);
    servoUserOffset = 0;
    Serial.print(F("Set current servo position as logical 0°. New logicalZero = "));
    Serial.println(servoLogicalZero);
    return;
  }

  // Allow overriding mode via serial
  if (s.equalsIgnoreCase("Mode 1")) {
    currentMode = 1;
    mode1Running = true;
    mode2Running = false;
    // reset Mode1 state
    m1_reversed = false;
    m1_previousPhaseDistance = 0.0;
    // reset tachometer counters for fresh run
    noInterrupts(); pulseCount = 0; interrupts();
    lastWindowMillis = millis();
    lastWindowPulseCount = 0;
    lastStatsMillis = millis();
    // start immediately
    setMotorForward(motorSpeedHigh);
    Serial.println(F("Mode 1 selected via Serial and started."));
    return;
  }

  if (s.equalsIgnoreCase("Mode 2")) {
    currentMode = 2;
    mode2Running = true;
    mode1Running = false;
    // reset tachometer counters for fresh run
    noInterrupts(); pulseCount = 0; interrupts();
    lastWindowMillis = millis();
    lastWindowPulseCount = 0;
    lastStatsMillis = millis();
    // reset Mode2 state
    m2_phase = M2_PHASE_START;
    m2_phaseStartDist = 0.0;
    Serial.println(F("Mode 2 selected via Serial and started."));
    return;
  }

  if (s.equalsIgnoreCase("Stop")) {
    stopMotor();
    mode1Running = false;
    mode2Running = false;
    currentMode = 0;
    Serial.println(F("Stopped and cancelled modes."));
    return;
  }

  if (s.startsWith("SetHigh")) {
    int idx = s.indexOf(' ');
    if (idx > 0) {
      String arg = s.substring(idx+1); arg.trim();
      int v = arg.toInt();
      v = constrain(v, 0, 255);
      motorSpeedHigh = v;
      Serial.print(F("motorSpeedHigh set to "));
      Serial.println(motorSpeedHigh);
    } else {
      Serial.println(F("Usage: SetHigh <0-255>"));
    }
    return;
  }

  if (s.startsWith("SetLow")) {
    int idx = s.indexOf(' ');
    if (idx > 0) {
      String arg = s.substring(idx+1); arg.trim();
      int v = arg.toInt();
      v = constrain(v, 0, 255);
      motorSpeedLow = v;
      Serial.print(F("motorSpeedLow set to "));
      Serial.println(motorSpeedLow);
    } else {
      Serial.println(F("Usage: SetLow <0-255>"));
    }
    return;
  }

  if (s.equalsIgnoreCase("GetSpeeds")) {
    Serial.print(F("motorSpeedHigh = "));
    Serial.print(motorSpeedHigh);
    Serial.print(F("  |  motorSpeedLow = "));
    Serial.println(motorSpeedLow);
    return;
  }

  // Unknown
  Serial.print(F("Unknown command: "));
  Serial.println(s);
}

// ---------------- Servo helpers ---------------- (Noah)
// Attach servo but offset initial position
void safeServoInit() {
  servo.attach(servoPin);
  delay(100); // give servo time to power up and stabilize — do NOT write a position
  servoLogicalZero = 0;  // logical reference: current physical position == logical 0°
  servoUserOffset = 43;   // no user offset at startup
  Serial.println(F("Servo initialized — current physical position set as logical 0° (no movement)."));
  servo.write(43);
}

void servoToZero() {
  servoWriteClamped(servoLogicalZero + servoUserOffset); // normally userOffset is 0 here
}

void servoRotateAndReset() {
  delay(1500);
  // rotate relative to logical zero, then return
  int target = servoLogicalZero + servoRotate;
  target = constrain(target, 0, 180);
  servoWriteClamped(target);
  delay(300);
  // return to logical zero (user offset should be 0)
  servoWriteClamped(servoLogicalZero);
}

// central helper that clamps and writes servo angle
void servoWriteClamped(int deg) {
  deg = constrain(deg, 0, 180);
  servo.write(deg);
}
