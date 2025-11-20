/*
  optimized tbwc control code
  authors: rayan and noah
  functionality: mode 1 (speed run) & mode 2 (launch)
*/

#include <Servo.h>

// --- pin definitions ---
const int pin_servo = 9;       // servo motor pin
const int pin_tach  = 3;       // tachometer/hall sensor pin (interrupt)
const int pin_pwm   = 11;      // motor speed control
const int pin_dir1  = 12;      // motor direction pin 1
const int pin_dir2  = 13;      // motor direction pin 2
const int pin_mode  = 7;       // mode select input
const int pin_pwr   = 5;       // provides high signal for mode pin

// --- constants & settings ---
const float wheel_circ = 0.16368; // wheel circ in meters (2 * pi * 0.02605)
const float dist_mode1 = 4.52;    // target distance for mode 1 (meters)
const float dist_mode2 = 2.44;    // target distance for mode 2 (meters)

// --- global variables ---
volatile unsigned long pulses = 0; // counts wheel rotations (isr)
volatile unsigned long last_time = 0; // for debounce
int speed_high = 255;             // max motor speed
int speed_low  = 90;              // slow motor speed
int mode = 0;                     // 0=idle, 1=speed run, 2=launch
int servo_zero = 0;               // logical zero for servo
int servo_offset = 43;            // manual adjustment
bool run_mode1 = false;           // flag for mode 1 active
bool run_mode2 = false;           // flag for mode 2 active
bool m1_reverse = false;          // flag for mode 1 reverse phase
float m1_prev_dist = 0;           // stores distance before reversing
int m2_stage = 0;                 // 0=start, 1=driving, 2=done
Servo my_servo;                   // servo object

// --- helper: motor control ---
// input: positive for fwd, negative for bwd, 0 for stop
void set_motor(int speed) {
  digitalWrite(pin_dir1, speed > 0);     // set direction 1
  digitalWrite(pin_dir2, speed < 0);     // set direction 2
  analogWrite(pin_pwm, constrain(abs(speed), 0, 255)); // set pwm
}

// --- helper: get distance ---
// converts pulse count to meters safely
float get_dist() {
  noInterrupts();                        // pause interrupts
  unsigned long p = pulses;              // copy variable safely
  interrupts();                          // resume interrupts
  return (p / 2.0) * wheel_circ;         // calc meters (2 pulses per rev)
}

// --- helper: safe servo write ---
// moves servo while keeping angle within limits
void set_servo(int angle) {
  my_servo.write(constrain(angle, 0, 180));
}

// --- interrupt service routine (isr) ---
// counts pulses from tachometer
void count_pulse() {
  unsigned long now = micros();
  // check if enough time passed (debounce)
  if (now - last_time >= 25000) {
    pulses++;
    last_time = now;
  }
}

void setup() {
  Serial.begin(115200); while(!Serial);  // start serial
  
  // config pins
  pinMode(pin_tach, INPUT);
  pinMode(pin_pwm, OUTPUT); 
  pinMode(pin_dir1, OUTPUT); 
  pinMode(pin_dir2, OUTPUT);
  pinMode(pin_mode, INPUT);
  pinMode(pin_pwr, OUTPUT);
  digitalWrite(pin_pwr, HIGH);           // set pin 5 high for mode select
  
  // init hardware
  set_motor(0);                          // stop motor
  my_servo.attach(pin_servo);            // connect servo
  delay(100);                            // wait for power
  set_servo(servo_offset);               // move to start pos
  
  // attach interrupt for tachometer
  attachInterrupt(digitalPinToInterrupt(pin_tach), count_pulse, CHANGE);

  // check startup mode
  if (digitalRead(pin_mode)) {
    mode = 2; run_mode2 = true;          // pin 7 high -> mode 2
    Serial.println(F("startup: mode 2 selected"));
  } else {
    mode = 1; run_mode1 = true;          // pin 7 low -> mode 1
    Serial.println(F("startup: mode 1 selected"));
  }
  
  delay(3000);                           // safety delay before start
  
  // reset counters and start
  noInterrupts(); pulses = 0; interrupts();
  if (mode == 1) set_motor(speed_high);  // start mode 1 immediately
}

void loop() {
  // --- serial commands ---
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n'); cmd.trim();
    // adjust servo offset
    if (cmd == "+") { servo_offset++; set_servo(servo_zero + servo_offset); }
    else if (cmd == "-") { servo_offset--; set_servo(servo_zero + servo_offset); }
    // save current pos as zero
    else if (cmd.equalsIgnoreCase("SetServo")) { 
      servo_zero += servo_offset; servo_offset = 0; 
    }
    // emergency stop
    else if (cmd.equalsIgnoreCase("Stop")) { 
      set_motor(0); run_mode1 = 0; run_mode2 = 0; 
    }
    // manual overrides
    else if (cmd.equalsIgnoreCase("Mode 1")) { 
      mode = 1; run_mode1 = true; run_mode2 = false; 
      m1_reverse = false; noInterrupts(); pulses = 0; interrupts();
      set_motor(speed_high); 
    }
    else if (cmd.equalsIgnoreCase("Mode 2")) { 
      mode = 2; run_mode2 = true; run_mode1 = false; 
      m2_stage = 0; noInterrupts(); pulses = 0; interrupts(); 
    }
  }

  // --- stats printing (every 20ms) ---
  static unsigned long last_print = 0;
  if ((run_mode1 || run_mode2) && (millis() - last_print >= 20)) {
    last_print = millis();
    Serial.print(F("dist: ")); Serial.println(get_dist());
  }

  // --- mode 1 logic (speed run) ---
  if (run_mode1) {
    float d = get_dist();
    // phase 1: forward
    if (!m1_reverse) {
      if (d >= dist_mode1 + 0.25) {      // add buffer for stop
        m1_reverse = true;               // switch phase
        m1_prev_dist = d;                // save distance
        set_motor(-speed_high);          // reverse motor
      }
    } 
    // phase 2: backward
    else {
      if (d >= m1_prev_dist + dist_mode1) {
        set_motor(0);                    // stop
        run_mode1 = false;               // end mode
        Serial.println(F("mode 1 complete"));
      }
    }
  }

  // --- mode 2 logic (launch) ---
  if (run_mode2) {
    float d = get_dist();
    if (m2_stage == 0) {                 // start phase
      m2_stage = 1; 
      set_motor(speed_high);             // go forward
    } 
    else if (m2_stage == 1 && d >= dist_mode2) {
      set_motor(0);                      // stop at target
      delay(100);                        // settle
      set_servo(servo_zero + 120);       // dump trash
      delay(1500);                       // wait for dump
      set_servo(servo_zero);             // return servo
      m2_stage = 2;                      // done
      run_mode2 = false;                 // end mode
      Serial.println(F("mode 2 complete"));
    }
  }
  delay(2);                              // stability delay
}
