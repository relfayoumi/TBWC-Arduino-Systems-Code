# TBWC Arduino Systems Code

Arduino firmware for the Teddy Bear Wheelchair (TBWC) autonomous robot system. This code controls the robot's navigation, motor control, distance measurement, and servo actuation for competition tasks.

## Table of Contents
- [About the TBWC Project](#about-the-tbwc-project)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Pin Configuration](#pin-configuration)
- [Installation & Setup](#installation--setup)
- [Operation Modes](#operation-modes)
- [Serial Commands](#serial-commands)
- [Configuration & Calibration](#configuration--calibration)
- [Technical Details](#technical-details)
- [Troubleshooting](#troubleshooting)

## About the TBWC Project

The Teddy Bear Wheelchair (TBWC) is an autonomous robot designed for precision navigation and task completion. This Arduino code implements the control system for:
- Precise distance-based navigation using wheel tachometry
- Two independent operational modes for different competition tasks
- Real-time speed and position feedback
- Servo-controlled actuation for task completion

The system uses interrupt-driven wheel encoding for accurate distance measurement and supports both autonomous startup and serial command control.

## Features

### Core Functionality
- **Dual Operation Modes**: Two pre-programmed autonomous routines (Mode 1 and Mode 2)
- **Precision Distance Tracking**: Interrupt-based tachometer with real-time distance calculation
- **Automated Startup**: Pin-based mode selection at power-on with 3-second delay
- **Speed Monitoring**: Real-time wheel RPM and linear speed (m/s) calculation
- **Servo Control**: Calibrated servo actuation for task completion in Mode 2
- **Serial Interface**: Full manual control and debugging via serial commands

### Mode 1: Round-Trip Navigation
- Forward travel: 4.52 meters (452 cm)
- Automatic reversal at target distance
- Backward travel: 4.52 meters (452 cm)
- Total distance: 9.04 meters
- Optimized for speed run tasks

### Mode 2: Distance + Servo Action
- Forward travel: 2.44 meters (244 cm)
- Stops at target distance
- Executes 120° servo rotation and return
- Designed for trash launch or actuation tasks

### Real-Time Feedback
- Distance traveled (meters, 3 decimal precision)
- Wheel RPM (revolutions per minute)
- Linear speed (meters per second)
- 20ms update interval during operation

## Hardware Requirements

### Electronics
- **Arduino Board**: Uno, Nano, or compatible (ATmega328P-based)
- **Motor Driver**: H-bridge driver (e.g., L298N, L293D) supporting PWM control
- **IR Sensor**: Optical tachometer or photointerrupter for wheel speed measurement
- **Servo Motor**: Standard hobby servo (0-180°)
- **Power Supply**: Appropriate for motors and electronics (typically 7-12V for motors, 5V for Arduino)

### Mechanical Components
- Drive wheels with radius: 26.05mm (2.605 cm)
- Two black bands per wheel for tachometry (2 pulses per revolution)
- Servo mounting for actuation mechanism

### Wiring Requirements
- Common ground between Arduino, motor driver, and sensors
- Separate power supply for motors (do not power motors from Arduino 5V pin)
- Interrupt-capable pin for tachometer (Pin 3 recommended)

## Pin Configuration

```cpp
// Motor Control
Pin 11 (PWM)  - Motor Enable (speed control)
Pin 12        - Motor Direction IN1
Pin 13        - Motor Direction IN2

// Sensors & Actuators
Pin 3         - IR Tachometer (interrupt-capable, REQUIRED)
Pin 9 (PWM)   - Servo Signal

// Mode Selection
Pin 7         - Mode Select Input (read at startup)
Pin 5         - Status Output (set HIGH at startup)
```

### Pin 7 Mode Selection Logic
- **HIGH** at startup → Mode 2 (2.44m + servo action)
- **LOW** at startup → Mode 1 (4.52m forward, 4.52m reverse)

## Installation & Setup

### Arduino IDE
1. Install Arduino IDE (version 1.8.x or 2.x)
2. Install the Servo library (usually included by default)
3. Open `CompletedTWBCOptimizedCode.ino`
4. Select **Tools → Board → Arduino Uno** (or your board)
5. Select **Tools → Port** → Your Arduino's COM port
6. Click **Upload** (→ button)
7. Open **Tools → Serial Monitor** at **115200 baud**

### Hardware Setup
1. Connect motor driver to Arduino (pins 11, 12, 13)
2. Connect IR tachometer to Pin 3 (must be interrupt-capable)
3. Connect servo to Pin 9
4. Wire Pin 7 for mode selection (HIGH/LOW switch)
5. Verify Pin 5 output is available
6. Ensure common ground across all components
7. Power motors from external supply, not Arduino 5V

### First Run Verification
```
1. Upload code with Pin 7 set LOW (Mode 1)
2. Serial Monitor should show:
   - "Startup: pin 5 LOW -> Mode 1 selected."
   - "Waiting 3 seconds before starting..."
   - Real-time distance/speed/RPM data
3. Robot should drive forward, then reverse automatically
```

## Operation Modes

### Automatic Startup Sequence
1. Power on Arduino
2. System reads Pin 7 to determine mode
3. Pin 5 is set HIGH (status indicator)
4. 3-second delay before starting
5. Selected mode begins automatically

### Mode 1: Round-Trip Run
**Purpose**: Speed run or long-distance navigation

**Sequence**:
1. Drive forward at high speed (PWM 255)
2. Monitor distance until 4.52m reached (with 0.25m buffer)
3. Reverse direction automatically
4. Drive backward at high speed
5. Stop when total distance reaches 9.04m
6. Print final statistics

**Use Case**: Competition speed runs, straight-line tests

### Mode 2: Approach & Actuate
**Purpose**: Precision positioning with servo action

**Sequence**:
1. Drive forward at high speed (PWM 255)
2. Monitor distance until 2.44m reached
3. Stop motor
4. Wait 1.5 seconds
5. Rotate servo 120° from home position
6. Wait 300ms
7. Return servo to home position
8. Task complete

**Use Case**: Trash launch, object manipulation, target activation

## Serial Commands

Open Serial Monitor at **115200 baud** for real-time control and debugging.

### Mode Control
```
Mode 1       - Start Mode 1 (4.52m forward + 4.52m reverse)
Mode 2       - Start Mode 2 (2.44m + servo action)
Stop         - Emergency stop (cancels current mode)
```

### Motor Speed Adjustment
```
SetHigh <0-255>   - Set high-speed PWM value (default: 255)
SetLow <0-255>    - Set low-speed PWM value (default: 90)
GetSpeeds         - Display current speed settings
```

### Servo Calibration
```
+            - Increment servo position by 1 degree
-            - Decrement servo position by 1 degree
SetServo     - Set current position as logical zero (0°)
```

### Example Session
```
> Mode 1
Mode 1 selected via Serial and started.
Distance (m): 0.164  |  Wheel RPM: 145.23  |  Speed (m/s): 0.396
Distance (m): 0.328  |  Wheel RPM: 147.81  |  Speed (m/s): 0.402
...
Mode1: reached forward distance. Reversing...
...
Mode 1 complete. Final stats:
Total distance (m): 9.0412

> SetHigh 200
motorSpeedHigh set to 200

> +
Servo moved to 44 deg (user offset incremented).
```

## Configuration & Calibration

### Physical Constants (lines 26-34)
```cpp
const float wheelRadius_m = 0.02605;    // Wheel radius in meters (26.05mm)
const int pulsesPerRev = 2;             // Black bands per wheel rotation
const float MODE1_PHASE_DISTANCE = 4.52; // Mode 1 distance in meters
const float MODE2_DISTANCE = 2.44;       // Mode 2 distance in meters
```

**Calibration**:
- Measure actual wheel radius with calipers
- Count tachometer pulses per wheel revolution
- Adjust MODE1/MODE2 distances based on competition requirements

### Motor Speed Settings (lines 42-43)
```cpp
int motorSpeedHigh = 255;  // Maximum PWM (full speed)
int motorSpeedLow  = 90;   // Reduced PWM (not actively used)
```

**Tuning**:
- Reduce `motorSpeedHigh` if robot is too fast or overshoots
- Increase for faster completion times
- Test on actual surface for optimal grip/speed balance

### Servo Configuration (lines 54-56)
```cpp
int servoLogicalZero = 0;    // Home position reference
int servoUserOffset = 43;     // Initial offset in degrees
const int servoRotate = 120;  // Rotation amount for Mode 2
```

**Calibration Process**:
1. Upload code and power on
2. Use `+` / `-` commands to move servo to desired home position
3. Send `SetServo` to save position as new zero
4. Test Mode 2 to verify 120° rotation reaches target

### Tachometer Debouncing (line 39)
```cpp
const unsigned long debounceMicros = 25000UL; // 25ms debounce
```

**Adjustment**:
- Increase if seeing erratic pulse counts (noise/bouncing)
- Decrease for higher maximum measurable speed

## Technical Details

### Distance Measurement System

**Method**: Interrupt-driven optical tachometry
- IR sensor detects black bands on drive wheel
- Each pulse triggers hardware interrupt (Pin 3)
- ISR (Interrupt Service Routine) increments pulse counter with debouncing
- Distance calculated from: pulses → revolutions → meters

**Formula**:
```
Revolutions = Pulses / PulsesPerRevolution
Distance (m) = Revolutions × WheelCircumference
WheelCircumference = 2 × π × WheelRadius
```

**Accuracy**: ±0.1mm per revolution (limited by wheel radius precision)

### Speed Calculation

**RPM (Revolutions Per Minute)**:
```
1. Count pulses over time window (default 20ms)
2. PulsesPerMinute = (DeltaPulses / TimeInMinutes)
3. WheelRPM = PulsesPerMinute / PulsesPerRev
```

**Linear Speed (m/s)**:
```
1. Measure distance traveled in time window
2. Speed = DistanceMeters / TimeSeconds
```

**Update Rate**: 20ms (50 Hz) - provides smooth real-time feedback without overwhelming serial output

### Motor Control

**H-Bridge Configuration**:
- IN1 HIGH, IN2 LOW = Forward
- IN1 LOW, IN2 HIGH = Backward  
- Both LOW = Coast stop (enable pin controls speed via PWM)

**PWM Control**: Pin 11 uses hardware PWM (490 Hz on Uno) for speed regulation

### Servo Control System

**Safe Initialization**: 
- Servo attaches without commanding initial position
- Prevents sudden movements that could damage mechanism
- Current physical position treated as logical 0° at startup

**Calibration Features**:
- Incremental adjustment (±1° via serial)
- Zero-point setting (`SetServo` command)
- Clamped writes (0-180° enforced) prevent servo damage

### Interrupt System

**Critical Requirement**: Pin 3 must support hardware interrupts
- Arduino Uno/Nano: Pins 2 and 3 support `attachInterrupt()`
- Code validates interrupt capability at startup
- ISR kept minimal (debounce check + counter increment) for reliability

### Code Structure

**Authors**: Noah (tachometer, interrupts, servo) & Rayan (motor control, modes, serial)

**Key Functions**:
- `tachISR()`: Interrupt service routine for tachometer
- `getDistanceMeters()`: Safe read of pulse counter with interrupt protection
- `getSpeed_m_s()` & `getRPM()`: Windowed speed calculations
- `setMotorForward/Backward()`: H-bridge control abstraction
- `handleSerial()`: Command parsing and execution
- `safeServoInit()`: No-movement servo attachment

## Troubleshooting

### Robot Doesn't Start
- **Check**: Pin 7 connection (mode select)
- **Check**: Serial Monitor at 115200 baud - should show startup messages
- **Check**: Power supply adequate for motors
- **Fix**: Verify 3-second delay message appears, indicating code is running

### Distance Measurement Errors
- **Symptom**: Distance not increasing or increasing too fast
- **Check**: IR sensor alignment with wheel bands
- **Check**: Pin 3 connection to tachometer
- **Fix**: Verify "attachInterrupt() on pin 3" message in Serial Monitor
- **Fix**: Adjust debounce value if seeing erratic counts

### Motor Not Running
- **Check**: Motor driver power supply (separate from Arduino)
- **Check**: Pins 11, 12, 13 connections to motor driver
- **Check**: Motor driver enable pin has PWM signal
- **Fix**: Test with `SetHigh 255` command to ensure speed is set

### Servo Doesn't Move (Mode 2)
- **Check**: Servo power (needs 5V, adequate current)
- **Check**: Pin 9 connection to servo signal wire
- **Check**: Servo is not mechanically jammed
- **Fix**: Use `+`/`-` commands to test servo manually before running Mode 2

### Overshooting Target Distance
- **Cause**: Robot momentum, surface friction, or wheel radius miscalibration
- **Fix**: Reduce `motorSpeedHigh` for more controlled stopping
- **Fix**: Measure actual wheel radius and update `wheelRadius_m` constant
- **Fix**: Add stopping buffer in code (already has 0.25m in Mode 1)

### Wrong Mode Starts
- **Check**: Pin 7 state at power-on (use multimeter or LED)
- **Symptom**: "pin 5 HIGH" but expected LOW (or vice versa)
- **Fix**: Verify mode select switch/jumper wiring
- **Override**: Use Serial commands (`Mode 1` or `Mode 2`) to manually select

### Code Won't Upload
- **Check**: Correct board selected (Tools → Board → Arduino Uno)
- **Check**: Correct port selected (Tools → Port)
- **Error**: "avrdude: stk500_recv(): programmer is not responding"
  - Disconnect Pin 0/1 if using them (TX/RX conflict)
  - Try different USB cable or port
  - Press reset button on Arduino right before upload

### Interrupt Error at Startup
- **Error**: "chosen IR pin is not interrupt-capable"
- **Cause**: Tachometer not connected to Pin 3 (or Pin 2)
- **Fix**: Rewire tachometer to Pin 3 (required for Uno/Nano)

### Serial Monitor Shows Garbage Characters
- **Fix**: Set baud rate to 115200 in Serial Monitor dropdown
- **Fix**: Wait for Arduino to fully reset after upload

### Real-Time Stats Not Printing
- **Cause**: Mode not running (stopped or never started)
- **Check**: Send `Mode 1` or `Mode 2` command
- **Check**: Stats only print when mode is actively running

---

**Project Repository**: [relfayoumi/TBWC-Arduino-Systems-Code](https://github.com/relfayoumi/TBWC-Arduino-Systems-Code)

For issues, questions, or contributions, please open an issue on GitHub.
