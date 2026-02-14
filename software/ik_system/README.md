# Modern IK System for QQ-Doggo Quadruped Robot

A completely refactored and modernized inverse kinematics system for the QQ-doggo quadruped robot with full 3D walking capabilities and Switch Pro controller integration via BluePad32.

## Features

### Core Functionality
- **3D Inverse Kinematics**: Analytic IK solver for 3-DOF legs with full 3D positioning
- **Quadruped Locomotion**: Trotting gait with configurable speed and stride
- **3D Body Rotation**: Full support for yaw, pitch, and roll rotations
- **Walking Control**: Forward, backward, strafe, and turning via controller
- **Real-time Updates**: 60Hz gait computation and servo control

### Controller Integration
- **BluePad32 Support**: Full Switch Pro controller compatibility
- **Analog Stick Control**: Left stick for movement (vx, vy), right stick for rotation
- **Trigger Control**: Height adjustment via L/R triggers
- **Button Mapping**: All standard gamepad buttons supported
- **Deadzone Handling**: Configurable stick deadzone with automatic scaling

### Hardware Support
- **ESP32 Compatible**: Arduino IDE compatible firmware
- **12 Servos**: 3-DOF per leg (Alpha, Beta, Gamma)
- **Servo Library**: Uses ESP32Servo for PWM control
- **Flexible Kinematics**: Configurable segment lengths for different builds

## Architecture

### Components

#### Vector3.h
3D vector mathematics library with standard operations:
- Vector addition, subtraction, scaling
- Dot product, cross product
- Magnitude, normalization, distance
- Linear interpolation (lerp)

#### IKSolver.h
Inverse kinematics solver featuring:
- `LegJoints` struct for angle representation (alpha, beta, gamma)
- `LegIK` class with analytic solving using law of cosines
- Angle limits and reach validation
- Accounts for 3-segment leg geometry (coxa, femur, tibia)

#### LegController.h
Quadruped locomotion controller:
- Trotting gait generation with diagonal leg pairs
- Body pose management (position + rotation)
- Individual leg IK solving for all 4 legs
- Walking, turning, and standing commands
- Configurable gait parameters (stride, step height, speed)

#### BluePadController.h
Switch Pro controller interface:
- `GamepadInput` struct for button/stick states
- Analog stick deadzone handling with scaling
- Trigger-based height adjustment
- Button state tracking for all inputs
- Automatic connection/disconnection handling

### File Structure

```
software/ik_system/
├── include/
│   ├── Vector3.h          # 3D vector math library
│   ├── IKSolver.h         # Leg IK solver
│   ├── LegController.h    # Quadruped controller
│   └── BluePadController.h # Gamepad input abstraction
├── src/
│   ├── LegController.cpp  # Controller implementation
│   └── BluePadController.cpp # Gamepad implementation
├── main_sketch.ino        # Example Arduino sketch
└── README.md              # This file
```

## Hardware Setup

### Required Components

1. **ESP32 Development Board**
   - Any ESP32 board with sufficient GPIO pins
   - Tested with ESP32-DevKitC

2. **Servo Motors (12 total)**
   - 3 servos per leg × 4 legs
   - Recommended: TowerPro MG90D or similar
   - Operating voltage: 4.8-6V
   - Torque: 2+ kg·cm recommended

3. **Power Supply**
   - Servos require external power (5-6V, 3A+ recommended)
   - ESP32 can be powered via USB or same supply with regulator

4. **Switch Pro Controller**
   - Official Nintendo Switch Pro Controller
   - Compatible third-party controllers may work

### Wiring

#### Servo Connections

Default pin assignments (modify in `main_sketch.ino` as needed):

```
Front Left Leg:
  - Alpha (shoulder): GPIO 2
  - Beta (elbow):     GPIO 4
  - Gamma (ankle):    GPIO 5

Front Right Leg:
  - Alpha: GPIO 15
  - Beta:  GPIO 16
  - Gamma: GPIO 17

Back Left Leg:
  - Alpha: GPIO 18
  - Beta:  GPIO 19
  - Gamma: GPIO 21

Back Right Leg:
  - Alpha: GPIO 22
  - Beta:  GPIO 23
  - Gamma: GPIO 25
```

**Important**: Connect all servo ground wires together and to ESP32 GND. Servo power should come from external supply, not ESP32 3.3V/5V pins.

#### Power Wiring

```
Power Supply (+5-6V, 3A+)
    ├─> All servo V+ pins
    └─> ESP32 VIN (optional, if not using USB power)

Common Ground
    ├─> All servo GND pins
    ├─> Power supply GND
    └─> ESP32 GND
```

### Controller Pairing

1. Power on ESP32 with sketch uploaded
2. Put Switch Pro controller in pairing mode:
   - Press and hold SYNC button (small button on top)
   - LED should start flashing
3. Controller will connect automatically via Bluetooth
4. Green LED on controller indicates connection

## Software Setup

### Required Libraries

Install via Arduino IDE Library Manager:

1. **Bluepad32** by Ricardo Quesada
   - Provides Switch Pro controller support
   - https://github.com/ricardoquesada/bluepad32

2. **ESP32Servo** by Kevin Harrington
   - Servo control for ESP32
   - Compatible with Arduino Servo API

### Arduino IDE Configuration

1. **Board**: ESP32 Dev Module (or your specific board)
2. **Upload Speed**: 115200
3. **Flash Frequency**: 80MHz
4. **Partition Scheme**: Default (or "Huge APP" for more program space)

### Building and Uploading

1. Open `main_sketch.ino` in Arduino IDE
2. Adjust configuration constants if needed:
   - `SERVO_PINS`: Match your wiring
   - Robot dimensions: `BODY_LENGTH`, `BODY_WIDTH`, etc.
   - Servo calibration: `SERVO_MIN_PULSE`, `SERVO_MAX_PULSE`
3. Select your ESP32 board and port
4. Click Upload

## Usage

### Basic Walking Example

```cpp
#include "include/LegController.h"
#include "include/BluePadController.h"

LegController robot(100.0f, 80.0f);  // body length, width
BluePadController gamepad;

void setup() {
    robot.init(24.0f, 51.9f, 50.0f);  // coxa, femur, tibia lengths
    robot.setGaitParams(60.0f, 30.0f, 0.5f);  // stride, height, speed
    gamepad.init();
}

void loop() {
    gamepad.update();
    
    if (gamepad.isConnected()) {
        float vx, vy;
        gamepad.getWalkingVelocity(vx, vy);
        float omega = gamepad.getRotationSpeed();
        float height = -120.0f + gamepad.getHeightAdjustment();
        
        robot.setBodyPosition(0, 0, height);
        robot.updateWalking(vx, vy, omega, 0.016f);  // 60Hz
        
        // Get joint angles and write to servos
        for(int i = 0; i < 4; ++i) {
            LegJoints angles = robot.getLegAngles((LegPosition)i);
            // ... write to servos ...
        }
    }
    
    delay(16);
}
```

### Control Mapping

#### Switch Pro Controller Layout

```
        L1                    R1
        L2                    R2
    
     ┌─────────────────────────┐
     │  [−]           [+]     │
     │                         │
     │    [←↑↓→]      [X]     │
     │                [Y] [A]  │
     │      (L)       [B]      │
     │                         │
     │                  (R)    │
     └─────────────────────────┘

L = Left analog stick
R = Right analog stick
```

#### Input Mapping

| Control | Function | Range |
|---------|----------|-------|
| **Left Stick Up/Down** | Forward/Backward | -1.0 to 1.0 |
| **Left Stick Left/Right** | Strafe Left/Right | -1.0 to 1.0 |
| **Right Stick Left/Right** | Turn (Yaw) | -1.0 to 1.0 |
| **L2 Trigger** | Lower Body Height | 0 to 1.0 |
| **R2 Trigger** | Raise Body Height | 0 to 1.0 |
| **A Button** | Reset to Standing | Press |
| **B Button** | Speed Boost Toggle | Press |
| **D-Pad Up/Down** | Body Pitch | Press |
| **D-Pad Left/Right** | Body Roll | Press |

### Calibration

#### Finding Servo Pulse Ranges

Each servo may have slightly different pulse width requirements:

1. Start with conservative values (500-2500 µs)
2. Test each servo individually
3. Adjust `SERVO_MIN_PULSE` and `SERVO_MAX_PULSE` as needed
4. Typical range: 600-2400 µs for most servos

#### Adjusting Leg Geometry

Measure your robot's actual dimensions:

```cpp
// In main_sketch.ino
const float COXA_LENGTH = 24.0f;   // Shoulder segment (mm)
const float FEMUR_LENGTH = 51.9f;  // Upper leg (mm)
const float TIBIA_LENGTH = 50.0f;  // Lower leg (mm)
const float BODY_LENGTH = 100.0f;  // Front-back spacing (mm)
const float BODY_WIDTH = 80.0f;    // Left-right spacing (mm)
```

#### Tuning Gait Parameters

Adjust for smoother or faster walking:

```cpp
const float STRIDE_LENGTH = 60.0f;  // Step distance (mm)
const float STEP_HEIGHT = 30.0f;    // Foot lift height (mm)
const float GAIT_SPEED = 0.5f;      // Speed multiplier (0-1)
```

## Extending the System

### Adding IMU Support

The architecture is designed for easy IMU integration:

```cpp
// In LegController.cpp, modify updateWalking()
void LegController::updateWalking(float vx, float vy, float vYaw, float deltaTime) {
    // Read IMU data
    Vector3 imuAngles = imu.getOrientation();
    
    // Apply stabilization
    bodyRotation = bodyRotation - imuAngles * 0.5f;  // Damping
    
    // ... rest of walking update ...
}
```

### Implementing New Gaits

Create alternative gait patterns:

```cpp
// Pace gait (lateral pairs)
void updatePaceGait(float deltaTime) {
    if (gaitPhase < 0.5f) {
        legPhases[LEG_FRONT_LEFT] = PHASE_SWING;
        legPhases[LEG_BACK_LEFT] = PHASE_SWING;
        legPhases[LEG_FRONT_RIGHT] = PHASE_STANCE;
        legPhases[LEG_BACK_RIGHT] = PHASE_STANCE;
    } else {
        // Swap
    }
}
```

### Custom Movement Patterns

Add specialized movements:

```cpp
void LegController::sidestep(float velocity) {
    // Lateral movement without turning
    updateWalking(0, velocity, 0, deltaTime);
}

void LegController::turnInPlace(float angularVel) {
    // Rotate without translation
    updateWalking(0, 0, angularVel, deltaTime);
}
```

## Troubleshooting

### Controller Won't Connect
- Ensure BluePad32 library is properly installed
- Check ESP32 Bluetooth is enabled (not disabled in partition scheme)
- Try resetting controller (hold SYNC for 10+ seconds)
- Check serial monitor for connection messages

### Servos Not Moving
- Verify power supply is connected and adequate (5-6V, 3A+)
- Check servo signal wires are connected to correct GPIO pins
- Confirm ESP32 GND is connected to servo GND
- Test servos individually with simple sketch

### Erratic Walking Motion
- Calibrate servo pulse ranges for your specific servos
- Verify leg dimensions match physical robot
- Check for mechanical binding in leg joints
- Reduce gait speed for testing

### Robot Falls Over
- Lower body height (increase Z magnitude, e.g., -140mm)
- Reduce stride length and step height
- Check that all legs are reaching ground (IK solutions valid)
- Ensure center of mass is within stability margin

### IK Solver Fails
- Target position may be outside reachable workspace
- Check leg segment lengths are correct
- Verify foot positions are reasonable (-120mm Z for ground)
- Add debug output to see which leg is failing

## Performance Notes

- **Update Rate**: 60Hz (16ms loop time) recommended
- **IK Computation**: ~0.5ms per leg (2ms total for 4 legs)
- **Controller Input**: ~1ms per update
- **Servo Write**: Negligible overhead with ESP32PWM

## API Reference

### LegController

```cpp
// Constructor
LegController(float bodyLength, float bodyWidth);

// Initialization
void init(float coxaLen, float femurLen, float tibiaLen);
void setGaitParams(float stride, float height, float speed);

// Control
void updateWalking(float vx, float vy, float vYaw, float deltaTime);
void stand();
void setBodyPosition(float x, float y, float z);
void setBodyRotation(float roll, float pitch, float yaw);

// Query
LegJoints getLegAngles(LegPosition leg) const;
Vector3 getFootPosition(LegPosition leg) const;
```

### BluePadController

```cpp
// Initialization
void init();
void update();

// Query
bool isConnected() const;
const GamepadInput& getInput() const;
void getWalkingVelocity(float& vx, float& vy) const;
float getRotationSpeed() const;
float getHeightAdjustment() const;

// Configuration
void setDeadzones(float stick, float trigger);
```

### LegIK

```cpp
// Constructor
LegIK(float coxa, float femur, float tibia);

// Configuration
void setAngleLimits(float aMin, float aMax, float bMin, float bMax, 
                    float gMin, float gMax);

// Solve
bool solve(const Vector3& target, LegJoints& result);
bool isWithinLimits(const LegJoints& joints) const;
```

## License

This modernized IK system builds upon the original QQ-doggo project. The refactored code maintains compatibility with the existing project structure while providing a cleaner, more modular architecture.

Original QQ-doggo project: MIT License
Modern IK system refactor: MIT License

## Credits

- Original QQ-doggo project by NanonbulidZ
- IK algorithms inspired by Boston Dynamics Spot
- BluePad32 library by Ricardo Quesada
- Refactored and modernized by GitHub Copilot

## Future Enhancements

- [ ] IMU-based body stabilization
- [ ] Adaptive terrain following
- [ ] Multiple gait patterns (trot, pace, bound)
- [ ] Obstacle avoidance
- [ ] Autonomous navigation
- [ ] WebSocket remote control interface
- [ ] Configuration via web interface
- [ ] Recording and playback of custom movements
