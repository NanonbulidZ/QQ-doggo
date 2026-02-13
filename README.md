# Modern IK System for QQ-Doggo

A completely refactored and modernized inverse kinematics system for the QQ-doggo quadruped robot with Switch Pro controller integration via BluePad.

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
- **TowerPro mg90d Servos**: Optimized for standard servo library
- **Flexible Kinematics**: Configurable segment lengths for different builds

## Architecture

### Header Files

#### Vector3.h
3D vector mathematics library with standard operations:
- Vector addition, subtraction, scaling
- Dot product, cross product
- Magnitude, normalization, distance

#### IKSolver.h
Inverse kinematics solver featuring:
- `LegJoints` struct for angle representation
- `LegIK` class with analytic solving
- Angle limits and reach validation
- Law of cosines-based computation

#### LegController.h
Quadruped locomotion controller:
- Trotting gait generation
- Body pose management (position + rotation)
- Individual leg IK solving
- Walking, turning, standing commands

#### BluePadController.h
Switch Pro controller interface:
- `GamepadInput` struct for button/stick states
- Analog stick deadzone handling
- Trigger-based height adjustment
- Button state tracking

## Usage

### Basic Walking Example
```cpp
LegController robot;
BluePadController gamepad;

robot.init();
gamepad.init();

while(true) {
    gamepad.update();
    
    float vx, vy;
    gamepad.getWalkingVelocity(vx, vy);
    float omega = gamepad.getRotationSpeed();
    float height = -120.0f + gamepad.getHeightAdjustment();
    
    robot.walk(vx, vy, omega, height);
    
    // Get servo angles
    for(int i = 0; i < 4; ++i) {
        LegJoints angles = robot.getLegAngles((LegPosition)i);
        // Write to servo PWM
    }
    
    delay(16); // ~60Hz
}
