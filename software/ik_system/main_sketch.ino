/**
 * main_sketch.ino
 * Modern IK System Example for QQ-Doggo Quadruped
 * 
 * Demonstrates 3D walking, turning, and standing control
 * using a Switch Pro controller via BluePad32
 * 
 * Hardware Requirements:
 * - ESP32 board
 * - 12x servo motors (3 per leg)
 * - Switch Pro controller
 * - Power supply (servos require external power)
 * 
 * Library Dependencies:
 * - Bluepad32 (https://github.com/ricardoquesada/bluepad32)
 * - ESP32Servo (for Arduino servo library compatibility)
 */

#include <ESP32Servo.h>
#include "include/LegController.h"
#include "include/BluePadController.h"

// ===== Configuration =====

// Servo pin assignments (adjust for your wiring)
// Format: [leg][joint] where joint: 0=alpha, 1=beta, 2=gamma
const int SERVO_PINS[4][3] = {
    {2, 4, 5},    // Front Left: alpha, beta, gamma
    {15, 16, 17}, // Front Right
    {18, 19, 21}, // Back Left
    {22, 23, 25}  // Back Right
};

// Robot dimensions (mm)
const float BODY_LENGTH = 100.0f;  // Front-to-back
const float BODY_WIDTH = 80.0f;    // Left-to-right
const float COXA_LENGTH = 24.0f;   // Shoulder segment
const float FEMUR_LENGTH = 51.9f;  // Upper leg segment
const float TIBIA_LENGTH = 50.0f;  // Lower leg segment

// Gait parameters
const float STRIDE_LENGTH = 60.0f;  // mm
const float STEP_HEIGHT = 30.0f;    // mm
const float GAIT_SPEED = 0.5f;      // 0-1 multiplier

// Body height limits (mm, negative = below body)
const float HEIGHT_MIN = -150.0f;   // Lowest body position (legs most extended)
const float HEIGHT_MAX = -80.0f;    // Highest body position (legs most compressed)
const float HEIGHT_DEFAULT = -120.0f; // Default standing height

// Servo calibration (adjust for your servos)
const int SERVO_MIN_PULSE = 500;   // microseconds
const int SERVO_MAX_PULSE = 2500;  // microseconds
const int SERVO_FREQUENCY = 50;    // Hz

// Control loop timing
const int LOOP_DELAY_MS = 16;      // ~60Hz update rate

// ===== Global Objects =====

LegController robot(BODY_LENGTH, BODY_WIDTH);
BluePadController controller;
Servo servos[4][3];  // [leg][joint]

// ===== Servo Conversion Functions =====

/**
 * Convert joint angle (radians) to servo pulse width (microseconds)
 * Maps 0-π radians to servo min-max pulse range
 */
int angleToPulse(float angleRad) {
    // Constrain angle to valid range
    if (angleRad < 0) angleRad = 0;
    if (angleRad > M_PI) angleRad = M_PI;
    
    // Linear mapping from 0-π to min-max pulse
    float normalized = angleRad / M_PI;
    int pulse = SERVO_MIN_PULSE + (int)(normalized * (SERVO_MAX_PULSE - SERVO_MIN_PULSE));
    
    return pulse;
}

/**
 * Write joint angles to servos for a specific leg
 */
void writeServos(int legIndex, const LegJoints& joints) {
    // Alpha (shoulder/coxa)
    int pulseAlpha = angleToPulse(joints.alpha);
    servos[legIndex][0].writeMicroseconds(pulseAlpha);
    
    // Beta (elbow/femur)
    int pulseBeta = angleToPulse(joints.beta);
    servos[legIndex][1].writeMicroseconds(pulseBeta);
    
    // Gamma (ankle/tibia)
    int pulseGamma = angleToPulse(joints.gamma);
    servos[legIndex][2].writeMicroseconds(pulseGamma);
}

/**
 * Write all servo positions based on current robot state
 */
void updateAllServos() {
    for (int leg = 0; leg < 4; leg++) {
        LegJoints angles = robot.getLegAngles((LegPosition)leg);
        writeServos(leg, angles);
    }
}

// ===== Setup =====

void setup() {
    Serial.begin(115200);
    Serial.println("=== QQ-Doggo Modern IK System ===");
    Serial.println("Initializing...");
    
    // Initialize servo library
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    
    // Attach servos to pins
    for (int leg = 0; leg < 4; leg++) {
        for (int joint = 0; joint < 3; joint++) {
            servos[leg][joint].setPeriodHertz(SERVO_FREQUENCY);
            servos[leg][joint].attach(SERVO_PINS[leg][joint], 
                                     SERVO_MIN_PULSE, 
                                     SERVO_MAX_PULSE);
        }
    }
    Serial.println("Servos attached");
    
    // Initialize leg controller
    robot.init(COXA_LENGTH, FEMUR_LENGTH, TIBIA_LENGTH);
    robot.setGaitParams(STRIDE_LENGTH, STEP_HEIGHT, GAIT_SPEED);
    robot.setBodyPosition(0, 0, HEIGHT_DEFAULT);
    Serial.println("Robot controller initialized");
    
    // Initialize BluePad32 controller
    controller.init();
    controller.setDeadzones(0.1f, 0.05f);
    Serial.println("Waiting for controller connection...");
    
    // Start in standing position
    robot.stand();
    updateAllServos();
    
    delay(1000);
    Serial.println("Ready!");
}

// ===== Main Loop =====

void loop() {
    static unsigned long lastUpdate = 0;
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastUpdate) / 1000.0f;  // Convert to seconds
    lastUpdate = currentTime;
    
    // Update controller state
    controller.update();
    
    // Check if controller is connected
    if (!controller.isConnected()) {
        // No controller - stay in safe standing position
        robot.stand();
        updateAllServos();
        delay(LOOP_DELAY_MS);
        return;
    }
    
    // Get controller input
    float vx, vy;
    controller.getWalkingVelocity(vx, vy);
    float vYaw = controller.getRotationSpeed();
    float heightDelta = controller.getHeightAdjustment();
    
    // Update body height
    static float currentHeight = HEIGHT_DEFAULT;
    currentHeight += heightDelta * deltaTime;
    currentHeight = constrain(currentHeight, HEIGHT_MIN, HEIGHT_MAX);
    robot.setBodyPosition(0, 0, currentHeight);
    
    // Button controls
    const GamepadInput& input = controller.getInput();
    
    // A button: Reset to standing
    if (input.buttonA) {
        robot.stand();
        currentHeight = HEIGHT_DEFAULT;
        robot.setBodyPosition(0, 0, currentHeight);
    }
    
    // B button: Toggle gait speed boost
    static bool speedBoost = false;
    static bool bButtonPrev = false;
    if (input.buttonB && !bButtonPrev) {
        speedBoost = !speedBoost;
        float speed = speedBoost ? 1.0f : 0.5f;
        robot.setGaitParams(STRIDE_LENGTH, STEP_HEIGHT, speed);
        Serial.print("Speed boost: ");
        Serial.println(speedBoost ? "ON" : "OFF");
    }
    bButtonPrev = input.buttonB;
    
    // D-pad: Body rotation control
    float roll = 0, pitch = 0, yaw = 0;
    if (input.dpadLeft) roll = 0.1f;
    if (input.dpadRight) roll = -0.1f;
    if (input.dpadUp) pitch = 0.1f;
    if (input.dpadDown) pitch = -0.1f;
    robot.setBodyRotation(roll, pitch, yaw);
    
    // Update walking motion
    if (fabs(vx) > 0.01f || fabs(vy) > 0.01f || fabs(vYaw) > 0.01f) {
        // Walking mode
        robot.updateWalking(vx, vy, vYaw, deltaTime);
    } else {
        // Standing mode (no stick input)
        robot.stand();
    }
    
    // Write servo positions
    updateAllServos();
    
    // Debug output (every second)
    static unsigned long lastDebug = 0;
    if (currentTime - lastDebug > 1000) {
        lastDebug = currentTime;
        Serial.print("Vel: ");
        Serial.print(vx, 2);
        Serial.print(", ");
        Serial.print(vy, 2);
        Serial.print(", Yaw: ");
        Serial.print(vYaw, 2);
        Serial.print(", Height: ");
        Serial.println(currentHeight, 1);
    }
    
    // Maintain loop timing
    delay(LOOP_DELAY_MS);
}

// ===== Control Reference =====
/*
 * SWITCH PRO CONTROLLER MAPPING:
 * 
 * Left Stick:
 *   - Up/Down: Forward/Backward walking
 *   - Left/Right: Strafe left/right
 * 
 * Right Stick:
 *   - Left/Right: Turn left/right (yaw rotation)
 * 
 * L2 Trigger: Lower body height
 * R2 Trigger: Raise body height
 * 
 * A Button: Reset to standing pose
 * B Button: Toggle speed boost (0.5x / 1.0x)
 * 
 * D-Pad:
 *   - Up/Down: Body pitch forward/backward
 *   - Left/Right: Body roll left/right
 * 
 * WIRING:
 * Servo connections as defined in SERVO_PINS array.
 * All servos require external power supply (5-6V).
 * ESP32 GPIO pins connect to servo signal wires.
 */
