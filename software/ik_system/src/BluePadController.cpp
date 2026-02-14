/**
 * BluePadController.cpp
 * Implementation of Switch Pro controller input handling
 */

#include "../include/BluePadController.h"

// Static instance for callbacks
BluePadController* BluePadController::instance = nullptr;

// Controller input value ranges
static constexpr float STICK_MAX_VALUE = 512.0f;    // Analog stick maximum raw value
static constexpr float TRIGGER_MAX_VALUE = 1023.0f; // Trigger maximum raw value

void BluePadController::init() {
    // Store singleton instance for callbacks
    instance = this;
    
    // Initialize BluePad32 library
    BP32.setup(&onConnectedController, &onDisconnectedController);
    
    // Enable virtual devices (for multiple controllers)
    BP32.enableVirtualDevice(false);
}

void BluePadController::update() {
    // Update BluePad32 state
    BP32.update();
    
    if (!connected || gamepad == nullptr) {
        return;
    }
    
    // Read analog sticks (range: -512 to 512)
    int lx = gamepad->axisX();
    int ly = gamepad->axisY();
    int rx = gamepad->axisRX();
    int ry = gamepad->axisRY();
    
    // Normalize to -1.0 to 1.0
    input.leftX = applyDeadzone(lx / STICK_MAX_VALUE, stickDeadzone);
    input.leftY = applyDeadzone(ly / STICK_MAX_VALUE, stickDeadzone);
    input.rightX = applyDeadzone(rx / STICK_MAX_VALUE, stickDeadzone);
    input.rightY = applyDeadzone(ry / STICK_MAX_VALUE, stickDeadzone);
    
    // Read triggers (range: 0 to 1023)
    input.leftTrigger = applyDeadzone(gamepad->brake() / TRIGGER_MAX_VALUE, triggerDeadzone);
    input.rightTrigger = applyDeadzone(gamepad->throttle() / TRIGGER_MAX_VALUE, triggerDeadzone);
    
    // Read D-pad
    input.dpadUp = gamepad->dpad() & 0x01;
    input.dpadDown = gamepad->dpad() & 0x02;
    input.dpadRight = gamepad->dpad() & 0x04;
    input.dpadLeft = gamepad->dpad() & 0x08;
    
    // Read face buttons
    input.buttonA = gamepad->a();
    input.buttonB = gamepad->b();
    input.buttonX = gamepad->x();
    input.buttonY = gamepad->y();
    
    // Read shoulder buttons
    input.buttonL1 = gamepad->l1();
    input.buttonR1 = gamepad->r1();
    
    // Read special buttons
    input.buttonPlus = gamepad->buttons() & 0x10;   // Start
    input.buttonMinus = gamepad->buttons() & 0x08;  // Select
    input.buttonHome = gamepad->buttons() & 0x1000; // Home
}

void BluePadController::getWalkingVelocity(float& vx, float& vy) const {
    // Left stick controls walking
    // Y-axis: forward/backward
    // X-axis: left/right strafe
    vx = input.leftY;   // Forward is positive Y
    vy = -input.leftX;  // Right is negative X (inverted for body frame)
}

float BluePadController::getRotationSpeed() const {
    // Right stick X-axis controls rotation
    return input.rightX;
}

float BluePadController::getHeightAdjustment() const {
    // Triggers control height: L2 lowers, R2 raises
    // Return in mm units
    float delta = (input.rightTrigger - input.leftTrigger) * 50.0f;
    return delta;
}

bool BluePadController::isButtonPressed(int buttonMask) const {
    if (!connected || gamepad == nullptr) {
        return false;
    }
    return (gamepad->buttons() & buttonMask) != 0;
}

float BluePadController::applyDeadzone(float value, float deadzone) const {
    if (fabs(value) < deadzone) {
        return 0.0f;
    }
    
    // Scale remaining range to full output
    float sign = (value > 0) ? 1.0f : -1.0f;
    float scaled = (fabs(value) - deadzone) / (1.0f - deadzone);
    return sign * scaled;
}

// Static callback for controller connection
void BluePadController::onConnectedController(ControllerPtr ctl) {
    if (instance == nullptr) return;
    
    // Check if controller is already set
    if (instance->gamepad == nullptr) {
        Serial.println("Controller connected!");
        instance->gamepad = ctl;
        instance->connected = true;
        
        // Optional: Set controller color/rumble
        ctl->setColorLED(0, 255, 0); // Green LED
    }
}

// Static callback for controller disconnection
void BluePadController::onDisconnectedController(ControllerPtr ctl) {
    if (instance == nullptr) return;
    
    if (instance->gamepad == ctl) {
        Serial.println("Controller disconnected!");
        instance->gamepad = nullptr;
        instance->connected = false;
        
        // Reset input state
        instance->input = GamepadInput();
    }
}
