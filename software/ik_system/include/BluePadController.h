/**
 * BluePadController.h
 * Switch Pro Controller input abstraction for BluePad32
 * 
 * Provides easy access to analog sticks, triggers, and buttons
 * with deadzone handling and normalized output values
 */

#pragma once

#include <Bluepad32.h>

// Gamepad input state structure
struct GamepadInput {
    // Left analog stick (movement)
    float leftX;      // -1.0 (left) to 1.0 (right)
    float leftY;      // -1.0 (down) to 1.0 (up)
    
    // Right analog stick (rotation/camera)
    float rightX;     // -1.0 (left) to 1.0 (right)
    float rightY;     // -1.0 (down) to 1.0 (up)
    
    // Triggers (0 to 1.0)
    float leftTrigger;   // L2
    float rightTrigger;  // R2
    
    // D-pad
    bool dpadUp;
    bool dpadDown;
    bool dpadLeft;
    bool dpadRight;
    
    // Face buttons
    bool buttonA;
    bool buttonB;
    bool buttonX;
    bool buttonY;
    
    // Shoulder buttons
    bool buttonL1;
    bool buttonR1;
    
    // Special buttons
    bool buttonPlus;   // Start/Plus
    bool buttonMinus;  // Select/Minus
    bool buttonHome;
    
    GamepadInput() : leftX(0), leftY(0), rightX(0), rightY(0),
                     leftTrigger(0), rightTrigger(0),
                     dpadUp(false), dpadDown(false), dpadLeft(false), dpadRight(false),
                     buttonA(false), buttonB(false), buttonX(false), buttonY(false),
                     buttonL1(false), buttonR1(false),
                     buttonPlus(false), buttonMinus(false), buttonHome(false) {}
};

class BluePadController {
public:
    // Deadzone for analog sticks (0-1)
    float stickDeadzone;
    
    // Trigger deadzone (0-1)
    float triggerDeadzone;
    
    // Constructor
    BluePadController() : stickDeadzone(0.1f), triggerDeadzone(0.05f), 
                          gamepad(nullptr), connected(false) {}

    /**
     * Initialize BluePad32 and register callbacks
     */
    void init();

    /**
     * Update controller state (call each frame)
     */
    void update();

    /**
     * Check if controller is connected
     */
    bool isConnected() const { return connected; }

    /**
     * Get current input state
     */
    const GamepadInput& getInput() const { return input; }

    /**
     * Get walking velocity from left stick
     * @param vx Forward/backward velocity output (-1 to 1)
     * @param vy Left/right strafe velocity output (-1 to 1)
     */
    void getWalkingVelocity(float& vx, float& vy) const;

    /**
     * Get rotation speed from right stick X-axis
     * @return Rotation velocity (-1 to 1)
     */
    float getRotationSpeed() const;

    /**
     * Get height adjustment from triggers
     * L2 = lower, R2 = raise
     * @return Height delta in mm (-50 to 50)
     */
    float getHeightAdjustment() const;

    /**
     * Check if a specific button is pressed
     */
    bool isButtonPressed(int buttonMask) const;

    /**
     * Set deadzone thresholds
     */
    void setDeadzones(float stick, float trigger) {
        stickDeadzone = stick;
        triggerDeadzone = trigger;
    }

private:
    GamepadInput input;
    ControllerPtr gamepad;
    bool connected;

    /**
     * Apply deadzone to analog value
     */
    float applyDeadzone(float value, float deadzone) const;

    /**
     * BluePad32 callbacks (static for C-style callback registration)
     */
    static void onConnectedController(ControllerPtr ctl);
    static void onDisconnectedController(ControllerPtr ctl);
    
    // Singleton instance for static callbacks
    static BluePadController* instance;
};
