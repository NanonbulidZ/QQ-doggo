/**
 * LegController.h
 * Quadruped locomotion controller with inverse kinematics
 * 
 * Manages 4 legs with trotting gait, body pose control,
 * and full 3D walking capabilities (forward, strafe, turn)
 */

#pragma once

#include "Vector3.h"
#include "IKSolver.h"
#include <math.h>

// Leg position identifiers
enum LegPosition {
    LEG_FRONT_LEFT = 0,
    LEG_FRONT_RIGHT = 1,
    LEG_BACK_LEFT = 2,
    LEG_BACK_RIGHT = 3
};

// Gait phase for each leg
enum GaitPhase {
    PHASE_STANCE,  // Leg on ground
    PHASE_SWING    // Leg in air
};

class LegController {
public:
    // Body dimensions (mm)
    float bodyLength;   // Front-to-back distance between leg attach points
    float bodyWidth;    // Left-to-right distance between leg attach points
    
    // Gait parameters
    float strideLength;    // Maximum stride length (mm)
    float stepHeight;      // Height of foot lift during swing (mm)
    float gaitSpeed;       // Gait speed multiplier (0-1)
    float gaitFrequency;   // Steps per second
    
    // Current body pose
    Vector3 bodyPosition;  // Body center position (x, y, z)
    Vector3 bodyRotation;  // Body rotation (roll, pitch, yaw) in radians

    // Constructor
    LegController(float length = 100.0f, float width = 80.0f);

    /**
     * Initialize leg controllers with segment lengths
     */
    void init(float coxaLen = 24.0f, float femurLen = 51.9f, float tibiaLen = 50.0f);

    /**
     * Set gait parameters
     */
    void setGaitParams(float stride = 60.0f, float height = 30.0f, float speed = 0.5f);

    /**
     * Set body position (height adjustment)
     */
    void setBodyPosition(float x, float y, float z);

    /**
     * Set body rotation (roll, pitch, yaw)
     */
    void setBodyRotation(float roll, float pitch, float yaw);

    /**
     * Update walking motion
     * @param vx Forward/backward velocity (-1 to 1)
     * @param vy Left/right strafe velocity (-1 to 1)
     * @param vYaw Rotation velocity (-1 to 1)
     * @param deltaTime Time since last update (seconds)
     */
    void updateWalking(float vx, float vy, float vYaw, float deltaTime);

    /**
     * Stand in default pose (no walking)
     */
    void stand();

    /**
     * Get joint angles for a specific leg
     * @param leg Leg position identifier
     * @return Joint angles (alpha, beta, gamma)
     */
    LegJoints getLegAngles(LegPosition leg) const;

    /**
     * Get current foot position for a leg (for debugging)
     */
    Vector3 getFootPosition(LegPosition leg) const;

private:
    // IK solvers for each leg
    LegIK legIK[4];

    // Current foot positions (relative to body center)
    Vector3 footPositions[4];

    // Default foot positions (neutral stance)
    Vector3 defaultFootPositions[4];

    // Gait state
    float gaitPhase;         // Current phase in gait cycle (0-1)
    GaitPhase legPhases[4];  // Current phase for each leg
    Vector3 swingStart[4];   // Foot position at start of swing
    Vector3 swingTarget[4];  // Target foot position for swing

    /**
     * Initialize default foot positions based on body dimensions
     */
    void initDefaultStance();

    /**
     * Update gait phase and determine which legs are in swing/stance
     */
    void updateGaitPhase(float deltaTime, float vx, float vy, float vYaw);

    /**
     * Calculate foot position during stance phase
     */
    Vector3 calcStanceFootPos(LegPosition leg, float vx, float vy, float vYaw, float deltaTime);

    /**
     * Calculate foot position during swing phase
     */
    Vector3 calcSwingFootPos(LegPosition leg, float swingProgress);

    /**
     * Transform foot position from body frame to leg frame
     * Accounts for body rotation and leg attachment point
     */
    Vector3 bodyToLegFrame(const Vector3& footInBody, LegPosition leg) const;

    /**
     * Get leg attachment point in body frame
     */
    Vector3 getLegAttachPoint(LegPosition leg) const;

    /**
     * Apply rotation matrix to vector
     */
    Vector3 rotateVector(const Vector3& v, const Vector3& rotation) const;

    /**
     * Solve IK for a leg and store result
     */
    bool solveAndStoreLeg(LegPosition leg, const Vector3& targetFootPos);
    
    // Stored joint angles from last IK solve
    LegJoints currentAngles[4];
};
