/**
 * LegController.cpp
 * Implementation of quadruped locomotion controller
 */

#include "../include/LegController.h"

// Constructor
LegController::LegController(float length, float width) 
    : bodyLength(length), bodyWidth(width),
      strideLength(60.0f), stepHeight(30.0f), 
      gaitSpeed(0.5f), gaitFrequency(1.0f),
      bodyPosition(0, 0, -120.0f), bodyRotation(0, 0, 0),
      gaitPhase(0) {
    
    // Initialize leg phases (trot gait: diagonals move together)
    legPhases[LEG_FRONT_LEFT] = PHASE_STANCE;
    legPhases[LEG_FRONT_RIGHT] = PHASE_SWING;
    legPhases[LEG_BACK_LEFT] = PHASE_SWING;
    legPhases[LEG_BACK_RIGHT] = PHASE_STANCE;
    
    // Initialize previous phases
    for (int i = 0; i < 4; i++) {
        prevLegPhases[i] = legPhases[i];
    }
}

void LegController::init(float coxaLen, float femurLen, float tibiaLen) {
    // Initialize all leg IK solvers with same dimensions
    for (int i = 0; i < 4; i++) {
        legIK[i] = LegIK(coxaLen, femurLen, tibiaLen);
        legIK[i].setAngleLimits(0, M_PI, 0, M_PI, 0, M_PI);
    }
    
    // Set up default foot positions
    initDefaultStance();
    
    // Initialize current foot positions to default
    for (int i = 0; i < 4; i++) {
        footPositions[i] = defaultFootPositions[i];
    }
}

void LegController::initDefaultStance() {
    // Foot positions in body frame (neutral stance)
    // Body center is at origin, legs spread to corners
    float halfLength = bodyLength / 2.0f;
    float halfWidth = bodyWidth / 2.0f;
    
    // Front left
    defaultFootPositions[LEG_FRONT_LEFT].set(
        halfLength + 40.0f,   // x: forward
        halfWidth + 40.0f,    // y: left
        -120.0f               // z: down (ground level)
    );
    
    // Front right
    defaultFootPositions[LEG_FRONT_RIGHT].set(
        halfLength + 40.0f,   // x: forward
        -halfWidth - 40.0f,   // y: right
        -120.0f               // z: down
    );
    
    // Back left
    defaultFootPositions[LEG_BACK_LEFT].set(
        -halfLength - 40.0f,  // x: backward
        halfWidth + 40.0f,    // y: left
        -120.0f               // z: down
    );
    
    // Back right
    defaultFootPositions[LEG_BACK_RIGHT].set(
        -halfLength - 40.0f,  // x: backward
        -halfWidth - 40.0f,   // y: right
        -120.0f               // z: down
    );
}

void LegController::setGaitParams(float stride, float height, float speed) {
    strideLength = stride;
    stepHeight = height;
    gaitSpeed = speed;
}

void LegController::setBodyPosition(float x, float y, float z) {
    bodyPosition.set(x, y, z);
}

void LegController::setBodyRotation(float roll, float pitch, float yaw) {
    bodyRotation.set(roll, pitch, yaw);
}

void LegController::updateWalking(float vx, float vy, float vYaw, float deltaTime) {
    // Update gait phase
    updateGaitPhase(deltaTime, vx, vy, vYaw);
    
    // Update each leg based on its phase
    for (int leg = 0; leg < 4; leg++) {
        LegPosition legPos = (LegPosition)leg;
        
        if (legPhases[leg] == PHASE_STANCE) {
            // Stance phase: foot moves backward relative to body
            footPositions[leg] = calcStanceFootPos(legPos, vx, vy, vYaw, deltaTime);
        } else {
            // Swing phase: foot lifts and moves forward
            // Calculate swing progress based on which diagonal pair is swinging
            float swingProgress;
            if (leg == LEG_FRONT_LEFT || leg == LEG_BACK_RIGHT) {
                // FL/BR swing in second half (0.5-1.0)
                swingProgress = (gaitPhase - 0.5f) * 2.0f;
            } else {
                // FR/BL swing in first half (0.0-0.5)
                swingProgress = gaitPhase * 2.0f;
            }
            footPositions[leg] = calcSwingFootPos(legPos, swingProgress);
        }
    }
    
    // Solve IK for all legs
    for (int leg = 0; leg < 4; leg++) {
        LegPosition legPos = (LegPosition)leg;
        Vector3 footInLegFrame = bodyToLegFrame(footPositions[leg], legPos);
        solveAndStoreLeg(legPos, footInLegFrame);
    }
}

void LegController::updateGaitPhase(float deltaTime, float vx, float vy, float vYaw) {
    // Store previous phases for transition detection
    for (int i = 0; i < 4; i++) {
        prevLegPhases[i] = legPhases[i];
    }
    
    // Advance gait phase
    gaitPhase += gaitFrequency * gaitSpeed * deltaTime;
    
    // Wrap phase to 0-1 range
    while (gaitPhase >= 1.0f) {
        gaitPhase -= 1.0f;
    }
    
    // Update leg phases (trot gait pattern)
    // Phase 0.0-0.5: FL and BR in stance, FR and BL in swing
    // Phase 0.5-1.0: FL and BR in swing, FR and BL in stance
    if (gaitPhase < 0.5f) {
        legPhases[LEG_FRONT_LEFT] = PHASE_STANCE;
        legPhases[LEG_FRONT_RIGHT] = PHASE_SWING;
        legPhases[LEG_BACK_LEFT] = PHASE_SWING;
        legPhases[LEG_BACK_RIGHT] = PHASE_STANCE;
    } else {
        legPhases[LEG_FRONT_LEFT] = PHASE_SWING;
        legPhases[LEG_FRONT_RIGHT] = PHASE_STANCE;
        legPhases[LEG_BACK_LEFT] = PHASE_STANCE;
        legPhases[LEG_BACK_RIGHT] = PHASE_SWING;
    }
    
    // Detect phase transitions and update swing targets
    for (int leg = 0; leg < 4; leg++) {
        // Check if leg just transitioned from stance to swing
        if (prevLegPhases[leg] == PHASE_STANCE && legPhases[leg] == PHASE_SWING) {
            swingStart[leg] = footPositions[leg];
            
            // Calculate target position at end of swing with rotation effect
            Vector3 legAttach = getLegAttachPoint((LegPosition)leg);
            Vector3 rotEffect(-legAttach.y * vYaw * 0.5f, legAttach.x * vYaw * 0.5f, 0);
            Vector3 velocity(vx, vy, 0);
            
            swingTarget[leg] = defaultFootPositions[leg] + 
                               (velocity * strideLength + rotEffect) * 0.5f;
        }
    }
}


Vector3 LegController::calcStanceFootPos(LegPosition leg, float vx, float vy, float vYaw, float deltaTime) {
    // During stance, foot position moves backward relative to body
    Vector3 velocity(vx * strideLength, vy * strideLength, 0);
    
    // Apply rotation effect (feet move laterally during turn)
    Vector3 legAttach = getLegAttachPoint(leg);
    Vector3 rotEffect(-legAttach.y * vYaw * 0.5f, legAttach.x * vYaw * 0.5f, 0);
    
    Vector3 movement = (velocity + rotEffect) * deltaTime * gaitSpeed * 2.0f;
    
    return footPositions[leg] - movement;
}

Vector3 LegController::calcSwingFootPos(LegPosition leg, float swingProgress) {
    // Swing phase: interpolate from start to target with parabolic lift
    Vector3 pos = Vector3::lerp(swingStart[leg], swingTarget[leg], swingProgress);
    
    // Add parabolic height (peaks at middle of swing)
    float liftHeight = stepHeight * 4.0f * swingProgress * (1.0f - swingProgress);
    pos.z += liftHeight;
    
    return pos;
}

void LegController::stand() {
    // Reset to default stance
    gaitPhase = 0;
    gaitSpeed = 0;
    
    for (int i = 0; i < 4; i++) {
        footPositions[i] = defaultFootPositions[i];
        legPhases[i] = PHASE_STANCE;
    }
    
    // Solve IK for standing position
    for (int leg = 0; leg < 4; leg++) {
        LegPosition legPos = (LegPosition)leg;
        Vector3 footInLegFrame = bodyToLegFrame(footPositions[leg], legPos);
        solveAndStoreLeg(legPos, footInLegFrame);
    }
}

LegJoints LegController::getLegAngles(LegPosition leg) const {
    return currentAngles[leg];
}

Vector3 LegController::getFootPosition(LegPosition leg) const {
    return footPositions[leg];
}

Vector3 LegController::bodyToLegFrame(const Vector3& footInBody, LegPosition leg) const {
    // Get leg attachment point
    Vector3 attachPoint = getLegAttachPoint(leg);
    
    // Apply body rotation to foot position
    Vector3 rotatedFoot = rotateVector(footInBody - bodyPosition, bodyRotation);
    
    // Transform to leg frame (relative to attachment point)
    Vector3 legFrame = rotatedFoot - attachPoint;
    
    return legFrame;
}

Vector3 LegController::getLegAttachPoint(LegPosition leg) const {
    float halfLength = bodyLength / 2.0f;
    float halfWidth = bodyWidth / 2.0f;
    
    switch (leg) {
        case LEG_FRONT_LEFT:
            return Vector3(halfLength, halfWidth, 0);
        case LEG_FRONT_RIGHT:
            return Vector3(halfLength, -halfWidth, 0);
        case LEG_BACK_LEFT:
            return Vector3(-halfLength, halfWidth, 0);
        case LEG_BACK_RIGHT:
            return Vector3(-halfLength, -halfWidth, 0);
        default:
            return Vector3(0, 0, 0);
    }
}

Vector3 LegController::rotateVector(const Vector3& v, const Vector3& rotation) const {
    // Apply rotation using Euler angles (roll, pitch, yaw)
    float cr = cosf(rotation.x);  // roll
    float sr = sinf(rotation.x);
    float cp = cosf(rotation.y);  // pitch
    float sp = sinf(rotation.y);
    float cy = cosf(rotation.z);  // yaw
    float sy = sinf(rotation.z);
    
    // Rotation matrix multiplication (ZYX order)
    Vector3 result;
    result.x = v.x * (cy * cp) + v.y * (cy * sp * sr - sy * cr) + v.z * (cy * sp * cr + sy * sr);
    result.y = v.x * (sy * cp) + v.y * (sy * sp * sr + cy * cr) + v.z * (sy * sp * cr - cy * sr);
    result.z = v.x * (-sp) + v.y * (cp * sr) + v.z * (cp * cr);
    
    return result;
}

bool LegController::solveAndStoreLeg(LegPosition leg, const Vector3& targetFootPos) {
    LegJoints angles;
    bool success = legIK[leg].solve(targetFootPos, angles);
    
    if (success) {
        currentAngles[leg] = angles;
    }
    
    return success;
}
