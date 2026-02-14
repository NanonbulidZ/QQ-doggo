/**
 * IKSolver.h
 * Inverse kinematics solver for 3-DOF robot legs
 * 
 * Implements analytical IK solution using law of cosines
 * for a 3-segment leg (coxa, femur, tibia)
 */

#pragma once

#include "Vector3.h"
#include <math.h>

// Joint angles for a single leg (radians)
struct LegJoints {
    float alpha;  // Shoulder/coxa angle (rotation around Z-axis)
    float beta;   // Elbow/femur angle
    float gamma;  // Ankle/tibia angle

    LegJoints() : alpha(0), beta(0), gamma(0) {}
    LegJoints(float a, float b, float g) : alpha(a), beta(b), gamma(g) {}
};

// 3-DOF Leg Inverse Kinematics Solver
class LegIK {
public:
    // IK solver constants
    static constexpr float REACH_SAFETY_FACTOR = 0.95f;  // Max reach safety margin
    static constexpr float ELBOW_REACH_SAFETY = 0.99f;   // Elbow joint reach safety
    
    // Leg segment lengths (mm)
    float coxaLength;   // l1: shoulder to elbow
    float femurLength;  // l2: elbow to ankle
    float tibiaLength;  // l3: ankle to foot

    // Joint angle limits (radians)
    float alphaMin, alphaMax;
    float betaMin, betaMax;
    float gammaMin, gammaMax;

    // Constructor with segment lengths
    LegIK(float coxa = 24.0f, float femur = 51.9f, float tibia = 50.0f) 
        : coxaLength(coxa), femurLength(femur), tibiaLength(tibia),
          alphaMin(0), alphaMax(M_PI),
          betaMin(0), betaMax(M_PI),
          gammaMin(0), gammaMax(M_PI) {}

    // Set joint angle limits
    void setAngleLimits(float aMin, float aMax, float bMin, float bMax, float gMin, float gMax) {
        alphaMin = aMin; alphaMax = aMax;
        betaMin = bMin; betaMax = bMax;
        gammaMin = gMin; gammaMax = gMax;
    }

    /**
     * Solve IK for target foot position relative to leg base
     * @param target Target foot position (x, y, z) in leg's local coordinate frame
     * @param result Output joint angles
     * @return true if solution is valid, false if unreachable
     */
    bool solve(const Vector3& target, LegJoints& result) {
        // Project onto XZ plane for alpha (shoulder rotation)
        float r_xz = sqrtf(target.x * target.x + target.z * target.z);
        
        // Check if target is reachable (simplified check)
        float maxReach = coxaLength + femurLength + tibiaLength;
        float targetDist = target.magnitude();
        if (targetDist > maxReach * REACH_SAFETY_FACTOR) {
            return false; // Too far
        }

        // Alpha: shoulder rotation (atan2 in XZ plane)
        // Adjust for coxa offset
        if (r_xz < 0.0001f) {
            result.alpha = 0;
        } else {
            result.alpha = atan2f(target.z, target.x) - atan2f(coxaLength, 0);
        }

        // Distance from shoulder joint to target in XZ plane (after alpha rotation)
        float d_xz = r_xz - coxaLength;
        
        // 3D distance from elbow to target
        float d = sqrtf(d_xz * d_xz + target.y * target.y);

        // Check if within femur+tibia reach
        if (d > (femurLength + tibiaLength) * ELBOW_REACH_SAFETY || d < fabs(femurLength - tibiaLength)) {
            return false; // Unreachable
        }

        // Beta: elbow angle using law of cosines
        // Angle between femur and the line to target
        float cosGamma = (femurLength * femurLength + tibiaLength * tibiaLength - d * d) 
                       / (2.0f * femurLength * tibiaLength);
        cosGamma = constrain(cosGamma, -1.0f, 1.0f);
        result.gamma = acosf(cosGamma);

        // Beta calculation with elevation angle
        float cosBeta = (femurLength * femurLength + d * d - tibiaLength * tibiaLength) 
                      / (2.0f * femurLength * d);
        cosBeta = constrain(cosBeta, -1.0f, 1.0f);
        float beta_temp = acosf(cosBeta);
        
        // Elevation angle to target
        float elevation = atan2f(target.y, d_xz);
        result.beta = elevation + beta_temp;

        // Validate angle limits
        if (!isWithinLimits(result)) {
            return false;
        }

        return true;
    }

    /**
     * Check if joint angles are within limits
     */
    bool isWithinLimits(const LegJoints& joints) const {
        return (joints.alpha >= alphaMin && joints.alpha <= alphaMax &&
                joints.beta >= betaMin && joints.beta <= betaMax &&
                joints.gamma >= gammaMin && joints.gamma <= gammaMax);
    }

private:
    // Constrain value to range
    float constrain(float value, float min, float max) const {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }
};
