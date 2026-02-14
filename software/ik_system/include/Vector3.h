/**
 * Vector3.h
 * 3D vector mathematics library for inverse kinematics
 * 
 * Provides essential vector operations for 3D robotics calculations
 */

#pragma once

#include <math.h>

class Vector3 {
public:
    float x, y, z;

    // Constructors
    Vector3() : x(0), y(0), z(0) {}
    Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

    // Vector addition
    Vector3 operator+(const Vector3& v) const {
        return Vector3(x + v.x, y + v.y, z + v.z);
    }

    // Vector subtraction
    Vector3 operator-(const Vector3& v) const {
        return Vector3(x - v.x, y - v.y, z - v.z);
    }

    // Scalar multiplication
    Vector3 operator*(float scalar) const {
        return Vector3(x * scalar, y * scalar, z * scalar);
    }

    // Scalar division
    Vector3 operator/(float scalar) const {
        return Vector3(x / scalar, y / scalar, z / scalar);
    }

    // Compound assignment operators
    Vector3& operator+=(const Vector3& v) {
        x += v.x; y += v.y; z += v.z;
        return *this;
    }

    Vector3& operator-=(const Vector3& v) {
        x -= v.x; y -= v.y; z -= v.z;
        return *this;
    }

    Vector3& operator*=(float scalar) {
        x *= scalar; y *= scalar; z *= scalar;
        return *this;
    }

    // Dot product
    float dot(const Vector3& v) const {
        return x * v.x + y * v.y + z * v.z;
    }

    // Cross product
    Vector3 cross(const Vector3& v) const {
        return Vector3(
            y * v.z - z * v.y,
            z * v.x - x * v.z,
            x * v.y - y * v.x
        );
    }

    // Magnitude (length) of vector
    float magnitude() const {
        return sqrtf(x * x + y * y + z * z);
    }

    // Squared magnitude (avoids sqrt for comparison)
    float magnitudeSquared() const {
        return x * x + y * y + z * z;
    }

    // Normalize vector to unit length
    Vector3 normalized() const {
        float mag = magnitude();
        if (mag > 0.0001f) {
            return *this / mag;
        }
        return Vector3(0, 0, 0);
    }

    // Normalize in place
    void normalize() {
        float mag = magnitude();
        if (mag > 0.0001f) {
            x /= mag;
            y /= mag;
            z /= mag;
        }
    }

    // Distance between two points
    static float distance(const Vector3& a, const Vector3& b) {
        return (a - b).magnitude();
    }

    // Linear interpolation
    static Vector3 lerp(const Vector3& a, const Vector3& b, float t) {
        return a + (b - a) * t;
    }

    // Set all components
    void set(float nx, float ny, float nz) {
        x = nx; y = ny; z = nz;
    }
};