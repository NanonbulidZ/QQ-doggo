// 3D vector mathematics library
// Modern C++ implementation with complete vector operations

#pragma once

#include <cmath>

class Vector3 {
public:
    float x, y, z;

    // Constructors
    Vector3() : x(0), y(0), z(0) {}
    Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

    // Addition
    Vector3 operator+(const Vector3& v) const {
        return Vector3(x + v.x, y + v.y, z + v.z);
    }

    // Subtraction
    Vector3 operator-(const Vector3& v) const {
        return Vector3(x - v.x, y - v.y, z - v.z);
    }

    // Scalar multiplication (scaling)
    Vector3 operator*(float scalar) const {
        return Vector3(x * scalar, y * scalar, z * scalar);
    }

    // Scalar division
    Vector3 operator/(float scalar) const {
        return Vector3(x / scalar, y / scalar, z / scalar);
    }

    // Compound assignment operators
    Vector3& operator+=(const Vector3& v) {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }

    Vector3& operator-=(const Vector3& v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }

    Vector3& operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    Vector3& operator/=(float scalar) {
        x /= scalar;
        y /= scalar;
        z /= scalar;
        return *this;
    }

    // Unary minus (negation)
    Vector3 operator-() const {
        return Vector3(-x, -y, -z);
    }

    // Comparison operators
    bool operator==(const Vector3& v) const {
        return x == v.x && y == v.y && z == v.z;
    }

    bool operator!=(const Vector3& v) const {
        return !(*this == v);
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

    // Magnitude (length) of the vector
    float magnitude() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    // Squared magnitude (useful to avoid sqrt in comparisons)
    float magnitudeSquared() const {
        return x * x + y * y + z * z;
    }

    // Normalize the vector (return unit vector)
    Vector3 normalized() const {
        float mag = magnitude();
        if (mag > 0.0f) {
            return *this / mag;
        }
        return Vector3(0, 0, 0);
    }

    // Normalize in place
    void normalize() {
        float mag = magnitude();
        if (mag > 0.0f) {
            *this /= mag;
        }
    }

    // Distance to another vector
    float distance(const Vector3& v) const {
        return (*this - v).magnitude();
    }

    // Squared distance (useful to avoid sqrt in comparisons)
    float distanceSquared(const Vector3& v) const {
        return (*this - v).magnitudeSquared();
    }

    // Length (alias for magnitude)
    float length() const {
        return magnitude();
    }

    // Squared length (alias for magnitudeSquared)
    float lengthSquared() const {
        return magnitudeSquared();
    }
};

// Scalar multiplication from left (scalar * vector)
inline Vector3 operator*(float scalar, const Vector3& v) {
    return v * scalar;
}