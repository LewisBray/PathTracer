#include "vec3.h"

#include <cmath>

static Vec3 operator+(const Vec3& lhs, const Vec3& rhs) {
    return Vec3{lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

static Vec3 operator-(const Vec3& v) {
    return Vec3{-v.x, -v.y, -v.z};
}

static Vec3 operator-(const Vec3& lhs, const Vec3& rhs) {
    return Vec3{lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

static Vec3 operator*(const real scalar, const Vec3& v) {
    return Vec3{scalar * v.x, scalar * v.y, scalar * v.z};
}

static real operator*(const Vec3& lhs, const Vec3& rhs) {
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

static Vec3 operator^(const Vec3& lhs, const Vec3& rhs) {
    return Vec3{
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x
    };
}

static real magnitude(const Vec3& v) {
    return std::sqrt(v * v);
}

static Vec3 normalise(const Vec3& v) {
    const real inverse_magnitude = 1.0f / magnitude(v);
    return inverse_magnitude * v;
}
