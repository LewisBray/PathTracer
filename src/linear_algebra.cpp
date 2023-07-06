#include "linear_algebra.h"

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

static Vec3 operator*(const Mat3& m, const Vec3& v) {
    return Vec3{
        m.rows[0][0] * v.x + m.rows[0][1] * v.y + m.rows[0][2] * v.z,
        m.rows[1][0] * v.x + m.rows[1][1] * v.y + m.rows[1][2] * v.z,
        m.rows[2][0] * v.x + m.rows[2][1] * v.y + m.rows[2][2] * v.z
    };
}

static Mat3 operator*(const Mat3& lhs, const Mat3& rhs) {
    Mat3 result = {};

    result.rows[0][0] = lhs.rows[0][0] * rhs.rows[0][0] + lhs.rows[0][1] * rhs.rows[1][0] + lhs.rows[0][2] * rhs.rows[2][0];
    result.rows[0][1] = lhs.rows[0][0] * rhs.rows[0][1] + lhs.rows[0][1] * rhs.rows[1][1] + lhs.rows[0][2] * rhs.rows[2][1];
    result.rows[0][2] = lhs.rows[0][0] * rhs.rows[0][2] + lhs.rows[0][1] * rhs.rows[1][2] + lhs.rows[0][2] * rhs.rows[2][2];

    result.rows[1][0] = lhs.rows[1][0] * rhs.rows[0][0] + lhs.rows[1][1] * rhs.rows[1][0] + lhs.rows[1][2] * rhs.rows[2][0];
    result.rows[1][1] = lhs.rows[1][0] * rhs.rows[0][1] + lhs.rows[1][1] * rhs.rows[1][1] + lhs.rows[1][2] * rhs.rows[2][1];
    result.rows[1][2] = lhs.rows[1][0] * rhs.rows[0][2] + lhs.rows[1][1] * rhs.rows[1][2] + lhs.rows[1][2] * rhs.rows[2][2];

    result.rows[2][0] = lhs.rows[2][0] * rhs.rows[0][0] + lhs.rows[2][1] * rhs.rows[1][0] + lhs.rows[2][2] * rhs.rows[2][0];
    result.rows[2][1] = lhs.rows[2][0] * rhs.rows[0][1] + lhs.rows[2][1] * rhs.rows[1][1] + lhs.rows[2][2] * rhs.rows[2][1];
    result.rows[2][2] = lhs.rows[2][0] * rhs.rows[0][2] + lhs.rows[2][1] * rhs.rows[1][2] + lhs.rows[2][2] * rhs.rows[2][2];
    
    return result;
}

static Mat3 scaling_matrix(const real x_scale, const real y_scale, const real z_scale) {
    Mat3 result = {};
    result.rows[0][0] = x_scale;
    result.rows[1][1] = y_scale;
    result.rows[2][2] = z_scale;

    return result;
}

static Mat3 rotation_matrix(const real angle, const real axis_x, const real axis_y, const real axis_z) {
    const Vec3 rotation_vector = normalise(Vec3{axis_x, axis_y, axis_z});
    const real sin_angle = std::sin(angle);
    const real cos_angle = std::cos(angle);

    Mat3 result = {};

    result.rows[0][0] = (1.0f - cos_angle) * rotation_vector.x * rotation_vector.x + cos_angle;
    result.rows[0][1] = (1.0f - cos_angle) * rotation_vector.x * rotation_vector.y - sin_angle * rotation_vector.z;
    result.rows[0][2] = (1.0f - cos_angle) * rotation_vector.x * rotation_vector.z + sin_angle * rotation_vector.y;

    result.rows[1][0] = (1.0f - cos_angle) * rotation_vector.x * rotation_vector.y + sin_angle * rotation_vector.z;
    result.rows[1][1] = (1.0f - cos_angle) * rotation_vector.y * rotation_vector.y + cos_angle;
    result.rows[1][2] = (1.0f - cos_angle) * rotation_vector.y * rotation_vector.z - sin_angle * rotation_vector.x;

    result.rows[2][0] = (1.0f - cos_angle) * rotation_vector.x * rotation_vector.z - sin_angle * rotation_vector.y;
    result.rows[2][1] = (1.0f - cos_angle) * rotation_vector.y * rotation_vector.z + sin_angle * rotation_vector.x;
    result.rows[2][2] = (1.0f - cos_angle) * rotation_vector.z * rotation_vector.z + cos_angle;

    return result;
}
