#ifndef LINEAR_ALGEBRA_H
#define LINEAR_ALGEBRA_H

#include "types.h"

struct Vec3 {
    real x;
    real y;
    real z;
};

static Vec3 operator+(const Vec3& lhs, const Vec3& rhs);
static Vec3 operator-(const Vec3& v);
static Vec3 operator-(const Vec3& lhs, const Vec3& rhs);
static Vec3 operator*(real scalar, const Vec3& v);
static real operator*(const Vec3& lhs, const Vec3& rhs);
static Vec3 operator^(const Vec3& lhs, const Vec3& rhs);

static real magnitude(const Vec3& v);
static Vec3 normalise(const Vec3& v);

struct Mat3 {
    real rows[3][3];
};

static Vec3 operator*(const Mat3& m, const Vec3& v);
static Mat3 operator*(const Mat3& lhs, const Mat3& rhs);
static Mat3 scaling_matrix(real x_scale, real y_scale, real z_scale);
static Mat3 rotation_matrix(real angle, real axis_x, real axis_y, real axis_z);

#endif
