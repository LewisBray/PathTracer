#ifndef VEC3_H
#define VEC3_H

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

#endif
