#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "linear_algebra.h"
#include "types.h"

#include "maybe.hpp"

struct Ray {
    Vec3 origin;
    Vec3 direction;
};

struct AABB {
    Vec3 min;
    Vec3 max;
};

static AABB& operator+=(AABB& lhs, const AABB& rhs);

struct AABBIntersections {
    real min_distance;
    real max_distance;
};

static AABBIntersections intersect(const Ray& ray, const AABB& aabb);

struct Sphere {
    Vec3 centre;
    real radius;
};

static AABB construct_aabb(const Sphere& sphere);

struct SphereIntersections {
    real min_distance;
    real max_distance;
};

static Maybe<SphereIntersections> intersect(const Ray& ray, const Sphere& sphere);

struct Triangle {
    Vec3 a;
    Vec3 b;
    Vec3 c;
};

static Vec3 unit_normal(const Triangle& triangle);
static AABB construct_aabb(const Triangle& triangle);
static Maybe<real> intersect(const Ray& ray, const Triangle& triangle);

#endif
