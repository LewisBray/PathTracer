#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "types.h"
#include "vec3.h"

#include "maybe.hpp"

struct Ray {
    Vec3 origin;
    Vec3 direction;
};

struct Sphere {
    Vec3 centre;
    real radius;
};

struct SphereIntersections {
    real distance_1;
    real distance_2;
};

static Maybe<SphereIntersections> intersect(const Ray& ray, const Sphere& sphere);

struct ClosestSphereIntersection {
    int index;
    real distance;
};

static Maybe<ClosestSphereIntersection> intersect(const Ray& ray, const Sphere* spheres, int count);

#endif
