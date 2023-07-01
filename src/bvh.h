#ifndef BVH_H
#define BVH_H

#include "geometry.h"

#include <vector>

struct Node {
    AABB aabb;
    int index;
    int left;
    int right;
};

using BVH = std::vector<Node>;

static BVH construct_sphere_bvh(const Sphere* spheres, int count);

#endif
