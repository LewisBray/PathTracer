#ifndef PATH_TRACING_H
#define PATH_TRACING_H

#include "geometry.h"
#include "material.h"
#include "colour.h"
#include "bvh.h"

struct Scene {
    const Material* materials;
    
    const Sphere* spheres;
    const BVH* sphere_bvh;
    const int* sphere_material_indices;
    int sphere_count;
};

static Colour intersect(Ray ray, const Scene& scene);

#endif
