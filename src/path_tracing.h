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

    const Triangle* triangles;
    const BVH* triangle_bvh;
    const int* triangle_material_indices;

    Colour background_gradient_start;
    Colour background_gradient_end;
};

static Colour intersect(Ray ray, const Scene& scene);

// TODO: should this have an aspect ratio? would need to handle resizing of window
struct Camera {
    Mat3 orientation;
    Vec3 target;
    real distance;

    real fov_y;
    real aperture;
    real focus_distance;
};

static Vec3 get_position(const Camera& camera);

#endif
