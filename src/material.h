#ifndef MATERIAL_H
#define MATERIAL_H

#include "types.h"
#include "colour.h"

struct Lambertian {
    Colour albedo;
};

struct Metal {
    Colour albedo;
    real fuzziness;
};

struct Dielectric {
    real refraction_index;
};

struct Material {
    enum Type {
        LAMBERTIAN = 0,
        METAL = 1,
        DIELECTRIC = 2
    };

    union {
        Lambertian lambertian;
        Metal metal;
        Dielectric dielectric;
    };

    Type type;
};

static Material construct_lambertian_material(const Colour& albedo);
static Material construct_metal_material(const Colour& albedo, real fuzziness);
static Material construct_dielectric_material(real refraction_index);
static Colour get_colour(const Material& material);

#endif
