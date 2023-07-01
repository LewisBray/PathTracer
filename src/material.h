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

struct DiffuseLight {
    Colour emission_colour;
    real emission_power;
};

struct Material {
    enum Type {
        LAMBERTIAN = 0,
        METAL = 1,
        DIELECTRIC = 2,
        DIFFUSE_LIGHT = 3
    };

    union {
        Lambertian lambertian;
        Metal metal;
        Dielectric dielectric;
        DiffuseLight diffuse_light;
    };

    Type type;
};

static Material construct_lambertian_material(const Colour& albedo);
static Material construct_metal_material(const Colour& albedo, real fuzziness);
static Material construct_dielectric_material(real refraction_index);
static Material construct_diffuse_light_material(const Colour& emission_colour, real emission_power);
static Colour get_colour(const Material& material);
static Colour get_emission(const Material& material);

#endif
