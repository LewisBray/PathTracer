#include "material.h"

#include <cassert>

static Material construct_lambertian_material(const Colour& albedo) {
    Material material = {};
    material.lambertian.albedo = albedo;
    material.type = Material::Type::LAMBERTIAN;

    return material;
}

static Material construct_metal_material(const Colour& albedo, const real fuzziness) {
    Material material = {};
    material.metal.albedo = albedo;
    material.metal.fuzziness = fuzziness;
    material.type = Material::Type::METAL;

    return material;
}

static Material construct_dielectric_material(const real refraction_index) {
    Material material = {};
    material.dielectric.refraction_index = refraction_index;
    material.type = Material::Type::DIELECTRIC;

    return material;
}

static Material construct_diffuse_light_material(const Colour& emission_colour, const real emission_power) {
    Material material = {};
    material.diffuse_light.emission_colour = emission_colour;
    material.diffuse_light.emission_power = emission_power;
    material.type = Material::Type::DIFFUSE_LIGHT;

    return material;
}

static Colour get_colour(const Material& material) {
    switch (material.type) {
        case Material::Type::LAMBERTIAN: {
            return material.lambertian.albedo;
        }

        case Material::Type::METAL: {
            return material.metal.albedo;
        }

        case Material::Type::DIELECTRIC: {
            return Colour{1.0f, 1.0f, 1.0f};
        }

        case Material::Type::DIFFUSE_LIGHT: {
            return Colour{0.0f, 0.0f, 0.0f};
        }

        default: {
            assert(false);
            return Colour{0.0f, 0.0f, 0.0f};
        }
    }
}

static Colour get_emission(const Material& material) {
    switch (material.type) {
        case Material::Type::LAMBERTIAN:    // fallthrough
        case Material::Type::METAL:         // fallthrough
        case Material::Type::DIELECTRIC: {
            return Colour{0.0f, 0.0f, 0.0f};
        }

        case Material::Type::DIFFUSE_LIGHT: {
            return material.diffuse_light.emission_power * material.diffuse_light.emission_colour;
        }

        default: {
            assert(false);
            return Colour{0.0f, 0.0f, 0.0f};
        }
    }
}
