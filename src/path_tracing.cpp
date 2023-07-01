#include "path_tracing.h"
#include "bvh.h"

#include <cassert>
#include <cmath>

static Vec3 reflect(const Vec3& direction, const Vec3& unit_normal) {
    assert(std::abs(magnitude(direction) - 1.0f) < 1.0e-6);
    assert(std::abs(magnitude(unit_normal) - 1.0f) < 1.0e-6);

    return direction - 2.0f * (direction * unit_normal) * unit_normal;
}

static Vec3 refract(const Vec3& direction, const Vec3& unit_normal, const real refraction_ratio) {
    assert(std::abs(magnitude(direction) - 1.0f) < 1.0e-6);
    assert(std::abs(magnitude(unit_normal) - 1.0f) < 1.0e-6);

    const real cos_theta = -direction * unit_normal;
    assert(-1.0f <= cos_theta && cos_theta <= 1.0f);
    const Vec3 refracted_direction_perpendicular = refraction_ratio * (direction + cos_theta * unit_normal);
    const Vec3 refracted_direction_parallel = -std::sqrt(std::abs(1.0f - refracted_direction_perpendicular * refracted_direction_perpendicular)) * unit_normal;

    return normalise(refracted_direction_perpendicular + refracted_direction_parallel);
}

// Uses Schlick's approximation of reflectance
static real reflectance(const real cos_theta, const real refraction_ratio) {
    const real ratio = (1.0f - refraction_ratio) / (1.0f + refraction_ratio);
    const real r0 = ratio * ratio;
    const real difference = (1.0f - cos_theta);

    return r0 + (1.0f - r0) * difference * difference * difference * difference * difference;
}

static Maybe<Ray> scatter(const Ray& ray, const Material& material, const Vec3& point, const Vec3& point_unit_normal) {
    assert(std::abs(magnitude(ray.direction) - 1.0f) < 1.0e-6);
    assert(std::abs(magnitude(point_unit_normal) - 1.0f) < 1.0e-6);

    static constexpr real NUDGE_FACTOR = 0.001f;

    switch (material.type) {
        case Material::Type::LAMBERTIAN: {
            Maybe<Ray> scattered_ray = {};
            const Vec3 random = random_unit_vector(point_unit_normal);
            const real sign = (random * point_unit_normal < 0.0f) ? -1.0f : 1.0f;
            scattered_ray.value.origin = point + NUDGE_FACTOR * point_unit_normal;
            scattered_ray.value.direction = sign * random;
            scattered_ray.is_valid = true;
            
            return scattered_ray;
        }

        case Material::Type::METAL: {
            Maybe<Ray> scattered_ray = {};
            const Vec3 random = random_unit_vector(point_unit_normal);
            const real fuzziness = material.metal.fuzziness;
            const Vec3 reflected_direction = reflect(ray.direction, point_unit_normal);
            scattered_ray.value.origin = point + NUDGE_FACTOR * point_unit_normal;
            scattered_ray.value.direction = normalise(reflected_direction + fuzziness * random);
            scattered_ray.is_valid = (scattered_ray.value.direction * point_unit_normal > 0.0f);
            
            return scattered_ray;
        }

        case Material::Type::DIELECTRIC: {
            Maybe<Ray> scattered_ray = {};
            const bool front_face = (ray.direction * point_unit_normal < 0.0f);
            const real refraction_ratio = front_face ? 1.0f / material.dielectric.refraction_index : material.dielectric.refraction_index;
            const Vec3 unit_normal = front_face ? point_unit_normal : -point_unit_normal;

            const real cos_theta = -ray.direction * unit_normal;
            const real sin_theta = std::sqrt(1.0f - cos_theta * cos_theta);
            const u32 reflectance_rng = noise_1d(static_cast<u32>(1000000000.0f * sin_theta));
            const real reflectance_threshold = real_from_rng(reflectance_rng);
            if (refraction_ratio * sin_theta > 1.0f || reflectance(cos_theta, refraction_ratio) > reflectance_threshold) {
                scattered_ray.value.origin = point + NUDGE_FACTOR * unit_normal;
                scattered_ray.value.direction = reflect(ray.direction, unit_normal);
            } else {
                scattered_ray.value.origin = point - NUDGE_FACTOR * unit_normal;
                scattered_ray.value.direction = refract(ray.direction, unit_normal, refraction_ratio);
            }

            scattered_ray.is_valid = true;

            return scattered_ray;
        }

        default: {
            assert(false);
            return Maybe<Ray>{};
        }
    }
}

static Colour background_gradient(const Ray& ray) {
    const real t = 0.5f * (ray.direction.y + 1.0f);
    return (1.0f - t) * Colour{1.0, 1.0, 1.0} + t * Colour{0.5, 0.7, 1.0};
}

struct ClosestSphereIntersection {
    int index;
    real distance;
};

static constexpr ClosestSphereIntersection MISS{-1, REAL_MAX};

static ClosestSphereIntersection intersect(const Ray& ray, const BVH& bvh, const int node_index, const Sphere* const spheres) {
    const Node& node = bvh[node_index];
    const AABBIntersections aabb_intersections = intersect(ray, node.aabb);
    if (aabb_intersections.max_distance < 0.0f || aabb_intersections.min_distance > aabb_intersections.max_distance) {
        return MISS;
    }

    if (node.left == 0 && node.right == 0) {
        const Sphere& sphere = spheres[node.index];
        const Maybe<SphereIntersections> sphere_intersections = intersect(ray, sphere);
        ClosestSphereIntersection sphere_intersection = MISS;
        if (sphere_intersections.is_valid && (sphere_intersections.value.min_distance > 0.0f || sphere_intersections.value.max_distance > 0.0f)) {
            sphere_intersection.distance = sphere_intersections.value.min_distance > 0.0f ? sphere_intersections.value.min_distance : sphere_intersections.value.max_distance;
            sphere_intersection.index = node.index;
        }

        return sphere_intersection;
    } else {
        const ClosestSphereIntersection left_intersection = intersect(ray, bvh, node.left, spheres);
        const ClosestSphereIntersection right_intersection = intersect(ray, bvh, node.right, spheres);
        if (left_intersection.distance < right_intersection.distance) {
            return left_intersection;
        } else if (right_intersection.distance < left_intersection.distance) {
            return right_intersection;
        } else {
            return MISS;
        }
    }
}

static Colour intersect(Ray ray, const Scene& scene) {
    static constexpr int MAX_BOUNCE_COUNT = 50;

    Colour colour = {0.0f, 0.0f, 0.0f};
    Colour attenuation = {1.0f, 1.0f, 1.0f};
    for (int bounce_index = 0; bounce_index < MAX_BOUNCE_COUNT; ++bounce_index) {
        const ClosestSphereIntersection closest_sphere_intersection = intersect(ray, *scene.sphere_bvh, 0, scene.spheres);
        if (closest_sphere_intersection.distance < REAL_MAX && closest_sphere_intersection.index != -1) {
            const int sphere_index = closest_sphere_intersection.index;
            const Sphere& sphere = scene.spheres[sphere_index];
            const Vec3 intersection_point = ray.origin + closest_sphere_intersection.distance * ray.direction;
            const real sign = (sphere.radius < 0.0f) ? -1.0f : 1.0f;    // trick to model hollow spheres, don't want this polluting the scatter routine
            const Vec3 sphere_unit_normal = sign * normalise(intersection_point - sphere.centre);

            const int sphere_material_index = scene.sphere_material_indices[sphere_index];
            const Material& sphere_material = scene.materials[sphere_material_index];

            const Maybe<Ray> scattered_ray = scatter(ray, sphere_material, intersection_point, sphere_unit_normal);
            if (scattered_ray.is_valid) {
                ray = scattered_ray.value;

                const Colour& sphere_material_colour = get_colour(sphere_material);
                attenuation *= sphere_material_colour;
            } else {
                colour = Colour{0.0f, 0.0f, 0.0f};
                break;
            }
        } else {
            colour += attenuation * background_gradient(ray);
            break;
        }
    }

    return colour;
}
