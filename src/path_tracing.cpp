#include "path_tracing.h"
#include "bvh.h"

#include <cassert>
#include <cmath>

static Vec3 reflect(const Vec3& direction, const Vec3& unit_normal) {
    assert(std::abs(direction * direction - 1.0f) < 1.0e-6f);
    assert(std::abs(unit_normal * unit_normal - 1.0f) < 1.0e-6f);

    return direction - 2.0f * (direction * unit_normal) * unit_normal;
}

static Vec3 refract(const Vec3& direction, const Vec3& unit_normal, const real refraction_ratio) {
    assert(std::abs(direction * direction - 1.0f) < 1.0e-6f);
    assert(std::abs(unit_normal * unit_normal - 1.0f) < 1.0e-6f);

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
    assert(std::abs(ray.direction * ray.direction - 1.0f) < 1.0e-6f);
    assert(std::abs(point_unit_normal * point_unit_normal - 1.0f) < 1.0e-6f);

    static constexpr real NUDGE_FACTOR = 0.001f;

    switch (material.type) {
        case Material::Type::LAMBERTIAN: {
            Maybe<Ray> scattered_ray = {};
            const Vec3 random = random_unit_vector(ray.direction);
            scattered_ray.value.origin = point + NUDGE_FACTOR * point_unit_normal;
            scattered_ray.value.direction = normalise(point_unit_normal + 0.99f * random);
            scattered_ray.is_valid = true;
            
            return scattered_ray;
        }

        case Material::Type::METAL: {
            Maybe<Ray> scattered_ray = {};
            const Vec3 random = random_unit_vector(ray.direction);
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

        case Material::Type::DIFFUSE_LIGHT: {
            Maybe<Ray> scattered_ray = {};
            return scattered_ray;
        }

        default: {
            assert(false);
            return Maybe<Ray>{};
        }
    }
}

static Colour background_gradient(const Colour& start, const Colour& end, const real ray_direction_y) {
    const real t = 0.5f * (ray_direction_y + 1.0f);
    return (1.0f - t) * start + t * end;
}

struct ClosestShapeIntersection {
    int index;
    real distance;
};

static constexpr ClosestShapeIntersection MISS{-1, REAL_MAX};

static ClosestShapeIntersection intersect(const Ray& ray, const BVH& bvh, const int node_index, const Sphere* const spheres) {
    const Node& node = bvh[node_index];
    const AABBIntersections aabb_intersections = intersect(ray, node.aabb);
    if (aabb_intersections.max_distance < 0.0f || aabb_intersections.min_distance > aabb_intersections.max_distance) {
        return MISS;
    }

    if (node.left == 0 && node.right == 0) {
        const Sphere& sphere = spheres[node.index];
        const Maybe<SphereIntersections> sphere_intersections = intersect(ray, sphere);
        ClosestShapeIntersection sphere_intersection = MISS;
        if (sphere_intersections.is_valid && (sphere_intersections.value.min_distance > 0.0f || sphere_intersections.value.max_distance > 0.0f)) {
            sphere_intersection.distance = sphere_intersections.value.min_distance > 0.0f ? sphere_intersections.value.min_distance : sphere_intersections.value.max_distance;
            sphere_intersection.index = node.index;
        }

        return sphere_intersection;
    } else {
        const ClosestShapeIntersection left_intersection = intersect(ray, bvh, node.left, spheres);
        const ClosestShapeIntersection right_intersection = intersect(ray, bvh, node.right, spheres);
        if (left_intersection.distance < right_intersection.distance) {
            return left_intersection;
        } else if (right_intersection.distance < left_intersection.distance) {
            return right_intersection;
        } else {
            return MISS;
        }
    }
}

static ClosestShapeIntersection intersect(const Ray& ray, const BVH& bvh, const int node_index, const Triangle* const triangles) {
    const Node& node = bvh[node_index];
    const AABBIntersections aabb_intersections = intersect(ray, node.aabb);
    if (aabb_intersections.max_distance < 0.0f || aabb_intersections.min_distance > aabb_intersections.max_distance) {
        return MISS;
    }

    if (node.left == 0 && node.right == 0) {
        const Triangle& triangle = triangles[node.index];
        const Maybe<real> triangle_intersection = intersect(ray, triangle); // TODO: should this routine check < 0, sphere intersection doesn't
        ClosestShapeIntersection result = MISS;
        if (triangle_intersection.is_valid) {
            result.distance = triangle_intersection.value;
            result.index = node.index;
        }

        return result;
    } else {
        const ClosestShapeIntersection left_intersection = intersect(ray, bvh, node.left, triangles);
        const ClosestShapeIntersection right_intersection = intersect(ray, bvh, node.right, triangles);
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
        const ClosestShapeIntersection closest_sphere_intersection = (scene.sphere_bvh != nullptr) ? intersect(ray, *scene.sphere_bvh, 0, scene.spheres) : MISS;
        const ClosestShapeIntersection closest_triangle_intersection = (scene.triangle_bvh != nullptr) ? intersect(ray, *scene.triangle_bvh, 0, scene.triangles) : MISS;
        if (closest_sphere_intersection.index != -1 || closest_triangle_intersection.index != -1) {
            Vec3 intersection_point = {};
            Vec3 shape_unit_normal = {};
            Material material = {};
            if (closest_sphere_intersection.distance < closest_triangle_intersection.distance) {
                assert(closest_sphere_intersection.distance < REAL_MAX);

                const int sphere_index = closest_sphere_intersection.index;
                const Sphere& sphere = scene.spheres[sphere_index];
                intersection_point = ray.origin + closest_sphere_intersection.distance * ray.direction;
                
                const real sign = (sphere.radius < 0.0f) ? -1.0f : 1.0f;    // trick to model hollow spheres, don't want this polluting the scatter routine
                shape_unit_normal = sign * normalise(intersection_point - sphere.centre);   // TODO: can divide by radius instead

                const int sphere_material_index = scene.sphere_material_indices[sphere_index];
                material = scene.materials[sphere_material_index];
            } else {
                assert(closest_triangle_intersection.distance < REAL_MAX);

                const int triangle_index = closest_triangle_intersection.index;
                const Triangle& triangle = scene.triangles[triangle_index];
                intersection_point = ray.origin + closest_triangle_intersection.distance * ray.direction;
                
                shape_unit_normal = unit_normal(triangle);

                const int triangle_material_index = scene.triangle_material_indices[triangle_index];
                material = scene.materials[triangle_material_index];
            }

            const Colour material_emission = get_emission(material);
            const Maybe<Ray> scattered_ray = scatter(ray, material, intersection_point, shape_unit_normal);
            if (scattered_ray.is_valid) {
                ray = scattered_ray.value;

                const Colour& material_colour = get_colour(material);
                colour += attenuation * material_emission;
                attenuation *= material_colour;
            } else {
                colour += attenuation * material_emission;
                break;
            }
        } else {
            colour += attenuation * background_gradient(scene.background_gradient_start, scene.background_gradient_end, ray.direction.y);
            break;
        }
    }

    return colour;
}

static Vec3 get_position(const Camera& camera) {
    return camera.orientation * Vec3{0.0f, 0.0f, camera.distance} + camera.target;
}
