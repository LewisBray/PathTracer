#include "geometry.h"

#include <cassert>
#include <cmath>

static Maybe<SphereIntersections> intersect(const Ray& ray, const Sphere& sphere) {
    assert(std::abs(magnitude(ray.direction) - 1.0f) < 1.0e-6f);

    Maybe<SphereIntersections> result = {};

    const Vec3 ray_origin_to_sphere_centre = sphere.centre - ray.origin;
    const real intersections_mid_point_distance = ray_origin_to_sphere_centre * ray.direction;

    const real ray_origin_to_sphere_centre_distance_squared = ray_origin_to_sphere_centre * ray_origin_to_sphere_centre;
    const real intersections_mid_point_distance_squared = intersections_mid_point_distance * intersections_mid_point_distance;
    const real sphere_centre_to_intersections_mid_point_squared = ray_origin_to_sphere_centre_distance_squared - intersections_mid_point_distance_squared;
    const real sphere_radius_squared = sphere.radius * sphere.radius;
    const real intersections_mid_point_to_intersections_distance_squared = sphere_radius_squared - sphere_centre_to_intersections_mid_point_squared;
    if (intersections_mid_point_to_intersections_distance_squared < 0.0f) { // TODO: removing this branch causes a huge slowdown, look into that
        return result;
    }

    const real intersections_mid_point_to_intersections_distance = std::sqrt(intersections_mid_point_to_intersections_distance_squared);
    result.value.distance_1 = intersections_mid_point_distance - intersections_mid_point_to_intersections_distance;
    result.value.distance_2 = intersections_mid_point_distance + intersections_mid_point_to_intersections_distance;
    result.is_valid = true;

    return result;
}

static Maybe<ClosestSphereIntersection> intersect(const Ray& ray, const Sphere* const spheres, const int count) {
    Maybe<ClosestSphereIntersection> closest_intersection = {};
    closest_intersection.is_valid = false;
    closest_intersection.value.index = -1;
    closest_intersection.value.distance = REAL_MAX;
    for (int sphere_index = 0; sphere_index < count; ++sphere_index) {
        const Maybe<SphereIntersections> intersection_result = intersect(ray, spheres[sphere_index]);
        if (intersection_result.is_valid) {
            if (0.0f < intersection_result.value.distance_1 && intersection_result.value.distance_1 < closest_intersection.value.distance) {
                closest_intersection.is_valid = true;
                closest_intersection.value.index = sphere_index;
                closest_intersection.value.distance = intersection_result.value.distance_1;
            } else if (0.0f < intersection_result.value.distance_2 && intersection_result.value.distance_2 < closest_intersection.value.distance) {
                closest_intersection.is_valid = true;
                closest_intersection.value.index = sphere_index;
                closest_intersection.value.distance = intersection_result.value.distance_2;
            }
        }
    }

    return closest_intersection;
}
