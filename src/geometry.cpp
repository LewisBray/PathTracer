#include "geometry.h"

#include <algorithm>
#include <cassert>
#include <cmath>

static AABBIntersections intersect(const Ray& ray, const AABB& aabb) {
    static_assert(std::numeric_limits<real>::is_iec559, "IEEE754 floating-point implementation required");
    assert(std::abs(magnitude(ray.direction) - 1.0f) < 1.0e-6);

    const real inverse_x = 1.0f / ray.direction.x;
    const real min_x_plane_intersection_time = (aabb.min.x - ray.origin.x) * inverse_x;
    const real max_x_plane_intersection_time = (aabb.max.x - ray.origin.x) * inverse_x;

    const real min_x_intersection_time = std::min(min_x_plane_intersection_time, max_x_plane_intersection_time);
    const real max_x_intersection_time = std::max(min_x_plane_intersection_time, max_x_plane_intersection_time);

    const real inverse_y = 1.0f / ray.direction.y;
    const real min_y_plane_intersection_time = (aabb.min.y - ray.origin.y) * inverse_y;
    const real max_y_plane_intersection_time = (aabb.max.y - ray.origin.y) * inverse_y;

    const real min_y_intersection_time = std::min(min_y_plane_intersection_time, max_y_plane_intersection_time);
    const real max_y_intersection_time = std::max(min_y_plane_intersection_time, max_y_plane_intersection_time);

    const real inverse_z = 1.0f / ray.direction.z;
    const real min_z_plane_intersection_time = (aabb.min.z - ray.origin.z) * inverse_z;
    const real max_z_plane_intersection_time = (aabb.max.z - ray.origin.z) * inverse_z;

    const real min_z_intersection_time = std::min(min_z_plane_intersection_time, max_z_plane_intersection_time);
    const real max_z_intersection_time = std::max(min_z_plane_intersection_time, max_z_plane_intersection_time);

    AABBIntersections result = {};
    result.min_distance = std::max(min_z_intersection_time, std::max(min_y_intersection_time, min_x_intersection_time));
    result.max_distance = std::min(max_z_intersection_time, std::min(max_y_intersection_time, max_x_intersection_time));

    return result;
}

static AABB& operator+=(AABB& lhs, const AABB& rhs) {
    lhs.min.x = std::min(lhs.min.x, rhs.min.x);
    lhs.min.y = std::min(lhs.min.y, rhs.min.y);
    lhs.min.z = std::min(lhs.min.z, rhs.min.z);

    lhs.max.x = std::max(lhs.max.x, rhs.max.x);
    lhs.max.y = std::max(lhs.max.y, rhs.max.y);
    lhs.max.z = std::max(lhs.max.z, rhs.max.z);

    return lhs;
}

static AABB construct_aabb(const Sphere& sphere) {
    AABB aabb = {};
    aabb.min = sphere.centre - Vec3{sphere.radius, sphere.radius, sphere.radius};
    aabb.max = sphere.centre + Vec3{sphere.radius, sphere.radius, sphere.radius};

    return aabb;
}

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
    result.value.min_distance = intersections_mid_point_distance - intersections_mid_point_to_intersections_distance;
    result.value.max_distance = intersections_mid_point_distance + intersections_mid_point_to_intersections_distance;
    result.is_valid = true;

    return result;
}
