#include "bvh.h"

#include <algorithm>
#include <cassert>

using AABBLessThan = bool(*)(const std::vector<AABB>&, int, int);

static bool aabb_less_than_x(const std::vector<AABB>& aabbs, const int lhs_index, const int rhs_index) {
    return aabbs[lhs_index].min.x < aabbs[rhs_index].min.x;
}

static bool aabb_less_than_y(const std::vector<AABB>& aabbs, const int lhs_index, const int rhs_index) {
    return aabbs[lhs_index].min.y < aabbs[rhs_index].min.y;
}

static bool aabb_less_than_z(const std::vector<AABB>& aabbs, const int lhs_index, const int rhs_index) {
    return aabbs[lhs_index].min.z < aabbs[rhs_index].min.z;
}

static int add_node(BVH& bvh, const int axis, const std::vector<AABB>& aabbs, std::vector<int>& indices, const int start_index, const int end_index) {
    bvh.push_back(Node{});
    const int node_index = bvh.size() - 1;

    const int count = end_index - start_index;
    if (count == 1) {
        Node& node = bvh[node_index];
        const int aabb_index = indices[start_index];
        node.aabb = aabbs[aabb_index];
        node.index = aabb_index;
        return node_index;
    }

    const AABBLessThan aabb_less_than = (axis == 0) ? aabb_less_than_x : ((axis == 1) ? aabb_less_than_y : aabb_less_than_z);
    std::sort(
        indices.begin() + start_index,
        indices.begin() + end_index,
        [&aabbs, aabb_less_than](const int lhs_index, const int rhs_index) { return aabb_less_than(aabbs, lhs_index, rhs_index); }
    );

    const int next_axis = (axis + 1) % 3;
    const int mid_index = start_index + (count / 2);
    bvh[node_index].left = add_node(bvh, next_axis, aabbs, indices, start_index, mid_index);
    bvh[node_index].right = add_node(bvh, next_axis, aabbs, indices, mid_index, end_index);

    Node& node = bvh[node_index];
    node.aabb = bvh[node.left].aabb;
    node.aabb += bvh[node.right].aabb;

    const int prev_axis = (axis + 2) % 3;
    const AABBLessThan prev_aabb_less_than = (prev_axis == 0) ? aabb_less_than_x : ((prev_axis == 1) ? aabb_less_than_y : aabb_less_than_z);
    std::sort(
        indices.begin() + start_index,
        indices.begin() + end_index,
        [&aabbs, prev_aabb_less_than](const int lhs_index, const int rhs_index) { return prev_aabb_less_than(aabbs, lhs_index, rhs_index); }
    );

    return node_index;
}

static BVH construct_sphere_bvh(const Sphere* const spheres, const int count) {
    std::vector<AABB> sphere_aabbs;
    sphere_aabbs.resize(count);
    std::vector<int> sphere_indices;
    sphere_indices.resize(count);
    for (int sphere_index = 0; sphere_index < count; ++sphere_index) {
        sphere_aabbs[sphere_index] = construct_aabb(spheres[sphere_index]);
        sphere_indices[sphere_index] = sphere_index;
    }

    const std::size_t leaf_count = count;
    const std::size_t max_node_count = 4 * leaf_count - 1;

    BVH bvh;
    bvh.reserve(max_node_count);
    const int first_node_index = add_node(bvh, 0, sphere_aabbs, sphere_indices, 0, count);
    assert(first_node_index == 0);

    return bvh;
}

static BVH construct_triangle_bvh(const Triangle* const triangles, const int count) {
    std::vector<AABB> triangle_aabbs;
    triangle_aabbs.resize(count);
    std::vector<int> triangle_indices;
    triangle_indices.resize(count);
    for (int triangle_index = 0; triangle_index < count; ++triangle_index) {
        triangle_aabbs[triangle_index] = construct_aabb(triangles[triangle_index]);
        triangle_indices[triangle_index] = triangle_index;
    }

    const std::size_t leaf_count = count;
    const std::size_t max_node_count = 4 * leaf_count - 1;

    BVH bvh;
    bvh.reserve(max_node_count);
    const int first_node_index = add_node(bvh, 0, triangle_aabbs, triangle_indices, 0, count);
    assert(first_node_index == 0);

    return bvh;
}
