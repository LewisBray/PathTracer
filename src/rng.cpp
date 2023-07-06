#include "rng.h"

static constexpr u32 NOISE_SEED = 1;

static u32 noise_1d(const int x) {
    static constexpr u32 BIT_NOISE_1 = 0xB5297A4D;
    static constexpr u32 BIT_NOISE_2 = 0x68E31DA4;
    static constexpr u32 BIT_NOISE_3 = 0x1B56C4E9;

    u32 mangled = static_cast<u32>(x);
    mangled *= BIT_NOISE_1;
    mangled += NOISE_SEED;
    mangled ^= (mangled >> 8);
    mangled += BIT_NOISE_2;
    mangled ^= (mangled << 8);
    mangled *= BIT_NOISE_3;
    mangled ^= (mangled >> 8);

    return mangled;
}

static u32 noise_3d(const int x, const int y, const int z) {
    static constexpr int PRIME_1 = 198491317;
    static constexpr int PRIME_2 = 6542989;
    const int new_x = x + PRIME_1 * y + PRIME_2 * z;
    const u32 result = noise_1d(new_x);
    return result;
}

static real real_from_rng(const u32 rng) {
    return static_cast<real>(rng) / static_cast<real>(UINT_MAX);
}

static Vec3 random_unit_vector(const Vec3& seed) {
    const int x = static_cast<int>(1000000000.0f * seed.x);
    const int y = static_cast<int>(1000000000.0f * seed.y);
    const int z = static_cast<int>(1000000000.0f * seed.z);

    u32 rng = noise_3d(x, y, z);

    rng = random_number(rng);
    const real x_randomness = real_from_rng(rng);
    const real random_x = 2.0f * x_randomness - 1.0f;

    const real y_max = std::sqrt(1.0f - random_x * random_x);
    const real y_min = -y_max;

    rng = random_number(rng);
    const real y_randomness = real_from_rng(rng);
    const real random_y = (y_max - y_min) * y_randomness + y_min;

    const real z_max = std::sqrt(1.0f - random_x * random_x - random_y * random_y);
    const real z_min = -z_max;

    rng = random_number(rng);
    const real z_randomness = real_from_rng(rng);
    const real random_z = (z_max - z_min) * z_randomness + z_min;

    const Vec3 random_vector = Vec3{random_x, random_y, random_z};
    return normalise(random_vector);
}

static u32 random_number(u32 seed) {
    seed ^= (seed << 13);
    seed ^= (seed >> 17);
    seed ^= (seed << 5);

    return seed;
}
