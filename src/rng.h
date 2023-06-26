#ifndef RNG_H
#define RNG_H

#include "types.h"

// noise
static u32 noise_1d(int x);
static u32 noise_3d(int x, int y, int z);
static real real_from_rng(u32 rng);
static Vec3 random_unit_vector(const Vec3& seed);

// pseudo
static u32 random_number(u32 seed);

#endif
