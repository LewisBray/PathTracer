#ifndef TYPES_H
#define TYPES_H

#include <cfloat>

using u32 = unsigned int;
using u64 = unsigned long long;

using real = double;
static constexpr real REAL_MAX = DBL_MAX;

// TODO: maybe a #define for single vs. double precision? can define this elsewhere then
#define strtor strtod

#endif
