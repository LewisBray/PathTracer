#ifndef COLOUR_H
#define COLOUR_H

#include "types.h"

struct Colour {
    real r;
    real g;
    real b;
};

static Colour operator+(const Colour& lhs, const Colour& rhs);
static Colour operator*(real scalar, const Colour& colour);
static Colour operator*(const Colour& lhs, const Colour& rhs);
static Colour& operator+=(Colour& lhs, const Colour& rhs);
static Colour& operator*=(Colour& lhs, const Colour& rhs);
static Colour& operator/=(Colour& lhs, const real scalar);

#endif
