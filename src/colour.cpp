#include "colour.h"

static Colour operator+(const Colour& lhs, const Colour& rhs) {
    return Colour{lhs.r + rhs.r, lhs.g + rhs.g, lhs.b + rhs.b};
}

static Colour operator*(const real scalar, const Colour& colour) {
    return Colour{scalar * colour.r, scalar * colour.g, scalar * colour.b};
}

static Colour operator*(const Colour& lhs, const Colour& rhs) {
    return Colour{lhs.r * rhs.r, lhs.g * rhs.g, lhs.b * rhs.b};
}

static Colour& operator+=(Colour& lhs, const Colour& rhs) {
    lhs.r += rhs.r;
    lhs.g += rhs.g;
    lhs.b += rhs.b;

    return lhs;
}

static Colour& operator*=(Colour& lhs, const Colour& rhs) {
    lhs.r *= rhs.r;
    lhs.g *= rhs.g;
    lhs.b *= rhs.b;

    return lhs;
}

static Colour& operator/=(Colour& lhs, const real scalar) {
    lhs.r /= scalar;
    lhs.g /= scalar;
    lhs.b /= scalar;

    return lhs;
}
