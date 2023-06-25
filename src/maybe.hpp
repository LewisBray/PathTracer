#ifndef MAYBE_HPP
#define MAYBE_HPP

template <typename T>
struct Maybe {
    T value;
    bool is_valid;
};

#endif
