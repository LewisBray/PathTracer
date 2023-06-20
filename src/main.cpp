#include <cassert>
#include <cfloat>
#include <cmath>

using u32 = unsigned int;

using real = double;
static constexpr real REAL_MAX = DBL_MAX;

static u32 noise_1d(const int x, const u32 seed) {
    static constexpr u32 BIT_NOISE_1 = 0x68E31DA4;
    static constexpr u32 BIT_NOISE_2 = 0xB5297A4D;
    static constexpr u32 BIT_NOISE_3 = 0x1B56C4E9;

    u32 mangled = static_cast<u32>(x);
    mangled *= BIT_NOISE_1;
    mangled += seed;
    mangled ^= (mangled >> 8);
    mangled += BIT_NOISE_2;
    mangled ^= (mangled << 8);
    mangled *= BIT_NOISE_3;
    mangled ^= (mangled >> 8);

    return mangled;
}

static u32 noise_3d(const int x, const int y, const int z, const u32 seed) {
    static constexpr int PRIME_1 = 198491317;
    static constexpr int PRIME_2 = 6542989;
    const int new_x = x + PRIME_1 * y + PRIME_2 * z;
    const u32 result = noise_1d(new_x, seed);
    return result;
}

static real real_from_rng_seed(const u32 rng_seed) {
    return static_cast<real>(rng_seed) / static_cast<real>(UINT_MAX);
}

struct Vec3 {
    real x;
    real y;
    real z;
};

static Vec3 operator+(const Vec3& lhs, const Vec3& rhs) {
    return Vec3{lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

static Vec3 operator-(const Vec3& v) {
    return Vec3{-v.x, -v.y, -v.z};
}

static Vec3 operator-(const Vec3& lhs, const Vec3& rhs) {
    return Vec3{lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

static Vec3 operator*(const real scalar, const Vec3& v) {
    return Vec3{scalar * v.x, scalar * v.y, scalar * v.z};
}

static real operator*(const Vec3& lhs, const Vec3& rhs) {
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

static real magnitude(const Vec3& v) {
    return std::sqrt(v * v);
}

static Vec3 normalise(const Vec3& v) {
    const real inverse_magnitude = 1.0f / magnitude(v);
    return inverse_magnitude * v;
}

static constexpr u32 RNG_SEED = 1;

static Vec3 random_unit_vector(const Vec3 seed) {
    const int x = static_cast<int>(1000000000.0f * seed.x);
    const int y = static_cast<int>(1000000000.0f * seed.y);
    const int z = static_cast<int>(1000000000.0f * seed.z);

    const u32 x_noise = noise_1d(x, RNG_SEED);
    const u32 y_noise = noise_1d(y, RNG_SEED);
    const u32 z_noise = noise_1d(z, RNG_SEED);

    const real random_x = real_from_rng_seed(x_noise) - 0.5f;
    const real random_y = real_from_rng_seed(y_noise) - 0.5f;
    const real random_z = real_from_rng_seed(z_noise) - 0.5f;
    const Vec3 random_vector = Vec3{random_x, random_y, random_z};
    return normalise(random_vector);
}

struct Ray {
    Vec3 origin;
    Vec3 direction;
};

struct Sphere {
    Vec3 centre;
    real radius;
};

template <typename T>
struct Maybe {
    T value;
    bool valid;
};

struct SphereIntersections {
    real distance_1;
    real distance_2;
};

static Maybe<SphereIntersections> intersect(const Ray& ray, const Sphere& sphere) noexcept {
    assert(std::abs(magnitude(ray.direction) - 1.0f) < 1.0e-6f);

    Maybe<SphereIntersections> result = {};

    const Vec3 ray_origin_to_sphere_centre = sphere.centre - ray.origin;
    const real intersections_mid_point_distance = ray_origin_to_sphere_centre * ray.direction;

    const real ray_start_to_sphere_centre_distance_squared = ray_origin_to_sphere_centre * ray_origin_to_sphere_centre;
    const real intersections_mid_point_distance_squared = intersections_mid_point_distance * intersections_mid_point_distance;
    const real sphere_centre_to_intersections_mid_point_squared = ray_start_to_sphere_centre_distance_squared - intersections_mid_point_distance_squared;
    const real sphere_radius_squared = sphere.radius * sphere.radius;
    const real intersections_mid_point_to_intersections_distance_squared = sphere_radius_squared - sphere_centre_to_intersections_mid_point_squared;
    if (intersections_mid_point_to_intersections_distance_squared < 0.0f) {
        return result;
    }

    const real intersections_mid_point_to_intersections_distance = std::sqrt(intersections_mid_point_to_intersections_distance_squared);
    result.value.distance_1 = intersections_mid_point_distance - intersections_mid_point_to_intersections_distance;
    result.value.distance_2 = intersections_mid_point_distance + intersections_mid_point_to_intersections_distance;
    result.valid = true;

    return result;
}

struct ClosestSphereIntersection {
    int index;
    real distance;
};

static Maybe<ClosestSphereIntersection> intersect(const Ray& ray, const Sphere* const spheres, const int count) {
    Maybe<ClosestSphereIntersection> closest_intersection = {};
    closest_intersection.valid = false;
    closest_intersection.value.index = -1;
    closest_intersection.value.distance = REAL_MAX;
    for (int sphere_index = 0; sphere_index < count; ++sphere_index) {
        const Maybe<SphereIntersections> intersection_result = intersect(ray, spheres[sphere_index]);
        if (intersection_result.valid) {
            if (0.0f < intersection_result.value.distance_1 && intersection_result.value.distance_1 < closest_intersection.value.distance) {
                closest_intersection.valid = true;
                closest_intersection.value.index = sphere_index;
                closest_intersection.value.distance = intersection_result.value.distance_1;
            } else if (0.0f < intersection_result.value.distance_2 && intersection_result.value.distance_2 < closest_intersection.value.distance) {
                closest_intersection.valid = true;
                closest_intersection.value.index = sphere_index;
                closest_intersection.value.distance = intersection_result.value.distance_2;
            }
        }
    }

    return closest_intersection;
}

struct Colour {
    real r;
    real g;
    real b;
};

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

static Colour background_gradient(const Ray& ray) {
    const real t = 0.5f * (ray.direction.y + 1.0f);
    return (1.0f - t) * Colour{1.0, 1.0, 1.0} + t * Colour{0.5, 0.7, 1.0};
}

struct Lambertian {
    Colour albedo;
};

struct Metal {
    Colour albedo;
    real fuzziness;
};

struct Dielectric {
    real refraction_index;
};

struct Material {
    enum Type {
        LAMBERTIAN = 0,
        METAL = 1,
        DIELECTRIC = 2
    };

    union {
        Lambertian lambertian;
        Metal metal;
        Dielectric dielectric;
    };

    Type type;
};

static Material construct_lambertian_material(const Colour& albedo) {
    Material material = {};
    material.lambertian.albedo = albedo;
    material.type = Material::Type::LAMBERTIAN;

    return material;
}

static Material construct_metal_material(const Colour& albedo, const real fuzziness) {
    Material material = {};
    material.metal.albedo = albedo;
    material.metal.fuzziness = fuzziness;
    material.type = Material::Type::METAL;

    return material;
}

static Material construct_dielectric_material(const real refraction_index) {
    Material material = {};
    material.dielectric.refraction_index = refraction_index;
    material.type = Material::Type::DIELECTRIC;

    return material;
}

static Colour get_colour(const Material& material) {
    switch (material.type) {
        case Material::Type::LAMBERTIAN: {
            return material.lambertian.albedo;
        }

        case Material::Type::METAL: {
            return material.metal.albedo;
        }

        case Material::Type::DIELECTRIC: {
            return Colour{1.0f, 1.0f, 1.0f};
        }

        default: {
            assert(false);
            return Colour{0.0f, 0.0f, 0.0f};
        }
    }
}

struct Scene {
    const Material* materials;
    
    const Sphere* spheres;
    const int* sphere_material_indices;
    int sphere_count;
};

static Vec3 reflect(const Vec3& direction, const Vec3& unit_normal) {
    assert(std::abs(magnitude(direction) - 1.0f) < 1.0e-6);
    assert(std::abs(magnitude(unit_normal) - 1.0f) < 1.0e-6);

    return direction - 2.0f * (direction * unit_normal) * unit_normal;
}

static Vec3 refract(const Vec3& direction, const Vec3& unit_normal, const real refraction_ratio) {
    assert(std::abs(magnitude(direction) - 1.0f) < 1.0e-6);
    assert(std::abs(magnitude(unit_normal) - 1.0f) < 1.0e-6);

    const real cos_theta = -direction * unit_normal;
    assert(cos_theta >= -1.0f);
    assert(cos_theta <= 1.0f);
    const Vec3 refracted_direction_perpendicular = refraction_ratio * (direction + cos_theta * unit_normal);
    const Vec3 refracted_direction_parallel = -std::sqrt(std::abs(1.0f - refracted_direction_perpendicular * refracted_direction_perpendicular)) * unit_normal;

    return normalise(refracted_direction_perpendicular + refracted_direction_parallel);
}

// Uses Schlick's approximation of reflectance
static double reflectance(const real cos_theta, const real refraction_ratio) {
    const real ratio = (1.0f - refraction_ratio) / (1.0f + refraction_ratio);
    const real r0 = ratio * ratio;
    const real difference = (1.0f - cos_theta);

    return r0 + (1.0f - r0) * difference * difference * difference * difference * difference;
}

static Maybe<Ray> scatter(const Ray& ray, const Vec3& point, const Vec3& point_unit_normal, const Material& material) {
    assert(std::abs(magnitude(ray.direction) - 1.0f) < 1.0e-6);
    assert(std::abs(magnitude(point_unit_normal) - 1.0f) < 1.0e-6);

    switch (material.type) {
        case Material::Type::LAMBERTIAN: {
            const Vec3 random = random_unit_vector(point);
            Maybe<Ray> scattered_ray = {};
            scattered_ray.value.origin = point + 0.001 * point_unit_normal;
            scattered_ray.value.direction = normalise(point_unit_normal + random);
            scattered_ray.valid = true;
            
            return scattered_ray;
        }

        case Material::Type::METAL: {
            Maybe<Ray> scattered_ray = {};
            const Vec3 random = random_unit_vector(point);
            const real fuzziness = material.metal.fuzziness;
            scattered_ray.value.origin = point + 0.001 * point_unit_normal;
            scattered_ray.value.direction = normalise(reflect(ray.direction, point_unit_normal) + fuzziness * random);
            scattered_ray.valid = (scattered_ray.value.direction * point_unit_normal > 0.0f);
            
            return scattered_ray;
        }

        case Material::Type::DIELECTRIC: {
            Maybe<Ray> scattered_ray = {};
            const bool front_face = (ray.direction * point_unit_normal < 0.0f);
            const real refraction_ratio = front_face ? 1.0f / material.dielectric.refraction_index : material.dielectric.refraction_index;
            const Vec3 unit_normal = front_face ? point_unit_normal : -point_unit_normal;

            const real cos_theta = -ray.direction * unit_normal;
            const real sin_theta = std::sqrt(1.0f - cos_theta * cos_theta);
            const u32 rng_seed = noise_1d(static_cast<u32>(1000000000.0f * sin_theta), RNG_SEED);
            if (refraction_ratio * sin_theta > 1.0f || reflectance(cos_theta, refraction_ratio) > real_from_rng_seed(rng_seed)) {
                scattered_ray.value.origin = point + 0.001 * unit_normal;
                scattered_ray.value.direction = reflect(ray.direction, unit_normal);
            } else {
                scattered_ray.value.origin = point - 0.001 * unit_normal;
                scattered_ray.value.direction = refract(ray.direction, unit_normal, refraction_ratio);
            }

            scattered_ray.valid = true;

            return scattered_ray;
        }

        default: {
            assert(false);
            return Maybe<Ray>{};
        }
    }
}

static Colour intersect(Ray ray, const Scene& scene) {
    static constexpr int MAX_BOUNCE_COUNT = 50;

    Colour colour = {0.0f, 0.0f, 0.0f};
    Colour attenuation = {1.0f, 1.0f, 1.0f};
    for (int bounce_index = 0; bounce_index < MAX_BOUNCE_COUNT; ++bounce_index) {
        const Maybe<ClosestSphereIntersection> closest_sphere_intersection = intersect(ray, scene.spheres, scene.sphere_count);
        if (closest_sphere_intersection.valid) {
            const int sphere_index = closest_sphere_intersection.value.index;
            const Sphere& sphere = scene.spheres[sphere_index];
            const Vec3 intersection_point = ray.origin + closest_sphere_intersection.value.distance * ray.direction;
            const real sign = (sphere.radius < 0.0f) ? -1.0f : 1.0f;    // trick to model hollow spheres, don't want this polluting the scatter routine
            const Vec3 sphere_unit_normal = sign * normalise(intersection_point - sphere.centre);

            const int sphere_material_index = scene.sphere_material_indices[sphere_index];
            const Material& sphere_material = scene.materials[sphere_material_index];

            const Maybe<Ray> scattered_ray = scatter(ray, intersection_point, sphere_unit_normal, sphere_material);
            if (scattered_ray.valid) {
                ray = scattered_ray.value;

                const Colour& sphere_material_colour = get_colour(sphere_material);
                attenuation *= sphere_material_colour;
            } else {
                colour = Colour{0.0f, 0.0f, 0.0f};
                break;
            }
        } else {
            colour += attenuation * background_gradient(ray);
            break;
        }
    }

    return colour;
}

#include <Windows.h>

static LRESULT CALLBACK main_window_proc(const HWND window, const UINT message, const WPARAM w_param, const LPARAM l_param) {
    assert(window != NULL);

    switch (message) {
        case WM_CLOSE: {
            PostQuitMessage(0);
            return 0;
        }

        default: {
            return DefWindowProcW(window, message, w_param, l_param);
        }
    }
}

int WINAPI wWinMain(const HINSTANCE instance, HINSTANCE, PWSTR, int) {
    static constexpr const wchar_t* WINDOW_CLASS_NAME = L"Main Window";

    const HCURSOR cursor = LoadCursorA(NULL, IDC_ARROW);

    WNDCLASSW window_class = {};
    window_class.style = CS_OWNDC;
    window_class.lpfnWndProc = main_window_proc;
    window_class.cbClsExtra = 0;
    window_class.cbWndExtra = 0;
    window_class.hInstance = instance;
    window_class.hIcon = NULL;
    window_class.hCursor = cursor;
    window_class.hbrBackground = NULL;
    window_class.lpszMenuName = nullptr;
    window_class.lpszClassName = WINDOW_CLASS_NAME;

    const ATOM atom = RegisterClassW(&window_class);
    assert(atom != 0);

    static constexpr real ASPECT_RATIO = 16.0f / 9.0f;
    static constexpr int CLIENT_WIDTH = 400;
    static constexpr int CLIENT_HEIGHT = static_cast<int>(static_cast<real>(CLIENT_WIDTH) / ASPECT_RATIO);

    static constexpr DWORD WINDOW_STYLE = WS_CAPTION | WS_SYSMENU;

    RECT window_rect = {};
    window_rect.left = 100;
    window_rect.top = 100;
    window_rect.right = window_rect.left + CLIENT_WIDTH;
    window_rect.bottom = window_rect.top + CLIENT_HEIGHT;

    const BOOL found_window_dimensions = AdjustWindowRect(&window_rect, WINDOW_STYLE, FALSE);
    assert(found_window_dimensions != FALSE);

    const HWND window = CreateWindowExW(
        0,
        WINDOW_CLASS_NAME,
        L"PBR Ray Tracer",
        WINDOW_STYLE,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        window_rect.right - window_rect.left,
        window_rect.bottom - window_rect.top,
        NULL,
        NULL,
        instance,
        nullptr
    );

    assert(window != NULL);

    const HDC window_device_context = GetDC(window);
    assert(window_device_context != NULL);

    const BOOL window_was_visible = ShowWindow(window, SW_SHOW);
    assert(window_was_visible == FALSE);

    BITMAPINFO bitmap_info = {};
    bitmap_info.bmiHeader.biSize = sizeof(bitmap_info.bmiHeader);
    bitmap_info.bmiHeader.biWidth = CLIENT_WIDTH;
    bitmap_info.bmiHeader.biHeight = CLIENT_HEIGHT;
    bitmap_info.bmiHeader.biPlanes = 1;
    bitmap_info.bmiHeader.biBitCount = 32;
    bitmap_info.bmiHeader.biCompression = BI_RGB;
    bitmap_info.bmiHeader.biSizeImage = 0;
    bitmap_info.bmiHeader.biXPelsPerMeter = 0;
    bitmap_info.bmiHeader.biYPelsPerMeter = 0;
    bitmap_info.bmiHeader.biClrUsed = 0;
    bitmap_info.bmiHeader.biClrImportant = 0;

    unsigned char* pixels = static_cast<unsigned char*>(VirtualAlloc(0, 4 * CLIENT_WIDTH * CLIENT_HEIGHT, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE));
    assert(pixels != nullptr);

    static constexpr real VIEWPORT_HEIGHT = 2.0f;
    static constexpr real VIEWPORT_WIDTH = ASPECT_RATIO * VIEWPORT_HEIGHT;
    static constexpr real FOCAL_LENGTH = 1.0f;

    static constexpr Vec3 BOTTOM_LEFT{-0.5f * VIEWPORT_WIDTH, -0.5f * VIEWPORT_HEIGHT, -FOCAL_LENGTH};

    const Material materials[4] = {
        construct_lambertian_material(Colour{0.8f, 0.8f, 0.0f}),    // ground
        construct_lambertian_material(Colour{0.1f, 0.2f, 0.5f}),    // middle
        construct_dielectric_material(1.5f),                        // left
        construct_metal_material(Colour{0.8f, 0.6f, 0.2f}, 0.0f)    // right
    };

    static constexpr int SPHERE_COUNT = 5;
    static constexpr Sphere SPHERES[SPHERE_COUNT] = {
        Sphere{Vec3{0.0f, -100.5f, -1.0f}, 100.0f}, // ground
        Sphere{Vec3{0.0f, 0.0f, -1.0f}, 0.5f},      // middle
        Sphere{Vec3{-1.0f, 0.0f, -1.0f}, 0.5f},     // left
        Sphere{Vec3{-1.0f, 0.0f, -1.0f}, -0.4f},    // left
        Sphere{Vec3{1.0f, 0.0f, -1.0f}, 0.5f}       // right
    };

    static constexpr int SPHERE_MATERIAL_INDICES[SPHERE_COUNT] = {0, 1, 2, 2, 3};

    const Scene scene{
        materials,
        SPHERES,
        SPHERE_MATERIAL_INDICES,
        SPHERE_COUNT
    };

    static constexpr int SAMPLES_PER_PIXEL = 100;

    for (int row = 0; row < CLIENT_HEIGHT; ++row) {
        for (int column = 0; column < CLIENT_WIDTH; ++column) {
            Colour colour = {};
            for (int sample_index = 0; sample_index < SAMPLES_PER_PIXEL; ++sample_index) {
                const u32 u_rng_seed = noise_3d(row, column, sample_index + 0, RNG_SEED);
                const real u_randomness = real_from_rng_seed(u_rng_seed);
                const real u = (static_cast<real>(column) + u_randomness) / static_cast<real>(CLIENT_WIDTH - 1);

                const u32 v_rng_seed = noise_3d(row, column, sample_index + 1, RNG_SEED);
                const real v_randomness = real_from_rng_seed(v_rng_seed);
                const real v = (static_cast<real>(row) + v_randomness) / static_cast<real>(CLIENT_HEIGHT - 1);

                const Vec3 ray_origin{0.0f, 0.0f, 0.0f};
                const Vec3 ray_direction = normalise(Vec3{BOTTOM_LEFT + u * Vec3{VIEWPORT_WIDTH, 0.0f, 0.0f} + v * Vec3{0.0f, VIEWPORT_HEIGHT, 0.0f}});
                const Ray ray{ray_origin, ray_direction};

                colour += intersect(ray, scene);
            }

            colour /= static_cast<real>(SAMPLES_PER_PIXEL);

            // gamma correct
            colour.r = std::sqrt(colour.r);
            colour.g = std::sqrt(colour.g);
            colour.b = std::sqrt(colour.b);

            const int index = 4 * (row * CLIENT_WIDTH + column);
            pixels[index + 0] = static_cast<unsigned char>(255.0f * colour.b);
            pixels[index + 1] = static_cast<unsigned char>(255.0f * colour.g);
            pixels[index + 2] = static_cast<unsigned char>(255.0f * colour.r);
            pixels[index + 3] = 0;
        }
    }

    bool quit = false;
    while (!quit) {
        const int scanlines_copied = StretchDIBits(
          window_device_context,
          0,
          0,
          CLIENT_WIDTH,
          CLIENT_HEIGHT,
          0,
          0,
          CLIENT_WIDTH,
          CLIENT_HEIGHT,
          pixels,
          &bitmap_info,
          DIB_RGB_COLORS,
          SRCCOPY
        );

        assert(scanlines_copied == CLIENT_HEIGHT);

        MSG window_message = {};
        while (PeekMessageW(&window_message, NULL, 0, 0, PM_REMOVE) != FALSE) {
            TranslateMessage(&window_message);
            DispatchMessageW(&window_message);

            if (window_message.message == WM_QUIT) {
                quit = true;
            }
        }
    }

    return 0;
}
