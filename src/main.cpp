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

static constexpr u32 RNG_SEED = 0;

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

struct Material {
    Colour albedo;
};

struct Scene {
    const Material* materials;
    
    const Sphere* spheres;
    const int* sphere_material_indices;
    int sphere_count;
};

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
            const Vec3 sphere_unit_normal = normalise(intersection_point - sphere.centre);
            ray.origin = intersection_point + 0.001 * sphere_unit_normal;
            const Vec3 random = random_unit_vector(ray.origin);
            ray.direction = normalise(sphere_unit_normal + random);

            const int sphere_material_index = scene.sphere_material_indices[sphere_index];
            const Material& sphere_material = scene.materials[sphere_material_index];
            attenuation *= sphere_material.albedo;
        } else {
            colour += attenuation * background_gradient(ray);
            break;
        }
    }

    return colour;
}

#include <Windows.h>

LRESULT CALLBACK main_window_proc(const HWND window, const UINT message, const WPARAM w_param, const LPARAM l_param) {
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

    static constexpr Material MATERIALS[2] = {
        Material{Colour{1.0f, 0.0f, 0.0f}},
        Material{Colour{0.0f, 1.0f, 0.0f}}
    };

    static constexpr int SPHERE_COUNT = 2;
    static constexpr Sphere SPHERES[SPHERE_COUNT] = {
        Sphere{Vec3{0.0f, 0.0f, -1.0f}, 0.5f},
        Sphere{Vec3{0.0f, -100.5f, -1.0f}, 100.0f}
    };

    static constexpr int SPHERE_MATERIAL_INDICES[SPHERE_COUNT] = {0, 1};

    const Scene scene{
        MATERIALS,
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
