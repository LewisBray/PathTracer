#include <cassert>
#include <cfloat>
#include <cmath>

using u32 = unsigned int;
using u64 = unsigned long long;

using real = double;
static constexpr real REAL_MAX = DBL_MAX;

static u32 noise_1d(const int x, const u32 seed) {
    static constexpr u32 BIT_NOISE_1 = 0xB5297A4D;
    static constexpr u32 BIT_NOISE_2 = 0x68E31DA4;
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

static Vec3 operator^(const Vec3& lhs, const Vec3& rhs) {
    return Vec3{
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x
    };
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

    static constexpr real NUDGE_FACTOR = 0.001f;

    switch (material.type) {
        case Material::Type::LAMBERTIAN: {
            const Vec3 random = random_unit_vector(point_unit_normal);
            Maybe<Ray> scattered_ray = {};
            scattered_ray.value.origin = point + NUDGE_FACTOR * point_unit_normal;
            scattered_ray.value.direction = normalise(point_unit_normal + random);
            scattered_ray.valid = true;
            
            return scattered_ray;
        }

        case Material::Type::METAL: {
            Maybe<Ray> scattered_ray = {};
            const Vec3 random = random_unit_vector(point_unit_normal);
            const real fuzziness = material.metal.fuzziness;
            scattered_ray.value.origin = point + NUDGE_FACTOR * point_unit_normal;
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
                scattered_ray.value.origin = point + NUDGE_FACTOR * unit_normal;
                scattered_ray.value.direction = reflect(ray.direction, unit_normal);
            } else {
                scattered_ray.value.origin = point - NUDGE_FACTOR * unit_normal;
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

static u32 random_number(u32 seed) {
    seed ^= (seed << 13);
    seed ^= (seed >> 17);
    seed ^= (seed << 5);

    return seed;
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

void write_pixel_data_to_file(unsigned char* const pixels, const u32 pixel_byte_count) {
    const HANDLE file_handle = CreateFileA(
        "pixels.bin",
        GENERIC_WRITE,
        0,
        nullptr,
        CREATE_ALWAYS,
        FILE_ATTRIBUTE_NORMAL,
        NULL
    );

    assert(file_handle != INVALID_HANDLE_VALUE);

    // swap r and b around for dumping as binary file
    for (u32 byte_index = 0; byte_index < pixel_byte_count; byte_index += 4) {
        const unsigned char temp = pixels[byte_index + 0];
        pixels[byte_index + 0] = pixels[byte_index + 2];
        pixels[byte_index + 2] = temp;
    }

    DWORD bytes_written = 0;
    const BOOL wrote_to_file = WriteFile(file_handle, pixels, pixel_byte_count, &bytes_written, nullptr);
    assert(wrote_to_file != FALSE);
    assert(bytes_written == pixel_byte_count);

    const BOOL closed_handle = CloseHandle(file_handle);
    assert(closed_handle != FALSE);
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

    static constexpr real ASPECT_RATIO = 3.0f / 2.0f;
    static constexpr int CLIENT_WIDTH = 1200;
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

    static constexpr u32 PIXEL_BYTE_COUNT = 4 * CLIENT_WIDTH * CLIENT_HEIGHT;
    unsigned char* pixels = static_cast<unsigned char*>(VirtualAlloc(0, PIXEL_BYTE_COUNT, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE));
    assert(pixels != nullptr);

    static constexpr real FOV_Y_DEGREES = 20.0f;
    static constexpr real FOCAL_LENGTH = 1.0f;

    static constexpr real PI = 3.14159265358979323846264f;

    const real fov_y = FOV_Y_DEGREES / 180.0f * PI;
    const real viewport_height = 2.0f * std::tan(0.5f * fov_y);
    const real viewport_width = ASPECT_RATIO * viewport_height;

    static constexpr Vec3 camera_position{13.0f, 2.0f, 3.0f};
    static constexpr Vec3 camera_target{0.0f, 0.0f, 0.0f};

    const Vec3 camera_z = normalise(camera_position - camera_target);
    const Vec3 camera_x = normalise(Vec3{0.0f, 1.0f, 0.0f} ^ camera_z);
    const Vec3 camera_y = camera_z ^ camera_x;

    static constexpr real APERTURE = 0.1f;
    static constexpr real LENS_RADIUS = 0.5f * APERTURE;

    const real focus_distance = 10.0f;
    const Vec3 step_x = focus_distance * viewport_width * camera_x;
    const Vec3 step_y = focus_distance * viewport_height * camera_y;

    const Vec3 bottom_left = camera_position - 0.5f * step_x - 0.5f * step_y - focus_distance * camera_z;

    static constexpr int SPHERE_COUNT = 22 * 22 + 4;
    Material materials[SPHERE_COUNT] = {};
    Sphere spheres[SPHERE_COUNT] = {};
    int sphere_material_indices[SPHERE_COUNT] = {};

    int sphere_index = 0;
    materials[sphere_index] = construct_lambertian_material(Colour{0.5f, 0.5f, 0.5f});
    spheres[sphere_index] = Sphere{Vec3{0.0f, -1000.0f, 0.0f}, 1000.0f};
    sphere_material_indices[sphere_index] = sphere_index;
    ++sphere_index;

    u32 rand_seed = 479001599;
    for (int a = -11; a < 11; ++a) {
        for (int b = -11; b < 11; ++b) {
            rand_seed = random_number(rand_seed);
            const real material_choice = real_from_rng_seed(rand_seed);

            rand_seed = random_number(rand_seed);
            const real x_offset = 0.9f * real_from_rng_seed(rand_seed);

            rand_seed = random_number(rand_seed);
            const real z_offset = 0.9f * real_from_rng_seed(rand_seed);

            const Vec3 sphere_centre{static_cast<real>(a) + x_offset, 0.2f, static_cast<real>(b) + z_offset};
            spheres[sphere_index] = Sphere{sphere_centre, 0.2f};

            if (material_choice < 0.8f) {
                rand_seed = random_number(rand_seed);
                const real colour_1_r = real_from_rng_seed(rand_seed);
                rand_seed = random_number(rand_seed);
                const real colour_1_g = real_from_rng_seed(rand_seed);
                rand_seed = random_number(rand_seed);
                const real colour_1_b = real_from_rng_seed(rand_seed);

                const Colour colour_1{colour_1_r, colour_1_g, colour_1_b};

                rand_seed = random_number(rand_seed);
                const real colour_2_r = real_from_rng_seed(rand_seed);
                rand_seed = random_number(rand_seed);
                const real colour_2_g = real_from_rng_seed(rand_seed);
                rand_seed = random_number(rand_seed);
                const real colour_2_b = real_from_rng_seed(rand_seed);

                const Colour colour_2{colour_2_r, colour_2_g, colour_2_b};

                const Colour albedo = colour_1 * colour_2;
                materials[sphere_index] = construct_lambertian_material(albedo);
            } else if (material_choice < 0.95f) {
                rand_seed = random_number(rand_seed);
                const real colour_r = 0.5f * real_from_rng_seed(rand_seed) + 0.5f;
                rand_seed = random_number(rand_seed);
                const real colour_g = 0.5f * real_from_rng_seed(rand_seed) + 0.5f;
                rand_seed = random_number(rand_seed);
                const real colour_b = 0.5f * real_from_rng_seed(rand_seed) + 0.5f;                

                const Colour albedo{colour_r, colour_g, colour_b};

                rand_seed = random_number(rand_seed);
                const real fuzziness = real_from_rng_seed(rand_seed);                

                materials[sphere_index] = construct_metal_material(albedo, fuzziness);
            } else {
                materials[sphere_index] = construct_dielectric_material(1.5f);
            }

            sphere_material_indices[sphere_index] = sphere_index;
            ++sphere_index;
        }
    }

    assert(sphere_index == SPHERE_COUNT - 3);

    materials[sphere_index] = construct_dielectric_material(1.5f);
    spheres[sphere_index] = Sphere{Vec3{0.0f, 1.0f, 0.0f}, 1.0f};
    sphere_material_indices[sphere_index] = sphere_index;
    ++sphere_index;

    materials[sphere_index] = construct_lambertian_material(Colour{0.4f, 0.2f, 0.1f});
    spheres[sphere_index] = Sphere{Vec3{-4.0f, 1.0f, 0.0f}, 1.0f};
    sphere_material_indices[sphere_index] = sphere_index;
    ++sphere_index;

    materials[sphere_index] = construct_metal_material(Colour{0.7f, 0.6f, 0.5f}, 0.0f);
    spheres[sphere_index] = Sphere{Vec3{4.0f, 1.0f, 0.0f}, 1.0f};
    sphere_material_indices[sphere_index] = sphere_index;
    ++sphere_index;

    assert(sphere_index == SPHERE_COUNT);

    const Scene scene{
        materials,
        spheres,
        sphere_material_indices,
        SPHERE_COUNT
    };

    static constexpr int SAMPLES_PER_PIXEL = 500;

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

                const u32 a_rng_seed = noise_3d(column, row, sample_index + 0, RNG_SEED);
                const real a_randomness = real_from_rng_seed(a_rng_seed);
                const real a = APERTURE * a_randomness - LENS_RADIUS;

                const real b_max = std::sqrt(LENS_RADIUS * LENS_RADIUS - a * a);
                const real b_min = -b_max;

                const real b_rng_seed = noise_3d(column, row, sample_index + 1, RNG_SEED);
                const real b_randomness = real_from_rng_seed(b_rng_seed);
                const real b = (b_max - b_min) * b_randomness + b_min;

                const Vec3 random_offset = a * camera_x + b * camera_y;

                const Vec3 ray_direction = normalise(Vec3{bottom_left + u * step_x + v * step_y - camera_position - random_offset});
                const Ray ray{camera_position + random_offset, ray_direction};

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
            pixels[index + 3] = 255;
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
