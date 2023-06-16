#include <cassert>
#include <cfloat>
#include <cmath>

struct Vec3 {
    float x;
    float y;
    float z;
};

Vec3 operator+(const Vec3& lhs, const Vec3& rhs) {
    return Vec3{lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

Vec3 operator-(const Vec3& lhs, const Vec3& rhs) {
    return Vec3{lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

Vec3 operator*(const float scalar, const Vec3& v) {
    return Vec3{scalar * v.x, scalar * v.y, scalar * v.z};
}

float operator*(const Vec3& lhs, const Vec3& rhs) {
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

float magnitude(const Vec3& v) {
    return std::sqrt(v * v);
}

Vec3 normalise(const Vec3& v) {
    const float inverse_magnitude = 1.0f / magnitude(v);
    return inverse_magnitude * v;
}

struct Ray {
    Vec3 origin;
    Vec3 direction;
};

struct Sphere {
    Vec3 centre;
    float radius;
};

template <typename T>
struct Maybe {
    T value;
    bool valid;
};

struct SphereIntersections {
    float distance_1;
    float distance_2;
};

static Maybe<SphereIntersections> intersect(const Ray& ray, const Sphere& sphere) noexcept {
    assert(std::abs(magnitude(ray.direction) - 1.0f) < 1.0e-6f);

    Maybe<SphereIntersections> result = {};

    const Vec3 ray_origin_to_sphere_centre = sphere.centre - ray.origin;
    const float intersections_mid_point_distance = ray_origin_to_sphere_centre * ray.direction;

    const float ray_start_to_sphere_centre_distance_squared = ray_origin_to_sphere_centre * ray_origin_to_sphere_centre;
    const float intersections_mid_point_distance_squared = intersections_mid_point_distance * intersections_mid_point_distance;
    const float sphere_centre_to_intersections_mid_point_squared = ray_start_to_sphere_centre_distance_squared - intersections_mid_point_distance_squared;
    const float sphere_radius_squared = sphere.radius * sphere.radius;
    const float intersections_mid_point_to_intersections_distance_squared = sphere_radius_squared - sphere_centre_to_intersections_mid_point_squared;
    if (intersections_mid_point_to_intersections_distance_squared < 0.0f) {
        return result;
    }

    const float intersections_mid_point_to_intersections_distance = std::sqrt(intersections_mid_point_to_intersections_distance_squared);
    result.value.distance_1 = intersections_mid_point_distance - intersections_mid_point_to_intersections_distance;
    result.value.distance_2 = intersections_mid_point_distance + intersections_mid_point_to_intersections_distance;
    result.valid = true;

    return result;
}

struct Colour {
    float r;
    float g;
    float b;
};

Colour operator+(const Colour& lhs, const Colour& rhs) {
    return Colour{lhs.r + rhs.r, lhs.g + rhs.g, lhs.b + rhs.b};
}

Colour operator*(const float scalar, const Colour& colour) {
    return Colour{scalar * colour.r, scalar * colour.g, scalar * colour.b};
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

Colour background_gradient(const Ray& ray) {
    const float t = 0.5f * (ray.direction.y + 1.0f);
    return (1.0f - t) * Colour{1.0, 1.0, 1.0} + t * Colour{0.5, 0.7, 1.0};
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

    static constexpr float ASPECT_RATIO = 16.0f / 9.0f;
    static constexpr int CLIENT_WIDTH = 400;
    static constexpr int CLIENT_HEIGHT = static_cast<int>(static_cast<float>(CLIENT_WIDTH) / ASPECT_RATIO);

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

    unsigned char* pixels = static_cast<unsigned char*>(VirtualAlloc(0, CLIENT_WIDTH * CLIENT_HEIGHT * sizeof(unsigned int), MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE));
    assert(pixels != nullptr);

    static constexpr float VIEWPORT_HEIGHT = 2.0f;
    static constexpr float VIEWPORT_WIDTH = ASPECT_RATIO * VIEWPORT_HEIGHT;
    static constexpr float FOCAL_LENGTH = 1.0f;

    static constexpr Vec3 BOTTOM_LEFT{-0.5f * VIEWPORT_WIDTH, -0.5f * VIEWPORT_HEIGHT, -FOCAL_LENGTH};

    static constexpr int SPHERE_COUNT = 2;
    static constexpr Sphere SPHERES[SPHERE_COUNT] = {
        Sphere{Vec3{0.0f, 0.0f, -1.0f}, 0.5f},
        Sphere{Vec3{0.0f, -100.5f, -1.0f}, 100.0f}
    };

    for (int row = 0; row < CLIENT_HEIGHT; ++row) {
        const float v = static_cast<float>(row) / static_cast<float>(CLIENT_HEIGHT - 1);
        for (int column = 0; column < CLIENT_WIDTH; ++column) {
            const float u = static_cast<float>(column) / static_cast<float>(CLIENT_WIDTH - 1);
            const Ray ray{Vec3{0.0f, 0.0f, 0.0f}, normalise(Vec3{BOTTOM_LEFT + u * Vec3{VIEWPORT_WIDTH, 0.0f, 0.0f} + v * Vec3{0.0f, VIEWPORT_HEIGHT, 0.0f}})};

            int closest_intersecting_sphere_index = -1;
            float closest_intersecting_sphere_distance = FLT_MAX;
            for (int sphere_index = 0; sphere_index < SPHERE_COUNT; ++sphere_index) {
                const Maybe<SphereIntersections> intersection_result = intersect(ray, SPHERES[sphere_index]);
                if (intersection_result.valid) {
                    if (0.0f < intersection_result.value.distance_1 && intersection_result.value.distance_1 < closest_intersecting_sphere_distance) {
                        closest_intersecting_sphere_index = sphere_index;
                        closest_intersecting_sphere_distance = intersection_result.value.distance_1;
                    } else if (0.0f < intersection_result.value.distance_2 && intersection_result.value.distance_2 < closest_intersecting_sphere_distance) {
                        closest_intersecting_sphere_index = sphere_index;
                        closest_intersecting_sphere_distance = intersection_result.value.distance_2;
                    }
                }
            }

            Colour colour = {};
            if (closest_intersecting_sphere_index != -1) {
                const Sphere& sphere = SPHERES[closest_intersecting_sphere_index];
                const Vec3 intersection_point = ray.origin + closest_intersecting_sphere_distance * ray.direction;
                const Vec3 sphere_unit_normal = normalise(intersection_point - sphere.centre);
                colour = Colour{0.5f * (sphere_unit_normal.x + 1.0f), 0.5f * (sphere_unit_normal.y + 1.0f), 0.5f * (sphere_unit_normal.z + 1.0f)};
            } else {
                colour = background_gradient(ray);
            }

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
