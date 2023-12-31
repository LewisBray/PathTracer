#include "linear_algebra.h"
#include "model_loading.h"
#include "path_tracing.h"
#include "geometry.h"
#include "material.h"
#include "colour.h"
#include "types.h"
#include "bvh.h"
#include "rng.h"

#include "linear_algebra.cpp"
#include "model_loading.cpp"
#include "path_tracing.cpp"
#include "geometry.cpp"
#include "material.cpp"
#include "colour.cpp"
#include "bvh.cpp"
#include "rng.cpp"

#include <algorithm>
#include <cassert>
#include <cfloat>

#define NOMINMAX
#include <Windows.h>

static constexpr real PI = 3.14159265358979323846264f;
static real degrees_to_radians(const real degrees) {
    return degrees / 180.0f * PI;
}

// app settings
static constexpr real SPHERES_ASPECT_RATIO = 3.0f / 2.0f;
static constexpr real CORNELL_ASPECT_RATIO = 1.0f;
static constexpr real MODEL_ASPECT_RATIO = 1.0f;

static constexpr real ASPECT_RATIO = MODEL_ASPECT_RATIO;
static constexpr int CLIENT_WIDTH = 600;
static constexpr int CLIENT_HEIGHT = static_cast<int>(static_cast<real>(CLIENT_WIDTH) / ASPECT_RATIO);

struct RenderWorkQueue {
    struct Entry {
        int row;
        int sample;
        Scene scene;
        real aperture;
        Vec3 camera_position;
        Vec3 camera_x;
        Vec3 camera_y;
        Vec3 bottom_left;
        Vec3 step_x;
        Vec3 step_y;
        real* pixels;
    };

    static constexpr int CAPACITY = CLIENT_HEIGHT;
    Entry entries[CAPACITY];
    volatile LONG entry_count;
    volatile LONG next_entry_to_do_index;
    volatile LONG completed_entry_count;
    HANDLE semaphore;
};

static void push_entry(RenderWorkQueue& work_queue, const RenderWorkQueue::Entry& entry) {
    assert(work_queue.entry_count < work_queue.CAPACITY);

    work_queue.entries[work_queue.entry_count] = entry;

    _mm_sfence();
    ++work_queue.entry_count;
    ReleaseSemaphore(work_queue.semaphore, 1, nullptr);
}

static void render_scanline(
    const int row,
    const int sample,
    const Scene& scene,
    const real aperture,
    const Vec3& camera_position,
    const Vec3& camera_x,
    const Vec3& camera_y,
    const Vec3& bottom_left,
    const Vec3& step_x,
    const Vec3& step_y,
    real* const pixels
) {
    const real lens_radius = 0.5f * aperture;
    for (int column = 0; column < CLIENT_WIDTH; ++column) {
        u32 rng = noise_3d(row, column, sample);
        
        rng = random_number(rng);
        const real u_randomness = real_from_rng(rng);
        const real u = (static_cast<real>(column) + u_randomness) / static_cast<real>(CLIENT_WIDTH - 1);

        rng = random_number(rng);
        const real v_randomness = real_from_rng(rng);
        const real v = (static_cast<real>(row) + v_randomness) / static_cast<real>(CLIENT_HEIGHT - 1);

        rng = random_number(rng);
        const real a_randomness = real_from_rng(rng);
        const real a = aperture * a_randomness - lens_radius;

        const real b_max = std::sqrt(lens_radius * lens_radius - a * a);
        const real b_min = -b_max;

        rng = random_number(rng);
        const real b_randomness = real_from_rng(rng);
        const real b = (b_max - b_min) * b_randomness + b_min;

        const Vec3 random_offset = a * camera_x + b * camera_y;

        const Vec3 ray_direction = normalise(Vec3{bottom_left + u * step_x + v * step_y - camera_position - random_offset});
        const Ray ray{camera_position + random_offset, ray_direction};

        const Colour colour = intersect(ray, scene);

        const int index = 4 * (row * CLIENT_WIDTH + column);
        pixels[index + 0] += colour.b;
        pixels[index + 1] += colour.g;
        pixels[index + 2] += colour.r;
        pixels[index + 3] += 1.0f;
    }
}

static bool process_work_queue_entry(RenderWorkQueue& work_queue) {
    bool had_entry_to_process = false;

    const LONG original_entry_to_do_index = work_queue.next_entry_to_do_index;
    if (original_entry_to_do_index < work_queue.entry_count) {
        const LONG entry_to_do_index = InterlockedCompareExchange(&work_queue.next_entry_to_do_index, original_entry_to_do_index + 1, original_entry_to_do_index);
        if (entry_to_do_index == original_entry_to_do_index) {
            const RenderWorkQueue::Entry& entry_to_do = work_queue.entries[entry_to_do_index];
            render_scanline(
                entry_to_do.row,
                entry_to_do.sample,
                entry_to_do.scene,
                entry_to_do.aperture,
                entry_to_do.camera_position,
                entry_to_do.camera_x,
                entry_to_do.camera_y,
                entry_to_do.bottom_left,
                entry_to_do.step_x,
                entry_to_do.step_y,
                entry_to_do.pixels
            );

            InterlockedIncrement(&work_queue.completed_entry_count);
            ReleaseSemaphore(work_queue.semaphore, 1, nullptr);
            had_entry_to_process = true;
        }
    }

    return had_entry_to_process;
}

static bool work_in_progress(const RenderWorkQueue& work_queue) {
    return (work_queue.completed_entry_count != work_queue.entry_count);
}

static void reset(RenderWorkQueue& work_queue) {
    work_queue.entry_count = 0;
    work_queue.next_entry_to_do_index = 0;
    work_queue.completed_entry_count = 0;
}

static DWORD thread_proc(const LPVOID parameter) {
    RenderWorkQueue* const work_queue = static_cast<RenderWorkQueue*>(parameter);

    while (true) {
        const bool had_entry_to_process = process_work_queue_entry(*work_queue);
        if (!had_entry_to_process) {
            WaitForSingleObject(work_queue->semaphore, INFINITE);
        }
    }

    return 0;
}

static void write_pixel_data_to_file(unsigned char* const pixels, const u32 pixel_byte_count) {
    const HANDLE file_handle = CreateFileA(
        "pixels.data",
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

    for (u32 byte_index = 0; byte_index < pixel_byte_count; byte_index += 4) {
        const unsigned char temp = pixels[byte_index + 0];
        pixels[byte_index + 0] = pixels[byte_index + 2];
        pixels[byte_index + 2] = temp;
    }

    const BOOL closed_handle = CloseHandle(file_handle);
    assert(closed_handle != FALSE);
}

struct KeyboardInput {
    bool a;
    bool d;
    bool e;
    bool q;
    bool s;
    bool w;
    bool ctrl;
    bool up;
    bool down;
    bool left;
    bool right;
};

struct MouseInput {
    int x;
    int y;
    int scroll;
    bool left;
};

struct ApplicationState {
    KeyboardInput keyboard_input;
    MouseInput mouse_input;
};

static ApplicationState& get_application_state(const HWND window) {
    const LONG_PTR user_data = GetWindowLongPtrA(window, GWLP_USERDATA);
    ApplicationState* const application_state = reinterpret_cast<ApplicationState*>(user_data);
    assert(application_state != nullptr);
    return *application_state;
}

static LRESULT CALLBACK main_window_proc(const HWND window, const UINT message, const WPARAM w_param, const LPARAM l_param) {
    assert(window != NULL);

    switch (message) {
        case WM_CREATE: {
            assert(l_param != 0);
            CREATESTRUCTA* const create_struct = reinterpret_cast<CREATESTRUCTA*>(l_param);
            assert(create_struct->lpCreateParams != nullptr);   // window should be created with CreateWindowEx
            SetWindowLongPtrA(window, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(create_struct->lpCreateParams));

            return 0;
        }

        case WM_CLOSE: {
            PostQuitMessage(0);
            return 0;
        }

        case WM_KEYDOWN: {
            ApplicationState& application_state = get_application_state(window);
            KeyboardInput& keyboard_input = application_state.keyboard_input;
            const char vk_key = static_cast<char>(w_param);
            switch (vk_key) {
                case 'A': {
                    keyboard_input.a = true;
                    return 0;
                }

                case 'D': {
                    keyboard_input.d = true;
                    return 0;
                }

                case 'E': {
                    keyboard_input.e = true;
                    return 0;
                }

                case 'Q': {
                    keyboard_input.q = true;
                    return 0;
                }

                case 'S': {
                    keyboard_input.s = true;
                    return 0;
                }

                case 'W': {
                    keyboard_input.w = true;
                    return 0;
                }

                case VK_CONTROL: {
                    keyboard_input.ctrl = true;
                    return 0;
                }

                case VK_UP: {
                    keyboard_input.up = true;
                    return 0;
                }

                case VK_DOWN: {
                    keyboard_input.down = true;
                    return 0;
                }

                case VK_LEFT: {
                    keyboard_input.left = true;
                    return 0;
                }

                case VK_RIGHT: {
                    keyboard_input.right = true;
                    return 0;
                }

                default: {
                    return DefWindowProcA(window, message, w_param, l_param);
                }
            }
        }

        case WM_KEYUP: {
            ApplicationState& application_state = get_application_state(window);
            KeyboardInput& keyboard_input = application_state.keyboard_input;
            const char vk_key = static_cast<char>(w_param);
            switch (vk_key) {
                case 'A': {
                    keyboard_input.a = false;
                    return 0;
                }

                case 'D': {
                    keyboard_input.d = false;
                    return 0;
                }

                case 'E': {
                    keyboard_input.e = false;
                    return 0;
                }

                case 'Q': {
                    keyboard_input.q = false;
                    return 0;
                }

                case 'S': {
                    keyboard_input.s = false;
                    return 0;
                }

                case 'W': {
                    keyboard_input.w = false;
                    return 0;
                }

                case VK_CONTROL: {
                    keyboard_input.ctrl = false;
                    return 0;
                }

                case VK_UP: {
                    keyboard_input.up = false;
                    return 0;
                }

                case VK_DOWN: {
                    keyboard_input.down = false;
                    return 0;
                }

                case VK_LEFT: {
                    keyboard_input.left = false;
                    return 0;
                }

                case VK_RIGHT: {
                    keyboard_input.right = false;
                    return 0;
                }

                default: {
                    return DefWindowProcA(window, message, w_param, l_param);
                }
            }
        }

        case WM_LBUTTONDOWN: {
            ApplicationState& application_state = get_application_state(window);
            MouseInput& mouse_input = application_state.mouse_input;

            mouse_input.left = true;
            const HWND capturing_window = SetCapture(window);
            assert(capturing_window == NULL);

            return 0;
        }

        case WM_MOUSEMOVE: {
            ApplicationState& application_state = get_application_state(window);
            MouseInput& mouse_input = application_state.mouse_input;

            mouse_input.x = l_param & 0xFFFF;
            mouse_input.y = (l_param & 0xFFFF0000) >> 16;

            return 0;
        }

        case WM_LBUTTONUP: {
            ApplicationState& application_state = get_application_state(window);
            MouseInput& mouse_input = application_state.mouse_input;

            mouse_input.left = false;
            const BOOL capture_released = ReleaseCapture();
            assert(capture_released != FALSE);

            return 0;
        }

        case WM_MOUSEWHEEL: {
            ApplicationState& application_state = get_application_state(window);
            MouseInput& mouse_input = application_state.mouse_input;

            const short high_word = (w_param & 0xFFFF0000) >> 16;
            mouse_input.scroll = high_word / WHEEL_DELTA;

            return 0;
        }

        default: {
            return DefWindowProcW(window, message, w_param, l_param);
        }
    }
}

HWND create_window(const HINSTANCE instance, const int client_width, const int client_height, ApplicationState& application_state) {
    static constexpr const wchar_t* WINDOW_CLASS_NAME = L"Main Window";

    const HCURSOR arrow_cursor = LoadCursorA(NULL, IDC_ARROW);

    WNDCLASSW window_class = {};
    window_class.style = CS_OWNDC;
    window_class.lpfnWndProc = main_window_proc;
    window_class.cbClsExtra = 0;
    window_class.cbWndExtra = 0;
    window_class.hInstance = instance;
    window_class.hIcon = NULL;
    window_class.hCursor = arrow_cursor;
    window_class.hbrBackground = NULL;
    window_class.lpszMenuName = nullptr;
    window_class.lpszClassName = WINDOW_CLASS_NAME;

    const ATOM atom = RegisterClassW(&window_class);
    assert(atom != 0);

    static constexpr DWORD WINDOW_STYLE = WS_CAPTION | WS_SYSMENU;

    RECT window_rect = {};
    window_rect.left = 100;
    window_rect.top = 100;
    window_rect.right = window_rect.left + client_width;
    window_rect.bottom = window_rect.top + client_height;

    const BOOL found_window_dimensions = AdjustWindowRect(&window_rect, WINDOW_STYLE, FALSE);
    assert(found_window_dimensions != FALSE);

    const HWND window = CreateWindowExW(
        0,
        WINDOW_CLASS_NAME,
        L"Path Tracer",
        WINDOW_STYLE,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        window_rect.right - window_rect.left,
        window_rect.bottom - window_rect.top,
        NULL,
        NULL,
        instance,
        &application_state
    );

    assert(window != NULL);
    return window;
}

int WINAPI wWinMain(const HINSTANCE instance, HINSTANCE, PWSTR, int) {
    LARGE_INTEGER tick_frequency = {};
    const BOOL queried_performance_frequency = QueryPerformanceFrequency(&tick_frequency);
    assert(queried_performance_frequency != FALSE);

    SYSTEM_INFO system_info = {};
    GetSystemInfo(&system_info);

    const DWORD core_count = system_info.dwNumberOfProcessors;
    assert(core_count > 0);
    const DWORD thread_count = core_count - 1;

    RenderWorkQueue work_queue = {};
    work_queue.semaphore = CreateSemaphoreA(nullptr, 0, thread_count, nullptr);

    for (DWORD thread_index = 0; thread_index < thread_count; ++thread_index) {
        DWORD thread_id = 0;
        const HANDLE thread_handle = CreateThread(nullptr, 0, thread_proc, static_cast<LPVOID>(&work_queue), 0, &thread_id);
        CloseHandle(thread_handle);
    }

    static constexpr int SPHERE_COUNT = 22 * 22 + 4;
    Material materials[SPHERE_COUNT] = {};
    Sphere spheres[SPHERE_COUNT] = {};
    int sphere_material_indices[SPHERE_COUNT] = {};

    int sphere_index = 0;
    materials[sphere_index] = construct_lambertian_material(Colour{0.5f, 0.5f, 0.5f});
    spheres[sphere_index] = Sphere{Vec3{0.0f, -1000.0f, 0.0f}, 1000.0f};
    sphere_material_indices[sphere_index] = sphere_index;
    ++sphere_index;

    u32 rng = 479001599;
    for (int a = -11; a < 11; ++a) {
        for (int b = -11; b < 11; ++b) {
            rng = random_number(rng);
            const real material_choice = real_from_rng(rng);

            rng = random_number(rng);
            const real x_offset = 0.9f * real_from_rng(rng);

            rng = random_number(rng);
            const real z_offset = 0.9f * real_from_rng(rng);

            const Vec3 sphere_centre{static_cast<real>(a) + x_offset, 0.2f, static_cast<real>(b) + z_offset};
            spheres[sphere_index] = Sphere{sphere_centre, 0.2f};

            if (material_choice < 0.8f) {
                rng = random_number(rng);
                const real colour_1_r = real_from_rng(rng);
                rng = random_number(rng);
                const real colour_1_g = real_from_rng(rng);
                rng = random_number(rng);
                const real colour_1_b = real_from_rng(rng);

                const Colour colour_1{colour_1_r, colour_1_g, colour_1_b};

                rng = random_number(rng);
                const real colour_2_r = real_from_rng(rng);
                rng = random_number(rng);
                const real colour_2_g = real_from_rng(rng);
                rng = random_number(rng);
                const real colour_2_b = real_from_rng(rng);

                const Colour colour_2{colour_2_r, colour_2_g, colour_2_b};

                const Colour albedo = colour_1 * colour_2;
                materials[sphere_index] = construct_lambertian_material(albedo);
            } else if (material_choice < 0.95f) {
                rng = random_number(rng);
                const real colour_r = 0.5f * real_from_rng(rng) + 0.5f;
                rng = random_number(rng);
                const real colour_g = 0.5f * real_from_rng(rng) + 0.5f;
                rng = random_number(rng);
                const real colour_b = 0.5f * real_from_rng(rng) + 0.5f;                

                const Colour albedo{colour_r, colour_g, colour_b};

                rng = random_number(rng);
                const real fuzziness = real_from_rng(rng);                

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

    const BVH sphere_bvh = construct_sphere_bvh(spheres, SPHERE_COUNT);
    const Scene random_spheres{
        materials,
        spheres,
        &sphere_bvh,
        sphere_material_indices,
        nullptr,
        nullptr,
        nullptr,
        Colour{1.0f, 1.0f, 1.0f},
        Colour{0.5f, 0.7f, 1.0f}
    };

    const Vec3 sphere_camera_start_position{13.0f, 2.0f, 3.0f};
    Camera sphere_camera = {};
    sphere_camera.target = Vec3{0.0f, 0.0f, 0.0f};
    sphere_camera.orientation = look_at_matrix(sphere_camera_start_position, sphere_camera.target);
    sphere_camera.distance = magnitude(sphere_camera_start_position - sphere_camera.target);
    sphere_camera.fov_y = degrees_to_radians(20.0f);
    sphere_camera.aperture = 0.1f;
    sphere_camera.focus_distance = 10.0f;

    const Material cornell_materials[] = {
        construct_lambertian_material(Colour{0.65f, 0.05f, 0.05f}), // red
        construct_lambertian_material(Colour{0.73f, 0.73f, 0.73f}), // white
        construct_lambertian_material(Colour{0.12f, 0.45f, 0.15f}), // green
        construct_diffuse_light_material(Colour{1.0f, 1.0f, 1.0f}, 15.0f),  // light
        construct_dielectric_material(1.5f)
    };

    const Sphere cornell_spheres[1] = {
        Sphere{Vec3{183.0f, 240.0f, 169.0f}, 75.0f}
    };

    const int cornell_sphere_material_indices[1] = {4};

    const BVH cornell_sphere_bvh = construct_sphere_bvh(cornell_spheres, 1);

    const Vec3 unit_box_vertices[] = {
        // +z
        Vec3{0.0f, 1.0f, 1.0f}, Vec3{0.0f, 0.0f, 1.0f}, Vec3{1.0f, 0.0f, 1.0f},
        Vec3{1.0f, 0.0f, 1.0f}, Vec3{1.0f, 1.0f, 1.0f}, Vec3{0.0f, 1.0f, 1.0f},

        // +x
        Vec3{1.0f, 1.0f, 1.0f}, Vec3{1.0f, 0.0f, 1.0f}, Vec3{1.0f, 0.0f, 0.0f},
        Vec3{1.0f, 0.0f, 0.0f}, Vec3{1.0f, 1.0f, 0.0f}, Vec3{1.0f, 1.0f, 1.0f},

        // -z
        Vec3{1.0f, 1.0f, 0.0f}, Vec3{1.0f, 0.0f, 0.0f}, Vec3{0.0f, 0.0f, 0.0f},
        Vec3{0.0f, 0.0f, 0.0f}, Vec3{0.0f, 1.0f, 0.0f}, Vec3{1.0f, 1.0f, 0.0f},

        // -x
        Vec3{0.0f, 1.0f, 0.0f}, Vec3{0.0f, 0.0f, 0.0f}, Vec3{0.0f, 0.0f, 1.0f},
        Vec3{0.0f, 0.0f, 1.0f}, Vec3{0.0f, 1.0f, 1.0f}, Vec3{0.0f, 1.0f, 0.0f},

        // +y
        Vec3{1.0f, 1.0f, 1.0f}, Vec3{1.0f, 1.0f, 0.0f}, Vec3{0.0f, 1.0f, 0.0f},
        Vec3{0.0f, 1.0f, 0.0f}, Vec3{0.0f, 1.0f, 1.0f}, Vec3{1.0f, 1.0f, 1.0f},

        // -y
        Vec3{0.0f, 0.0f, 1.0f}, Vec3{0.0f, 0.0f, 0.0f}, Vec3{1.0f, 0.0f, 0.0f},
        Vec3{1.0f, 0.0f, 0.0f}, Vec3{1.0f, 0.0f, 1.0f}, Vec3{0.0f, 0.0f, 1.0f}
    };

    const Mat3 right_box_transform = scaling_matrix(165.0f, 165.0f, 165.0f) * rotation_matrix(-PI / 10.0f, 0.0f, 1.0f, 0.0f);
    Vec3 right_box_vertices[36] = {};
    for (int i = 0; i < 36; ++i) {
        right_box_vertices[i] = right_box_transform * unit_box_vertices[i] + Vec3{130.0f, 0.0f, 65.0f};
    }

    Vec3 location = right_box_transform * Vec3{0.5f, 1.0f, 0.5f} + Vec3{130.0f, 0.0f, 65.0f};

    assert(sizeof(unit_box_vertices) == sizeof(right_box_vertices));

    const Mat3 left_box_transform = scaling_matrix(165.0f, 330.0f, 165.0f) * rotation_matrix(PI / 12.0f, 0.0f, 1.0f, 0.0f);
    Vec3 left_box_vertices[36] = {};
    for (int i = 0; i < 36; ++i) {
        left_box_vertices[i] = left_box_transform * unit_box_vertices[i] + Vec3{265.0f, 0.0f, 295.0f};
    }

    assert(sizeof(unit_box_vertices) == sizeof(left_box_vertices));

    const Triangle cornell_triangles[36] = {
        // left wall
        Triangle{Vec3{555.0f, 0.0f, 0.0f}, Vec3{555.0f, 0.0f, 555.0f}, Vec3{555.0f, 555.0f, 555.0f}},
        Triangle{Vec3{555.0f, 555.0f, 555.0f}, Vec3{555.0f, 555.0f, 0.0f}, Vec3{555.0f, 0.0f, 0.0f}},

        // right wall
        Triangle{Vec3{0.0f, 0.0f, 0.0f}, Vec3{0.0f, 555.0f, 0.0f}, Vec3{0.0f, 555.0f, 555.0f}},
        Triangle{Vec3{0.0f, 555.0f, 555.0f}, Vec3{0.0f, 0.0f, 555.0f}, Vec3{0.0f, 0.0f, 0.0f}},

        // floor
        Triangle{Vec3{0.0f, 0.0f, 0.0f}, Vec3{0.0f, 0.0f, 555.0f}, Vec3{555.0f, 0.0f, 555.0f}},
        Triangle{Vec3{555.0f, 0.0f, 555.0f}, Vec3{555.0f, 0.0f, 0.0f}, Vec3{0.0f, 0.0f, 0.0f}},

        // ceiling
        Triangle{Vec3{0.0f, 555.0f, 0.0f}, Vec3{555.0f, 555.0f, 0.0f}, Vec3{555.0f, 555.0f, 555.0f}},
        Triangle{Vec3{555.0f, 555.0f, 555.0f}, Vec3{0.0f, 555.0f, 555.0f}, Vec3{0.0f, 555.0f, 0.0f}},

        // back
        Triangle{Vec3{0.0f, 0.0f, 555.0f}, Vec3{0.0f, 555.0f, 555.0f}, Vec3{555.0f, 555.0f, 555.0f}},
        Triangle{Vec3{555.0f, 555.0f, 555.0f}, Vec3{555.0f, 0.0f, 555.0f}, Vec3{0.0f, 0.0f, 555.0f}},

        // light
        Triangle{Vec3{213.0f, 554.0f, 227.0f}, Vec3{343.0f, 554.0f, 227.0f}, Vec3{343.0f, 554.0f, 332.0f}},
        Triangle{Vec3{343.0f, 554.0f, 332.0f}, Vec3{213.0f, 554.0f, 332.0f}, Vec3{213.0f, 554.0f, 227.0f}},

        // right box
        Triangle{right_box_vertices[0], right_box_vertices[1], right_box_vertices[2]},
        Triangle{right_box_vertices[3], right_box_vertices[4], right_box_vertices[5]},

        Triangle{right_box_vertices[6], right_box_vertices[7], right_box_vertices[8]},
        Triangle{right_box_vertices[9], right_box_vertices[10], right_box_vertices[11]},

        Triangle{right_box_vertices[12], right_box_vertices[13], right_box_vertices[14]},
        Triangle{right_box_vertices[15], right_box_vertices[16], right_box_vertices[17]},

        Triangle{right_box_vertices[18], right_box_vertices[19], right_box_vertices[20]},
        Triangle{right_box_vertices[21], right_box_vertices[22], right_box_vertices[23]},

        Triangle{right_box_vertices[24], right_box_vertices[25], right_box_vertices[26]},
        Triangle{right_box_vertices[27], right_box_vertices[28], right_box_vertices[29]},

        Triangle{right_box_vertices[30], right_box_vertices[31], right_box_vertices[32]},
        Triangle{right_box_vertices[33], right_box_vertices[34], right_box_vertices[35]},

        // left box
        Triangle{left_box_vertices[0], left_box_vertices[1], left_box_vertices[2]},
        Triangle{left_box_vertices[3], left_box_vertices[4], left_box_vertices[5]},

        Triangle{left_box_vertices[6], left_box_vertices[7], left_box_vertices[8]},
        Triangle{left_box_vertices[9], left_box_vertices[10], left_box_vertices[11]},

        Triangle{left_box_vertices[12], left_box_vertices[13], left_box_vertices[14]},
        Triangle{left_box_vertices[15], left_box_vertices[16], left_box_vertices[17]},

        Triangle{left_box_vertices[18], left_box_vertices[19], left_box_vertices[20]},
        Triangle{left_box_vertices[21], left_box_vertices[22], left_box_vertices[23]},

        Triangle{left_box_vertices[24], left_box_vertices[25], left_box_vertices[26]},
        Triangle{left_box_vertices[27], left_box_vertices[28], left_box_vertices[29]},

        Triangle{left_box_vertices[30], left_box_vertices[31], left_box_vertices[32]},
        Triangle{left_box_vertices[33], left_box_vertices[34], left_box_vertices[35]}
    };

    const int cornell_triangle_material_indices[36] = {
        // left wall
        2, 2,

        // right wall
        0, 0,

        // floor
        1, 1,

        // ceiling
        1, 1,

        // back
        1, 1,

        // light
        3, 3,

        // right box
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,

        // left box
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
    };

    const BVH cornell_triangle_bvh = construct_triangle_bvh(cornell_triangles, 36);

    const Scene cornell_box{
        cornell_materials,
        cornell_spheres,
        &cornell_sphere_bvh,
        cornell_sphere_material_indices,
        cornell_triangles,
        &cornell_triangle_bvh,
        cornell_triangle_material_indices,
        Colour{0.0f, 0.0f, 0.0f},
        Colour{0.0f, 0.0f, 0.0f}
    };

    const Vec3 cornell_camera_start_position{278.0f, 278.0f, -800.0f};
    Camera cornell_camera = {};
    cornell_camera.target = Vec3{278.0f, 278.0f, 0.0f};
    cornell_camera.orientation = look_at_matrix(cornell_camera_start_position, cornell_camera.target);
    cornell_camera.distance = magnitude(cornell_camera_start_position - cornell_camera.target);
    cornell_camera.fov_y = degrees_to_radians(40.0f);
    cornell_camera.aperture = 0.1f;
    cornell_camera.focus_distance = cornell_camera.distance;

    const Material model_materials[2] = {
        construct_lambertian_material(Colour{6.0f / 255.0f, 4.0f / 255.0f, 3.0f / 255.0f}),
        construct_diffuse_light_material(Colour{1.0f, 1.0f, 1.0f}, 10.0f)
    };

    std::vector<Triangle> model_triangles = load_triangles_file(".\\models\\rook.triangles");
    const std::vector<int> model_triangle_material_indices(model_triangles.size(), 0);

    const Mat3 model_transform = rotation_matrix(-PI / 2.0f, 1.0f, 0.0f, 0.0f);
    for (Triangle& triangle : model_triangles) {
        triangle.a = model_transform * triangle.a;
        triangle.b = model_transform * triangle.b;
        triangle.c = model_transform * triangle.c;
    }

    const BVH model_triangle_bvh = construct_triangle_bvh(model_triangles.data(), model_triangles.size());

    const Sphere model_light{Vec3{20.0f, 80.0f, 10.0f}, 20.0f};
    const int model_light_material_index = 1;
    const BVH model_light_bvh = construct_sphere_bvh(&model_light, 1);

    const Scene model{
        model_materials,
        &model_light,
        &model_light_bvh,
        &model_light_material_index,
        model_triangles.data(),
        &model_triangle_bvh,
        model_triangle_material_indices.data(),
        Colour{0.01f, 0.01f, 0.01f},
        Colour{0.01f, 0.01f, 0.01f}
    };

    const Vec3 model_camera_start_position{0.0f, 150.0f, 150.0f};
    Camera model_camera = {};
    model_camera.target = Vec3{0.0f, 0.0f, 0.0f};
    model_camera.orientation = look_at_matrix(model_camera_start_position, model_camera.target);
    model_camera.distance = magnitude(model_camera_start_position - model_camera.target);
    model_camera.fov_y = degrees_to_radians(40.0f);
    model_camera.aperture = 0.1f;
    model_camera.focus_distance = model_camera.distance;

    ApplicationState application_state = {};
    const HWND window = create_window(instance, CLIENT_WIDTH, CLIENT_HEIGHT, application_state);

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

    static constexpr u32 PIXEL_COUNT = CLIENT_WIDTH * CLIENT_HEIGHT;
    real* const pixels_real = static_cast<real*>(VirtualAlloc(0, 4 * PIXEL_COUNT * sizeof(real), MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE));    // TODO: 3x, not 4x
    assert(pixels_real != nullptr);

    unsigned char* const pixels_u8 = static_cast<unsigned char*>(VirtualAlloc(0, 4 * PIXEL_COUNT, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE));
    assert(pixels_u8 != nullptr);    

    LARGE_INTEGER previous_time = {};
    const BOOL read_previous_time = QueryPerformanceCounter(&previous_time);
    assert(read_previous_time != FALSE);

    const Scene& scene = model;
    Camera& camera = model_camera;

    ApplicationState previous_application_state = application_state;

    bool quit = false;
    int sample = 0;
    while (!quit) {
        LARGE_INTEGER current_time = {};
        const BOOL read_current_time = QueryPerformanceCounter(&current_time);
        assert(read_current_time != FALSE);

        const real frame_duration = static_cast<real>(current_time.QuadPart - previous_time.QuadPart) / static_cast<real>(tick_frequency.QuadPart);
        previous_time = current_time;

        bool camera_modified = false;

        MouseInput& mouse_input = application_state.mouse_input;
        const MouseInput& previous_mouse_input = previous_application_state.mouse_input;
        
        const bool left_mouse_button_held = mouse_input.left && previous_mouse_input.left;
        const int mouse_delta_x = mouse_input.x - previous_mouse_input.x;
        const int mouse_delta_y = mouse_input.y - previous_mouse_input.y;
        if (left_mouse_button_held && (mouse_delta_x != 0 || mouse_delta_y != 0)) {
            static constexpr real MOUSE_SENSITIVITY = 4.0f;
            const real camera_yaw_delta = static_cast<real>(mouse_delta_x) * MOUSE_SENSITIVITY * frame_duration;
            const real camera_pitch_delta = -static_cast<real>(mouse_delta_y) * MOUSE_SENSITIVITY * frame_duration;

            const Mat3 camera_yaw_delta_rotation = rotation_matrix(degrees_to_radians(camera_yaw_delta), 0.0f, 0.0f, 1.0f);
            const Mat3 camera_pitch_delta_rotation = rotation_matrix(degrees_to_radians(camera_pitch_delta), 1.0f, 0.0f, 0.0f);
            camera.orientation = camera_yaw_delta_rotation * camera.orientation * camera_pitch_delta_rotation;

            camera_modified = true;
        }

        static constexpr float SCROLL_SENSITIVITY = 0.95f;
        if (mouse_input.scroll > 0) {
          for (int i = 0; i < mouse_input.scroll; ++i) {
            camera.distance *= SCROLL_SENSITIVITY;
          }

          camera_modified = true;
        }
        else if (mouse_input.scroll < 0) {
          for (int i = mouse_input.scroll; i < 0; ++i) {
            camera.distance /= SCROLL_SENSITIVITY;
          }

          camera_modified = true;
        }

        mouse_input.scroll = 0; // scroll handled

        const Vec3 camera_x = get_column(camera.orientation, 0);
        const Vec3 camera_y = get_column(camera.orientation, 1);
        const Vec3 camera_z = get_column(camera.orientation, 2);

        static constexpr float CAMERA_SPEED = 0.6f;
        const KeyboardInput& keyboard_input = application_state.keyboard_input;
        if (keyboard_input.a) {
            camera.target = camera.target - camera.distance * CAMERA_SPEED * frame_duration * camera_x;
            camera_modified = true;
        }

        if (keyboard_input.d) {
            camera.target = camera.target + camera.distance * CAMERA_SPEED * frame_duration * camera_x;
            camera_modified = true;
        }

        if (keyboard_input.w) {
            camera.target = camera.target + camera.distance * CAMERA_SPEED * frame_duration * camera_y;
            camera_modified = true;
        }

        if (keyboard_input.s) {
            camera.target = camera.target - camera.distance * CAMERA_SPEED * frame_duration * camera_y;
            camera_modified = true;
        }

        static constexpr real FOV_SENSITIVITY = 0.1f;
        if (keyboard_input.q) {
            camera.fov_y -= FOV_SENSITIVITY * frame_duration;
            camera_modified = true;
        }

        if (keyboard_input.e) {
            camera.fov_y += FOV_SENSITIVITY * frame_duration;
            camera_modified = true;
        }

        static constexpr real FOCUS_SENSITIVITY = 1.0f;
        if (keyboard_input.up) {
            camera.focus_distance += FOCUS_SENSITIVITY * frame_duration * camera.distance;
            camera_modified = true;
        }

        if (keyboard_input.down) {
            camera.focus_distance -= FOCUS_SENSITIVITY * frame_duration * camera.distance;
            camera_modified = true;
        }

        static constexpr real APERTURE_SENSITIVITY = 0.1f;
        if (keyboard_input.left) {
            camera.aperture -= APERTURE_SENSITIVITY * frame_duration;
            camera_modified = true;
        }

        if (keyboard_input.right) {
            camera.aperture += APERTURE_SENSITIVITY * frame_duration;
            camera_modified = true;
        }

        const Vec3 camera_position = get_position(camera);

        const real viewport_height = 2.0f * std::tan(0.5f * camera.fov_y);
        const real viewport_width = ASPECT_RATIO * viewport_height;

        const Vec3 step_x = camera.focus_distance * viewport_width * camera_x;
        const Vec3 step_y = camera.focus_distance * viewport_height * camera_y;
        const Vec3 bottom_left = camera_position - 0.5f * step_x - 0.5f * step_y - camera.focus_distance * camera_z;

        if (camera_modified) {
            memset(pixels_real, 0, 4 * sizeof(real) * PIXEL_COUNT);
            sample = 0;
        }

        for (int row = 0; row < CLIENT_HEIGHT; ++row) {
            RenderWorkQueue::Entry entry = {};
            entry.row = row;
            entry.sample = sample;
            entry.scene = scene;
            entry.aperture = camera.aperture;
            entry.camera_position = camera_position;
            entry.camera_x = camera_x;
            entry.camera_y = camera_y;
            entry.bottom_left = bottom_left;
            entry.step_x = step_x;
            entry.step_y = step_y;
            entry.pixels = pixels_real;

            push_entry(work_queue, entry);
        }

        while (work_in_progress(work_queue)) {
            process_work_queue_entry(work_queue);
        }

        reset(work_queue);

        for (int pixel_index = 0; pixel_index < PIXEL_COUNT; ++pixel_index) {
            const int index = 4 * pixel_index;
            const real b = std::min<real>(std::sqrt(pixels_real[index + 0] / static_cast<real>(sample + 1)), 1.0f);
            const real g = std::min<real>(std::sqrt(pixels_real[index + 1] / static_cast<real>(sample + 1)), 1.0f);
            const real r = std::min<real>(std::sqrt(pixels_real[index + 2] / static_cast<real>(sample + 1)), 1.0f);

            pixels_u8[index + 0] = static_cast<unsigned char>(255.0f * b);
            pixels_u8[index + 1] = static_cast<unsigned char>(255.0f * g);
            pixels_u8[index + 2] = static_cast<unsigned char>(255.0f * r);
            pixels_u8[index + 3] = 255;
        }

        ++sample;

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
          pixels_u8,
          &bitmap_info,
          DIB_RGB_COLORS,
          SRCCOPY
        );

        assert(scanlines_copied == CLIENT_HEIGHT);

        const KeyboardInput& previous_keyboard_input = previous_application_state.keyboard_input;
        if (keyboard_input.ctrl && !keyboard_input.s && previous_keyboard_input.s) {
            write_pixel_data_to_file(pixels_u8, 4 * PIXEL_COUNT);
        }

        previous_application_state = application_state;

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
