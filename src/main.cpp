#include "path_tracing.h"
#include "geometry.h"
#include "material.h"
#include "colour.h"
#include "types.h"
#include "vec3.h"
#include "rng.h"

#include "path_tracing.cpp"
#include "geometry.cpp"
#include "material.cpp"
#include "colour.cpp"
#include "vec3.cpp"
#include "rng.cpp"

#include <cassert>
#include <cfloat>

#include <Windows.h>

static constexpr real PI = 3.14159265358979323846264f;

// app settings
static constexpr real ASPECT_RATIO = 3.0f / 2.0f;
static constexpr real FOV_Y_DEGREES = 20.0f;
static constexpr real APERTURE = 0.1f;
static constexpr int SAMPLES_PER_PIXEL = 500;
static constexpr Vec3 CAMERA_POSITION{13.0f, 2.0f, 3.0f};
static constexpr Vec3 CAMERA_TARGET{0.0f, 0.0f, 0.0f};
static constexpr int CLIENT_WIDTH = 1200;

static constexpr real LENS_RADIUS = 0.5f * APERTURE;
static constexpr int CLIENT_HEIGHT = static_cast<int>(static_cast<real>(CLIENT_WIDTH) / ASPECT_RATIO);

struct RenderWorkQueue {
    struct Entry {
        int row;
        Scene scene;
        Vec3 camera_x;
        Vec3 camera_y;
        Vec3 bottom_left;
        Vec3 step_x;
        Vec3 step_y;
        unsigned char* pixels;
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
    const Scene& scene,
    const Vec3& camera_x,
    const Vec3& camera_y,
    const Vec3& bottom_left,
    const Vec3& step_x,
    const Vec3& step_y,
    unsigned char* const pixels
) {
    for (int column = 0; column < CLIENT_WIDTH; ++column) {
        Colour colour = {};
        for (int sample_index = 0; sample_index < SAMPLES_PER_PIXEL; ++sample_index) {
            const u32 u_rng = noise_3d(row, column, sample_index + 0);
            const real u_randomness = real_from_rng(u_rng);
            const real u = (static_cast<real>(column) + u_randomness) / static_cast<real>(CLIENT_WIDTH - 1);

            const u32 v_rng = noise_3d(row, column, sample_index + 1);
            const real v_randomness = real_from_rng(v_rng);
            const real v = (static_cast<real>(row) + v_randomness) / static_cast<real>(CLIENT_HEIGHT - 1);

            const u32 a_rng = noise_3d(column, row, sample_index + 0);
            const real a_randomness = real_from_rng(a_rng);
            const real a = APERTURE * a_randomness - LENS_RADIUS;

            const real b_max = std::sqrt(LENS_RADIUS * LENS_RADIUS - a * a);
            const real b_min = -b_max;

            const real b_rng = noise_3d(column, row, sample_index + 1);
            const real b_randomness = real_from_rng(b_rng);
            const real b = (b_max - b_min) * b_randomness + b_min;

            const Vec3 random_offset = a * camera_x + b * camera_y;

            const Vec3 ray_direction = normalise(Vec3{bottom_left + u * step_x + v * step_y - CAMERA_POSITION - random_offset});
            const Ray ray{CAMERA_POSITION + random_offset, ray_direction};

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

static bool process_work_queue_entry(RenderWorkQueue& work_queue) {
    bool had_entry_to_process = false;

    const LONG original_entry_to_do_index = work_queue.next_entry_to_do_index;
    if (original_entry_to_do_index < work_queue.entry_count) {
        const LONG entry_to_do_index = InterlockedCompareExchange(&work_queue.next_entry_to_do_index, original_entry_to_do_index + 1, original_entry_to_do_index);
        if (entry_to_do_index == original_entry_to_do_index) {
            const RenderWorkQueue::Entry& entry_to_do = work_queue.entries[entry_to_do_index];
            render_scanline(
                entry_to_do.row,
                entry_to_do.scene,
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
    unsigned char* const pixels = static_cast<unsigned char*>(VirtualAlloc(0, PIXEL_BYTE_COUNT, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE));
    assert(pixels != nullptr);

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

    static constexpr real FOV_Y = FOV_Y_DEGREES / 180.0f * PI;
    const real viewport_height = 2.0f * std::tan(0.5f * FOV_Y);
    const real viewport_width = ASPECT_RATIO * viewport_height;

    const Vec3 camera_z = normalise(CAMERA_POSITION - CAMERA_TARGET);
    const Vec3 camera_x = normalise(Vec3{0.0f, 1.0f, 0.0f} ^ camera_z);
    const Vec3 camera_y = camera_z ^ camera_x;

    const real focus_distance = 10.0f;
    const Vec3 step_x = focus_distance * viewport_width * camera_x;
    const Vec3 step_y = focus_distance * viewport_height * camera_y;

    const Vec3 bottom_left = CAMERA_POSITION - 0.5f * step_x - 0.5f * step_y - focus_distance * camera_z;

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

    const Scene scene{
        materials,
        spheres,
        sphere_material_indices,
        SPHERE_COUNT
    };

    LARGE_INTEGER render_start_time = {};
    const BOOL read_start_time = QueryPerformanceCounter(&render_start_time);
    assert(read_start_time != FALSE);

    for (int row = 0; row < CLIENT_HEIGHT; ++row) {
        RenderWorkQueue::Entry entry = {};
        entry.row = row;
        entry.scene = scene;
        entry.camera_x = camera_x;
        entry.camera_y = camera_y;
        entry.bottom_left = bottom_left;
        entry.step_x = step_x;
        entry.step_y = step_y;
        entry.pixels = pixels;

        push_entry(work_queue, entry);
    }

    while (work_in_progress(work_queue)) {
        process_work_queue_entry(work_queue);
    }

    LARGE_INTEGER render_finish_time = {};
    const BOOL read_finish_time = QueryPerformanceCounter(&render_finish_time);
    assert(read_finish_time != FALSE);

    const real render_duration = static_cast<real>(render_finish_time.QuadPart - render_start_time.QuadPart) / static_cast<real>(tick_frequency.QuadPart);

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
