#include <Windows.h>

#include <cassert>

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

    static constexpr DWORD WINDOW_STYLE = WS_CAPTION | WS_SYSMENU;
    static constexpr DWORD CLIENT_WIDTH = 256;
    static constexpr DWORD CLIENT_HEIGHT = 256;

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

    unsigned int pixels[CLIENT_WIDTH * CLIENT_HEIGHT] = {};

    for (int row = 0; row < CLIENT_HEIGHT; ++row) {
        for (int column = 0; column < CLIENT_WIDTH; ++column) {
            pixels[row * CLIENT_WIDTH + column] = 0xFF00;
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
