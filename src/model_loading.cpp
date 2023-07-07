#include "model_loading.h"
#include "types.h"

#include <cassert>

// TODO: a bit weird this file depends on windows?
#define NOMINMAX
#include <Windows.h>

static std::vector<Triangle> load_triangles_file(const char* const filename) {
    const HANDLE file_handle = CreateFileA(
        filename,
        GENERIC_READ,
        0,
        nullptr,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        NULL
    );

    assert(file_handle != INVALID_HANDLE_VALUE);

    LARGE_INTEGER file_size = {};
    const BOOL got_file_size = GetFileSizeEx(file_handle, &file_size);
    assert(got_file_size != FALSE);
    assert(file_size.QuadPart % sizeof(Triangle) == 0);

    std::vector<Triangle> triangles(file_size.QuadPart / sizeof(Triangle));

    DWORD bytes_read = 0;
    const BOOL read_file_triangles = ReadFile(file_handle, triangles.data(), file_size.QuadPart, &bytes_read, nullptr);
    assert(read_file_triangles != FALSE);
    assert(bytes_read == file_size.QuadPart);

    const BOOL closed_file_handle = CloseHandle(file_handle);
    assert(closed_file_handle != FALSE);

    return triangles;
}

static void save_triangles_file(const std::vector<Triangle>& triangles, const char* const filename) {
    const HANDLE file_handle = CreateFileA(
        filename,
        GENERIC_WRITE,
        0,
        nullptr,
        CREATE_ALWAYS,
        FILE_ATTRIBUTE_NORMAL,
        NULL
    );

    assert(file_handle != INVALID_HANDLE_VALUE);

    DWORD bytes_written = 0;
    const BOOL wrote_file = WriteFile(file_handle, triangles.data(), sizeof(Triangle) * triangles.size(), &bytes_written, nullptr);
    assert(wrote_file != FALSE);
    assert(bytes_written == sizeof(Triangle) * triangles.size());

    const BOOL closed_file_handle = CloseHandle(file_handle);
    assert(closed_file_handle != FALSE);
}

struct ByteStream {
    char* data;
    u32 size;
    u32 index;
};

struct Token {
    const char* string;
    u32 length;
};

static bool is_whitespace(const char c) {
    return (c == ' ') || (c == '\n') || (c == '\t') || (c == '\r');
}

Token get_next_token(ByteStream& byte_stream) {
    while (byte_stream.index < byte_stream.size && is_whitespace(byte_stream.data[byte_stream.index])) {
        ++byte_stream.index;
    }

    const DWORD token_start = byte_stream.index;
    while (byte_stream.index < byte_stream.size && !is_whitespace(byte_stream.data[byte_stream.index])) {
        ++byte_stream.index;
    }

    return Token{byte_stream.data + token_start, byte_stream.index - token_start};
}

static Vec3 parse_stl_vec3(ByteStream& byte_stream) {
    const Token x_token = get_next_token(byte_stream);    
    char* x_conversion_end = nullptr;
    const real x = strtor(x_token.string, &x_conversion_end);
    assert(x_conversion_end == x_token.string + x_token.length);

    const Token y_token = get_next_token(byte_stream);
    char* y_conversion_end = nullptr;
    const real y = strtor(y_token.string, &y_conversion_end);
    assert(y_conversion_end == y_token.string + y_token.length);

    const Token z_token = get_next_token(byte_stream);
    char* z_conversion_end = nullptr;
    const real z = strtor(z_token.string, &z_conversion_end);
    assert(z_conversion_end == z_token.string + z_token.length);

    return Vec3{x, y, z};
}

static std::vector<Triangle> parse_stl_file(const char* const filename) {
    const HANDLE file_handle = CreateFileA(
        filename,
        GENERIC_READ,
        0,
        nullptr,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        NULL
    );

    assert(file_handle != INVALID_HANDLE_VALUE);

    LARGE_INTEGER file_size = {};
    const BOOL got_file_size = GetFileSizeEx(file_handle, &file_size);
    assert(got_file_size != FALSE);

    ByteStream stl_file = {};
    stl_file.data = static_cast<char*>(VirtualAlloc(0, file_size.QuadPart + 1, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE));
    assert(stl_file.data != nullptr);

    DWORD bytes_read = 0;
    const BOOL file_read = ReadFile(file_handle, stl_file.data, file_size.QuadPart, &bytes_read, nullptr);
    assert(file_read != FALSE);
    assert(bytes_read == file_size.QuadPart);
    stl_file.size = file_size.QuadPart;
    stl_file.index = 0;

    std::vector<Triangle> triangles;

    int vertex_count = 0;
    bool parsing_facet = false;
    Vec3 current_normal{0.0f, 0.0f, 0.0f};
    while (stl_file.index < stl_file.size) {
        const Token token = get_next_token(stl_file);
        if (token.length == 0) {
            break;
        }

        if (strncmp(token.string, "facet", token.length) == 0) {
            assert(parsing_facet == false);
            parsing_facet = true;
            triangles.push_back(Triangle{});
        } else if (strncmp(token.string, "endfacet", token.length) == 0) {
            assert(parsing_facet == true);
            parsing_facet = false;
        } else if (strncmp(token.string, "normal", token.length) == 0) {
            assert(parsing_facet);
            current_normal = parse_stl_vec3(stl_file);
        } else if (strncmp(token.string, "loop", token.length) == 0) {
            vertex_count = 0;
        } else if (strncmp(token.string, "endloop", token.length) == 0) {
            assert(vertex_count == 3);
        } else if (strncmp(token.string, "vertex", token.length) == 0) {
            assert(parsing_facet);

            const Vec3 vertex = parse_stl_vec3(stl_file);
            switch (vertex_count) {
                case 0: {
                    triangles.back().a = vertex;
                    break;
                }
                case 1: {
                    triangles.back().b = vertex;
                    break;
                }
                case 2: {
                    triangles.back().c = vertex;
                    break;
                }
                default: {
                    assert(false);
                    break;
                }
            }

            ++vertex_count;
        }
    }

    VirtualFree(stl_file.data, file_size.QuadPart + 1, MEM_RELEASE);

    const BOOL closed_file_handle = CloseHandle(file_handle);
    assert(closed_file_handle != FALSE);

    return triangles;
}
