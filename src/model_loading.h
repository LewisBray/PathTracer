#ifndef MODEL_LOADING_H
#define MODEL_LOADING_H

#include "geometry.h"

#include <vector>

static std::vector<Triangle> load_triangles_file(const char* filename);
static void save_triangles_file(const std::vector<Triangle>& triangles, const char* const filename);

#endif
