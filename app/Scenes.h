#pragma once
#include "core/World.h"
#include <vector>

struct Scene {
    const char* name;
    void (*setup)(pe::World&);
};

const std::vector<Scene>& scenes();
pe::Shape randomShape(float size);
