#pragma once
#include "core/World.h"
#include <vector>

struct Category {
    const char* name;
    const char* description;
    const char* status; // nullptr when playable
};

struct Scene {
    const char* name;
    int category;
    void (*setup)(pe::World&);
};

const std::vector<Category>& categories();
const std::vector<Scene>& scenes();
std::vector<int> scenesIn(int category);
pe::Shape randomShape(float size);
