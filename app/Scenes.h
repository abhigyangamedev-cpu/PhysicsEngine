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
    void (*update)(pe::World&, float dt) = nullptr;
    // Return true when the scene handled the click itself (button: 1 left, 3 right).
    bool (*click)(pe::World&, pe::Vec2 point, int button) = nullptr;
    pe::Vec2 cameraCenter{0.0f, 7.0f};
    float cameraScale = 40.0f;
    bool trails = false;
    bool fieldLines = false;
    bool stir = false; // left-drag on empty space stirs the fluid
};

const std::vector<Category>& categories();
const std::vector<Scene>& scenes();
std::vector<int> scenesIn(int category);
pe::Shape randomShape(float size);
