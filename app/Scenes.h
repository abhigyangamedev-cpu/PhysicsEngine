#pragma once
#include "core/World.h"
#include <string>
#include <vector>

struct Category {
    const char* name;
    const char* description;
    const char* status; // nullptr when playable
};

// Live measurements a scene puts on screen, rebuilt every frame.
struct Readout {
    struct Line {
        pe::Vec2 a, b;
        uint32_t color; // 0xRRGGBBAA
    };
    std::vector<std::string> text;
    std::vector<Line> lines;          // world-space overlay: rays, ropes, gear teeth, arrows
    std::vector<float> histogram;     // optional bar chart, drawn under the text
    std::vector<float> histogramCurve; // optional theory curve over the same bins
    std::string histogramLabel;

    void add(const char* fmt, ...);
    void line(pe::Vec2 a, pe::Vec2 b, uint32_t color) { lines.push_back({a, b, color}); }
};

struct Scene {
    const char* name;
    int category;
    void (*setup)(pe::World&);
    void (*update)(pe::World&, float dt) = nullptr;
    // Return true when the scene handled the click itself (button: 1 left, 3 right).
    bool (*click)(pe::World&, pe::Vec2 point, int button) = nullptr;
    void (*readout)(const pe::World&, Readout&) = nullptr;
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

// Scene lists for the categories that live in their own files (app/scenes/*.cpp).
std::vector<Scene> machineScenes(int category);
std::vector<Scene> oscillationScenes(int category);
std::vector<Scene> collisionScenes(int category);
std::vector<Scene> projectileScenes(int category);
std::vector<Scene> rotationScenes(int category);
std::vector<Scene> thermoScenes(int category);
std::vector<Scene> opticsScenes(int category);
