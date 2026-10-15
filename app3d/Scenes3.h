#pragma once
#include "core/World3.h"
#include <string>
#include <vector>

namespace scenes3 {

struct Scene {
    std::string name;
    void (*build)(pe3::World&);
    pe3::Vec3 focus{0, 2, 0}; // initial camera target and distance
    float distance = 18.0f;
    bool settles = true; // false for scenes meant to keep moving (undamped pendulums, motors)
    int category = 0;     // index into categories()
    const char* description = "";
};

const std::vector<Scene>& all();
const std::vector<const char*>& categories();
pe3::Body* addGround(pe3::World& w, float half = 30.0f);

} // namespace scenes3
