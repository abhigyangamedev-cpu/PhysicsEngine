#pragma once
// Real-life problems solved by the 3D simulation: each builds a scene, then measures the asked-for
// quantity as it runs so the answer can be checked against the textbook one.
#include "EditorScene3.h"
#include <string>
#include <vector>

namespace editor3 {

// What a problem has measured so far in the current run.
struct ProblemRun {
    bool done = false, primed = false;
    float result = 0.0f; // the asked-for quantity, once done
    float prev[6]{};
    std::vector<std::string> lines;
};

struct Problem {
    const char* name;
    const char* topic;
    const char* question;
    const char* answer;
    float expected; // textbook value of the asked-for quantity
    const char* unit;
    Scene (*setup)();
    Vec3 cameraTarget;
    float cameraDistance, cameraYaw, cameraPitch;
    const char* focus; // object selected on load
    // Called after every step; returns true to pause (the asked-for moment was reached).
    bool (*monitor)(const Simulation&, ProblemRun&);
};

const std::vector<Problem>& problems();

} // namespace editor3
