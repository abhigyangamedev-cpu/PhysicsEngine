#pragma once
#include "editor/Simulation.h"
#include <string>
#include <vector>

namespace editor {

// What an example has measured so far in the current run.
struct ExampleRun {
    bool done = false, primed = false;
    float result = 0.0f; // the asked-for quantity, once done
    float prev[4]{};
    std::vector<std::string> lines;
};

// A real-life problem the 2D simulation answers.
struct Example {
    const char* name;
    const char* topic;
    const char* question;
    const char* answer;
    float expected; // textbook value of the asked-for quantity
    Scene (*setup)();
    Vec2 cameraCenter;
    float cameraScale;
    const char* focus; // object selected and graphed on load
    int plot;          // editor::Plot shown on load
    // Called after every step; returns true to pause (the asked-for moment was reached).
    bool (*monitor)(const Simulation&, ExampleRun&);
};

const std::vector<Example>& examples();

} // namespace editor
