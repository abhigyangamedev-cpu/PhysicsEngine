#pragma once
#include "editor/Simulation.h"
#include <functional>
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
    std::function<Scene()> setup;
    Vec2 cameraCenter;
    float cameraScale;
    const char* focus; // object selected and graphed on load
    int plot;          // editor::Plot shown on load
    // Called after every step; returns true to pause (the asked-for moment was reached).
    std::function<bool(const Simulation&, ExampleRun&)> monitor;
    const char* unit = ""; // of expected, shown next to it when set
};

// A problem made at run time (e.g. from an uploaded question). It owns the text its example points
// into, so it is shared rather than copied.
struct CustomExample {
    std::string name, topic, question, answer, focus, unit;
    Example example{};

    CustomExample() = default;
    CustomExample(const CustomExample&) = delete;
    CustomExample& operator=(const CustomExample&) = delete;
};

const std::vector<Example>& examples();

} // namespace editor
