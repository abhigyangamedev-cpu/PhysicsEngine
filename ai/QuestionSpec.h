#pragma once
// A simulation generated from a physics question: the scene (2D or 3D), the moment and quantity to
// measure, and the worked textbook answer to compare against. Read from the JSON the model writes;
// free of SDL and the network so the tests run it headless.
#include "Json.h"
#include "Problems3.h"
#include "editor/Examples.h"
#include <memory>
#include <string>
#include <vector>

namespace ai {

enum class Space { TwoD, ThreeD };

// The moment the run stops at.
enum class Event { Time, Lands, CrossesX, CrossesY, CrossesZ, Apex, Stops, Distance, Reversals };
// What is read off at that moment.
enum class Quantity { Time, X, Y, Z, Speed, Vx, Vy, Vz, Distance, Range, MaxHeight, MaxSpeed, KineticEnergy, Period };

struct Measure {
    Event event = Event::Time;
    std::string eventObject;
    float value = 0.0f; // time, coordinate, landing height, distance or reversal count, by event
    int axis = 1;       // velocity component counted by Reversals: 0 x, 1 y, 2 z
    Quantity quantity = Quantity::Time;
    std::string object; // whose quantity
    std::string label;  // e.g. "Time to reach the ground"
    float timeout = 60.0f; // simulated seconds to wait for the event
};

struct Spec {
    std::string title, topic, question, answer, unit;
    float expected = 0.0f; // textbook value of the measured quantity
    Space space = Space::TwoD;
    editor::Scene scene2;
    editor3::Scene scene3;
    Measure measure;
    pe3::Vec3 viewCenter{0.0f, 2.0f, 0.0f};
    float viewSize = 12.0f; // metres the view should span
    std::string focus;      // object selected and graphed on open
    json::Value source;     // what this was read from, for saving
};

// Reads a spec from the model's JSON. On failure returns null and says what is wrong, in terms the
// model can act on. A question the engine can't simulate comes back as an error starting "cannot simulate".
std::shared_ptr<Spec> readSpec(const json::Value& v, std::string& error);

// One headless run of the spec, the way the editor will run it.
struct Check {
    bool reached = false; // the event happened before the timeout
    bool blewUp = false;
    float result = 0.0f, time = 0.0f;
    std::vector<std::string> lines;
    std::string problem; // empty when the run is usable; otherwise what went wrong
};
Check verify(const Spec& spec);

// The spec as a problem for the 2D editor or the 3D window, measuring as it runs.
std::shared_ptr<const editor::CustomExample> toExample(std::shared_ptr<const Spec> spec);
std::shared_ptr<const editor3::CustomProblem> toProblem(std::shared_ptr<const Spec> spec);

const json::Value& outputSchema(); // the JSON schema the model's answer must follow
const char* systemPrompt();        // how the engine works and how to fill the schema

std::string formatValue(float v); // 4 significant figures, no exponent for everyday sizes

} // namespace ai
