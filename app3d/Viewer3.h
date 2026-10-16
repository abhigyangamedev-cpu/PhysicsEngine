#pragma once
#include <memory>

namespace editor3 {
struct CustomProblem;
}

namespace view3d {

enum class Exit { Back, Quit };

// What the 3D window opens on.
struct Launch {
    enum class Mode { Scenes, Editor, Problem, Custom };
    Mode mode = Mode::Scenes;
    int problem = 0; // index into editor3::problems() for Mode::Problem
    std::shared_ptr<const editor3::CustomProblem> custom = nullptr; // for Mode::Custom
};

// Called after each frame is drawn, before the swap; lets tests read pixels and inject input.
using FrameHook = void (*)(int frame, int pixelWidth, int pixelHeight);

// Opens its own OpenGL window and runs until ESC/Back (Back) or the window is closed (Quit).
// SDL video must already be initialised.
Exit run(int width, int height, Launch launch = {}, FrameHook hook = nullptr);

} // namespace view3d
