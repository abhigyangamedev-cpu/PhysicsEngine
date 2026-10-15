#pragma once

namespace view3d {

enum class Exit { Back, Quit };

// What the 3D window opens on.
struct Launch {
    enum class Mode { Scenes, Editor, Problem };
    Mode mode = Mode::Scenes;
    int problem = 0; // index into editor3::problems() for Mode::Problem
};

// Called after each frame is drawn, before the swap; lets tests read pixels and inject input.
using FrameHook = void (*)(int frame, int pixelWidth, int pixelHeight);

// Opens its own OpenGL window and runs until ESC/Back (Back) or the window is closed (Quit).
// SDL video must already be initialised.
Exit run(int width, int height, Launch launch = {}, FrameHook hook = nullptr);

} // namespace view3d
