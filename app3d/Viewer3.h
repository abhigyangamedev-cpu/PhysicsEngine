#pragma once

namespace view3d {

enum class Exit { Back, Quit };

// Called after each frame is drawn, before the swap; lets tests read pixels and inject input.
using FrameHook = void (*)(int frame, int pixelWidth, int pixelHeight);

// Opens its own OpenGL window and runs the 3D scenes until ESC (Back) or the window is closed (Quit).
// SDL video must already be initialised.
Exit run(int width, int height, FrameHook hook = nullptr);

} // namespace view3d
