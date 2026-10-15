#pragma once
// Shared by Editor.cpp and EditorPanels.cpp.
#include "editor/Editor.h"
#include <numbers>

namespace editor::layout {

inline constexpr float kDeg = std::numbers::pi_v<float> / 180.0f;
inline constexpr float kTop = 52.0f, kBar = 40.0f, kHint = 26.0f, kGraphH = 210.0f, kInspW = 350.0f;
inline constexpr const char* kSavePath = "editor_scene.txt";
inline constexpr const char* kToolNames[] = {"Select", "Block", "Ball", "Wedge", "Ground", "Spring", "Rod", "Rope", "Hinge", "Pulley"};
inline constexpr const char* kPlotNames[] = {"x", "y", "vx", "vy", "|v|", "ax", "ay", "|a|", "KE", "E total"};
inline constexpr const char* kLinkNames[] = {"Spring", "Rod", "Rope", "Hinge", "Pulley"};

} // namespace editor::layout
