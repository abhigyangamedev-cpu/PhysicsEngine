#pragma once
#include "Renderer.h"
#include <string>

namespace theme {
inline constexpr Color background{18, 20, 26};
inline constexpr Color panel{30, 33, 42};
inline constexpr Color panelHover{40, 44, 56};
inline constexpr Color border{58, 63, 78};
inline constexpr Color accent{42, 157, 143};
inline constexpr Color accentHover{58, 186, 170};
inline constexpr Color text{232, 234, 240};
inline constexpr Color textDim{140, 146, 162};
inline constexpr Color textFaint{90, 95, 110};
inline constexpr Color warning{233, 196, 106};
} // namespace theme

// Immediate-mode widgets: call every frame, a widget returns true on the frame it is clicked.
class Ui {
public:
    explicit Ui(Renderer& r) : gfx_(r) {}

    void begin(float mouseX, float mouseY, bool clicked);
    bool hovered(SDL_FRect r) const;
    bool overUi() const { return overUi_; }
    void scroll(float wheel) { pendingWheel_ += wheel; }

    bool button(SDL_FRect r, const char* label, bool active = false, float scale = 1.5f);
    // Large selectable card with title, subtitle and optional badge; disabled cards are dimmed.
    bool card(SDL_FRect r, const char* title, const char* subtitle, const char* details, const char* badge, bool enabled,
              Color stripe, float titleScale = 2.0f);

    void panel(SDL_FRect r, Color c = theme::panel);
    void label(float x, float y, const char* str, Color c = theme::text, float scale = 1.5f);
    void labelCentered(float cx, float y, const char* str, Color c = theme::text, float scale = 1.5f);
    // Click to type a value (see Expr.h); Enter or clicking away commits, Esc cancels, the wheel
    // nudges by step. Returns true on the frame the value changes. id must be unique and nonzero.
    bool numberField(SDL_FRect r, int id, float& value, float step = 0.0f, bool enabled = true);
    // Click to type free text, edited in place. Enter adds a line when multiline (otherwise finishes),
    // Ctrl+V pastes, Esc or clicking away finishes. Returns true on the frames the text changes.
    bool textArea(SDL_FRect r, int id, std::string& text, const char* placeholder, bool multiline = true,
                  bool masked = false, size_t maxLength = 6000);
    bool editing() const { return editId_ != 0; }
    void textInput(const char* utf8);
    void keyDown(SDL_Keycode key);

    // Word-wraps to maxWidth; returns the height used.
    float paragraph(float x, float y, float maxWidth, const char* str, Color c, float scale, float lineGap = 6.0f);

private:
    Renderer& gfx_;
    float mx_ = 0, my_ = 0;
    bool clicked_ = false;
    bool overUi_ = false;
    float wheel_ = 0.0f, pendingWheel_ = 0.0f;

    int editId_ = 0;
    bool editSeen_ = false, commit_ = false, cancel_ = false;
    bool textMode_ = false, multiline_ = false; // the field being edited is a textArea
    std::string buffer_;
};
