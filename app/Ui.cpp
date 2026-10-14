#include "Ui.h"
#include <string>

void Ui::begin(float mouseX, float mouseY, bool clicked) {
    mx_ = mouseX;
    my_ = mouseY;
    clicked_ = clicked;
    overUi_ = false;
}

bool Ui::hovered(SDL_FRect r) const {
    return mx_ >= r.x && mx_ < r.x + r.w && my_ >= r.y && my_ < r.y + r.h;
}

void Ui::panel(SDL_FRect r, Color c) {
    gfx_.fillRect(r, c);
    if (hovered(r)) overUi_ = true;
}

void Ui::label(float x, float y, const char* str, Color c, float scale) { gfx_.text(x, y, str, c, scale); }

void Ui::labelCentered(float cx, float y, const char* str, Color c, float scale) {
    gfx_.text(cx - Renderer::textWidth(str, scale) * 0.5f, y, str, c, scale);
}

bool Ui::button(SDL_FRect r, const char* label, bool active, float scale) {
    const bool hot = hovered(r);
    if (hot) overUi_ = true;
    Color fill = active ? (hot ? theme::accentHover : theme::accent) : (hot ? theme::panelHover : theme::panel);
    gfx_.fillRect(r, fill);
    gfx_.strokeRect(r, active ? theme::accentHover : theme::border);
    float th = Renderer::textHeight(scale);
    labelCentered(r.x + r.w * 0.5f, r.y + (r.h - th) * 0.5f, label, theme::text, scale);
    return hot && clicked_;
}

float Ui::paragraph(float x, float y, float maxWidth, const char* str, Color c, float scale, float lineGap) {
    const size_t maxChars = std::max<size_t>(1, static_cast<size_t>(maxWidth / Renderer::textWidth("M", scale)));
    const float lineH = Renderer::textHeight(scale) + lineGap;
    std::string text(str), line;
    float used = 0.0f;
    size_t pos = 0;
    while (pos < text.size()) {
        size_t end = text.find(' ', pos);
        if (end == std::string::npos) end = text.size();
        std::string word = text.substr(pos, end - pos);
        if (!line.empty() && line.size() + 1 + word.size() > maxChars) {
            gfx_.text(x, y + used, line.c_str(), c, scale);
            used += lineH;
            line.clear();
        }
        line += (line.empty() ? "" : " ") + word;
        pos = end + 1;
    }
    if (!line.empty()) {
        gfx_.text(x, y + used, line.c_str(), c, scale);
        used += lineH;
    }
    return used;
}

bool Ui::card(SDL_FRect r, const char* title, const char* subtitle, const char* details, const char* badge, bool enabled,
              Color stripe, float titleScale) {
    const bool hot = enabled && hovered(r);
    if (hovered(r)) overUi_ = true;

    gfx_.fillRect(r, hot ? theme::panelHover : theme::panel);
    gfx_.strokeRect(r, hot ? stripe : theme::border);
    gfx_.fillRect({r.x, r.y, 6.0f, r.h}, enabled ? stripe : theme::border);

    const float pad = 22.0f, inner = r.w - 2.0f * pad;
    float y = r.y + pad;
    label(r.x + pad, y, title, enabled ? theme::text : theme::textFaint, titleScale);
    y += Renderer::textHeight(titleScale) + 14.0f;
    if (subtitle) paragraph(r.x + pad, y, inner, subtitle, enabled ? theme::textDim : theme::textFaint, 1.25f);

    const float s = 1.25f;
    const float bottom = r.y + r.h - pad - Renderer::textHeight(s);
    float badgeW = 0.0f;
    if (badge) {
        badgeW = Renderer::textWidth(badge, s) + 16.0f;
        SDL_FRect b{r.x + r.w - pad - badgeW, bottom - 5.0f, badgeW, Renderer::textHeight(s) + 10.0f};
        Color bc = enabled ? stripe : theme::warning;
        gfx_.strokeRect(b, bc);
        label(b.x + 8.0f, b.y + 5.0f, badge, bc, s);
    }
    if (details) {
        float detailW = inner - badgeW - 12.0f;
        float h = Renderer::textHeight(1.0f) + 5.0f;
        size_t maxChars = static_cast<size_t>(detailW / Renderer::textWidth("M", 1.0f));
        int lines = static_cast<int>((std::string(details).size() + maxChars - 1) / std::max<size_t>(maxChars, 1));
        paragraph(r.x + pad, bottom + Renderer::textHeight(s) - lines * h + 5.0f, detailW, details,
                  enabled ? theme::textDim : theme::textFaint, 1.0f, 5.0f);
    }
    return hot && clicked_;
}
