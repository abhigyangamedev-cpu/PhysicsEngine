#include "Ui.h"
#include "Expr.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

void popChar(std::string& s) { // one UTF-8 character
    while (!s.empty() && (static_cast<unsigned char>(s.back()) & 0xc0) == 0x80) s.pop_back();
    if (!s.empty()) s.pop_back();
}

// Lines of at most maxChars, breaking at spaces where it can and at explicit newlines.
std::vector<std::string> wrap(const std::string& text, size_t maxChars) {
    std::vector<std::string> lines;
    size_t start = 0;
    while (start <= text.size()) {
        size_t end = text.find('\n', start);
        if (end == std::string::npos) end = text.size();
        std::string para = text.substr(start, end - start);
        do {
            if (para.size() <= maxChars) {
                lines.push_back(para);
                break;
            }
            size_t cut = para.rfind(' ', maxChars);
            if (cut == std::string::npos || cut == 0) cut = maxChars;
            lines.push_back(para.substr(0, cut));
            para.erase(0, para[cut] == ' ' ? cut + 1 : cut);
        } while (true);
        start = end + 1;
    }
    return lines;
}

} // namespace

void Ui::begin(float mouseX, float mouseY, bool clicked) {
    mx_ = mouseX;
    my_ = mouseY;
    clicked_ = clicked;
    overUi_ = false;
    wheel_ = pendingWheel_;
    pendingWheel_ = 0.0f;
    if (editId_ && !editSeen_) editId_ = 0; // the field being edited is no longer drawn
    editSeen_ = false;
}

void Ui::textInput(const char* utf8) {
    if (editId_) buffer_ += utf8;
}

void Ui::keyDown(SDL_Keycode key) {
    if (!editId_) return;
    const bool enter = key == SDLK_RETURN || key == SDLK_KP_ENTER;
    if (key == SDLK_BACKSPACE) {
        popChar(buffer_);
    } else if (textMode_ && key == SDLK_V && (SDL_GetModState() & SDL_KMOD_CTRL)) {
        if (char* clip = SDL_GetClipboardText()) {
            for (const char* c = clip; *c; ++c)
                if (*c != '\r') buffer_ += multiline_ || *c != '\n' ? *c : ' ';
            SDL_free(clip);
        }
    } else if (textMode_ && multiline_ && enter) {
        buffer_ += '\n';
    } else if (enter || key == SDLK_TAB) {
        commit_ = true;
    }
    if (key == SDLK_ESCAPE) cancel_ = true;
}

bool Ui::textArea(SDL_FRect r, int id, std::string& text, const char* placeholder, bool multiline, bool masked,
                  size_t maxLength) {
    const bool hot = hovered(r);
    if (hot) overUi_ = true;
    bool changed = false;
    if (editId_ == id) {
        editSeen_ = true;
        if (buffer_.size() > maxLength) buffer_.resize(maxLength);
        if (buffer_ != text) {
            text = buffer_;
            changed = true;
        }
        if ((clicked_ && !hot) || commit_ || cancel_) {
            editId_ = 0;
            commit_ = cancel_ = false;
            textMode_ = false;
        }
    } else if (hot && clicked_) {
        buffer_ = text;
        editId_ = id;
        editSeen_ = true;
        commit_ = cancel_ = false;
        textMode_ = true;
        multiline_ = multiline;
    }

    const bool active = editId_ == id;
    gfx_.fillRect(r, active ? Color{14, 16, 21} : (hot ? theme::panelHover : Color{22, 25, 32}));
    gfx_.strokeRect(r, active ? theme::accentHover : theme::border);
    const float s = 1.25f, pad = 8.0f, lineH = Renderer::textHeight(s) + 6.0f;
    if (text.empty() && !active) {
        if (placeholder && multiline) paragraph(r.x + pad, r.y + pad, r.w - 2 * pad, placeholder, theme::textFaint, s);
        else if (placeholder) gfx_.text(r.x + pad, r.y + (r.h - Renderer::textHeight(s)) * 0.5f, placeholder, theme::textFaint, s);
        return changed;
    }
    std::string shown = masked ? std::string(text.size(), '*') : text;
    if (active) shown += '_';
    const size_t perLine = std::max<size_t>(1, static_cast<size_t>((r.w - 2 * pad) / Renderer::textWidth("M", s)));
    std::vector<std::string> lines = multiline ? wrap(shown, perLine) : std::vector<std::string>{shown};
    if (!multiline && shown.size() > perLine) lines[0] = shown.substr(shown.size() - perLine); // keep the end in view
    const size_t fit = std::max<size_t>(1, static_cast<size_t>((r.h - 2 * pad + 6.0f) / lineH));
    const size_t first = active && lines.size() > fit ? lines.size() - fit : 0; // typing: follow the cursor
    const float y0 = multiline ? r.y + pad : r.y + (r.h - Renderer::textHeight(s)) * 0.5f;
    for (size_t i = first; i < lines.size() && i < first + fit; ++i)
        gfx_.text(r.x + pad, y0 + (i - first) * lineH, lines[i].c_str(), theme::text, s);
    return changed;
}

bool Ui::numberField(SDL_FRect r, int id, float& value, float step, bool enabled) {
    const bool hot = hovered(r);
    if (hot) overUi_ = true;
    const bool active = editId_ == id;
    bool changed = false;

    if (active) {
        editSeen_ = true;
        if (clicked_ && !hot) commit_ = true;
        if (cancel_ || commit_) {
            if (commit_) {
                if (auto v = evaluate(buffer_)) {
                    changed = static_cast<float>(*v) != value;
                    value = static_cast<float>(*v);
                }
            }
            editId_ = 0;
            commit_ = cancel_ = false;
        }
    } else if (enabled && hot && clicked_) {
        textMode_ = false;
        char buf[32];
        std::snprintf(buf, sizeof buf, "%.6g", value);
        buffer_ = buf;
        editId_ = id;
        editSeen_ = true;
        commit_ = cancel_ = false;
    } else if (enabled && hot && step > 0.0f && wheel_ != 0.0f) {
        value = std::round((value + step * (wheel_ > 0.0f ? 1.0f : -1.0f)) / step) * step;
        if (std::abs(value) < step * 1e-3f) value = 0.0f;
        wheel_ = 0.0f;
        changed = true;
    }

    const bool editingNow = editId_ == id;
    gfx_.fillRect(r, editingNow ? Color{14, 16, 21} : (hot && enabled ? theme::panelHover : Color{22, 25, 32}));
    gfx_.strokeRect(r, editingNow ? theme::accentHover : theme::border);
    char buf[40];
    if (editingNow) std::snprintf(buf, sizeof buf, "%s_", buffer_.c_str());
    else std::snprintf(buf, sizeof buf, "%.5g", value);
    const float s = 1.25f;
    const size_t fit = static_cast<size_t>((r.w - 12.0f) / Renderer::textWidth("M", s));
    const size_t len = std::strlen(buf);
    const char* shown = len > fit ? buf + (len - fit) : buf; // keep the cursor end visible
    gfx_.text(r.x + 6.0f, r.y + (r.h - Renderer::textHeight(s)) * 0.5f, shown, enabled ? theme::text : theme::textFaint, s);
    return changed;
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
