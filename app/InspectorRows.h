#pragma once
// Inspector layout shared by the 2D and 3D editors: a label, then one to three number fields per row.
#include "Ui.h"
#include <cstdio>
#include <initializer_list>
#include <utility>

// Lays out inspector rows top to bottom, advancing y.
struct Rows {
    Ui& ui;
    float x, y, w;
    bool enabled;
    bool changed = false;
    static constexpr float kRowH = 28.0f, kLabelW = 118.0f;

    void header(const char* title, const char* sub = nullptr) {
        ui.label(x, y, title, theme::text, 1.75f);
        y += Renderer::textHeight(1.75f) + 6.0f;
        if (sub) {
            ui.label(x, y, sub, theme::textFaint, 1.0f);
            y += Renderer::textHeight(1.0f) + 6.0f;
        }
        y += 6.0f;
    }
    void section(const char* title) {
        y += 6.0f;
        ui.label(x, y, title, theme::textDim, 1.25f);
        y += Renderer::textHeight(1.25f) + 8.0f;
    }
    void label(const char* s) { ui.label(x, y + (kRowH - 4.0f - Renderer::textHeight(1.0f)) * 0.5f, s, theme::textDim, 1.0f); }
    bool one(const char* name, int id, float& v, float step) {
        label(name);
        bool c = ui.numberField({x + kLabelW, y, w - kLabelW, kRowH - 4.0f}, id, v, step, enabled);
        y += kRowH;
        changed |= c;
        return c;
    }
    bool three(const char* name, int id, float& a, float& b, float& c, float step) {
        label(name);
        const float fw = (w - kLabelW - 8.0f) / 3.0f;
        bool any = false;
        float* v[3] = {&a, &b, &c};
        for (int i = 0; i < 3; ++i) any |= ui.numberField({x + kLabelW + i * (fw + 4.0f), y, fw, kRowH - 4.0f}, id + i, *v[i], step, enabled);
        y += kRowH;
        changed |= any;
        return any;
    }
    bool two(const char* name, int id, float& a, float& b, float step, bool* first = nullptr) {
        label(name);
        const float fw = (w - kLabelW - 6.0f) * 0.5f;
        bool ca = ui.numberField({x + kLabelW, y, fw, kRowH - 4.0f}, id, a, step, enabled);
        bool cb = ui.numberField({x + kLabelW + fw + 6.0f, y, fw, kRowH - 4.0f}, id + 1, b, step, enabled);
        y += kRowH;
        if (first) *first = ca;
        changed |= ca || cb;
        return ca || cb;
    }
    bool toggles(std::initializer_list<std::pair<const char*, bool*>> items) {
        float tx = x;
        bool any = false;
        for (auto [name, v] : items) {
            float bw = Renderer::textWidth(name, 1.0f) + 18.0f;
            if (ui.button({tx, y, bw, kRowH - 6.0f}, name, *v, 1.0f) && enabled) {
                *v = !*v;
                any = true;
            }
            tx += bw + 6.0f;
        }
        y += kRowH;
        changed |= any;
        return any;
    }
    void text(const char* fmt, double a = 0, double b = 0, double c = 0) {
        char buf[96];
        std::snprintf(buf, sizeof buf, fmt, a, b, c);
        ui.label(x, y, buf, theme::text, 1.0f);
        y += Renderer::textHeight(1.0f) + 7.0f;
    }
    float note(const char* s) {
        float h = ui.paragraph(x, y, w, s, theme::textFaint, 1.0f, 5.0f);
        y += h + 4.0f;
        return h;
    }
};

