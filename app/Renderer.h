#pragma once
#include "math/Vec2.h"
#include <SDL3/SDL.h>
#include <span>
#include <vector>

struct Color {
    Uint8 r, g, b, a = 255;
};

struct Camera {
    pe::Vec2 center{0.0f, 7.0f};
    float scale = 40.0f; // pixels per meter
    int width = 1280, height = 720;
    float insetTop = 0.0f, insetBottom = 0.0f; // screen space covered by UI bars

    float midY() const { return insetTop + (height - insetTop - insetBottom) * 0.5f; }
    pe::Vec2 toScreen(pe::Vec2 w) const {
        return {(w.x - center.x) * scale + width * 0.5f, (center.y - w.y) * scale + midY()};
    }
    pe::Vec2 toWorld(pe::Vec2 s) const {
        return {(s.x - width * 0.5f) / scale + center.x, center.y - (s.y - midY()) / scale};
    }
};

class Renderer {
public:
    Renderer(SDL_Renderer* r, Camera& cam) : sdl_(r), cam_(cam) {}

    void clear(Color c);
    void line(pe::Vec2 a, pe::Vec2 b, Color c);
    void polygon(std::span<const pe::Vec2> pts, Color fill, Color outline);
    void circle(pe::Vec2 center, float radius, float angle, Color fill, Color outline);
    void point(pe::Vec2 p, float pixels, Color c);
    void text(float x, float y, const char* str, Color c, float scale = 1.0f);

    // Batched discs for drawing thousands of particles in one call.
    void discBatchAdd(pe::Vec2 center, float radius, Color c);
    void discBatchFlush();

    // screen-space helpers for UI
    void fillRect(SDL_FRect r, Color c);
    void strokeRect(SDL_FRect r, Color c);
    static float textWidth(const char* str, float scale = 1.0f);
    static float textHeight(float scale = 1.0f) { return SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE * scale; }

private:
    SDL_Renderer* sdl_;
    Camera& cam_;
    std::vector<SDL_Vertex> batchVerts_;
    std::vector<int> batchIndices_;
};
