#pragma once
#include "math/Vec2.h"
#include <SDL3/SDL.h>
#include <span>

struct Color {
    Uint8 r, g, b, a = 255;
};

struct Camera {
    pe::Vec2 center{0.0f, 7.0f};
    float scale = 40.0f; // pixels per meter
    int width = 1280, height = 720;

    pe::Vec2 toScreen(pe::Vec2 w) const {
        return {(w.x - center.x) * scale + width * 0.5f, (center.y - w.y) * scale + height * 0.5f};
    }
    pe::Vec2 toWorld(pe::Vec2 s) const {
        return {(s.x - width * 0.5f) / scale + center.x, center.y - (s.y - height * 0.5f) / scale};
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
    void text(float x, float y, const char* str, Color c);

private:
    SDL_Renderer* sdl_;
    Camera& cam_;
};
