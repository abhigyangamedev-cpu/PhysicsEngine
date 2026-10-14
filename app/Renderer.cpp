#include "Renderer.h"
#include <numbers>
#include <vector>

using pe::Vec2;

namespace {

SDL_FColor toF(Color c) { return {c.r / 255.0f, c.g / 255.0f, c.b / 255.0f, c.a / 255.0f}; }

} // namespace

void Renderer::clear(Color c) {
    SDL_SetRenderDrawColor(sdl_, c.r, c.g, c.b, c.a);
    SDL_RenderClear(sdl_);
}

void Renderer::line(Vec2 a, Vec2 b, Color c) {
    Vec2 sa = cam_.toScreen(a), sb = cam_.toScreen(b);
    SDL_SetRenderDrawColor(sdl_, c.r, c.g, c.b, c.a);
    SDL_RenderLine(sdl_, sa.x, sa.y, sb.x, sb.y);
}

void Renderer::polygon(std::span<const Vec2> pts, Color fill, Color outline) {
    const int n = static_cast<int>(pts.size());
    if (n < 3) return;

    std::vector<SDL_Vertex> verts(n);
    std::vector<SDL_FPoint> loop(n + 1);
    const SDL_FColor fc = toF(fill);
    for (int i = 0; i < n; ++i) {
        Vec2 s = cam_.toScreen(pts[i]);
        verts[i] = {{s.x, s.y}, fc, {0.0f, 0.0f}};
        loop[i] = {s.x, s.y};
    }
    loop[n] = loop[0];

    std::vector<int> indices;
    indices.reserve((n - 2) * 3);
    for (int i = 1; i < n - 1; ++i) indices.insert(indices.end(), {0, i, i + 1});

    SDL_RenderGeometry(sdl_, nullptr, verts.data(), n, indices.data(), static_cast<int>(indices.size()));
    SDL_SetRenderDrawColor(sdl_, outline.r, outline.g, outline.b, outline.a);
    SDL_RenderLines(sdl_, loop.data(), n + 1);
}

void Renderer::circle(Vec2 center, float radius, float angle, Color fill, Color outline) {
    constexpr int segments = 32;
    Vec2 pts[segments];
    for (int i = 0; i < segments; ++i) {
        float t = 2.0f * std::numbers::pi_v<float> * i / segments;
        pts[i] = center + Vec2{std::cos(t), std::sin(t)} * radius;
    }
    polygon(pts, fill, outline);
    line(center, center + Vec2{std::cos(angle), std::sin(angle)} * radius, outline);
}

void Renderer::point(Vec2 p, float px, Color c) {
    Vec2 s = cam_.toScreen(p);
    SDL_FRect r{s.x - px * 0.5f, s.y - px * 0.5f, px, px};
    SDL_SetRenderDrawColor(sdl_, c.r, c.g, c.b, c.a);
    SDL_RenderFillRect(sdl_, &r);
}

void Renderer::text(float x, float y, const char* str, Color c) {
    SDL_SetRenderDrawColor(sdl_, c.r, c.g, c.b, c.a);
    SDL_RenderDebugText(sdl_, x, y, str);
}
