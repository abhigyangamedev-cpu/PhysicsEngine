#include "Renderer.h"
#include <numbers>
#include <algorithm>
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

void Renderer::text(float x, float y, const char* str, Color c, float scale) {
    SDL_SetRenderDrawColor(sdl_, c.r, c.g, c.b, c.a);
    SDL_SetRenderScale(sdl_, scale, scale);
    SDL_RenderDebugText(sdl_, x / scale, y / scale, str);
    SDL_SetRenderScale(sdl_, 1.0f, 1.0f);
}

void Renderer::fillRect(SDL_FRect r, Color c) {
    SDL_SetRenderDrawColor(sdl_, c.r, c.g, c.b, c.a);
    SDL_RenderFillRect(sdl_, &r);
}

void Renderer::strokeRect(SDL_FRect r, Color c) {
    SDL_SetRenderDrawColor(sdl_, c.r, c.g, c.b, c.a);
    SDL_RenderRect(sdl_, &r);
}

float Renderer::textWidth(const char* str, float scale) {
    return static_cast<float>(SDL_strlen(str)) * SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE * scale;
}

void Renderer::discBatchAdd(Vec2 center, float radius, Color c) {
    constexpr int kSides = 6;
    static const Vec2 ring[kSides] = {{1.0f, 0.0f},  {0.5f, 0.866f},  {-0.5f, 0.866f},
                                      {-1.0f, 0.0f}, {-0.5f, -0.866f}, {0.5f, -0.866f}};
    const Vec2 s = cam_.toScreen(center);
    const float r = std::max(radius * cam_.scale, 1.0f);
    const SDL_FColor fc = toF(c);
    const int base = static_cast<int>(batchVerts_.size());
    batchVerts_.push_back({{s.x, s.y}, fc, {0.0f, 0.0f}});
    for (Vec2 d : ring) batchVerts_.push_back({{s.x + d.x * r, s.y - d.y * r}, fc, {0.0f, 0.0f}});
    for (int i = 0; i < kSides; ++i) batchIndices_.insert(batchIndices_.end(), {base, base + 1 + i, base + 1 + (i + 1) % kSides});
}

void Renderer::discBatchFlush() {
    if (!batchVerts_.empty())
        SDL_RenderGeometry(sdl_, nullptr, batchVerts_.data(), static_cast<int>(batchVerts_.size()), batchIndices_.data(),
                           static_cast<int>(batchIndices_.size()));
    batchVerts_.clear();
    batchIndices_.clear();
}
