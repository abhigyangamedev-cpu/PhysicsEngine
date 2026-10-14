#pragma once
#include <cmath>

namespace pe {

struct Vec2 {
    float x = 0.0f, y = 0.0f;

    constexpr Vec2() = default;
    constexpr Vec2(float x, float y) : x(x), y(y) {}

    constexpr Vec2 operator+(Vec2 o) const { return {x + o.x, y + o.y}; }
    constexpr Vec2 operator-(Vec2 o) const { return {x - o.x, y - o.y}; }
    constexpr Vec2 operator*(float s) const { return {x * s, y * s}; }
    constexpr Vec2 operator/(float s) const { return {x / s, y / s}; }
    constexpr Vec2 operator-() const { return {-x, -y}; }
    constexpr Vec2& operator+=(Vec2 o) { x += o.x; y += o.y; return *this; }
    constexpr Vec2& operator-=(Vec2 o) { x -= o.x; y -= o.y; return *this; }
    constexpr Vec2& operator*=(float s) { x *= s; y *= s; return *this; }

    float length() const { return std::sqrt(x * x + y * y); }
    constexpr float lengthSq() const { return x * x + y * y; }
    Vec2 normalized() const {
        float l = length();
        return l > 1e-12f ? Vec2{x / l, y / l} : Vec2{};
    }
    constexpr Vec2 perp() const { return {-y, x}; }
};

constexpr Vec2 operator*(float s, Vec2 v) { return v * s; }
constexpr float dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
constexpr float cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
// w × v for scalar angular velocity w
constexpr Vec2 cross(float w, Vec2 v) { return {-w * v.y, w * v.x}; }
constexpr Vec2 cross(Vec2 v, float w) { return {w * v.y, -w * v.x}; }

struct Rot {
    float c = 1.0f, s = 0.0f;

    Rot() = default;
    explicit Rot(float angle) : c(std::cos(angle)), s(std::sin(angle)) {}

    constexpr Vec2 apply(Vec2 v) const { return {c * v.x - s * v.y, s * v.x + c * v.y}; }
    constexpr Vec2 applyT(Vec2 v) const { return {c * v.x + s * v.y, -s * v.x + c * v.y}; }
};

struct Aabb {
    Vec2 min, max;

    constexpr bool overlaps(const Aabb& o) const {
        return min.x <= o.max.x && max.x >= o.min.x && min.y <= o.max.y && max.y >= o.min.y;
    }
};

} // namespace pe
