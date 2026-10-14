#pragma once
// Small helpers shared by the scene files.
#include "Scenes.h"
#include <algorithm>
#include <numbers>
#include <random>

namespace kit {

using pe::Body;
using pe::BodyDef;
using pe::Shape;
using pe::Vec2;
using pe::World;

inline constexpr float kPi = std::numbers::pi_v<float>;
inline constexpr float kG = 9.81f;

inline std::mt19937& rng() {
    static std::mt19937 gen{1337};
    return gen;
}

inline float randf(float lo, float hi) { return std::uniform_real_distribution<float>(lo, hi)(rng()); }

inline Body* addStaticBox(World& w, Vec2 pos, float hw, float hh, float angle = 0.0f) {
    BodyDef d;
    d.position = pos;
    d.angle = angle;
    d.isStatic = true;
    d.friction = 0.6f;
    return w.createBody(Shape::box(hw, hh), d);
}

// Static convex polygon from world-space points. Shape::polygon recenters vertices on the
// centroid, so the body is placed at that centroid to keep the points where they were given.
inline Body* addStaticPolygon(World& w, std::vector<Vec2> worldPoints, float friction = 0.6f) {
    Vec2 first = *std::min_element(worldPoints.begin(), worldPoints.end(),
                                   [](Vec2 a, Vec2 b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
    Shape shape = Shape::polygon(std::move(worldPoints));
    BodyDef d;
    d.position = first - shape.vertices[0]; // the hull starts at the lowest-leftmost point
    d.isStatic = true;
    d.friction = friction;
    return w.createBody(shape, d);
}

inline Body* addAnchor(World& w, Vec2 pos) {
    BodyDef d;
    d.position = pos;
    d.isStatic = true;
    return w.createBody(Shape::circle(0.08f), d);
}

// A static, non-colliding wheel drawn where a rope runs over a fixed pulley.
inline Body* addPulleyWheel(World& w, Vec2 center, float radius) {
    BodyDef d;
    d.position = center;
    d.isStatic = true;
    d.collides = false;
    d.color = 0x8a93a6ff;
    return w.createBody(Shape::circle(radius), d);
}

// Dynamic body with a given total mass rather than a density.
inline Body* addBody(World& w, const Shape& shape, Vec2 pos, float mass, uint32_t color = 0) {
    BodyDef d;
    d.position = pos;
    d.color = color;
    float area = shape.computeMass(1.0f).mass;
    d.density = area > 0.0f ? mass / area : 1.0f;
    return w.createBody(shape, d);
}

inline float kineticEnergy(const Body& b) {
    return 0.5f * b.mass * b.velocity.lengthSq() + 0.5f * b.inertia * b.angularVelocity * b.angularVelocity;
}

// Circle outline as overlay segments.
inline void ring(Readout& r, Vec2 c, float radius, uint32_t color, int segments = 32) {
    for (int i = 0; i < segments; ++i) {
        float a0 = 2.0f * kPi * i / segments, a1 = 2.0f * kPi * (i + 1) / segments;
        r.line(c + Vec2{std::cos(a0), std::sin(a0)} * radius, c + Vec2{std::cos(a1), std::sin(a1)} * radius, color);
    }
}

// Arc from angle a0 to a1 (radians, counterclockwise) as overlay segments.
inline void arc(Readout& r, Vec2 c, float radius, float a0, float a1, uint32_t color, int segments = 16) {
    for (int i = 0; i < segments; ++i) {
        float t0 = a0 + (a1 - a0) * i / segments, t1 = a0 + (a1 - a0) * (i + 1) / segments;
        r.line(c + Vec2{std::cos(t0), std::sin(t0)} * radius, c + Vec2{std::cos(t1), std::sin(t1)} * radius, color);
    }
}

inline constexpr uint32_t kRope = 0xc8965aff;

// Arrow from a to b as overlay segments.
inline void arrow(Readout& r, Vec2 a, Vec2 b, uint32_t color) {
    Vec2 d = b - a;
    float len = d.length();
    if (len < 1e-4f) return;
    Vec2 u = d / len, head = u * std::min(0.35f, 0.4f * len);
    r.line(a, b, color);
    r.line(b, b - head + head.perp() * 0.6f, color);
    r.line(b, b - head - head.perp() * 0.6f, color);
}

// Estimates a steady acceleration from velocity samples: the slope of v against t.
struct AccelMeter {
    double n = 0, st = 0, sv = 0, stt = 0, stv = 0;
    void reset() { *this = {}; }
    void add(float t, float v) {
        n += 1;
        st += t;
        sv += v;
        stt += double(t) * t;
        stv += double(t) * v;
    }
    bool ready() const { return n > 10; }
    float value() const {
        double den = n * stt - st * st;
        return den != 0.0 ? float((n * stv - st * sv) / den) : 0.0f;
    }
};

// Measures the period of an oscillating signal from its upward zero crossings.
struct PeriodMeter {
    float last = 0.0f, lastCross = -1.0f, period = 0.0f;
    bool primed = false;
    void add(float t, float signal, float dt) {
        if (primed && last < 0.0f && signal >= 0.0f) {
            float tc = t - dt * signal / (signal - last); // interpolate the crossing within the step
            if (lastCross >= 0.0f) period = tc - lastCross;
            lastCross = tc;
        }
        last = signal;
        primed = true;
    }
};

} // namespace kit
