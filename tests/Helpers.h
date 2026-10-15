#pragma once
// Scene builders and measurements shared by the tests.
#include "editor/Simulation.h"
#include <cmath>
#include <functional>
#include <numbers>

namespace th {

using namespace editor;
using pe::Vec2;

inline constexpr float kPi = std::numbers::pi_v<float>;
inline constexpr float kDeg = kPi / 180.0f;

inline Scene empty(float g = 0.0f) {
    Scene s;
    s.world.g = g;
    return s;
}

inline Scene withGround(float g = 9.81f) {
    Scene s = defaultScene();
    s.world.g = g;
    return s;
}

inline ObjectSpec block(Vec2 pos, float w, float h, float mass) {
    ObjectSpec o;
    o.kind = ObjKind::Block;
    o.pos = pos;
    o.width = w;
    o.height = h;
    o.mass = mass;
    o.noSpin = true;
    return o;
}

inline ObjectSpec ball(Vec2 pos, float r, float mass) {
    ObjectSpec o;
    o.kind = ObjKind::Ball;
    o.pos = pos;
    o.radius = r;
    o.mass = mass;
    o.noSpin = true;
    return o;
}

// Fixed wedge (or movable with fixed = false) whose bottom-left corner is at corner.
inline ObjectSpec wedge(Vec2 corner, float base, float slopeDeg, bool fixed = true) {
    ObjectSpec o;
    o.kind = ObjKind::Wedge;
    o.base = base;
    o.slopeDeg = slopeDeg;
    o.fixed = fixed;
    const float h = base * std::tan(slopeDeg * kDeg);
    o.pos = corner + Vec2{2.0f * base / 3.0f, h / 3.0f};
    return o;
}

// Square block of side size resting on the slope of a wedge from wedge(), d along the base from the corner.
inline ObjectSpec onSlope(Vec2 corner, float slopeDeg, float d, float size, float mass) {
    const float th = slopeDeg * kDeg;
    Vec2 surface = corner + Vec2{d, d * std::tan(th)};
    Vec2 normal{-std::sin(th), std::cos(th)};
    ObjectSpec o = block(surface + normal * (size * 0.5f), size, size, mass);
    o.angleDeg = slopeDeg;
    return o;
}

// Adds an object, giving it an id and a name if it has none; returns its id.
inline int add(Scene& s, ObjectSpec o) {
    o.id = s.nextId++;
    if (o.name.empty()) o.name = s.nextName();
    s.objects.push_back(o);
    return o.id;
}

inline int add(Scene& s, LinkSpec l) {
    l.id = s.nextId++;
    s.links.push_back(l);
    return l.id;
}

inline LinkSpec link(LinkKind kind, Endpoint a, Endpoint b) {
    LinkSpec l;
    l.kind = kind;
    l.a = a;
    l.b = b;
    return l;
}

inline Endpoint at(int obj) { return {obj, {}}; }
inline Endpoint fixedPoint(Vec2 p) { return {-1, p}; }

// Average acceleration of an object along x or y between two times.
inline Vec2 accelBetween(Simulation& sim, int index, float t0, float t1) {
    sim.runUntil(t0);
    Vec2 v0 = sim.body(index)->velocity;
    sim.runUntil(t1);
    return (sim.body(index)->velocity - v0) / (t1 - t0);
}

// Period of signal() from its upward zero crossings over the given duration.
inline float measurePeriod(Simulation& sim, float duration, const std::function<float()>& signal) {
    float last = signal(), first = -1.0f, latest = -1.0f;
    int crossings = 0;
    while (sim.time() < duration) {
        sim.step();
        float v = signal();
        if (last < 0.0f && v >= 0.0f) {
            float t = sim.time() - Simulation::kDt * v / (v - last);
            if (first < 0.0f) first = t;
            else ++crossings;
            latest = t;
        }
        last = v;
    }
    return crossings ? (latest - first) / crossings : 0.0f;
}

} // namespace th
