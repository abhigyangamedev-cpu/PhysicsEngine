#pragma once
#include "rigid/Body.h"
#include <algorithm>
#include <cmath>

namespace pe {

struct ParticleContact {
    Vec2 normal, point;
    float depth;
};

// Closest-feature contact between a circle of radius r at p and a body. Normal points out of the body.
inline bool particleContact(const Body& b, Vec2 p, float r, ParticleContact& out) {
    if (b.shape.type == ShapeType::Circle) {
        Vec2 d = p - b.position;
        float R = b.shape.radius + r;
        float distSq = d.lengthSq();
        if (distSq >= R * R) return false;
        float dist = std::sqrt(distSq);
        out.normal = dist > 1e-6f ? d / dist : Vec2{0.0f, 1.0f};
        out.depth = R - dist;
        out.point = b.position + out.normal * b.shape.radius;
        return true;
    }

    const auto& v = b.worldVertices;
    const auto& n = b.worldNormals;
    const size_t count = v.size();
    float maxSep = -1e30f;
    size_t face = 0;
    for (size_t i = 0; i < count; ++i) {
        float s = dot(n[i], p - v[i]);
        if (s > r) return false;
        if (s > maxSep) { maxSep = s; face = i; }
    }
    if (maxSep <= 0.0f) {
        out.normal = n[face];
        out.depth = r - maxSep;
        out.point = p - n[face] * maxSep;
        return true;
    }

    float bestSq = 1e30f;
    Vec2 closest;
    for (size_t i = 0; i < count; ++i) {
        Vec2 a = v[i], e = v[(i + 1) % count] - a;
        float t = std::clamp(dot(p - a, e) / e.lengthSq(), 0.0f, 1.0f);
        Vec2 q = a + e * t;
        float dSq = (p - q).lengthSq();
        if (dSq < bestSq) { bestSq = dSq; closest = q; }
    }
    if (bestSq >= r * r) return false;
    float dist = std::sqrt(bestSq);
    out.normal = dist > 1e-6f ? (p - closest) / dist : n[face]; // exactly on an edge
    out.depth = r - dist;
    out.point = closest;
    return true;
}

} // namespace pe
