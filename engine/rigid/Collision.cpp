#include "rigid/Collision.h"
#include <algorithm>
#include <utility>

namespace pe {

namespace {

bool circleCircle(Body* a, Body* b, Manifold& m) {
    Vec2 d = b->position - a->position;
    float r = a->shape.radius + b->shape.radius;
    float distSq = d.lengthSq();
    if (distSq > r * r) return false;

    float dist = std::sqrt(distSq);
    m.normal = dist > 1e-6f ? d / dist : Vec2{0.0f, 1.0f};
    m.count = 1;
    auto& c = m.contacts[0];
    c.depth = r - dist;
    c.point = a->position + m.normal * (a->shape.radius - 0.5f * c.depth);
    c.feature = 0;
    return true;
}

bool polygonCircle(Body* poly, Body* circle, Manifold& m) {
    const auto& verts = poly->worldVertices;
    const auto& norms = poly->worldNormals;
    const size_t n = verts.size();
    Vec2 c = circle->position;
    float r = circle->shape.radius;

    size_t face = 0;
    float maxSep = -1e30f;
    for (size_t i = 0; i < n; ++i) {
        float s = dot(norms[i], c - verts[i]);
        if (s > r) return false;
        if (s > maxSep) { maxSep = s; face = i; }
    }

    Vec2 v1 = verts[face], v2 = verts[(face + 1) % n];
    auto& cp = m.contacts[0];
    m.count = 1;
    cp.feature = static_cast<uint32_t>(face);

    if (maxSep > 1e-6f) {
        auto vertexRegion = [&](Vec2 v) {
            Vec2 d = c - v;
            float distSq = d.lengthSq();
            if (distSq > r * r) return false;
            float dist = std::sqrt(distSq);
            m.normal = d / dist;
            cp.depth = r - dist;
            cp.point = v;
            return true;
        };
        if (dot(c - v1, v2 - v1) <= 0.0f) return vertexRegion(v1);
        if (dot(c - v2, v1 - v2) <= 0.0f) return vertexRegion(v2);
    }

    m.normal = norms[face];
    cp.depth = r - maxSep;
    cp.point = c - m.normal * (r - 0.5f * cp.depth);
    return true;
}

float findMaxSeparation(const Body* a, const Body* b, size_t& edge) {
    float best = -1e30f;
    for (size_t i = 0; i < a->worldVertices.size(); ++i) {
        Vec2 n = a->worldNormals[i], v = a->worldVertices[i];
        float minSep = 1e30f;
        for (Vec2 w : b->worldVertices) minSep = std::min(minSep, dot(n, w - v));
        if (minSep > best) { best = minSep; edge = i; }
    }
    return best;
}

struct ClipVertex {
    Vec2 v;
    uint32_t id;
};

int clipSegment(ClipVertex out[2], const ClipVertex in[2], Vec2 n, float offset, uint32_t clipId) {
    int k = 0;
    float d0 = dot(n, in[0].v) - offset;
    float d1 = dot(n, in[1].v) - offset;
    if (d0 <= 0.0f) out[k++] = in[0];
    if (d1 <= 0.0f) out[k++] = in[1];
    if (d0 * d1 < 0.0f) {
        float t = d0 / (d0 - d1);
        out[k].v = in[0].v + (in[1].v - in[0].v) * t;
        out[k].id = (d0 > 0.0f ? in[0].id : in[1].id) | clipId;
        ++k;
    }
    return k;
}

bool polygonPolygon(Body* a, Body* b, Manifold& m) {
    size_t edgeA = 0, edgeB = 0;
    float sepA = findMaxSeparation(a, b, edgeA);
    if (sepA > 0.0f) return false;
    float sepB = findMaxSeparation(b, a, edgeB);
    if (sepB > 0.0f) return false;

    const Body* ref = a;
    const Body* inc = b;
    size_t refEdge = edgeA;
    bool flip = false;
    if (sepB > 0.98f * sepA + 0.001f) {
        std::swap(ref, inc);
        refEdge = edgeB;
        flip = true;
    }

    const size_t rn = ref->worldVertices.size();
    const size_t in = inc->worldVertices.size();
    Vec2 refN = ref->worldNormals[refEdge];

    size_t incEdge = 0;
    float minDot = 1e30f;
    for (size_t i = 0; i < in; ++i) {
        float d = dot(inc->worldNormals[i], refN);
        if (d < minDot) { minDot = d; incEdge = i; }
    }

    uint32_t base = (flip ? 1u << 31 : 0u) | static_cast<uint32_t>(refEdge) << 16 | static_cast<uint32_t>(incEdge) << 8;
    ClipVertex incident[2] = {
        {inc->worldVertices[incEdge], base | 0u},
        {inc->worldVertices[(incEdge + 1) % in], base | 1u},
    };

    Vec2 v1 = ref->worldVertices[refEdge];
    Vec2 v2 = ref->worldVertices[(refEdge + 1) % rn];
    Vec2 t = (v2 - v1).normalized();

    ClipVertex clip1[2], clip2[2];
    if (clipSegment(clip1, incident, -t, -dot(t, v1), 2u) < 2) return false;
    if (clipSegment(clip2, clip1, t, dot(t, v2), 4u) < 2) return false;

    m.normal = flip ? -refN : refN;
    m.count = 0;
    for (const auto& cv : clip2) {
        float sep = dot(refN, cv.v - v1);
        if (sep > 0.0f) continue;
        auto& cp = m.contacts[m.count++];
        cp.point = cv.v - refN * (0.5f * sep);
        cp.depth = -sep;
        cp.feature = cv.id;
    }
    return m.count > 0;
}

} // namespace

bool collide(Body* a, Body* b, Manifold& m) {
    const bool ca = a->shape.type == ShapeType::Circle;
    const bool cb = b->shape.type == ShapeType::Circle;
    if (ca && !cb) std::swap(a, b);
    m.a = a;
    m.b = b;
    if (ca && cb) return circleCircle(a, b, m);
    if (ca || cb) return polygonCircle(a, b, m);
    return polygonPolygon(a, b, m);
}

} // namespace pe
