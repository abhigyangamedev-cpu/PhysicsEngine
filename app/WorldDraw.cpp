#include "WorldDraw.h"
#include "field/Fields.h"
#include <algorithm>
#include <cmath>

using namespace pe;

Color rgba(uint32_t c) { return {Uint8(c >> 24), Uint8(c >> 16), Uint8(c >> 8), Uint8(c)}; }

Color bodyColor(const Body& b) {
    if (b.color) return rgba(b.color);
    if (b.charge > 0.0f) return {235, 87, 87};
    if (b.charge < 0.0f) return {72, 149, 239};
    if (b.isStatic) return {70, 75, 85};
    if (b.mass > 500.0f) return {255, 200, 87};
    static constexpr Color palette[] = {
        {231, 111, 81}, {244, 162, 97}, {233, 196, 106}, {42, 157, 143}, {94, 129, 172}, {180, 142, 173},
    };
    return palette[b.id % std::size(palette)];
}

Color darker(Color c) { return {Uint8(c.r * 0.6f), Uint8(c.g * 0.6f), Uint8(c.b * 0.6f), c.a}; }

// Zigzag spring between two points; the coil count stays fixed so stretching is visible.
void drawCoil(Renderer& gfx, Vec2 a, Vec2 b, Color c) {
    Vec2 d = b - a;
    float len = d.length();
    if (len < 1e-4f) return;
    Vec2 u = d / len, n = u.perp() * 0.18f;
    constexpr int kTurns = 12;
    const float lead = std::min(0.2f, 0.15f * len);
    Vec2 prev = a + u * lead;
    gfx.line(a, prev, c);
    for (int i = 1; i <= kTurns * 2; ++i) {
        float t = lead + (len - 2.0f * lead) * i / (kTurns * 2);
        Vec2 p = a + u * t + (i == kTurns * 2 ? Vec2{} : (i % 2 ? n : -n));
        gfx.line(prev, p, c);
        prev = p;
    }
    gfx.line(prev, b, c);
}

void drawWorld(Renderer& gfx, const World& world, bool showContacts) {
    const auto& ps = world.particles;
    for (const auto& pc : ps.pressures) {
        thread_local std::vector<Vec2> ring;
        ring.clear();
        for (uint32_t i : pc.ring) ring.push_back(ps.particles[i].position);
        gfx.polygon(ring, {94, 200, 172, 200}, {40, 120, 100});
    }

    for (const auto& b : world.bodies()) {
        Color c = bodyColor(*b);
        if (b->shape.type == ShapeType::Circle)
            gfx.circle(b->position, b->shape.radius, b->angle, c, darker(c));
        else
            gfx.polygon(b->worldVertices, c, darker(c));
    }

    const auto& fl = world.fluid;
    const float fr = fl.particleRadius() * 1.6f;
    for (size_t i = 0; i < fl.size(); ++i) {
        uint32_t base = fl.color[i] ? fl.color[i] : 0x2f7fd6ff;
        float t = std::min(fl.velocity[i].length() / 7.0f, 1.0f);
        auto mix = [t](uint32_t c, int shift, float target) {
            return Uint8(((c >> shift) & 0xff) * (1.0f - t) + target * t);
        };
        gfx.discBatchAdd(fl.position[i], fr, {mix(base, 24, 200), mix(base, 16, 235), mix(base, 8, 255), 190});
    }
    gfx.discBatchFlush();

    for (const auto& l : ps.links)
        if (!l.broken) gfx.line(ps.particles[l.a].position, ps.particles[l.b].position, {170, 190, 230, 160});
    for (const auto& p : ps.particles) {
        Color c = p.invMass == 0.0f ? Color{255, 80, 80} : Color{220, 230, 255};
        if (p.color) c = {Uint8(p.color >> 24), Uint8(p.color >> 16), Uint8(p.color >> 8), Uint8(p.color)};
        gfx.point(p.position, p.color ? 2.0f : 3.0f, c);
    }
    for (const auto& a : ps.attachments)
        gfx.line(ps.particles[a.particle].position, a.body->position + a.body->rot.apply(a.localAnchor), {255, 200, 80});

    for (const auto& f : world.fields()) {
        auto* sf = dynamic_cast<const SpringField*>(f.get());
        if (!sf) continue;
        for (const auto& s : sf->springs) {
            if (s.coil)
                drawCoil(gfx, sf->endA(s), sf->endB(s), s.pushOnly ? Color{120, 200, 255} : Color{250, 210, 80});
            else
                gfx.line(sf->endA(s), sf->endB(s), {220, 220, 230});
        }
    }

    for (const auto& j : world.joints()) {
        Color c{230, 230, 230};
        if (dynamic_cast<const GearJoint*>(j.get())) continue; // the gears themselves show it
        if (auto* p = dynamic_cast<const PulleyJoint*>(j.get())) {
            const Color rope{200, 150, 90};
            gfx.line(p->groundA, p->anchorA(), rope);
            // a block and tackle hangs body b from `ratio` parallel strands
            const int strands = std::max(1, static_cast<int>(std::lround(p->ratio)));
            const float spread = p->b->shape.type == ShapeType::Circle ? p->b->shape.radius * 0.8f : 0.3f;
            for (int k = 0; k < strands; ++k) {
                float off = strands == 1 ? 0.0f : spread * (2.0f * k / (strands - 1) - 1.0f);
                gfx.line(p->groundB + Vec2{off, 0.0f}, p->anchorB() + Vec2{off, 0.0f}, rope);
            }
            continue;
        }
        if (auto* d = dynamic_cast<const DistanceJoint*>(j.get())) {
            if (d->rope) c = {200, 150, 90};
            else if (d->hertz > 0.0f) c = {250, 210, 80};
        }
        gfx.line(j->anchorA(), j->anchorB(), c);
        gfx.point(j->anchorA(), 4.0f, c);
        gfx.point(j->anchorB(), 4.0f, c);
    }

    if (showContacts) {
        for (const auto& m : world.manifolds()) {
            for (int i = 0; i < m.count; ++i) {
                gfx.point(m.contacts[i].point, 5.0f, {255, 60, 60});
                gfx.line(m.contacts[i].point, m.contacts[i].point + m.normal * 0.3f, {255, 255, 255});
            }
        }
    }
}
