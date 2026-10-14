#include "soft/Builders.h"
#include <algorithm>
#include <numbers>

namespace pe {

std::vector<uint32_t> makeCloth(ParticleSystem& ps, const ClothDef& d) {
    const int group = ps.newGroup();
    std::vector<uint32_t> idx(d.columns * d.rows);
    for (int r = 0; r < d.rows; ++r)
        for (int c = 0; c < d.columns; ++c)
            idx[r * d.columns + c] = ps.addParticle(d.topLeft + Vec2{c * d.spacing, -r * d.spacing}, d.particleMass,
                                                    d.spacing * 0.45f, group);

    auto at = [&](int r, int c) { return idx[r * d.columns + c]; };
    for (int r = 0; r < d.rows; ++r) {
        for (int c = 0; c < d.columns; ++c) {
            if (c + 1 < d.columns) ps.link(at(r, c), at(r, c + 1), d.stretchCompliance, d.tearRatio);
            if (r + 1 < d.rows) ps.link(at(r, c), at(r + 1, c), d.stretchCompliance, d.tearRatio);
            if (c + 1 < d.columns && r + 1 < d.rows) {
                ps.link(at(r, c), at(r + 1, c + 1), d.shearCompliance, d.tearRatio);
                ps.link(at(r, c + 1), at(r + 1, c), d.shearCompliance, d.tearRatio);
            }
            if (c + 2 < d.columns) ps.link(at(r, c), at(r, c + 2), d.bendCompliance, d.tearRatio);
            if (r + 2 < d.rows) ps.link(at(r, c), at(r + 2, c), d.bendCompliance, d.tearRatio);
        }
    }
    return idx;
}

std::vector<uint32_t> makeRope(ParticleSystem& ps, Vec2 from, Vec2 to, int segments, float mass, float compliance) {
    const int group = ps.newGroup();
    const float segLen = (to - from).length() / segments;
    std::vector<uint32_t> idx;
    for (int i = 0; i <= segments; ++i) {
        idx.push_back(ps.addParticle(from + (to - from) * (static_cast<float>(i) / segments), mass / (segments + 1),
                                     segLen * 0.5f, group));
        if (i > 0) ps.link(idx[i - 1], idx[i], compliance);
    }
    return idx;
}

std::vector<uint32_t> makeBlob(ParticleSystem& ps, const BlobDef& d) {
    const int group = ps.newGroup();
    const float segLen = 2.0f * std::numbers::pi_v<float> * d.radius / d.segments;
    std::vector<uint32_t> ring;
    for (int i = 0; i < d.segments; ++i) {
        float a = 2.0f * std::numbers::pi_v<float> * i / d.segments;
        ring.push_back(ps.addParticle(d.center + Vec2{std::cos(a), std::sin(a)} * d.radius, d.mass / d.segments,
                                      segLen * 0.5f, group));
    }
    for (int i = 0; i < d.segments; ++i) ps.link(ring[i], ring[(i + 1) % d.segments], d.edgeCompliance);
    ps.addPressure(ring, d.pressure, d.pressureCompliance);
    return ring;
}

std::vector<uint32_t> makeJelly(ParticleSystem& ps, const JellyDef& d) {
    const int group = ps.newGroup();
    const float dx = d.width / (d.columns - 1), dy = d.height / (d.rows - 1);
    const Vec2 origin = d.center - Vec2{d.width, d.height} * 0.5f;
    const float m = d.mass / (d.columns * d.rows);
    std::vector<uint32_t> idx(d.columns * d.rows);
    for (int r = 0; r < d.rows; ++r)
        for (int c = 0; c < d.columns; ++c)
            idx[r * d.columns + c] = ps.addParticle(origin + Vec2{c * dx, r * dy}, m, 0.5f * std::min(dx, dy), group);

    auto at = [&](int r, int c) { return idx[r * d.columns + c]; };
    for (int r = 0; r < d.rows; ++r) {
        for (int c = 0; c < d.columns; ++c) {
            if (c + 1 < d.columns) ps.link(at(r, c), at(r, c + 1), d.compliance);
            if (r + 1 < d.rows) ps.link(at(r, c), at(r + 1, c), d.compliance);
            if (c + 1 < d.columns && r + 1 < d.rows) {
                ps.link(at(r, c), at(r + 1, c + 1), d.compliance);
                ps.link(at(r, c + 1), at(r + 1, c), d.compliance);
            }
        }
    }
    return idx;
}

} // namespace pe
