#include "field/Fields.h"
#include "core/World.h"
#include <cmath>

namespace pe {

void NBodyGravity::apply(World& world, float h) {
    positions_.clear();
    masses_.clear();
    for (const auto& b : world.bodies()) {
        positions_.push_back(b->position);
        masses_.push_back(b->mass);
    }
    auto& ps = world.particles.particles;
    if (includeParticles) {
        for (const auto& p : ps) {
            positions_.push_back(p.position);
            masses_.push_back(p.invMass > 0.0f ? 1.0f / p.invMass : 0.0f);
        }
    }
    tree_.build(positions_, masses_);

    const float eps2 = softening * softening;
    int index = 0;
    for (const auto& b : world.bodies()) {
        if (!b->isStatic) b->velocity += tree_.acceleration(b->position, G, eps2, theta, index) * h;
        ++index;
    }
    if (includeParticles) {
        for (auto& p : ps) {
            if (p.invMass > 0.0f) p.velocity += tree_.acceleration(p.position, G, eps2, theta, index) * h;
            ++index;
        }
    }
}

void CoulombField::apply(World& world, float h) {
    thread_local std::vector<Body*> charged;
    charged.clear();
    for (const auto& b : world.bodies())
        if (b->charge != 0.0f) charged.push_back(b.get());

    const float eps2 = softening * softening;
    for (size_t i = 0; i < charged.size(); ++i) {
        Body* a = charged[i];
        for (size_t j = i + 1; j < charged.size(); ++j) {
            Body* b = charged[j];
            Vec2 d = b->position - a->position;
            float r2 = d.lengthSq() + eps2;
            Vec2 f = d * (k * a->charge * b->charge / (r2 * std::sqrt(r2))); // force on b
            a->velocity -= f * (a->invMass * h);
            b->velocity += f * (b->invMass * h);
        }
    }
}

Vec2 CoulombField::electricField(const World& world, Vec2 p) const {
    Vec2 E;
    const float eps2 = softening * softening;
    for (const auto& b : world.bodies()) {
        if (b->charge == 0.0f) continue;
        Vec2 d = p - b->position;
        float r2 = d.lengthSq() + eps2;
        E += d * (k * b->charge / (r2 * std::sqrt(r2)));
    }
    return E;
}

void UniformElectricField::apply(World& world, float h) {
    for (const auto& b : world.bodies())
        if (!b->isStatic && b->charge != 0.0f && region.contains(b->position))
            b->velocity += E * (b->charge * b->invMass * h);
}

void UniformMagneticField::apply(World& world, float h) {
    for (const auto& b : world.bodies()) {
        if (b->isStatic || b->charge == 0.0f || !region.contains(b->position)) continue;
        // dv/dt = (q/m) v x B  ->  rotation at angular rate -qB/m
        float angle = -b->charge * B * b->invMass * h;
        Rot r(angle);
        b->velocity = r.apply(b->velocity);
    }
}

float orbitalSpeed(float G, float centralMass, float radius) { return std::sqrt(G * centralMass / radius); }

} // namespace pe
