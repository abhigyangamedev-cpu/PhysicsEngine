#include "field/Fields.h"
#include "core/World.h"
#include <cmath>
#include <numbers>

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

SpringField::Spring& SpringField::add(Body* a, Vec2 worldA, Body* b, Vec2 worldB, float k, float damping) {
    Spring s{};
    s.a = a;
    s.b = b;
    s.localA = a->rot.applyT(worldA - a->position);
    s.localB = b ? b->rot.applyT(worldB - b->position) : worldB;
    s.k = k;
    s.damping = damping;
    s.rest = (worldB - worldA).length();
    springs.push_back(s);
    return springs.back();
}

Vec2 SpringField::endA(const Spring& s) const { return s.a->position + s.a->rot.apply(s.localA); }

Vec2 SpringField::endB(const Spring& s) const {
    if (s.b) return s.b->position + s.b->rot.apply(s.localB);
    return s.localB + s.drive * std::sin(2.0f * std::numbers::pi_v<float> * s.driveHz * time);
}

void SpringField::apply(World&, float h) {
    time += h;
    const float omega = 2.0f * std::numbers::pi_v<float>;
    for (const Spring& s : springs) {
        Vec2 pA = endA(s), pB = endB(s);
        Vec2 d = pB - pA;
        float len = d.length();
        if (len < 1e-6f) continue;
        Vec2 u = d / len;
        float stretch = len - s.rest;
        if (s.pushOnly && stretch >= 0.0f) continue;

        Vec2 vB = s.b ? s.b->velocityAt(pB) : s.drive * (omega * s.driveHz * std::cos(omega * s.driveHz * time));
        float f = s.k * stretch + s.damping * dot(vB - s.a->velocityAt(pA), u);
        s.a->applyImpulse(u * (f * h), pA - s.a->position);
        if (s.b) s.b->applyImpulse(u * (-f * h), pB - s.b->position);
    }
}

float orbitalSpeed(float G, float centralMass, float radius) { return std::sqrt(G * centralMass / radius); }

} // namespace pe
