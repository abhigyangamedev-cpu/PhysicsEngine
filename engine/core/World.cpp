#include "core/World.h"
#include <algorithm>
#include <cmath>

namespace pe {

namespace {

uint64_t pairKey(const Body* a, const Body* b) {
    uint32_t i = a->id, j = b->id;
    if (i > j) std::swap(i, j);
    return static_cast<uint64_t>(i) << 32 | j;
}

} // namespace

Body* World::createBody(const Shape& shape, const BodyDef& def) {
    auto body = std::make_unique<Body>(shape, def);
    body->id = nextId_++;
    bodies_.push_back(std::move(body));
    return bodies_.back().get();
}

void World::destroyBody(Body* body) {
    std::erase_if(joints_, [body](const auto& j) { return j->a == body || j->b == body; });
    rebuildJointPairs();
    particles.removeBody(body);
    std::erase_if(manifolds_, [body](const auto& kv) { return kv.second.a == body || kv.second.b == body; });
    std::erase_if(bodies_, [body](const auto& p) { return p.get() == body; });
}

void World::destroyJoint(Joint* joint) {
    std::erase_if(joints_, [joint](const auto& j) { return j.get() == joint; });
    rebuildJointPairs();
}

void World::rebuildJointPairs() {
    jointPairs_.clear();
    for (const auto& j : joints_)
        if (j->a && j->b && !j->collideConnected) jointPairs_.insert(pairKey(j->a, j->b));
}

void World::clear() {
    joints_.clear();
    jointPairs_.clear();
    particles.clear();
    manifolds_.clear();
    bodies_.clear();
}

Body* World::bodyAt(Vec2 point) const {
    for (const auto& b : bodies_)
        if (!b->isStatic && b->contains(point)) return b.get();
    return nullptr;
}

void World::step(float dt) {
    const float h = dt / static_cast<float>(settings.substeps);
    for (int s = 0; s < settings.substeps; ++s) {
        integrateVelocities(h);
        detectContacts();
        prepareContacts(h);
        for (auto& j : joints_) j->prepare(h);
        for (int i = 0; i < settings.velocityIterations; ++i) {
            solveJoints(true);
            solveContacts(true);
        }
        integratePositions(h);
        // remove the velocity injected by position correction so it doesn't become kinetic energy
        for (int i = 0; i < settings.relaxIterations; ++i) {
            solveJoints(false);
            solveContacts(false);
        }
        applyRestitution();

        const float ph = h / static_cast<float>(settings.particleIterations);
        for (int i = 0; i < settings.particleIterations; ++i) particles.step(ph, settings.gravity, bodies_);
    }
    for (auto& b : bodies_) {
        b->force = {};
        b->torque = 0.0f;
    }
}

void World::integrateVelocities(float h) {
    const float linDamp = 1.0f / (1.0f + h * settings.linearDamping);
    const float angDamp = 1.0f / (1.0f + h * settings.angularDamping);
    for (auto& b : bodies_) {
        if (b->isStatic) continue;
        b->velocity += (settings.gravity + b->force * b->invMass) * h;
        b->angularVelocity += b->torque * b->invInertia * h;
        b->velocity *= linDamp;
        b->angularVelocity *= angDamp;
    }
}

void World::detectContacts() {
    std::vector<Body*> sorted;
    sorted.reserve(bodies_.size());
    for (auto& b : bodies_) sorted.push_back(b.get());
    std::sort(sorted.begin(), sorted.end(), [](Body* a, Body* b) { return a->aabb.min.x < b->aabb.min.x; });

    std::unordered_map<uint64_t, Manifold> next;
    next.reserve(manifolds_.size());

    for (size_t i = 0; i < sorted.size(); ++i) {
        Body* a = sorted[i];
        for (size_t j = i + 1; j < sorted.size(); ++j) {
            Body* b = sorted[j];
            if (b->aabb.min.x > a->aabb.max.x) break;
            if (a->isStatic && b->isStatic) continue;
            if (!a->aabb.overlaps(b->aabb)) continue;
            if (!jointPairs_.empty() && jointPairs_.contains(pairKey(a, b))) continue;

            Manifold m;
            if (!(a->id < b->id ? collide(a, b, m) : collide(b, a, m))) continue;

            const uint64_t key = pairKey(a, b);
            if (auto old = manifolds_.find(key); old != manifolds_.end()) {
                for (int k = 0; k < m.count; ++k) {
                    for (int o = 0; o < old->second.count; ++o) {
                        const auto& oc = old->second.contacts[o];
                        if (oc.feature == m.contacts[k].feature) {
                            m.contacts[k].normalImpulse = oc.normalImpulse;
                            m.contacts[k].tangentImpulse = oc.tangentImpulse;
                            break;
                        }
                    }
                }
            }
            next.emplace(key, m);
        }
    }
    manifolds_ = std::move(next);
}

void World::prepareContacts(float h) {
    contactSoft_ = Softness::make(std::min(settings.contactHertz, 0.25f / h), settings.contactDampingRatio, h);
    for (auto& [key, m] : manifolds_) {
        Body* a = m.a;
        Body* b = m.b;
        const Vec2 n = m.normal;
        const Vec2 t = cross(n, 1.0f);
        for (int i = 0; i < m.count; ++i) {
            auto& c = m.contacts[i];
            c.rA = c.point - a->position;
            c.rB = c.point - b->position;

            float rnA = cross(c.rA, n), rnB = cross(c.rB, n);
            c.normalMass = 1.0f / (a->invMass + b->invMass + a->invInertia * rnA * rnA + b->invInertia * rnB * rnB);
            float rtA = cross(c.rA, t), rtB = cross(c.rB, t);
            c.tangentMass = 1.0f / (a->invMass + b->invMass + a->invInertia * rtA * rtA + b->invInertia * rtB * rtB);

            c.relativeVelocity = dot(b->velocityAt(c.point) - a->velocityAt(c.point), n);
            c.bias = std::min(contactSoft_.biasRate * std::max(0.0f, c.depth - settings.penetrationSlop),
                              settings.maxCorrectionVelocity);

            Vec2 p = n * c.normalImpulse + t * c.tangentImpulse;
            a->applyImpulse(-p, c.rA);
            b->applyImpulse(p, c.rB);
        }
    }
}

void World::solveJoints(bool useBias) {
    for (auto& j : joints_) j->solve(useBias);
}

void World::applyRestitution() {
    for (auto& [key, m] : manifolds_) {
        Body* a = m.a;
        Body* b = m.b;
        const float e = std::max(a->restitution, b->restitution);
        if (e == 0.0f) continue;
        for (int i = 0; i < m.count; ++i) {
            auto& c = m.contacts[i];
            if (c.relativeVelocity > -settings.restitutionThreshold || c.normalImpulse == 0.0f) continue;
            Vec2 dv = b->velocity + cross(b->angularVelocity, c.rB) - a->velocity - cross(a->angularVelocity, c.rA);
            float vn = dot(dv, m.normal);
            float old = c.normalImpulse;
            c.normalImpulse = std::max(old - c.normalMass * (vn + e * c.relativeVelocity), 0.0f);
            Vec2 p = m.normal * (c.normalImpulse - old);
            a->applyImpulse(-p, c.rA);
            b->applyImpulse(p, c.rB);
        }
    }
}

void World::solveContacts(bool useBias) {
    for (auto& [key, m] : manifolds_) {
        Body* a = m.a;
        Body* b = m.b;
        const Vec2 n = m.normal;
        const Vec2 t = cross(n, 1.0f);
        const float mu = std::sqrt(a->friction * b->friction);

        for (int i = 0; i < m.count; ++i) {
            auto& c = m.contacts[i];

            Vec2 dv = b->velocity + cross(b->angularVelocity, c.rB) - a->velocity - cross(a->angularVelocity, c.rA);
            float lambda = -c.tangentMass * dot(dv, t);
            float maxF = mu * c.normalImpulse;
            float old = c.tangentImpulse;
            c.tangentImpulse = std::clamp(old + lambda, -maxF, maxF);
            Vec2 p = t * (c.tangentImpulse - old);
            a->applyImpulse(-p, c.rA);
            b->applyImpulse(p, c.rB);

            dv = b->velocity + cross(b->angularVelocity, c.rB) - a->velocity - cross(a->angularVelocity, c.rA);
            old = c.normalImpulse;
            if (useBias)
                lambda = contactSoft_.massScale * c.normalMass * (c.bias - dot(dv, n)) - contactSoft_.impulseScale * old;
            else
                lambda = -c.normalMass * dot(dv, n);
            c.normalImpulse = std::max(old + lambda, 0.0f);
            p = n * (c.normalImpulse - old);
            a->applyImpulse(-p, c.rA);
            b->applyImpulse(p, c.rB);
        }
    }
}

void World::integratePositions(float h) {
    for (auto& b : bodies_) {
        if (b->isStatic) continue;
        b->position += b->velocity * h;
        b->angle += b->angularVelocity * h;
        b->updateTransform();
    }
}

} // namespace pe
