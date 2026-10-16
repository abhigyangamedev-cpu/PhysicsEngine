#include "core/World.h"
#include <algorithm>
#include <cmath>
#include <numeric>

namespace pe {

namespace {

// Broadphase order puts contacts sharing a body next to each other, and solving them back to back
// stalls on each other's velocity writes. Visiting them with a golden-ratio stride keeps neighbours
// apart; the result lands in out.
void spreadOut(std::vector<Manifold>& in, std::vector<Manifold>& out) {
    const size_t n = in.size();
    if (n < 8) {
        std::swap(in, out);
        return;
    }
    size_t stride = n * 618 / 1000;
    while (std::gcd(stride, n) != 1) ++stride;
    out.resize(n);
    for (size_t i = 0, k = 0; i < n; ++i, k = (k + stride) % n) out[i] = in[k];
}

uint64_t pairKey(const Body* a, const Body* b) {
    uint32_t i = a->id, j = b->id;
    if (i > j) std::swap(i, j);
    return static_cast<uint64_t>(i) << 32 | j;
}

} // namespace

Body* World::createBody(const Shape& shape, const BodyDef& def) {
    auto body = std::make_unique<Body>(shape, def);
    body->id = nextId_++;
    sortedDirty_ = true;
    bodies_.push_back(std::move(body));
    return bodies_.back().get();
}

void World::destroyBody(Body* body) {
    std::erase_if(joints_, [body](const auto& j) { return j->a == body || j->b == body; });
    rebuildJointPairs();
    for (auto& f : fields_) f->removeBody(body);
    particles.removeBody(body);
    std::erase_if(manifolds_, [body](const Manifold& m) { return m.a == body || m.b == body; });
    std::erase_if(bodies_, [body](const auto& p) { return p.get() == body; });
    sortedDirty_ = true;
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
    fields_.clear();
    joints_.clear();
    jointPairs_.clear();
    particles.clear();
    fluid.clear();
    manifolds_.clear();
    bodies_.clear();
    sorted_.clear();
    sortedDirty_ = true;
}

Body* World::bodyAt(Vec2 point) const {
    for (const auto& b : bodies_)
        if ((!b->isStatic || b->draggable) && b->contains(point)) return b.get();
    return nullptr;
}

Vec2 World::electricFieldAt(Vec2 p) const {
    Vec2 E;
    for (const auto& f : fields_) E += f->electricField(*this, p);
    return E;
}

float World::magneticFieldAt(Vec2 p) const {
    float B = 0.0f;
    for (const auto& f : fields_) B += f->magneticField(p);
    return B;
}

void World::step(float dt) {
    if (!(dt > 0.0f) || !std::isfinite(dt)) return; // no time passes, and 1/h would poison everything
    const int substeps = std::max(1, settings.substeps);
    const float h = dt / static_cast<float>(substeps);
    for (int s = 0; s < substeps; ++s) {
        for (auto& f : fields_) f->apply(*this, h);
        integrateVelocities(h);
        detectContacts();
        prepareContacts(h);
        for (auto& j : joints_) j->prepare(h);
        for (int i = 0; i < settings.velocityIterations; ++i) {
            solveJoints(true);
            solveContacts(true);
        }
        particles.beginBodyStep();
        integratePositions(h);
        // remove the velocity injected by position correction so it doesn't become kinetic energy
        for (int i = 0; i < settings.relaxIterations; ++i) {
            solveJoints(false);
            solveContacts(false);
        }
        applyRestitution();

        particles.step(h, settings.gravity, bodies_, settings.particleIterations);
        // the fluid takes fewer, larger steps than the rigid solver
        const int every = std::max(1, substeps / std::max(1, fluid.stepsPerFrame));
        if ((s + 1) % every == 0) fluid.step(h * every, settings.gravity, bodies_);
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

void World::indexManifolds() {
    size_t size = 16;
    while (size < manifolds_.size() * 2) size *= 2;
    slotKeys_.assign(size, ~0ull);
    slotIndex_.resize(size);
    for (int i = 0; i < static_cast<int>(manifolds_.size()); ++i) {
        size_t s = (manifolds_[i].key * 0x9E3779B97F4A7C15ull) >> 32 & (size - 1);
        while (slotKeys_[s] != ~0ull) s = (s + 1) & (size - 1);
        slotKeys_[s] = manifolds_[i].key;
        slotIndex_[s] = i;
    }
}

const Manifold* World::findManifold(uint64_t key) const {
    const size_t mask = slotKeys_.size() - 1;
    for (size_t s = (key * 0x9E3779B97F4A7C15ull) >> 32 & mask; slotKeys_[s] != ~0ull; s = (s + 1) & mask)
        if (slotKeys_[s] == key) return &manifolds_[slotIndex_[s]];
    return nullptr;
}

void World::detectContacts() {
    // bodies barely move between substeps, so insertion sort on last order is close to linear
    auto& sorted = sorted_;
    if (sortedDirty_) {
        sorted.clear();
        for (auto& b : bodies_) sorted.push_back(b.get());
        sortedDirty_ = false;
    }
    for (size_t i = 1; i < sorted.size(); ++i) {
        Body* b = sorted[i];
        size_t j = i;
        for (; j > 0 && sorted[j - 1]->aabb.min.x > b->aabb.min.x; --j) sorted[j] = sorted[j - 1];
        sorted[j] = b;
    }
    indexManifolds();
    auto& next = nextManifolds_;
    next.clear();

    for (size_t i = 0; i < sorted.size(); ++i) {
        Body* a = sorted[i];
        for (size_t j = i + 1; j < sorted.size(); ++j) {
            Body* b = sorted[j];
            if (b->aabb.min.x > a->aabb.max.x) break;
            if ((a->isStatic && b->isStatic) || !a->collides || !b->collides) continue;
            if (a->group != 0 && a->group == b->group) continue;
            if (!a->aabb.overlaps(b->aabb)) continue;
            const uint64_t key = pairKey(a, b);
            if (!jointPairs_.empty() && jointPairs_.contains(key)) continue;

            Manifold& m = next.emplace_back();
            if (!(a->id < b->id ? collide(a, b, m) : collide(b, a, m))) {
                next.pop_back();
                continue;
            }
            m.key = key;
            if (const Manifold* old = findManifold(key)) {
                for (int k = 0; k < m.count; ++k) {
                    for (int o = 0; o < old->count; ++o) {
                        const auto& oc = old->contacts[o];
                        if (oc.feature == m.contacts[k].feature) {
                            m.contacts[k].normalImpulse = oc.normalImpulse;
                            m.contacts[k].tangentImpulse = oc.tangentImpulse;
                            break;
                        }
                    }
                }
            }
        }
    }
    spreadOut(next, manifolds_);
}

void World::prepareContacts(float h) {
    contactSoft_ = Softness::make(std::min(settings.contactHertz, 0.25f / h), settings.contactDampingRatio, h);
    for (Manifold& m : manifolds_) {
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
    // A few passes so two-point contacts (a box face) converge together; a single sequential
    // pass lets the first point's impulse spin the body and the second overshoot.
    for (int pass = 0; pass < 4; ++pass) {
    for (Manifold& m : manifolds_) {
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
}

void World::solveContacts(bool useBias) {
    for (Manifold& m : manifolds_) {
        Body* a = m.a;
        Body* b = m.b;
        const Vec2 n = m.normal;
        const Vec2 t = cross(n, 1.0f);
        const float mu = settings.minFriction ? std::min(a->friction, b->friction) : std::sqrt(a->friction * b->friction);

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
