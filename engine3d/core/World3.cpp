#include "core/World3.h"
#include <numbers>
#include <numeric>

namespace pe3 {

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

// impulse lambda along the normal, using the angular responses cached in prepareContacts
void applyNormal(Body* a, Body* b, const Manifold& m, const ContactPoint& c, float lambda) {
    const Vec3 p = m.normal * lambda;
    a->velocity -= p * a->invMass;
    a->angularVelocity -= c.angA[0] * lambda;
    b->velocity += p * b->invMass;
    b->angularVelocity += c.angB[0] * lambda;
}

Vec3 relativeVelocity(const Body* a, const Body* b, const ContactPoint& c) {
    return b->velocity + cross(b->angularVelocity, c.rB) - a->velocity - cross(a->angularVelocity, c.rA);
}

// Implicit gyroscopic torque (Catto, GDC 2015): one Newton step in body space keeps the
// spin of asymmetric bodies stable while conserving angular momentum.
Vec3 gyroscopic(const Body& b, float h) {
    const Vec3 inv = b.invInertiaLocal;
    const Vec3 I{1.0f / inv.x, 1.0f / inv.y, 1.0f / inv.z};
    const Vec3 w = b.rotation.mulT(b.angularVelocity);
    const Vec3 Iw = mul(I, w);
    const Vec3 f = cross(w, Iw) * h;
    const Mat3 J = Mat3::diagonal(I) + (Mat3::skew(w) * Mat3::diagonal(I) + Mat3::skew(Iw) * -1.0f) * h;
    return b.rotation * (w - J.solve(f));
}

bool rayBox(Vec3 o, Vec3 d, Vec3 e, float& t, Vec3& n) {
    float tMin = -1e30f, tMax = 1e30f;
    int axis = -1;
    float sign = 1.0f;
    for (int i = 0; i < 3; ++i) {
        if (std::abs(d[i]) < 1e-9f) {
            if (std::abs(o[i]) > e[i]) return false;
            continue;
        }
        float t1 = (-e[i] - o[i]) / d[i], t2 = (e[i] - o[i]) / d[i];
        float s = -1.0f;
        if (t1 > t2) std::swap(t1, t2), s = 1.0f;
        if (t1 > tMin) tMin = t1, axis = i, sign = s;
        tMax = std::min(tMax, t2);
        if (tMin > tMax) return false;
    }
    if (tMin < 0.0f || axis < 0) return false;
    t = tMin;
    n = {};
    n[axis] = sign;
    return true;
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
    wake(body);
    const Vec3 pad{0.05f, 0.05f, 0.05f};
    const Aabb region{body->aabb.min - pad, body->aabb.max + pad};
    for (auto& b : bodies_)
        if (!b->awake && region.overlaps(b->aabb)) wake(b.get());
    std::erase_if(joints_, [this, body](const auto& j) {
        if (j->a != body && j->b != body) return false;
        wake(j->a == body ? j->b : j->a);
        return true;
    });
    rebuildJointPairs();
    std::erase_if(manifolds_, [body](const Manifold& m) { return m.a == body || m.b == body; });
    std::erase_if(bodies_, [body](const auto& p) { return p.get() == body; });
    sortedDirty_ = true;
}

void World::destroyJoint(Joint* joint) {
    wake(joint->a);
    wake(joint->b);
    std::erase_if(joints_, [joint](const auto& j) { return j.get() == joint; });
    rebuildJointPairs();
}

void World::rebuildJointPairs() {
    jointPairs_.clear();
    for (const auto& j : joints_) {
        if (!j->collideConnected) jointPairs_.insert(pairKey(j->a, j->b));
        else wake(j->a), wake(j->b); // a pair that may now collide must be checked again
    }
}

void World::clear() {
    joints_.clear();
    jointPairs_.clear();
    manifolds_.clear();
    islands_.clear();
    bodies_.clear();
    sorted_.clear();
    sortedDirty_ = true;
}

RayHit World::raycast(Vec3 origin, Vec3 dir, float maxT) const {
    RayHit best;
    best.t = maxT;
    for (const auto& b : bodies_) {
        float t;
        Vec3 n;
        if (b->shape.type == ShapeType::Sphere) {
            Vec3 m = origin - b->position;
            float bq = dot(m, dir), c = m.lengthSq() - b->shape.radius * b->shape.radius;
            float disc = bq * bq - c;
            if (disc < 0.0f) continue;
            t = -bq - std::sqrt(disc);
            if (t < 0.0f) continue;
            n = (origin + dir * t - b->position).normalized();
        } else {
            if (!rayBox(b->toLocal(origin), b->rotation.mulT(dir), b->shape.halfExtents, t, n)) continue;
            n = b->rotation * n;
        }
        if (t < best.t) best = {b.get(), t, origin + dir * t, n};
    }
    return best;
}

void World::wake(Body* body) {
    if (body->awake || body->isStatic) return;
    if (body->island < 0) {
        body->awake = true;
        body->sleepTime = 0.0f;
        return;
    }
    auto& members = islands_[body->island];
    for (Body* b : members) {
        b->awake = true;
        b->sleepTime = 0.0f;
        b->island = -1;
    }
    members.clear();
}

int World::awakeCount() const {
    int n = 0;
    for (const auto& b : bodies_) n += !b->isStatic && b->awake;
    return n;
}

void World::wakeDisturbed() {
    for (auto& b : bodies_) {
        if (b->awake || b->isStatic) continue;
        const bool pushed = b->force.lengthSq() > 0.0f || b->torque.lengthSq() > 0.0f;
        const bool moved = b->velocity.lengthSq() > 0.0f || b->angularVelocity.lengthSq() > 0.0f;
        if (pushed || moved || !settings.enableSleep) wake(b.get());
    }
    for (auto& j : joints_) {
        const bool activeA = !j->a->isStatic && j->a->awake, activeB = !j->b->isStatic && j->b->awake;
        if (activeA || activeB || j->keepsAwake()) wake(j->a), wake(j->b);
    }
}

void World::updateSleep(float dt) {
    if (!settings.enableSleep) return;
    int n = 0;
    for (auto& b : bodies_) b->node = !b->isStatic && b->awake ? n++ : -1;
    parent_.resize(n);
    for (int i = 0; i < n; ++i) parent_[i] = i;
    auto find = [this](int i) {
        while (parent_[i] != i) i = parent_[i] = parent_[parent_[i]];
        return i;
    };

    for (const Manifold& m : manifolds_) {
        // speculative contacts count too: anything that could wake a body must share its island
        if (m.a->node >= 0 && m.b->node >= 0) parent_[find(m.a->node)] = find(m.b->node);
    }

    for (const auto& j : joints_) {
        if (j->keepsAwake()) {
            j->a->sleepTime = j->b->sleepTime = -1e30f;
            continue;
        }
        if (j->a->node >= 0 && j->b->node >= 0) parent_[find(j->a->node)] = find(j->b->node);
    }

    auto& minSleep = minSleep_;
    minSleep.assign(n, 1e30f);
    for (auto& b : bodies_) {
        if (b->node < 0) continue;
        const float speed = std::max(b->velocity.length(), b->angularVelocity.length() * b->shape.radius);
        b->sleepTime = speed > settings.sleepThreshold || b->sleepTime < 0.0f ? 0.0f : b->sleepTime + dt;
        float& s = minSleep[find(b->node)];
        s = std::min(s, b->sleepTime);
    }

    auto& islandOf = islandOf_;
    islandOf.assign(n, -1);
    for (auto& b : bodies_) {
        if (b->node < 0) continue;
        const int root = find(b->node);
        if (minSleep[root] < settings.timeToSleep) continue;
        if (islandOf[root] < 0) {
            auto slot = std::find_if(islands_.begin(), islands_.end(), [](const auto& v) { return v.empty(); });
            if (slot == islands_.end()) slot = islands_.emplace(islands_.end());
            islandOf[root] = static_cast<int>(slot - islands_.begin());
        }
        b->awake = false;
        b->island = islandOf[root];
        b->velocity = b->angularVelocity = {};
        islands_[b->island].push_back(b.get());
    }
}

void World::detectUntilStable() {
    for (int pass = 0; pass < 4 && detectContacts(); ++pass) {
    }
}

void World::step(float dt) {
    if (!(dt > 0.0f) || !std::isfinite(dt)) return; // no time passes, and 1/h would poison everything
    wakeDisturbed();
    const int substeps = std::max(1, settings.substeps);
    const float h = dt / static_cast<float>(substeps);
    for (int s = 0; s < substeps; ++s) {
        integrateVelocities(h);
        detectUntilStable();
        prepareContacts(h);
        for (auto& j : joints_)
            if (jointActive(*j)) j->prepare(h);
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
    }
    for (auto& b : bodies_) {
        b->force = {};
        b->torque = {};
    }
    updateSleep(dt);
}

bool World::jointActive(const Joint& j) const {
    return (!j.a->isStatic && j.a->awake) || (!j.b->isStatic && j.b->awake);
}

void World::solveJoints(bool useBias) {
    for (auto& j : joints_)
        if (jointActive(*j)) j->solve(useBias);
}

void World::integrateVelocities(float h) {
    const float linDamp = 1.0f / (1.0f + h * settings.linearDamping);
    const float angDamp = 1.0f / (1.0f + h * settings.angularDamping);
    for (auto& b : bodies_) {
        if (b->isStatic || !b->awake) continue;
        b->velocity += (settings.gravity + b->force * b->invMass) * h;
        b->angularVelocity += b->invInertiaWorld * b->torque * h;
        b->angularVelocity = gyroscopic(*b, h);
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

bool World::detectContacts() {
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
    bool woke = false;

    const float margin = settings.speculativeDistance;
    const Vec3 pad{margin, margin, margin};
    auto& next = nextManifolds_;
    next.clear();

    for (size_t i = 0; i < sorted.size(); ++i) {
        Body* a = sorted[i];
        const Aabb box{a->aabb.min - pad, a->aabb.max + pad};
        const bool activeA = !a->isStatic && a->awake;
        for (size_t j = i + 1; j < sorted.size(); ++j) {
            Body* b = sorted[j];
            if (b->aabb.min.x > box.max.x) break;
            const bool activeB = !b->isStatic && b->awake;
            if (!activeA && !activeB) continue;
            if (a->group != 0 && a->group == b->group) continue;
            if (!box.overlaps(b->aabb)) continue;
            const uint64_t key = pairKey(a, b);
            if (!jointPairs_.empty() && jointPairs_.contains(key)) continue;

            Body* first = a->id < b->id ? a : b;
            Body* second = first == a ? b : a;
            Manifold& m = next.emplace_back();
            if (!collide(first, second, margin, m)) {
                next.pop_back();
                continue;
            }
            m.key = key;
            if (!a->isStatic && !a->awake) wake(a), woke = true;
            if (!b->isStatic && !b->awake) wake(b), woke = true;

            if (const Manifold* old = findManifold(key)) {
                for (int k = 0; k < m.count; ++k) {
                    auto& c = m.contacts[k];
                    for (int o = 0; o < old->count; ++o) {
                        const auto& oc = old->contacts[o];
                        if ((oc.localA - c.localA).lengthSq() < 0.05f * 0.05f) {
                            c.normalImpulse = oc.normalImpulse;
                            // re-project friction into the new tangent basis
                            Vec3 ft = old->tangent[0] * oc.tangentImpulse[0] + old->tangent[1] * oc.tangentImpulse[1];
                            c.tangentImpulse[0] = dot(ft, m.tangent[0]);
                            c.tangentImpulse[1] = dot(ft, m.tangent[1]);
                            break;
                        }
                    }
                }
            }
            if (first->shape.type == ShapeType::Sphere || second->shape.type == ShapeType::Sphere) {
                float r = 1e30f;
                for (Body* s : {first, second})
                    if (s->shape.type == ShapeType::Sphere) r = std::min(r, s->shape.radius);
                m.rollingRadius = r * std::max(first->rollingResistance, second->rollingResistance);
            }
        }
    }
    spreadOut(next, manifolds_);
    return woke;
}

void World::prepareContacts(float h) {
    const float hertz = std::min(settings.contactHertz, 0.25f / h);
    const float omega = 2.0f * std::numbers::pi_v<float> * hertz;
    const float zeta = settings.contactDampingRatio;
    const float a1 = 2.0f * zeta + h * omega;
    const float a2 = h * omega * a1;
    biasRate_ = omega / a1;
    massScale_ = a2 / (1.0f + a2);
    impulseScale_ = 1.0f / (1.0f + a2);
    invH_ = 1.0f / h;

    for (Manifold& m : manifolds_) {
        Body* a = m.a;
        Body* b = m.b;
        if (m.rollingRadius > 0.0f) {
            auto trace = [](const Mat3& I) { return I.c[0].x + I.c[1].y + I.c[2].z; };
            const float k = (trace(a->invInertiaWorld) + trace(b->invInertiaWorld)) / 3.0f;
            m.rollingMass = k > 0.0f ? 1.0f / k : 0.0f;
        }
        const Vec3 dirs[3] = {m.normal, m.tangent[0], m.tangent[1]};
        for (int i = 0; i < m.count; ++i) {
            auto& c = m.contacts[i];
            c.rA = c.point - a->position;
            c.rB = c.point - b->position;
            float* masses[3] = {&c.normalMass, &c.tangentMass[0], &c.tangentMass[1]};
            for (int k = 0; k < 3; ++k) {
                const Vec3 ca = cross(c.rA, dirs[k]), cb = cross(c.rB, dirs[k]);
                c.angA[k] = a->invInertiaWorld * ca;
                c.angB[k] = b->invInertiaWorld * cb;
                const float invK = a->invMass + b->invMass + dot(ca, c.angA[k]) + dot(cb, c.angB[k]);
                *masses[k] = invK > 0.0f ? 1.0f / invK : 0.0f;
            }
            c.relativeVelocity = dot(relativeVelocity(a, b, c), m.normal);
        }
    }

    // separate pass: approach speeds above must not see other contacts' warm-start impulses,
    // or restitution mistakes a loaded resting contact for an impact
    for (Manifold& m : manifolds_) {
        for (int i = 0; i < m.count; ++i) {
            const auto& c = m.contacts[i];
            const float n = c.normalImpulse, t0 = c.tangentImpulse[0], t1 = c.tangentImpulse[1];
            const Vec3 p = m.normal * n + m.tangent[0] * t0 + m.tangent[1] * t1;
            m.a->velocity -= p * m.a->invMass;
            m.a->angularVelocity -= c.angA[0] * n + c.angA[1] * t0 + c.angA[2] * t1;
            m.b->velocity += p * m.b->invMass;
            m.b->angularVelocity += c.angB[0] * n + c.angB[1] * t0 + c.angB[2] * t1;
        }
    }
}

void World::solveContacts(bool useBias) {
    for (Manifold& m : manifolds_) {
        Body* a = m.a;
        Body* b = m.b;
        const float mu = std::sqrt(a->friction * b->friction);

        if (m.rollingRadius > 0.0f) {
            float total = 0.0f;
            for (int i = 0; i < m.count; ++i) total += m.contacts[i].normalImpulse;
            const float maxR = m.rollingRadius * total;
            const Vec3 old = m.rollingImpulse;
            Vec3 next = old - (b->angularVelocity - a->angularVelocity) * m.rollingMass;
            if (const float len = next.length(); len > maxR) next *= maxR / len;
            m.rollingImpulse = next;
            const Vec3 d = next - old;
            a->angularVelocity -= a->invInertiaWorld * d;
            b->angularVelocity += b->invInertiaWorld * d;
        }

        for (int i = 0; i < m.count; ++i) {
            auto& c = m.contacts[i];

            Vec3 dv = relativeVelocity(a, b, c);
            const float maxF = mu * c.normalImpulse;
            const float old0 = c.tangentImpulse[0], old1 = c.tangentImpulse[1];
            float t0 = old0 - c.tangentMass[0] * dot(dv, m.tangent[0]);
            float t1 = old1 - c.tangentMass[1] * dot(dv, m.tangent[1]);
            const float len = std::sqrt(t0 * t0 + t1 * t1);
            if (len > maxF) {
                const float s = len > 0.0f ? maxF / len : 0.0f;
                t0 *= s;
                t1 *= s;
            }
            c.tangentImpulse[0] = t0;
            c.tangentImpulse[1] = t1;
            const float d0 = t0 - old0, d1 = t1 - old1;
            Vec3 p = m.tangent[0] * d0 + m.tangent[1] * d1;
            a->velocity -= p * a->invMass;
            a->angularVelocity -= c.angA[1] * d0 + c.angA[2] * d1;
            b->velocity += p * b->invMass;
            b->angularVelocity += c.angB[1] * d0 + c.angB[2] * d1;

            dv = relativeVelocity(a, b, c);
            const float vn = dot(dv, m.normal);
            const float old = c.normalImpulse;
            float lambda;
            if (c.depth < 0.0f) {
                lambda = -c.normalMass * (vn - c.depth * invH_); // speculative: may close the gap, not cross it
            } else if (useBias) {
                const float bias = std::min(biasRate_ * std::max(0.0f, c.depth - settings.penetrationSlop),
                                            settings.maxCorrectionVelocity);
                lambda = massScale_ * c.normalMass * (bias - vn) - impulseScale_ * old;
            } else {
                lambda = -c.normalMass * vn;
            }
            c.normalImpulse = std::max(old + lambda, 0.0f);
            applyNormal(a, b, m, c, c.normalImpulse - old);
        }
    }
}

void World::applyRestitution() {
    for (int pass = 0; pass < 4; ++pass) {
        for (Manifold& m : manifolds_) {
            Body* a = m.a;
            Body* b = m.b;
            const float e = std::max(a->restitution, b->restitution);
            if (e == 0.0f) continue;
            for (int i = 0; i < m.count; ++i) {
                auto& c = m.contacts[i];
                if (c.relativeVelocity > -settings.restitutionThreshold || c.normalImpulse == 0.0f) continue;
                float vn = dot(relativeVelocity(a, b, c), m.normal);
                float old = c.normalImpulse;
                c.normalImpulse = std::max(old - c.normalMass * (vn + e * c.relativeVelocity), 0.0f);
                applyNormal(a, b, m, c, c.normalImpulse - old);
            }
        }
    }
}

void World::integratePositions(float h) {
    for (auto& b : bodies_) {
        if (b->isStatic || !b->awake) continue;
        b->position += b->velocity * h;
        b->orientation = b->orientation.integrated(b->angularVelocity, h);
        b->updateTransform();
    }
}

} // namespace pe3
