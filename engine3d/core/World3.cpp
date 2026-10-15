#include "core/World3.h"
#include <numbers>

namespace pe3 {

namespace {

uint64_t pairKey(const Body* a, const Body* b) {
    uint32_t i = a->id, j = b->id;
    if (i > j) std::swap(i, j);
    return static_cast<uint64_t>(i) << 32 | j;
}

float effectiveMass(const Body* a, const Body* b, Vec3 rA, Vec3 rB, Vec3 dir) {
    Vec3 ca = cross(rA, dir), cb = cross(rB, dir);
    float k = a->invMass + b->invMass + dot(ca, a->invInertiaWorld * ca) + dot(cb, b->invInertiaWorld * cb);
    return k > 0.0f ? 1.0f / k : 0.0f;
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
    std::erase_if(manifolds_, [body](const auto& kv) { return kv.second.a == body || kv.second.b == body; });
    std::erase_if(bodies_, [body](const auto& p) { return p.get() == body; });
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

    for (const auto& [key, m] : manifolds_) {
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

    std::vector<float> minSleep(n, 1e30f);
    for (auto& b : bodies_) {
        if (b->node < 0) continue;
        const float speed = std::max(b->velocity.length(), b->angularVelocity.length() * b->shape.radius);
        b->sleepTime = speed > settings.sleepThreshold || b->sleepTime < 0.0f ? 0.0f : b->sleepTime + dt;
        float& s = minSleep[find(b->node)];
        s = std::min(s, b->sleepTime);
    }

    std::vector<int> islandOf(n, -1);
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
    wakeDisturbed();
    const float h = dt / static_cast<float>(settings.substeps);
    for (int s = 0; s < settings.substeps; ++s) {
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

bool World::detectContacts() {
    auto& sorted = sorted_;
    sorted.clear();
    for (auto& b : bodies_) sorted.push_back(b.get());
    std::sort(sorted.begin(), sorted.end(), [](Body* a, Body* b) { return a->aabb.min.x < b->aabb.min.x; });
    bool woke = false;

    const float margin = settings.speculativeDistance;
    const Vec3 pad{margin, margin, margin};
    auto& next = nextManifolds_;
    next.clear();

    for (size_t i = 0; i < sorted.size(); ++i) {
        Body* a = sorted[i];
        const Aabb box{a->aabb.min - pad, a->aabb.max + pad};
        for (size_t j = i + 1; j < sorted.size(); ++j) {
            Body* b = sorted[j];
            if (b->aabb.min.x > box.max.x) break;
            const bool activeA = !a->isStatic && a->awake, activeB = !b->isStatic && b->awake;
            if (!activeA && !activeB) continue;
            if (a->group != 0 && a->group == b->group) continue;
            if (!box.overlaps(b->aabb)) continue;
            if (!jointPairs_.empty() && jointPairs_.contains(pairKey(a, b))) continue;

            Body* first = a->id < b->id ? a : b;
            Body* second = first == a ? b : a;
            Manifold m;
            if (!collide(first, second, margin, m)) continue;
            if (!a->isStatic && !a->awake) wake(a), woke = true;
            if (!b->isStatic && !b->awake) wake(b), woke = true;

            const uint64_t key = pairKey(a, b);
            if (auto old = manifolds_.find(key); old != manifolds_.end()) {
                for (int k = 0; k < m.count; ++k) {
                    auto& c = m.contacts[k];
                    for (int o = 0; o < old->second.count; ++o) {
                        const auto& oc = old->second.contacts[o];
                        if ((oc.localA - c.localA).lengthSq() < 0.05f * 0.05f) {
                            c.normalImpulse = oc.normalImpulse;
                            // re-project friction into the new tangent basis
                            Vec3 ft = old->second.tangent[0] * oc.tangentImpulse[0] + old->second.tangent[1] * oc.tangentImpulse[1];
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
            next.emplace(key, m);
        }
    }
    std::swap(manifolds_, next);
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

    for (auto& [key, m] : manifolds_) {
        Body* a = m.a;
        Body* b = m.b;
        if (m.rollingRadius > 0.0f) {
            auto trace = [](const Mat3& I) { return I.c[0].x + I.c[1].y + I.c[2].z; };
            const float k = (trace(a->invInertiaWorld) + trace(b->invInertiaWorld)) / 3.0f;
            m.rollingMass = k > 0.0f ? 1.0f / k : 0.0f;
        }
        for (int i = 0; i < m.count; ++i) {
            auto& c = m.contacts[i];
            c.rA = c.point - a->position;
            c.rB = c.point - b->position;
            c.normalMass = effectiveMass(a, b, c.rA, c.rB, m.normal);
            c.tangentMass[0] = effectiveMass(a, b, c.rA, c.rB, m.tangent[0]);
            c.tangentMass[1] = effectiveMass(a, b, c.rA, c.rB, m.tangent[1]);
            c.relativeVelocity = dot(relativeVelocity(a, b, c), m.normal);
        }
    }

    // separate pass: approach speeds above must not see other contacts' warm-start impulses,
    // or restitution mistakes a loaded resting contact for an impact
    for (auto& [key, m] : manifolds_) {
        for (int i = 0; i < m.count; ++i) {
            const auto& c = m.contacts[i];
            Vec3 p = m.normal * c.normalImpulse + m.tangent[0] * c.tangentImpulse[0] + m.tangent[1] * c.tangentImpulse[1];
            m.a->applyImpulse(-p, c.rA);
            m.b->applyImpulse(p, c.rB);
        }
    }
}

void World::solveContacts(bool useBias) {
    for (auto& [key, m] : manifolds_) {
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
            Vec3 p = m.tangent[0] * (t0 - old0) + m.tangent[1] * (t1 - old1);
            a->applyImpulse(-p, c.rA);
            b->applyImpulse(p, c.rB);

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
            p = m.normal * (c.normalImpulse - old);
            a->applyImpulse(-p, c.rA);
            b->applyImpulse(p, c.rB);
        }
    }
}

void World::applyRestitution() {
    for (int pass = 0; pass < 4; ++pass) {
        for (auto& [key, m] : manifolds_) {
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
                Vec3 p = m.normal * (c.normalImpulse - old);
                a->applyImpulse(-p, c.rA);
                b->applyImpulse(p, c.rB);
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
