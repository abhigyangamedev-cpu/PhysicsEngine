#include "soft/ParticleSystem.h"
#include "core/ParticleContact.h"
#include <algorithm>
#include <cmath>

namespace pe {

namespace {

uint32_t hashCell(int x, int y, uint32_t tableSize) {
    uint32_t h = (static_cast<uint32_t>(x) * 92837111u) ^ (static_cast<uint32_t>(y) * 689287499u);
    return h % tableSize;
}

} // namespace

uint32_t ParticleSystem::addParticle(Vec2 position, float mass, float radius, int group) {
    Particle p;
    p.position = p.previous = position;
    p.invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
    p.radius = radius;
    p.group = group;
    particles.push_back(p);
    return static_cast<uint32_t>(particles.size() - 1);
}

DistanceConstraint& ParticleSystem::link(uint32_t a, uint32_t b, float compliance, float tearRatio) {
    float len = (particles[b].position - particles[a].position).length();
    return links.emplace_back(DistanceConstraint{a, b, len, compliance, tearRatio});
}

PressureConstraint& ParticleSystem::addPressure(std::vector<uint32_t> ring, float pressure, float compliance) {
    PressureConstraint pc{std::move(ring), 0.0f, pressure, compliance};
    pc.restArea = area(pc);
    return pressures.emplace_back(std::move(pc));
}

void ParticleSystem::attach(uint32_t particle, Body* body, float compliance) {
    Vec2 local = body->rot.applyT(particles[particle].position - body->position);
    attachments.push_back({particle, body, local, compliance});
}

void ParticleSystem::removeBody(const Body* body) {
    std::erase_if(attachments, [body](const Attachment& a) { return a.body == body; });
    std::erase_if(carried_, [body](const Carried& c) { return c.body == body; });
}

void ParticleSystem::clear() {
    particles.clear();
    links.clear();
    pressures.clear();
    attachments.clear();
    carried_.clear();
    begun_ = false;
    groupCounter_ = 0;
}

float ParticleSystem::area(const PressureConstraint& pc) const {
    float a = 0.0f;
    const size_t n = pc.ring.size();
    for (size_t i = 0; i < n; ++i)
        a += cross(particles[pc.ring[i]].position, particles[pc.ring[(i + 1) % n]].position);
    return 0.5f * a;
}

int ParticleSystem::nearest(Vec2 point, float maxDistance) const {
    int best = -1;
    float bestSq = maxDistance * maxDistance;
    for (size_t i = 0; i < particles.size(); ++i) {
        float dSq = (particles[i].position - point).lengthSq();
        if (dSq < bestSq) { bestSq = dSq; best = static_cast<int>(i); }
    }
    return best;
}

void ParticleSystem::beginBodyStep() {
    carried_.clear();
    for (const auto& at : attachments) {
        Body* b = at.body;
        if (b->isStatic) continue;
        if (std::none_of(carried_.begin(), carried_.end(), [b](const Carried& c) { return c.body == b; }))
            carried_.push_back({b, b->position, b->angle, {}, 0.0f});
    }
    begun_ = true;
}

void ParticleSystem::step(float h, Vec2 gravity, const std::vector<std::unique_ptr<Body>>& bodies, int iterations) {
    if (!begun_) beginBodyStep(); // nobody moved the bodies: they start where they are
    begun_ = false;
    if (particles.empty()) return;
    iterations = std::max(1, iterations);
    const float ph = h / static_cast<float>(iterations);
    // rewind to the start of the step and split the rigid motion over the particle steps, so a
    // correction made in one of them is in time with the particle it pulls on
    for (auto& c : carried_) {
        Body& b = *c.body;
        std::swap(b.position, c.position);
        std::swap(b.angle, c.angle);
        c.position = (c.position - b.position) / static_cast<float>(iterations);
        c.angle = (c.angle - b.angle) / static_cast<float>(iterations);
        c.velocity = b.velocity;
        c.angularVelocity = b.angularVelocity;
    }
    for (int i = 0; i < iterations; ++i) {
        // the rigid motion, plus whatever velocity the attachments have added so far
        for (const auto& c : carried_) {
            Body& b = *c.body;
            b.position += c.position + (b.velocity - c.velocity) * ph;
            b.angle += c.angle + (b.angularVelocity - c.angularVelocity) * ph;
            b.updateTransform();
        }
        substep(ph, gravity, bodies);
    }
}

void ParticleSystem::substep(float h, Vec2 gravity, const std::vector<std::unique_ptr<Body>>& bodies) {
    const float damp = 1.0f / (1.0f + h * damping);
    for (auto& p : particles) {
        p.previous = p.position;
        if (p.invMass == 0.0f) continue;
        p.velocity = (p.velocity + gravity * h) * damp;
        p.position += p.velocity * h;
    }
    predicted_.resize(particles.size());
    for (size_t i = 0; i < particles.size(); ++i) predicted_[i] = particles[i].position;

    const float alphaScale = 1.0f / (h * h);
    solveLinks(alphaScale);
    solvePressures(alphaScale);
    solveAttachments(h, alphaScale);
    if (selfCollision) collideParticles();
    collideBodies(h, bodies);

    const float invH = 1.0f / h;
    // only the constraints' corrections change the velocity: re-deriving all of it from positions loses
    // the gravity increment to rounding once a particle is far from the origin and moving fast
    for (size_t i = 0; i < particles.size(); ++i) {
        Particle& p = particles[i];
        p.velocity = p.invMass > 0.0f ? p.velocity + (p.position - predicted_[i]) * invH : Vec2{};
    }
}

void ParticleSystem::solveLinks(float alphaScale) {
    for (auto& c : links) {
        if (c.broken) continue;
        Particle& a = particles[c.a];
        Particle& b = particles[c.b];
        float w = a.invMass + b.invMass;
        if (w == 0.0f) continue;

        Vec2 d = b.position - a.position;
        float len = d.length();
        if (len < 1e-6f) continue;
        if (c.tearRatio > 0.0f && len > c.restLength * c.tearRatio) {
            c.broken = true;
            continue;
        }
        Vec2 n = d / len;
        float lambda = -(len - c.restLength) / (w + c.compliance * alphaScale);
        a.position -= n * (lambda * a.invMass);
        b.position += n * (lambda * b.invMass);
    }
}

void ParticleSystem::solvePressures(float alphaScale) {
    for (auto& pc : pressures) {
        const size_t n = pc.ring.size();
        float C = area(pc) - pc.restArea * pc.pressure;

        float denom = pc.compliance * alphaScale;
        thread_local std::vector<Vec2> grads;
        grads.resize(n);
        for (size_t i = 0; i < n; ++i) {
            Vec2 next = particles[pc.ring[(i + 1) % n]].position;
            Vec2 prev = particles[pc.ring[(i + n - 1) % n]].position;
            grads[i] = Vec2{next.y - prev.y, prev.x - next.x} * 0.5f;
            denom += particles[pc.ring[i]].invMass * grads[i].lengthSq();
        }
        if (denom == 0.0f) continue;
        float lambda = -C / denom;
        for (size_t i = 0; i < n; ++i) {
            Particle& p = particles[pc.ring[i]];
            p.position += grads[i] * (lambda * p.invMass);
        }
    }
}

void ParticleSystem::solveAttachments(float h, float alphaScale) {
    for (auto& at : attachments) {
        Particle& p = particles[at.particle];
        Body& b = *at.body;
        Vec2 r = b.rot.apply(at.localAnchor);
        Vec2 d = b.position + r - p.position;
        float len = d.length();
        if (len < 1e-6f) continue;
        Vec2 n = d / len;
        float rn = cross(r, n);
        float wb = b.invMass + b.invInertia * rn * rn;
        float w = p.invMass + wb;
        if (w == 0.0f) continue;
        float lambda = len / (w + at.compliance * alphaScale);
        p.position += n * (lambda * p.invMass);
        if (b.isStatic) continue;
        // move the body now so later iterations see the correction instead of stacking impulses on it,
        // and give it the same momentum the particle gets from its side of the correction
        Vec2 j = -n * lambda;
        b.position += j * b.invMass;
        b.angle += cross(r, j) * b.invInertia;
        b.updateTransform();
        b.applyImpulse(j / h, r);
    }
}

void ParticleSystem::collideParticles() {
    const uint32_t n = static_cast<uint32_t>(particles.size());
    float maxR = 0.0f;
    for (const auto& p : particles) maxR = std::max(maxR, p.radius);
    const float cell = 2.0f * maxR;
    if (cell <= 0.0f) return;
    const float invCell = 1.0f / cell;
    const uint32_t tableSize = 2 * n;

    auto cellOf = [invCell](Vec2 p) {
        return std::pair{static_cast<int>(std::floor(p.x * invCell)), static_cast<int>(std::floor(p.y * invCell))};
    };

    cellStart_.assign(tableSize + 1, 0);
    cellEntries_.resize(n);
    for (const auto& p : particles) {
        auto [x, y] = cellOf(p.position);
        ++cellStart_[hashCell(x, y, tableSize)];
    }
    uint32_t sum = 0;
    for (uint32_t i = 0; i <= tableSize; ++i) {
        sum += cellStart_[i];
        cellStart_[i] = sum;
    }
    for (uint32_t i = 0; i < n; ++i) {
        auto [x, y] = cellOf(particles[i].position);
        cellEntries_[--cellStart_[hashCell(x, y, tableSize)]] = i;
    }

    for (uint32_t i = 0; i < n; ++i) {
        auto [cx, cy] = cellOf(particles[i].position);
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                uint32_t hsh = hashCell(cx + dx, cy + dy, tableSize);
                for (uint32_t k = cellStart_[hsh]; k < cellStart_[hsh + 1]; ++k) {
                    uint32_t j = cellEntries_[k];
                    if (j <= i) continue;
                    Particle& a = particles[i];
                    Particle& b = particles[j];
                    if (a.group == b.group && a.group != 0) continue;
                    float w = a.invMass + b.invMass;
                    if (w == 0.0f) continue;
                    Vec2 d = b.position - a.position;
                    float R = a.radius + b.radius;
                    float distSq = d.lengthSq();
                    if (distSq >= R * R || distSq < 1e-12f) continue;
                    float dist = std::sqrt(distSq);
                    Vec2 corr = d * ((R - dist) / (dist * w));
                    a.position -= corr * a.invMass;
                    b.position += corr * b.invMass;
                }
            }
        }
    }
}

void ParticleSystem::collideBodies(float h, const std::vector<std::unique_ptr<Body>>& bodies) {
    auto attachedTo = [this](size_t particle, const Body* body) {
        for (const auto& at : attachments)
            if (at.particle == particle && at.body == body) return true;
        return false;
    };
    for (size_t i = 0; i < particles.size(); ++i) {
        Particle& p = particles[i];
        if (p.invMass == 0.0f) continue;
        Aabb box{p.position - Vec2{p.radius, p.radius}, p.position + Vec2{p.radius, p.radius}};
        for (const auto& bp : bodies) {
            Body& b = *bp;
            if (!b.collides || !b.aabb.overlaps(box)) continue;
            ParticleContact c;
            if (!particleContact(b, p.position, p.radius, c) || attachedTo(i, &b)) continue;

            Vec2 r = c.point - b.position;
            float rn = cross(r, c.normal);
            float wb = b.invMass + b.invInertia * rn * rn;
            float w = p.invMass + wb;
            Vec2 vb = b.velocityAt(c.point);
            float vn = dot((p.position - p.previous) * (1.0f / h) - vb, c.normal);
            p.position += c.normal * (c.depth * p.invMass / w);

            // static friction cancels tangential slip up to mu * normal correction
            Vec2 slip = (p.position - p.previous) - vb * h;
            Vec2 t = slip - c.normal * dot(slip, c.normal);
            float tLen = t.length();
            float mu = std::sqrt(p.friction * b.friction);
            if (tLen > 1e-7f) {
                float corr = std::min(tLen, mu * c.depth);
                p.position -= t * (corr / tLen * p.invMass / w);
            }

            // inelastic momentum transfer: only the approaching normal velocity is exchanged
            if (!b.isStatic && vn < 0.0f) b.applyImpulse(c.normal * (vn / w), r);
        }
    }
}

} // namespace pe
