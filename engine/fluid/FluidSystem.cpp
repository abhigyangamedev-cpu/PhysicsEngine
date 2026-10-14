#include "fluid/FluidSystem.h"
#include "core/ParticleContact.h"
#include <algorithm>
#include <cmath>
#include <numbers>

namespace pe {

namespace {

constexpr float kPi = std::numbers::pi_v<float>;

uint32_t hashCell(int x, int y, uint32_t tableSize) {
    uint32_t h = (static_cast<uint32_t>(x) * 92837111u) ^ (static_cast<uint32_t>(y) * 689287499u);
    return h % tableSize;
}

} // namespace

void FluidSystem::configure(float particleSpacing) {
    spacing = particleSpacing;
    h_ = 2.0f * spacing;
    poly6_ = 4.0f / (kPi * std::pow(h_, 8.0f));
    spikyGrad_ = -30.0f / (kPi * std::pow(h_, 5.0f));
    wDeltaQ_ = W(0.04f * h_ * h_);

    // rest density of a perfect lattice at the chosen spacing (unit particle mass)
    restDensity_ = 0.0f;
    for (int i = -4; i <= 4; ++i)
        for (int j = -4; j <= 4; ++j) restDensity_ += W((i * i + j * j) * spacing * spacing);

    // density from a solid half-plane at distance d, sampled as rows of virtual particles
    for (int k = 0; k <= kWallSamples; ++k) {
        float d = h_ * k / kWallSamples;
        float sum = 0.0f;
        for (int row = 0; row < 4; ++row) {
            float y = d + (row + 0.5f) * spacing;
            for (int col = -6; col <= 6; ++col)
                for (float shift : {0.0f, 0.5f}) sum += 0.5f * W((col + shift) * (col + shift) * spacing * spacing + y * y);
        }
        wallDensity_[k] = sum;
    }
    for (int k = 0; k <= kWallSamples; ++k) {
        int a = std::max(k - 1, 0), b = std::min(k + 1, kWallSamples);
        wallGradient_[k] = (wallDensity_[b] - wallDensity_[a]) / (h_ * (b - a) / kWallSamples);
    }

    restStiffness_ = 0.0f;
    for (int i = -4; i <= 4; ++i)
        for (int j = -4; j <= 4; ++j) {
            Vec2 d{i * spacing, j * spacing};
            restStiffness_ += (gradW(d, d.length()) / restDensity_).lengthSq();
        }
}

float FluidSystem::W(float r2) const {
    float x = h_ * h_ - r2;
    return x > 0.0f ? poly6_ * x * x * x : 0.0f;
}

Vec2 FluidSystem::gradW(Vec2 d, float r) const {
    if (r <= 1e-6f || r >= h_) return {};
    float x = h_ - r;
    return d * (spikyGrad_ * x * x / r);
}

float FluidSystem::wallLookup(const float* table, float distance) const {
    float x = std::clamp(distance / h_, 0.0f, 1.0f) * kWallSamples;
    int i = std::min(static_cast<int>(x), kWallSamples - 1);
    float t = x - i;
    return table[i] * (1.0f - t) + table[i + 1] * t;
}

void FluidSystem::addParticle(Vec2 p, Vec2 v, float visc, uint32_t c) {
    if (poly6_ == 0.0f) configure(spacing);
    position.push_back(p);
    velocity.push_back(v);
    viscosity.push_back(visc);
    color.push_back(c);
}

void FluidSystem::addBlock(Vec2 min, Vec2 max, float visc, uint32_t c) {
    if (poly6_ == 0.0f) configure(spacing);
    for (float y = min.y + 0.5f * spacing; y < max.y; y += spacing)
        for (float x = min.x + 0.5f * spacing; x < max.x; x += spacing)
            addParticle({x + 0.01f * spacing * std::sin(13.0f * y), y}, {}, visc, c);
}

void FluidSystem::stir(Vec2 center, float radius, Vec2 v) {
    for (size_t i = 0; i < size(); ++i) {
        float d = (position[i] - center).length();
        if (d < radius) {
            float w = 1.0f - d / radius;
            velocity[i] += (v - velocity[i]) * (0.25f * w);
        }
    }
}

void FluidSystem::removeOutside(const Aabb& box) {
    size_t out = 0;
    for (size_t i = 0; i < size(); ++i) {
        Vec2 p = position[i];
        if (p.x < box.min.x || p.x > box.max.x || p.y < box.min.y || p.y > box.max.y) continue;
        position[out] = position[i];
        velocity[out] = velocity[i];
        viscosity[out] = viscosity[i];
        color[out] = color[i];
        ++out;
    }
    position.resize(out);
    velocity.resize(out);
    viscosity.resize(out);
    color.resize(out);
}

void FluidSystem::clear() {
    position.clear();
    velocity.clear();
    viscosity.clear();
    color.clear();
}

void FluidSystem::findNeighbors() {
    const uint32_t n = static_cast<uint32_t>(size());
    const uint32_t tableSize = 2 * n + 1;
    const float inv = 1.0f / h_;
    auto cellOf = [inv](Vec2 p) {
        return std::pair{static_cast<int>(std::floor(p.x * inv)), static_cast<int>(std::floor(p.y * inv))};
    };

    cellStart_.assign(tableSize + 1, 0);
    cellEntries_.resize(n);
    for (uint32_t i = 0; i < n; ++i) {
        auto [x, y] = cellOf(predicted_[i]);
        ++cellStart_[hashCell(x, y, tableSize)];
    }
    uint32_t sum = 0;
    for (uint32_t i = 0; i <= tableSize; ++i) {
        sum += cellStart_[i];
        cellStart_[i] = sum;
    }
    for (uint32_t i = 0; i < n; ++i) {
        auto [x, y] = cellOf(predicted_[i]);
        cellEntries_[--cellStart_[hashCell(x, y, tableSize)]] = i;
    }

    const float h2 = h_ * h_;
    neighborStart_.resize(n + 1);
    neighbors_.clear();
    for (uint32_t i = 0; i < n; ++i) {
        neighborStart_[i] = static_cast<uint32_t>(neighbors_.size());
        auto [cx, cy] = cellOf(predicted_[i]);
        uint32_t visited[9];
        int visitedCount = 0;
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                uint32_t hsh = hashCell(cx + dx, cy + dy, tableSize);
                // two cells can hash to the same bucket; scan each bucket once
                if (std::find(visited, visited + visitedCount, hsh) != visited + visitedCount) continue;
                visited[visitedCount++] = hsh;
                for (uint32_t k = cellStart_[hsh]; k < cellStart_[hsh + 1]; ++k) {
                    uint32_t j = cellEntries_[k];
                    if ((predicted_[j] - predicted_[i]).lengthSq() < h2) neighbors_.push_back(j);
                }
            }
        }
    }
    neighborStart_[n] = static_cast<uint32_t>(neighbors_.size());
}

void FluidSystem::collide(float dt, const std::vector<Body*>& bodies, bool transferMomentum) {
    const float r = particleRadius();
    const float m = particleMass();
    for (size_t i = 0; i < size(); ++i) {
        Vec2& p = predicted_[i];
        Aabb box{p - Vec2{r, r}, p + Vec2{r, r}};
        for (Body* b : bodies) {
            if (!b->aabb.overlaps(box)) continue;
            ParticleContact c;
            if (!particleContact(*b, p, r, c)) continue;
            p += c.normal * c.depth;
            // the fluid was stopped from moving depth into the body: hand that momentum to the body
            if (transferMomentum && !b->isStatic) b->applyImpulse(-c.normal * (m * c.depth / dt), c.point - b->position);
        }
    }
}

void FluidSystem::step(float dt, Vec2 gravity, const std::vector<std::unique_ptr<Body>>& allBodies) {
    const size_t n = size();
    if (n == 0) return;
    if (poly6_ == 0.0f) configure(spacing);

    predicted_.resize(n);
    delta_.resize(n);
    omegaGrad_.resize(n);
    rho.resize(n);
    lambda_.resize(n);
    omega_.resize(n);
    walls_.resize(n * kMaxWalls);
    wallCount_.resize(n);
    const float mass = particleMass();

    thread_local std::vector<Body*> bodies;
    bodies.clear();
    for (const auto& b : allBodies)
        if (b->collides) bodies.push_back(b.get());

    const float maxSpeedSq = maxSpeed * maxSpeed;
    for (size_t i = 0; i < n; ++i) {
        velocity[i] += gravity * dt;
        if (velocity[i].lengthSq() > maxSpeedSq) velocity[i] = velocity[i].normalized() * maxSpeed;
        predicted_[i] = position[i] + velocity[i] * dt;
    }
    collide(dt, bodies, true);
    findNeighbors();

    const float invRho0 = 1.0f / restDensity_;
    const float eps = relaxation * restStiffness_;
    const float tensileScale = tensileStrength / restStiffness_;

    for (int it = 0; it < iterations; ++it) {
        for (size_t i = 0; i < n; ++i) {
            float density = 0.0f, sumSq = 0.0f;
            Vec2 gi;
            for (uint32_t k = neighborStart_[i]; k < neighborStart_[i + 1]; ++k) {
                uint32_t j = neighbors_[k];
                Vec2 d = predicted_[i] - predicted_[j];
                float r2 = d.lengthSq();
                density += W(r2);
                if (j == i) continue;
                Vec2 gw = gradW(d, std::sqrt(r2)) * invRho0;
                gi += gw;
                sumSq += gw.lengthSq();
            }
            int count = 0;
            Aabb box{predicted_[i] - Vec2{h_, h_}, predicted_[i] + Vec2{h_, h_}};
            for (Body* b : bodies) {
                if (count == kMaxWalls) break;
                if (!b->aabb.overlaps(box)) continue;
                ParticleContact c;
                if (!particleContact(*b, predicted_[i], h_, c)) continue;
                float dist = std::max(h_ - c.depth, 0.0f);
                density += wallLookup(wallDensity_, dist);
                float g = wallLookup(wallGradient_, dist) * invRho0; // d(C)/d(distance), negative
                gi += c.normal * g;
                walls_[i * kMaxWalls + count++] = {b, c.normal, c.point, g};
            }
            wallCount_[i] = static_cast<uint8_t>(count);

            rho[i] = density;
            float C = std::max(density * invRho0 - 1.0f, 0.0f);
            lambda_[i] = -C / (gi.lengthSq() + sumSq + eps);
        }

        for (size_t i = 0; i < n; ++i) {
            Vec2 dp;
            for (uint32_t k = neighborStart_[i]; k < neighborStart_[i + 1]; ++k) {
                uint32_t j = neighbors_[k];
                if (j == i) continue;
                Vec2 d = predicted_[i] - predicted_[j];
                float r2 = d.lengthSq();
                float ratio = W(r2) / wDeltaQ_;
                float scorr = -tensileScale * ratio * ratio * ratio * ratio;
                dp += gradW(d, std::sqrt(r2)) * (lambda_[i] + lambda_[j] + scorr);
            }
            delta_[i] = dp * invRho0;
            for (int w = 0; w < wallCount_[i]; ++w) {
                const WallContact& wc = walls_[i * kMaxWalls + w];
                Vec2 push = wc.normal * (lambda_[i] * wc.gradient);
                delta_[i] += push;
                // pressure on the particle from the wall is pressure on the wall from the particle
                if (!wc.body->isStatic) wc.body->applyImpulse(-push * (mass / dt), wc.point - wc.body->position);
            }
        }
        for (size_t i = 0; i < n; ++i) predicted_[i] += delta_[i];
        collide(dt, bodies, true);
    }

    const float invDt = 1.0f / dt;
    for (size_t i = 0; i < n; ++i) velocity[i] = (predicted_[i] - position[i]) * invDt;

    if (vorticity > 0.0f) {
        for (size_t i = 0; i < n; ++i) {
            float w = 0.0f;
            for (uint32_t k = neighborStart_[i]; k < neighborStart_[i + 1]; ++k) {
                uint32_t j = neighbors_[k];
                if (j == i) continue;
                Vec2 d = predicted_[i] - predicted_[j];
                w += cross(velocity[j] - velocity[i], -gradW(d, d.length()));
            }
            omega_[i] = w;
        }
        for (size_t i = 0; i < n; ++i) {
            Vec2 eta;
            for (uint32_t k = neighborStart_[i]; k < neighborStart_[i + 1]; ++k) {
                uint32_t j = neighbors_[k];
                if (j == i) continue;
                Vec2 d = predicted_[i] - predicted_[j];
                eta += gradW(d, d.length()) * std::abs(omega_[j]);
            }
            Vec2 N = eta.normalized();
            omegaGrad_[i] = Vec2{N.y * omega_[i], -N.x * omega_[i]} * (vorticity * invRho0);
        }
        for (size_t i = 0; i < n; ++i) velocity[i] += omegaGrad_[i] * dt;
    }

    // XSPH viscosity
    for (size_t i = 0; i < n; ++i) {
        Vec2 dv;
        for (uint32_t k = neighborStart_[i]; k < neighborStart_[i + 1]; ++k) {
            uint32_t j = neighbors_[k];
            if (j == i) continue;
            dv += (velocity[j] - velocity[i]) * (W((predicted_[i] - predicted_[j]).lengthSq()) / std::max(rho[j], 1e-6f));
        }
        delta_[i] = dv * viscosity[i];
    }
    for (size_t i = 0; i < n; ++i) {
        velocity[i] += delta_[i];
        position[i] = predicted_[i];
    }
}

} // namespace pe
