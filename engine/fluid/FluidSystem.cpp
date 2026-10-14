#include "fluid/FluidSystem.h"
#include "core/Parallel.h"
#include "core/ParticleContact.h"
#include <algorithm>
#include <cmath>
#include <numbers>

namespace pe {

namespace {

constexpr float kPi = std::numbers::pi_v<float>;

// row-major sort key of a grid cell: y major, x minor, both signed
int64_t cellKeyOf(int x, int y) {
    return static_cast<int64_t>(y) * (int64_t{1} << 32) + static_cast<int64_t>(static_cast<uint32_t>(x) ^ 0x80000000u);
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

void FluidSystem::sortByCell() {
    const size_t n = size();
    const float inv = 1.0f / h_;
    order_.resize(n);
    for (size_t i = 0; i < n; ++i) {
        Vec2 p = predicted_[i] * inv;
        int x = static_cast<int>(std::floor(std::clamp(p.x, -1e6f, 1e6f)));
        int y = static_cast<int>(std::floor(std::clamp(p.y, -1e6f, 1e6f)));
        order_[i] = {cellKeyOf(x, y), static_cast<uint32_t>(i)};
    }
    std::sort(order_.begin(), order_.end());

    cellKey_.resize(n);
    for (size_t i = 0; i < n; ++i) cellKey_[i] = order_[i].first;
    auto permute = [&](auto& data, auto& scratch) {
        scratch.resize(n);
        for (size_t i = 0; i < n; ++i) scratch[i] = data[order_[i].second];
        data.swap(scratch);
    };
    permute(position, scratchVec_);
    permute(velocity, scratchVec_);
    permute(predicted_, scratchVec_);
    permute(viscosity, scratchFloat_);
    permute(color, scratchColor_);
}

void FluidSystem::findNeighbors() {
    const size_t n = size();
    const float inv = 1.0f / h_;
    const float h2 = h_ * h_;
    neighbors_.resize(n * kMaxNeighbors);
    neighborCount_.resize(n);
    pairW_.resize(n * kMaxNeighbors);
    pairGrad_.resize(n * kMaxNeighbors);

    parallelFor(n, [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            Vec2 p = predicted_[i] * inv;
            int cx = static_cast<int>(std::floor(std::clamp(p.x, -1e6f, 1e6f)));
            int cy = static_cast<int>(std::floor(std::clamp(p.y, -1e6f, 1e6f)));
            uint32_t* out = &neighbors_[i * kMaxNeighbors];
            int count = 0;
            for (int dy = -1; dy <= 1; ++dy) {
                // cells (cx-1..cx+1, cy+dy) are adjacent in the sorted order
                auto lo = std::lower_bound(cellKey_.begin(), cellKey_.end(), cellKeyOf(cx - 1, cy + dy));
                auto hi = std::upper_bound(lo, cellKey_.end(), cellKeyOf(cx + 1, cy + dy));
                for (auto it = lo; it != hi && count < kMaxNeighbors; ++it) {
                    uint32_t j = static_cast<uint32_t>(it - cellKey_.begin());
                    if ((predicted_[j] - predicted_[i]).lengthSq() < h2) out[count++] = j;
                }
            }
            neighborCount_[i] = static_cast<uint8_t>(count);
        }
    });
}

void FluidSystem::collide(float dt, const std::vector<Body*>& bodies) {
    const float r = particleRadius();
    const float m = particleMass();
    parallelFor(size(), [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            Vec2& p = predicted_[i];
            int count = 0;
            for (Body* b : bodies) {
                if (!b->aabb.overlaps({p - Vec2{r, r}, p + Vec2{r, r}})) continue;
                ParticleContact c;
                if (!particleContact(*b, p, r, c)) continue;
                p += c.normal * c.depth;
                // the fluid was stopped from moving depth into the body: hand that momentum to the body
                if (!b->isStatic && count < kMaxWalls)
                    impulses_[i * kMaxWalls + count++] = {b, -c.normal * (m * c.depth / dt), c.point - b->position};
            }
            impulseCount_[i] = static_cast<uint8_t>(count);
        }
    });
    applyImpulses();
}

void FluidSystem::applyImpulses() {
    for (size_t i = 0; i < size(); ++i)
        for (int k = 0; k < impulseCount_[i]; ++k) {
            const BodyImpulse& bi = impulses_[i * kMaxWalls + k];
            bi.body->applyImpulse(bi.impulse, bi.point);
        }
}

void FluidSystem::step(float dt, Vec2 gravity, const std::vector<std::unique_ptr<Body>>& allBodies) {
    if (std::any_of(position.begin(), position.end(), [&](Vec2 p) {
            return p.x < bounds.min.x || p.x > bounds.max.x || p.y < bounds.min.y || p.y > bounds.max.y;
        }))
        removeOutside(bounds);

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
    impulses_.resize(n * kMaxWalls);
    impulseCount_.resize(n);
    const float mass = particleMass();

    // a member, not thread_local: the worker threads read this list too
    bodies_.clear();
    for (const auto& b : allBodies)
        if (b->collides) bodies_.push_back(b.get());
    const std::vector<Body*>& bodies = bodies_;

    const float maxSpeedSq = maxSpeed * maxSpeed;
    for (size_t i = 0; i < n; ++i) {
        velocity[i] += gravity * dt;
        if (velocity[i].lengthSq() > maxSpeedSq) velocity[i] = velocity[i].normalized() * maxSpeed;
        predicted_[i] = position[i] + velocity[i] * dt;
    }
    collide(dt, bodies);
    sortByCell();
    findNeighbors();

    const float invRho0 = 1.0f / restDensity_;
    const float eps = relaxation * restStiffness_;
    const float tensileScale = tensileStrength / restStiffness_;

    for (int it = 0; it < iterations; ++it) {
        parallelFor(n, [&](size_t begin, size_t end) {
            for (size_t i = begin; i < end; ++i) {
                float density = 0.0f, sumSq = 0.0f;
                Vec2 gi;
                const size_t base = i * kMaxNeighbors;
                for (size_t k = base; k < base + neighborCount_[i]; ++k) {
                    uint32_t j = neighbors_[k];
                    Vec2 d = predicted_[i] - predicted_[j];
                    float r2 = d.lengthSq();
                    float w = W(r2);
                    density += w;
                    pairW_[k] = w;
                    if (j == i) {
                        pairGrad_[k] = {};
                        continue;
                    }
                    Vec2 gw = gradW(d, std::sqrt(r2));
                    pairGrad_[k] = gw;
                    gw *= invRho0;
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
                    walls_[i * kMaxWalls + count++] = {b, c.normal, c.point, {}, g};
                }
                wallCount_[i] = static_cast<uint8_t>(count);

                rho[i] = density;
                float C = std::max(density * invRho0 - 1.0f, 0.0f);
                lambda_[i] = -C / (gi.lengthSq() + sumSq + eps);
            }
        });

        parallelFor(n, [&](size_t begin, size_t end) {
            for (size_t i = begin; i < end; ++i) {
                Vec2 dp;
                const size_t base = i * kMaxNeighbors;
                for (size_t k = base; k < base + neighborCount_[i]; ++k) {
                    uint32_t j = neighbors_[k];
                    if (j == i) continue;
                    float ratio = pairW_[k] / wDeltaQ_;
                    float scorr = -tensileScale * ratio * ratio * ratio * ratio;
                    dp += pairGrad_[k] * (lambda_[i] + lambda_[j] + scorr);
                }
                delta_[i] = dp * invRho0;
                for (int w = 0; w < wallCount_[i]; ++w) {
                    WallContact& wc = walls_[i * kMaxWalls + w];
                    wc.push = wc.normal * (lambda_[i] * wc.gradient);
                    delta_[i] += wc.push;
                }
            }
        });
        // pressure on the particle from the wall is pressure on the wall from the particle
        for (size_t i = 0; i < n; ++i)
            for (int w = 0; w < wallCount_[i]; ++w) {
                const WallContact& wc = walls_[i * kMaxWalls + w];
                if (!wc.body->isStatic) wc.body->applyImpulse(-wc.push * (mass / dt), wc.point - wc.body->position);
            }

        for (size_t i = 0; i < n; ++i) predicted_[i] += delta_[i];
        collide(dt, bodies);
    }

    const float invDt = 1.0f / dt;
    for (size_t i = 0; i < n; ++i) velocity[i] = (predicted_[i] - position[i]) * invDt;

    if (vorticity > 0.0f) {
        parallelFor(n, [&](size_t begin, size_t end) {
            for (size_t i = begin; i < end; ++i) {
                float w = 0.0f;
                const size_t base = i * kMaxNeighbors;
                for (size_t k = base; k < base + neighborCount_[i]; ++k) {
                    uint32_t j = neighbors_[k];
                    if (j == i) continue;
                    Vec2 d = predicted_[i] - predicted_[j];
                    w += cross(velocity[j] - velocity[i], -gradW(d, d.length()));
                }
                omega_[i] = w;
            }
        });
        parallelFor(n, [&](size_t begin, size_t end) {
            for (size_t i = begin; i < end; ++i) {
                Vec2 eta;
                const size_t base = i * kMaxNeighbors;
                for (size_t k = base; k < base + neighborCount_[i]; ++k) {
                    uint32_t j = neighbors_[k];
                    if (j == i) continue;
                    Vec2 d = predicted_[i] - predicted_[j];
                    eta += gradW(d, d.length()) * std::abs(omega_[j]);
                }
                Vec2 N = eta.normalized();
                omegaGrad_[i] = Vec2{N.y * omega_[i], -N.x * omega_[i]} * (vorticity * invRho0);
            }
        });
        for (size_t i = 0; i < n; ++i) velocity[i] += omegaGrad_[i] * dt;
    }

    // XSPH viscosity
    parallelFor(n, [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            Vec2 dv;
            const size_t base = i * kMaxNeighbors;
            for (size_t k = base; k < base + neighborCount_[i]; ++k) {
                uint32_t j = neighbors_[k];
                if (j == i) continue;
                dv += (velocity[j] - velocity[i]) * (W((predicted_[i] - predicted_[j]).lengthSq()) / std::max(rho[j], 1e-6f));
            }
            delta_[i] = dv * viscosity[i];
        }
    });
    for (size_t i = 0; i < n; ++i) {
        velocity[i] += delta_[i];
        position[i] = predicted_[i];
    }
}

} // namespace pe
