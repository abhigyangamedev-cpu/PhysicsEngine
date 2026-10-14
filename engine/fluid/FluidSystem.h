#pragma once
#include "rigid/Body.h"
#include <cstdint>
#include <memory>
#include <vector>

namespace pe {

// Position Based Fluids (Macklin & Mueller 2013) with SPH kernels, XSPH viscosity,
// vorticity confinement and two-way rigid body coupling.
class FluidSystem {
public:
    float spacing = 0.25f;     // rest distance between particles
    float density = 1.0f;      // physical density, same units as BodyDef::density
    int iterations = 4;
    int stepsPerFrame = 2;
    float relaxation = 0.05f;  // softens the density constraint, relative to its stiffness at rest
    float tensileStrength = 0.02f;
    float vorticity = 0.15f;
    float maxSpeed = 30.0f;
    // particles that leave this box are removed at the start of the next step
    Aabb bounds{{-1e6f, -1e6f}, {1e6f, 1e6f}};

    void configure(float particleSpacing);
    void addParticle(Vec2 position, Vec2 velocity = {}, float viscosity = 0.02f, uint32_t color = 0);
    void addBlock(Vec2 min, Vec2 max, float viscosity = 0.02f, uint32_t color = 0);
    void stir(Vec2 center, float radius, Vec2 velocity);
    void removeOutside(const Aabb& box);
    void clear();

    void step(float dt, Vec2 gravity, const std::vector<std::unique_ptr<Body>>& bodies);

    size_t size() const { return position.size(); }
    float particleRadius() const { return 0.5f * spacing; }
    float particleMass() const { return density * spacing * spacing; }
    float densityRatio(size_t i) const { return rho[i] / restDensity_; }

    std::vector<Vec2> position, velocity;
    std::vector<float> viscosity;
    std::vector<uint32_t> color;

private:
    float h_ = 0.5f; // kernel radius
    float restDensity_ = 1.0f;
    float poly6_ = 0.0f, spikyGrad_ = 0.0f, wDeltaQ_ = 0.0f, restStiffness_ = 0.0f;

    std::vector<Body*> bodies_;
    std::vector<Vec2> predicted_, delta_, omegaGrad_;
    std::vector<float> rho, lambda_, omega_;

    // Particles are kept sorted by grid cell (row-major), so neighbors sit close in memory and
    // the cells of a 3x3 block form three contiguous runs found by binary search.
    std::vector<int64_t> cellKey_;
    std::vector<std::pair<int64_t, uint32_t>> order_;
    std::vector<Vec2> scratchVec_;
    std::vector<float> scratchFloat_;
    std::vector<uint32_t> scratchColor_;

    // fixed-stride neighbor lists (neighbors of i at i * kMaxNeighbors), plus each pair's
    // kernel value and gradient, computed in the density pass and reused for the position deltas
    static constexpr int kMaxNeighbors = 64;
    std::vector<uint32_t> neighbors_;
    std::vector<uint8_t> neighborCount_;
    std::vector<float> pairW_;
    std::vector<Vec2> pairGrad_;

    // Solid surfaces contribute density like a half-plane of fluid particles (Akinci-style boundaries).
    static constexpr int kWallSamples = 64;
    float wallDensity_[kWallSamples + 1] = {}, wallGradient_[kWallSamples + 1] = {};
    struct WallContact {
        Body* body;
        Vec2 normal, point, push;
        float gradient;
    };
    static constexpr int kMaxWalls = 2;
    std::vector<WallContact> walls_;
    std::vector<uint8_t> wallCount_;
    float wallLookup(const float* table, float distance) const;

    // impulses on dynamic bodies are collected per particle in parallel, then applied serially
    struct BodyImpulse {
        Body* body;
        Vec2 impulse, point;
    };
    std::vector<BodyImpulse> impulses_;
    std::vector<uint8_t> impulseCount_;
    void applyImpulses();

    float W(float r2) const;
    Vec2 gradW(Vec2 d, float r) const;
    void sortByCell();
    void findNeighbors();
    void collide(float dt, const std::vector<Body*>& bodies);
};

} // namespace pe
