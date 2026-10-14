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

    std::vector<Vec2> predicted_, delta_, omegaGrad_;
    std::vector<float> rho, lambda_, omega_;
    std::vector<uint32_t> cellStart_, cellEntries_, neighborStart_, neighbors_;

    // Solid surfaces contribute density like a half-plane of fluid particles (Akinci-style boundaries).
    static constexpr int kWallSamples = 64;
    float wallDensity_[kWallSamples + 1] = {}, wallGradient_[kWallSamples + 1] = {};
    struct WallContact {
        Body* body;
        Vec2 normal, point;
        float gradient;
    };
    static constexpr int kMaxWalls = 2;
    std::vector<WallContact> walls_;
    std::vector<uint8_t> wallCount_;
    float wallLookup(const float* table, float distance) const;

    float W(float r2) const;
    Vec2 gradW(Vec2 d, float r) const;
    void findNeighbors();
    void collide(float dt, const std::vector<Body*>& bodies, bool transferMomentum);
};

} // namespace pe
