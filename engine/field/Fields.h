#pragma once
#include "field/ForceField.h"
#include "field/QuadTree.h"

namespace pe {

// Mutual gravity between every dynamic body and (optionally) particle, via Barnes-Hut.
class NBodyGravity : public ForceField {
public:
    float G = 1.0f;
    float softening = 0.1f;
    float theta = 0.6f;
    bool includeParticles = true;

    void apply(World& world, float h) override;

private:
    QuadTree tree_;
    std::vector<Vec2> positions_;
    std::vector<float> masses_;
};

// Coulomb force between charged bodies. Static charged bodies act as fixed sources.
class CoulombField : public ForceField {
public:
    float k = 1.0f;
    float softening = 0.1f;

    void apply(World& world, float h) override;
    Vec2 electricField(const World& world, Vec2 p) const override;
};

class UniformElectricField : public ForceField {
public:
    Vec2 E;
    Region region;

    void apply(World& world, float h) override;
    Vec2 electricField(const World&, Vec2 p) const override { return region.contains(p) ? E : Vec2{}; }
};

// Out-of-plane magnetic field B (positive = toward the viewer). Velocity is rotated exactly,
// so charged bodies keep their speed instead of spiralling outward.
class UniformMagneticField : public ForceField {
public:
    float B = 0.0f;
    Region region;

    void apply(World& world, float h) override;
    float magneticField(Vec2 p) const override { return region.contains(p) ? B : 0.0f; }
};

float orbitalSpeed(float G, float centralMass, float radius);

} // namespace pe
