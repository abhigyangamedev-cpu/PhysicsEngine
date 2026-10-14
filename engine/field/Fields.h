#pragma once
#include "field/ForceField.h"
#include "field/QuadTree.h"
#include "rigid/Body.h"
#include <vector>

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

// Hookean springs, F = -k (length - rest) - damping * (rate of stretch), applied every substep.
// Unlike a DistanceJoint spring (which is tuned by frequency), k is a true stiffness in N/m,
// so a mass m on spring k oscillates with period 2 pi sqrt(m / k).
class SpringField : public ForceField {
public:
    struct Spring {
        Body* a;
        Body* b;            // null: end B is the fixed world point localB
        Vec2 localA, localB;
        float k, damping, rest;
        bool pushOnly = false; // a compressed spring that stops pushing once it reaches its rest length
        Vec2 drive;            // world end B oscillates as localB + drive * sin(2 pi driveHz t)
        float driveHz = 0.0f;
        bool coil = true;      // drawn as a coil, or as a plain line (strings)
    };

    std::vector<Spring> springs;
    float time = 0.0f;

    // Rest length is the current distance between the two points.
    Spring& add(Body* a, Vec2 worldA, Body* b, Vec2 worldB, float k, float damping = 0.0f);
    Vec2 endA(const Spring& s) const;
    Vec2 endB(const Spring& s) const;
    void apply(World& world, float h) override;
};

float orbitalSpeed(float G, float centralMass, float radius);

} // namespace pe
