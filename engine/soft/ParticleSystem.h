#pragma once
#include "rigid/Body.h"
#include <cstdint>
#include <memory>
#include <vector>

namespace pe {

struct Particle {
    Vec2 position, previous, velocity;
    float invMass = 1.0f;
    float radius = 0.1f;
    float friction = 0.4f;
    int group = 0; // particles in the same group don't collide with each other
    uint32_t color = 0; // 0xRRGGBBAA, 0 = renderer default
};

// compliance = inverse stiffness (0 = rigid). tearRatio > 0 breaks the link when stretched past it.
struct DistanceConstraint {
    uint32_t a, b;
    float restLength;
    float compliance = 0.0f;
    float tearRatio = 0.0f;
    bool broken = false;
};

// Keeps the area enclosed by a CCW ring of particles near restArea * pressure.
struct PressureConstraint {
    std::vector<uint32_t> ring;
    float restArea;
    float pressure = 1.0f;
    float compliance = 0.0f;
};

struct Attachment {
    uint32_t particle;
    Body* body;
    Vec2 localAnchor;
    float compliance = 0.0f;
};

class ParticleSystem {
public:
    uint32_t addParticle(Vec2 position, float mass, float radius, int group);
    DistanceConstraint& link(uint32_t a, uint32_t b, float compliance = 0.0f, float tearRatio = 0.0f);
    PressureConstraint& addPressure(std::vector<uint32_t> ring, float pressure, float compliance = 0.0f);
    void attach(uint32_t particle, Body* body, float compliance = 0.0f);
    int newGroup() { return ++groupCounter_; }

    // Call before the rigid solver moves the bodies over a step: the particle steps then walk attached
    // bodies along that motion with them instead of seeing them a whole step ahead.
    void beginBodyStep();
    // Advances h in `iterations` particle steps.
    void step(float h, Vec2 gravity, const std::vector<std::unique_ptr<Body>>& bodies, int iterations = 1);
    void removeBody(const Body* body);
    void clear();

    float area(const PressureConstraint& p) const;
    int nearest(Vec2 point, float maxDistance) const;

    std::vector<Particle> particles;
    std::vector<DistanceConstraint> links;
    std::vector<PressureConstraint> pressures;
    std::vector<Attachment> attachments;
    float damping = 0.5f; // per second
    bool selfCollision = true;

private:
    int groupCounter_ = 0;
    std::vector<uint32_t> cellStart_, cellEntries_;
    std::vector<Vec2> predicted_; // where each particle would be with no constraints, this step
    struct Carried {
        Body* body;
        Vec2 position; // where the body was before the rigid solver moved it
        float angle;
        Vec2 velocity; // and its velocity before the attachments changed it
        float angularVelocity;
    };
    std::vector<Carried> carried_; // the dynamic bodies something is attached to
    bool begun_ = false;

    void substep(float h, Vec2 gravity, const std::vector<std::unique_ptr<Body>>& bodies);

    void solveLinks(float alphaScale);
    void solvePressures(float alphaScale);
    void solveAttachments(float h, float alphaScale);
    void collideParticles();
    void collideBodies(float h, const std::vector<std::unique_ptr<Body>>& bodies);
};

} // namespace pe
