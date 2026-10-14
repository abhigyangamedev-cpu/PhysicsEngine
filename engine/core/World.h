#pragma once
#include "rigid/Body.h"
#include "rigid/Collision.h"
#include <memory>
#include <unordered_map>
#include <vector>

namespace pe {

struct WorldSettings {
    Vec2 gravity{0.0f, -9.81f};
    int substeps = 4;
    int velocityIterations = 8;
    float baumgarte = 0.2f;
    float penetrationSlop = 0.005f;
    float maxCorrectionVelocity = 3.0f;
    float restitutionThreshold = 1.0f;
    float linearDamping = 0.0f;
    float angularDamping = 0.05f;
};

class World {
public:
    WorldSettings settings;

    Body* createBody(const Shape& shape, const BodyDef& def);
    void destroyBody(Body* body);
    void clear();

    void step(float dt);

    Body* bodyAt(Vec2 point) const;
    const std::vector<std::unique_ptr<Body>>& bodies() const { return bodies_; }
    const std::unordered_map<uint64_t, Manifold>& manifolds() const { return manifolds_; }

private:
    std::vector<std::unique_ptr<Body>> bodies_;
    std::unordered_map<uint64_t, Manifold> manifolds_;
    uint32_t nextId_ = 0;

    void integrateVelocities(float h);
    void detectContacts();
    void prepareContacts(float h);
    void solveContacts();
    void integratePositions(float h);
};

} // namespace pe
