#pragma once
#include "core/Softness.h"
#include "rigid/Body.h"
#include "rigid/Collision.h"
#include "rigid/Joint.h"
#include "soft/ParticleSystem.h"
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace pe {

struct WorldSettings {
    Vec2 gravity{0.0f, -9.81f};
    int substeps = 4;
    int velocityIterations = 8;
    int relaxIterations = 3;
    float contactHertz = 30.0f;
    float contactDampingRatio = 10.0f;
    float penetrationSlop = 0.005f;
    float maxCorrectionVelocity = 3.0f;
    float restitutionThreshold = 1.0f;
    float linearDamping = 0.0f;
    float angularDamping = 0.05f;
    int particleIterations = 4;
};

class World {
public:
    WorldSettings settings;

    Body* createBody(const Shape& shape, const BodyDef& def);
    void destroyBody(Body* body);
    void clear();

    template <typename T, typename... Args>
    T* createJoint(Args&&... args) {
        auto joint = std::make_unique<T>(std::forward<Args>(args)...);
        T* raw = joint.get();
        joints_.push_back(std::move(joint));
        rebuildJointPairs();
        return raw;
    }
    void destroyJoint(Joint* joint);

    ParticleSystem particles;

    void step(float dt);

    Body* bodyAt(Vec2 point) const;
    const std::vector<std::unique_ptr<Body>>& bodies() const { return bodies_; }
    const std::unordered_map<uint64_t, Manifold>& manifolds() const { return manifolds_; }
    const std::vector<std::unique_ptr<Joint>>& joints() const { return joints_; }

private:
    std::vector<std::unique_ptr<Body>> bodies_;
    std::unordered_map<uint64_t, Manifold> manifolds_;
    std::vector<std::unique_ptr<Joint>> joints_;
    std::unordered_set<uint64_t> jointPairs_;
    Softness contactSoft_;
    uint32_t nextId_ = 0;

    void rebuildJointPairs();
    void integrateVelocities(float h);
    void detectContacts();
    void prepareContacts(float h);
    void solveContacts(bool useBias);
    void solveJoints(bool useBias);
    void applyRestitution();
    void integratePositions(float h);
};

} // namespace pe
