#pragma once
#include "rigid/Collision3.h"
#include "rigid/Joint3.h"
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace pe3 {

struct WorldSettings {
    Vec3 gravity{0.0f, -9.81f, 0.0f};
    int substeps = 4;
    int velocityIterations = 8;
    int relaxIterations = 3;
    float contactHertz = 30.0f;
    float contactDampingRatio = 10.0f;
    float penetrationSlop = 0.005f;
    float speculativeDistance = 0.02f;
    float maxCorrectionVelocity = 3.0f;
    float restitutionThreshold = 1.0f;
    float linearDamping = 0.0f;
    float angularDamping = 0.05f;
    bool enableSleep = true;
    float sleepThreshold = 0.05f; // m/s, angular speed scaled by body radius
    float timeToSleep = 0.5f;
};

struct RayHit {
    Body* body = nullptr;
    float t = 0.0f;
    Vec3 point, normal;
};

class World {
public:
    WorldSettings settings;

    Body* createBody(const Shape& shape, const BodyDef& def);
    void destroyBody(Body* body);
    void clear();

    // bodies may be nullptr to pin to the world
    template <typename T, typename... Args>
    T* createJoint(Args&&... args) {
        auto joint = std::make_unique<T>(std::forward<Args>(args)...);
        T* raw = joint.get();
        wake(raw->a);
        wake(raw->b);
        joints_.push_back(std::move(joint));
        rebuildJointPairs();
        return raw;
    }
    void destroyJoint(Joint* joint);
    void rebuildJointPairs(); // call after changing a joint's collideConnected
    const std::vector<std::unique_ptr<Joint>>& joints() const { return joints_; }

    void step(float dt);
    void wake(Body* body); // wakes the body's whole sleeping island
    int awakeCount() const;

    RayHit raycast(Vec3 origin, Vec3 dir, float maxT = 1e30f) const;

    const std::vector<std::unique_ptr<Body>>& bodies() const { return bodies_; }
    const std::unordered_map<uint64_t, Manifold>& manifolds() const { return manifolds_; }

private:
    std::vector<std::unique_ptr<Body>> bodies_;
    std::unordered_map<uint64_t, Manifold> manifolds_;
    std::unordered_map<uint64_t, Manifold> nextManifolds_;
    std::vector<std::unique_ptr<Joint>> joints_;
    std::unordered_set<uint64_t> jointPairs_;
    std::vector<Body*> sorted_;
    std::vector<std::vector<Body*>> islands_;
    std::vector<int> parent_;
    uint32_t nextId_ = 0;
    float biasRate_ = 0.0f, massScale_ = 1.0f, impulseScale_ = 0.0f, invH_ = 0.0f;

    void integrateVelocities(float h);
    void wakeDisturbed();
    void updateSleep(float dt);
    bool detectContacts(); // true if it woke a sleeping island
    void detectUntilStable();
    bool jointActive(const Joint& j) const;
    void solveJoints(bool useBias);
    void prepareContacts(float h);
    void solveContacts(bool useBias);
    void applyRestitution();
    void integratePositions(float h);
};

} // namespace pe3
