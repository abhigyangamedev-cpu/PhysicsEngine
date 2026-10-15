#pragma once
#include "core/Softness.h"
#include "field/ForceField.h"
#include "rigid/Body.h"
#include "rigid/Collision.h"
#include "rigid/Joint.h"
#include "soft/ParticleSystem.h"
#include "fluid/FluidSystem.h"
#include <memory>
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
    bool minFriction = false; // contact friction is the smaller of the two coefficients instead of their geometric mean
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
    void rebuildJointPairs(); // call after changing a joint's collideConnected

    template <typename T>
    T* addField() {
        auto field = std::make_unique<T>();
        T* raw = field.get();
        fields_.push_back(std::move(field));
        return raw;
    }
    const std::vector<std::unique_ptr<ForceField>>& fields() const { return fields_; }
    Vec2 electricFieldAt(Vec2 p) const;
    float magneticFieldAt(Vec2 p) const;

    ParticleSystem particles;
    FluidSystem fluid;

    void step(float dt);

    Body* bodyAt(Vec2 point) const;
    const std::vector<std::unique_ptr<Body>>& bodies() const { return bodies_; }
    const std::vector<Manifold>& manifolds() const { return manifolds_; } // Manifold::key names the pair
    const std::vector<std::unique_ptr<Joint>>& joints() const { return joints_; }

private:
    std::vector<std::unique_ptr<Body>> bodies_;
    std::vector<Manifold> manifolds_, nextManifolds_;
    std::vector<Body*> sorted_; // broadphase order, kept between steps so re-sorting is nearly free
    bool sortedDirty_ = true;
    std::vector<uint64_t> slotKeys_; // open-addressing index of manifolds_ by pair key, for warm starting
    std::vector<int> slotIndex_;
    std::vector<std::unique_ptr<Joint>> joints_;
    std::vector<std::unique_ptr<ForceField>> fields_;
    std::unordered_set<uint64_t> jointPairs_;
    Softness contactSoft_;
    uint32_t nextId_ = 0;

    void integrateVelocities(float h);
    void detectContacts();
    void indexManifolds();
    const Manifold* findManifold(uint64_t key) const;
    void prepareContacts(float h);
    void solveContacts(bool useBias);
    void solveJoints(bool useBias);
    void applyRestitution();
    void integratePositions(float h);
};

} // namespace pe
