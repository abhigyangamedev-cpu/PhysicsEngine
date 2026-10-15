#pragma once
#include "editor/EditorScene.h"

namespace editor {

float kineticEnergy(const pe::Body& b);

// Runs a scene in fixed steps with its applied forces and constant-velocity drives. Keeps its own
// copy of the scene, so indices into bodies() always match scene().objects.
class Simulation {
public:
    static constexpr float kDt = 1.0f / 120.0f;

    void load(const Scene& scene);
    void step();
    void runUntil(float t);

    float time() const { return time_; }
    const Scene& scene() const { return scene_; }
    const Built& built() const { return built_; }
    const pe::World& world() const { return world_; }
    size_t size() const { return built_.bodies.size(); }

    int indexOf(int objId) const;
    int indexNamed(const char* name) const;
    pe::Body* body(int index) const { return index >= 0 ? built_.bodies[index] : nullptr; }
    pe::Body* body(const char* name) const { return body(indexNamed(name)); }

    pe::Vec2 acceleration(int index) const { return accel_[index]; }
    float driveForce(int index) const { return drive_[index]; } // along the object's initial velocity
    float totalEnergy() const; // KE + mgy + spring PE
    bool blewUp() const { return blewUp_; } // a body went non-finite or absurdly far/fast

private:
    Scene scene_;
    pe::World world_;
    Built built_;
    float time_ = 0.0f;
    bool blewUp_ = false;
    std::vector<pe::Vec2> accel_;
    std::vector<float> drive_, driveSum_;
};

} // namespace editor
