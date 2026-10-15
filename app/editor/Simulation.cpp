#include "editor/Simulation.h"

namespace editor {

using namespace pe;

float kineticEnergy(const Body& b) {
    return 0.5f * b.mass * b.velocity.lengthSq() + 0.5f * b.inertia * b.angularVelocity * b.angularVelocity;
}

void Simulation::load(const Scene& scene) {
    scene_ = scene;
    build(scene_, world_, built_);
    time_ = 0.0f;
    accel_.assign(size(), {});
    drive_.assign(size(), 0.0f);
    driveSum_.assign(size(), 0.0f);
}

void Simulation::step() {
    const size_t n = size();
    thread_local std::vector<Vec2> before;
    before.resize(n);
    for (size_t i = 0; i < n; ++i) {
        Body* b = built_.bodies[i];
        before[i] = b->velocity;
        const ObjectSpec& o = scene_.objects[i];
        if (!b->isStatic) b->force = o.force + o.velocity.normalized() * drive_[i];
    }
    world_.step(kDt);
    time_ += kDt;

    for (size_t i = 0; i < n; ++i) {
        const ObjectSpec& o = scene_.objects[i];
        const Body* b = built_.bodies[i];
        if (b->isStatic) continue;
        accel_[i] = accel_[i] * 0.5f + (b->velocity - before[i]) * (0.5f / kDt);

        // constant-velocity drive: PI control on the speed along the initial velocity
        if (!o.holdVelocity || o.velocity.lengthSq() == 0.0f) continue;
        const float e = dot(o.velocity - b->velocity, o.velocity.normalized()) * b->mass / kDt;
        driveSum_[i] += 0.1f * e;
        drive_[i] = driveSum_[i] + 0.6f * e;
    }
}

void Simulation::runUntil(float t) {
    while (time_ < t - kDt * 0.5f) step();
}

int Simulation::indexOf(int objId) const {
    for (size_t i = 0; i < scene_.objects.size(); ++i)
        if (scene_.objects[i].id == objId) return static_cast<int>(i);
    return -1;
}

int Simulation::indexNamed(const char* name) const {
    for (size_t i = 0; i < scene_.objects.size(); ++i)
        if (scene_.objects[i].name == name) return static_cast<int>(i);
    return -1;
}

float Simulation::totalEnergy() const {
    float e = 0.0f;
    for (const Body* b : built_.bodies)
        if (!b->isStatic) e += kineticEnergy(*b) + b->mass * scene_.world.g * b->position.y;
    for (const auto& s : built_.springs->springs) {
        float x = (built_.springs->endB(s) - built_.springs->endA(s)).length() - s.rest;
        e += 0.5f * s.k * x * x;
    }
    return e;
}

} // namespace editor
