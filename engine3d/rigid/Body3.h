#pragma once
#include "math/Math3.h"
#include <cstdint>

namespace pe3 {

enum class ShapeType { Sphere, Box };

struct Shape {
    ShapeType type = ShapeType::Sphere;
    float radius = 0.5f;
    Vec3 halfExtents{0.5f, 0.5f, 0.5f};

    static Shape sphere(float r) { return {ShapeType::Sphere, r, {r, r, r}}; }
    static Shape box(Vec3 half) { return {ShapeType::Box, half.length(), half}; }

    float volume() const;
    Vec3 unitInertia() const; // principal inertia per unit mass
};

struct BodyDef {
    Vec3 position;
    Quat orientation;
    Vec3 velocity;
    Vec3 angularVelocity;
    float density = 1.0f;
    float friction = 0.5f;
    float restitution = 0.2f;
    float rollingResistance = 0.0f;
    bool isStatic = false;
    int group = 0; // bodies sharing a nonzero group don't collide with each other
    uint32_t color = 0;
};

class Body {
public:
    Body(const Shape& shape, const BodyDef& def);

    Shape shape;
    Vec3 position, velocity, force;
    Quat orientation;
    Vec3 angularVelocity, torque;
    float mass, invMass;
    Vec3 invInertiaLocal;
    float friction, restitution, rollingResistance;
    bool isStatic;
    int group;
    uint32_t color;
    uint32_t id = 0;

    bool awake = true;
    float sleepTime = 0.0f;
    int island = -1; // sleeping island index
    int node = -1;   // scratch index for island building

    Mat3 rotation;
    Mat3 invInertiaWorld;
    Aabb aabb;

    void applyForce(Vec3 f) { force += f; }
    void applyForceAt(Vec3 f, Vec3 worldPoint) {
        force += f;
        torque += cross(worldPoint - position, f);
    }
    void applyImpulse(Vec3 j, Vec3 r) {
        velocity += j * invMass;
        angularVelocity += invInertiaWorld * cross(r, j);
    }
    Vec3 velocityAt(Vec3 worldPoint) const { return velocity + cross(angularVelocity, worldPoint - position); }
    Vec3 toWorld(Vec3 local) const { return position + rotation * local; }
    Vec3 toLocal(Vec3 world) const { return rotation.mulT(world - position); }

    void updateTransform();
};

} // namespace pe3
