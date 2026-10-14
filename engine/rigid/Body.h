#pragma once
#include "rigid/Shape.h"
#include <cstdint>

namespace pe {

struct BodyDef {
    Vec2 position;
    float angle = 0.0f;
    Vec2 velocity;
    float angularVelocity = 0.0f;
    float density = 1.0f;
    float friction = 0.5f;
    float restitution = 0.2f;
    bool isStatic = false;
};

class Body {
public:
    Body(const Shape& shape, const BodyDef& def);

    Shape shape;
    Vec2 position, velocity, force;
    float angle, angularVelocity, torque = 0.0f;
    float mass, invMass, inertia, invInertia;
    float friction, restitution;
    bool isStatic;
    uint32_t id = 0;

    Rot rot;
    std::vector<Vec2> worldVertices;
    std::vector<Vec2> worldNormals;
    Aabb aabb;

    void applyForce(Vec2 f) { force += f; }
    void applyForceAt(Vec2 f, Vec2 worldPoint) {
        force += f;
        torque += cross(worldPoint - position, f);
    }
    void applyImpulse(Vec2 j, Vec2 r) {
        velocity += j * invMass;
        angularVelocity += cross(r, j) * invInertia;
    }
    Vec2 velocityAt(Vec2 worldPoint) const {
        return velocity + cross(angularVelocity, worldPoint - position);
    }
    bool contains(Vec2 worldPoint) const;

    void updateTransform();
};

} // namespace pe
