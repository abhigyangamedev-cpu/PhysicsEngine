#include "rigid/Body3.h"
#include <numbers>

namespace pe3 {

float Shape::volume() const {
    if (type == ShapeType::Sphere) return 4.0f / 3.0f * std::numbers::pi_v<float> * radius * radius * radius;
    return 8.0f * halfExtents.x * halfExtents.y * halfExtents.z;
}

Vec3 Shape::unitInertia() const {
    if (type == ShapeType::Sphere) {
        float i = 0.4f * radius * radius;
        return {i, i, i};
    }
    Vec3 e = halfExtents;
    return Vec3{e.y * e.y + e.z * e.z, e.x * e.x + e.z * e.z, e.x * e.x + e.y * e.y} / 3.0f;
}

Body::Body(const Shape& shape, const BodyDef& def)
    : shape(shape), position(def.position), velocity(def.velocity), orientation(def.orientation.normalized()),
      angularVelocity(def.angularVelocity), friction(def.friction), restitution(def.restitution), rollingResistance(def.rollingResistance),
      isStatic(def.isStatic), group(def.group), color(def.color) {
    if (isStatic) {
        mass = invMass = 0.0f;
        invInertiaLocal = {};
        velocity = angularVelocity = {};
    } else {
        mass = def.density * shape.volume();
        invMass = 1.0f / mass;
        Vec3 I = shape.unitInertia() * mass;
        invInertiaLocal = {1.0f / I.x, 1.0f / I.y, 1.0f / I.z};
    }
    updateTransform();
}

void Body::updateTransform() {
    rotation = orientation.toMat3();
    invInertiaWorld = rotation * Mat3::diagonal(invInertiaLocal) * rotation.transposed();
    Vec3 ext;
    if (shape.type == ShapeType::Sphere) {
        ext = {shape.radius, shape.radius, shape.radius};
    } else {
        for (int i = 0; i < 3; ++i) {
            Vec3 c = rotation.c[i] * shape.halfExtents[i];
            ext += Vec3{std::abs(c.x), std::abs(c.y), std::abs(c.z)};
        }
    }
    aabb = {position - ext, position + ext};
}

} // namespace pe3
