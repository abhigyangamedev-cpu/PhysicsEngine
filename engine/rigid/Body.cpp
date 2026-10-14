#include "rigid/Body.h"
#include <algorithm>

namespace pe {

Body::Body(const Shape& s, const BodyDef& def)
    : shape(s), position(def.position), velocity(def.velocity), angle(def.angle),
      angularVelocity(def.angularVelocity), friction(def.friction), restitution(def.restitution),
      charge(def.charge), isStatic(def.isStatic), collides(def.collides), draggable(def.draggable),
      group(def.group) {
    if (isStatic) {
        mass = inertia = invMass = invInertia = 0.0f;
        velocity = {};
        angularVelocity = 0.0f;
    } else {
        MassData md = shape.computeMass(def.density);
        mass = md.mass;
        inertia = md.inertia;
        invMass = 1.0f / mass;
        invInertia = 1.0f / inertia;
    }
    worldVertices.resize(shape.vertices.size());
    worldNormals.resize(shape.normals.size());
    updateTransform();
}

void Body::updateTransform() {
    rot = Rot(angle);
    if (shape.type == ShapeType::Circle) {
        Vec2 r{shape.radius, shape.radius};
        aabb = {position - r, position + r};
        return;
    }
    aabb = {{1e30f, 1e30f}, {-1e30f, -1e30f}};
    for (size_t i = 0; i < shape.vertices.size(); ++i) {
        Vec2 v = position + rot.apply(shape.vertices[i]);
        worldVertices[i] = v;
        worldNormals[i] = rot.apply(shape.normals[i]);
        aabb.min = {std::min(aabb.min.x, v.x), std::min(aabb.min.y, v.y)};
        aabb.max = {std::max(aabb.max.x, v.x), std::max(aabb.max.y, v.y)};
    }
}

bool Body::contains(Vec2 p) const {
    if (shape.type == ShapeType::Circle) return (p - position).lengthSq() <= shape.radius * shape.radius;
    for (size_t i = 0; i < worldVertices.size(); ++i)
        if (dot(worldNormals[i], p - worldVertices[i]) > 0.0f) return false;
    return true;
}

} // namespace pe
