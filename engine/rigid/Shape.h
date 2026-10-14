#pragma once
#include "math/Vec2.h"
#include <vector>

namespace pe {

enum class ShapeType { Circle, Polygon };

struct MassData {
    float mass = 0.0f;
    float inertia = 0.0f;
};

struct Shape {
    ShapeType type = ShapeType::Circle;
    float radius = 0.0f;
    std::vector<Vec2> vertices; // local space, CCW, centroid at origin
    std::vector<Vec2> normals;

    static Shape circle(float radius);
    static Shape box(float halfWidth, float halfHeight);
    static Shape polygon(std::vector<Vec2> points); // convex hull is taken

    MassData computeMass(float density) const;
};

} // namespace pe
