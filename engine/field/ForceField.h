#pragma once
#include "math/Vec2.h"

namespace pe {

class World;

// Applied once per substep before velocities are integrated; fields change velocities directly.
class ForceField {
public:
    virtual ~ForceField() = default;
    virtual void apply(World& world, float h) = 0;

    // For visualization: the field's contribution at a point.
    virtual Vec2 electricField(const World&, Vec2) const { return {}; }
    virtual float magneticField(Vec2) const { return 0.0f; }
};

// Field confined to a rectangle; an empty region (min > max) means everywhere.
struct Region {
    Aabb box{{1.0f, 1.0f}, {-1.0f, -1.0f}};

    bool everywhere() const { return box.min.x > box.max.x; }
    bool contains(Vec2 p) const {
        return everywhere() || (p.x >= box.min.x && p.x <= box.max.x && p.y >= box.min.y && p.y <= box.max.y);
    }
};

} // namespace pe
