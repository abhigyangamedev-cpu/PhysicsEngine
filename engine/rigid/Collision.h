#pragma once
#include "rigid/Body.h"
#include <cstdint>

namespace pe {

struct ContactPoint {
    Vec2 point;
    float depth = 0.0f;
    uint32_t feature = 0; // identifies the contact across frames for warm starting

    Vec2 rA, rB;
    float normalMass = 0.0f, tangentMass = 0.0f;
    float bias = 0.0f;
    float relativeVelocity = 0.0f;
    float normalImpulse = 0.0f, tangentImpulse = 0.0f;
};

struct Manifold {
    Body* a = nullptr;
    Body* b = nullptr;
    Vec2 normal; // points from a to b
    ContactPoint contacts[2];
    int count = 0;
};

bool collide(Body* a, Body* b, Manifold& m);

} // namespace pe
