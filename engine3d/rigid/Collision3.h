#pragma once
#include "rigid/Body3.h"

namespace pe3 {

constexpr int kMaxContacts = 4;

struct ContactPoint {
    Vec3 point;
    float depth = 0.0f; // negative = speculative gap
    Vec3 localA;        // matches the contact across frames for warm starting

    Vec3 rA, rB;
    // angular velocity change per unit impulse along the normal and the two tangents, for a and b
    Vec3 angA[3], angB[3];
    float normalMass = 0.0f, tangentMass[2] = {};
    float relativeVelocity = 0.0f;
    float normalImpulse = 0.0f, tangentImpulse[2] = {};
};

struct Manifold {
    uint64_t key = 0; // the body pair, lower id in the high half
    Body* a = nullptr;
    Body* b = nullptr;
    Vec3 normal; // points from a to b
    Vec3 tangent[2];
    ContactPoint contacts[kMaxContacts];
    int count = 0;

    float rollingRadius = 0.0f, rollingMass = 0.0f;
    Vec3 rollingImpulse;
};

// margin: keep speculative contacts up to this gap
bool collide(Body* a, Body* b, float margin, Manifold& m);

} // namespace pe3
