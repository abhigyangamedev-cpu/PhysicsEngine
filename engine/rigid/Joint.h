#pragma once
#include "core/Softness.h"
#include "rigid/Body.h"
#include <algorithm>

namespace pe {

class Joint {
public:
    Joint(Body* a, Body* b) : a(a), b(b) {}
    virtual ~Joint() = default;

    virtual void prepare(float h) = 0;
    virtual void solve(bool useBias) = 0;
    virtual Vec2 anchorA() const = 0;
    virtual Vec2 anchorB() const = 0;

    Body* a;
    Body* b;
    bool collideConnected = false;

    static constexpr float kDampingRatio = 2.0f;
    static float stiffness(float h) { return std::min(60.0f, 0.25f / h); }
};

// Rigid rod when hertz == 0, spring otherwise. A rope only resists stretching.
class DistanceJoint : public Joint {
public:
    DistanceJoint(Body* a, Body* b, Vec2 worldAnchorA, Vec2 worldAnchorB);

    float length;
    float hertz = 0.0f;
    float dampingRatio = 0.0f;
    bool rope = false;

    void prepare(float h) override;
    void solve(bool useBias) override;
    Vec2 anchorA() const override { return a->position + a->rot.apply(localA_); }
    Vec2 anchorB() const override { return b->position + b->rot.apply(localB_); }

private:
    Vec2 localA_, localB_, rA_, rB_, u_;
    float mass_ = 0.0f, gamma_ = 0.0f, bias_ = 0.0f, impulse_ = 0.0f;
    bool slack_ = false;
    Softness soft_;
};

class RevoluteJoint : public Joint {
public:
    RevoluteJoint(Body* a, Body* b, Vec2 worldAnchor);

    bool enableMotor = false;
    float motorSpeed = 0.0f;
    float maxMotorTorque = 0.0f;

    void prepare(float h) override;
    void solve(bool useBias) override;
    Vec2 anchorA() const override { return a->position + a->rot.apply(localA_); }
    Vec2 anchorB() const override { return b->position + b->rot.apply(localB_); }

private:
    Vec2 localA_, localB_, rA_, rB_, error_, impulse_;
    float k11_ = 0, k12_ = 0, k22_ = 0;
    Softness soft_;
    float motorMass_ = 0.0f, motorImpulse_ = 0.0f, maxMotorImpulse_ = 0.0f;
};

// A rope from body a up over a fixed pulley at groundA, across to groundB and down to body b.
// Keeps lengthA + ratio * lengthB from exceeding its initial value and only ever pulls.
// ratio > 1 is a block and tackle: b hangs from `ratio` strands, so it moves 1/ratio as far as a
// and is held with ratio times the rope tension.
class PulleyJoint : public Joint {
public:
    PulleyJoint(Body* a, Body* b, Vec2 groundA, Vec2 groundB, Vec2 worldAnchorA, Vec2 worldAnchorB, float ratio = 1.0f);

    Vec2 groundA, groundB;
    float ratio;
    float total; // lengthA + ratio * lengthB with the rope taut

    float lengthA() const { return (groundA - anchorA()).length(); }
    float lengthB() const { return (groundB - anchorB()).length(); }
    float tension() const { return h_ > 0.0f ? impulse_ / h_ : 0.0f; } // force in the rope, last substep

    void prepare(float h) override;
    void solve(bool useBias) override;
    Vec2 anchorA() const override { return a->position + a->rot.apply(localA_); }
    Vec2 anchorB() const override { return b->position + b->rot.apply(localB_); }

private:
    Vec2 localA_, localB_, rA_, rB_, uA_, uB_;
    float mass_ = 0.0f, bias_ = 0.0f, impulse_ = 0.0f, h_ = 0.0f;
    bool slack_ = false;
    Softness soft_;
};

// Couples rotations so that ratio * angleA + angleB stays constant. Meshing gears use
// ratio = radiusA / radiusB (they turn opposite ways); a belt uses -radiusA / radiusB.
class GearJoint : public Joint {
public:
    GearJoint(Body* a, Body* b, float ratio);

    float ratio;

    void prepare(float h) override;
    void solve(bool useBias) override;
    Vec2 anchorA() const override { return a->position; }
    Vec2 anchorB() const override { return b->position; }

private:
    float reference_, error_ = 0.0f, mass_ = 0.0f, impulse_ = 0.0f;
    Softness soft_;
};

// Soft constraint pulling a point on body b toward target. Body a is unused.
class MouseJoint : public Joint {
public:
    MouseJoint(Body* body, Vec2 worldAnchor);

    Vec2 target;
    float hertz = 5.0f;
    float dampingRatio = 0.7f;
    float maxForce;

    void prepare(float h) override;
    void solve(bool useBias) override;
    Vec2 anchorA() const override { return target; }
    Vec2 anchorB() const override { return b->position + b->rot.apply(localB_); }

private:
    Vec2 localB_, rB_, bias_, impulse_;
    float k11_ = 0, k12_ = 0, k22_ = 0, gamma_ = 0, maxImpulse_ = 0;
};

} // namespace pe
