#pragma once
#include "core/Softness3.h"
#include "rigid/Body3.h"
#include <algorithm>

namespace pe3 {

// Bodies passed as nullptr are pinned to the world.
class Joint {
public:
    Joint(Body* a, Body* b);
    virtual ~Joint() = default;

    virtual void prepare(float h) = 0;
    virtual void solve(bool useBias) = 0;
    virtual bool keepsAwake() const { return false; }

    Vec3 anchorA() const { return a->toWorld(localA_); }
    Vec3 anchorB() const { return b->toWorld(localB_); }

    Body* a;
    Body* b;
    bool collideConnected = false;

    static Body* world(); // static anchor standing in for nullptr
    static constexpr float kDampingRatio = 2.0f;
    static float stiffness(float h) { return std::min(60.0f, 0.25f / h); }

protected:
    Vec3 localA_, localB_, rA_, rB_;
    float invH_ = 0.0f;

    void setAnchors(Vec3 worldA, Vec3 worldB);
    void computeArms() { rA_ = a->rotation * localA_, rB_ = b->rotation * localB_; }
    Vec3 pointVelocityError() const;   // relative velocity of the anchors, b minus a
    Mat3 pointMass() const;            // inverse effective mass of the anchor point
    void applyLinear(Vec3 impulse);    // +impulse at b's anchor, -impulse at a's
    void applyAngular(Vec3 impulse);   // +impulse to b, -impulse to a
};

// Ball and socket: the anchors coincide, rotation is free apart from optional joint friction
// and an optional swing cone.
class BallJoint : public Joint {
public:
    BallJoint(Body* a, Body* b, Vec3 worldAnchor);

    float maxFrictionTorque = 0.0f; // resists relative spin, like a stiff shoulder

    // Keeps a direction fixed in b (worldAxis as placed) within `angle` radians of the same direction
    // fixed in a, like a neck that can nod and tilt but not fold the head into the chest.
    void limitSwing(Vec3 worldAxis, float angle);
    bool enableCone = false;
    float coneAngle = 0.0f;
    float swingAngle() const; // current angle between the two directions

    void prepare(float h) override;
    void solve(bool useBias) override;

private:
    Vec3 error_, impulse_, frictionImpulse_;
    Mat3 k_, kAng_;
    float maxFrictionImpulse_ = 0.0f;
    Vec3 coneA_, coneB_, coneAxis_;
    float coneMass_ = 0.0f, swing_ = 0.0f, coneImpulse_ = 0.0f;
    bool pivotTurn_ = false; // one side is fixed: the limit turns the other about the pivot
    float pivotInertia_ = 0.0f;

    void applyCone(float impulse); // angular impulse along coneAxis_: + to b, - to a
    Softness soft_;
};

// Rotation about one axis only, with optional angle limits and motor.
class HingeJoint : public Joint {
public:
    HingeJoint(Body* a, Body* b, Vec3 worldAnchor, Vec3 worldAxis);

    bool enableLimit = false;
    float lowerAngle = 0.0f, upperAngle = 0.0f;
    bool enableMotor = false;
    float motorSpeed = 0.0f; // rad/s of b relative to a about the axis
    float maxMotorTorque = 0.0f; // with motorSpeed 0 this is joint friction

    float angle() const;
    Vec3 axis() const { return a->rotation * axisA_; }

    void prepare(float h) override;
    void solve(bool useBias) override;

private:
    Vec3 axisA_, axisB_, refA_, refB_;
    Vec3 pointError_, pointImpulse_;
    Mat3 k_;
    Vec3 axis_, row1_, row2_;
    float k11_ = 0, k12_ = 0, k22_ = 0, err1_ = 0, err2_ = 0, ang1_ = 0, ang2_ = 0;
    float axialMass_ = 0, angle_ = 0;
    float motorImpulse_ = 0, maxMotorImpulse_ = 0, lowerImpulse_ = 0, upperImpulse_ = 0;
    Softness soft_;

    void solveLimit(float C, float sign, float& impulse, bool useBias);
};

// Rigid rod when hertz == 0, spring otherwise. A rope only resists stretching.
class DistanceJoint : public Joint {
public:
    DistanceJoint(Body* a, Body* b, Vec3 worldAnchorA, Vec3 worldAnchorB);

    float length;
    float hertz = 0.0f;
    float dampingRatio = 0.0f;
    bool rope = false;

    void prepare(float h) override;
    void solve(bool useBias) override;

private:
    Vec3 u_;
    float mass_ = 0, gamma_ = 0, bias_ = 0, impulse_ = 0;
    bool slack_ = false;
    Softness soft_;
};

// Locks the relative position and orientation.
class WeldJoint : public Joint {
public:
    WeldJoint(Body* a, Body* b, Vec3 worldAnchor);

    void prepare(float h) override;
    void solve(bool useBias) override;

private:
    Quat relative_; // qB relative to qA at creation
    Vec3 pointError_, angleError_, pointImpulse_, angleImpulse_;
    Mat3 k_, kAng_;
    Softness soft_;
};

// Soft spring pulling a point on b toward target; a is the world.
class MouseJoint : public Joint {
public:
    MouseJoint(Body* body, Vec3 worldAnchor);

    Vec3 target;
    float hertz = 5.0f;
    float dampingRatio = 0.7f;
    float maxForce;

    bool keepsAwake() const override { return true; }
    void prepare(float h) override;
    void solve(bool useBias) override;

private:
    Vec3 bias_, impulse_;
    Mat3 k_;
    float gamma_ = 0, maxImpulse_ = 0;
};

} // namespace pe3
