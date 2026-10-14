#include "rigid/Joint.h"
#include <algorithm>
#include <numbers>

namespace pe {

namespace {

struct Soft {
    float gamma = 0.0f, biasRate = 0.0f;
};

// Box2D-style soft constraint: maps spring frequency/damping to impulse-space gamma and bias.
Soft softParams(float mass, float hertz, float zeta, float h) {
    float omega = 2.0f * std::numbers::pi_v<float> * hertz;
    float k = mass * omega * omega;
    float c = 2.0f * mass * zeta * omega;
    float g = h * (c + h * k);
    if (g == 0.0f) return {};
    return {1.0f / g, h * k / g};
}

// Solve the 2x2 symmetric system K x = rhs.
Vec2 solve22(float k11, float k12, float k22, Vec2 rhs) {
    float det = k11 * k22 - k12 * k12;
    if (det != 0.0f) det = 1.0f / det;
    return {det * (k22 * rhs.x - k12 * rhs.y), det * (k11 * rhs.y - k12 * rhs.x)};
}

Vec2 relativeVelocity(const Body* a, Vec2 rA, const Body* b, Vec2 rB) {
    Vec2 vb = b->velocity + cross(b->angularVelocity, rB);
    return a ? vb - a->velocity - cross(a->angularVelocity, rA) : vb;
}

} // namespace

DistanceJoint::DistanceJoint(Body* a, Body* b, Vec2 worldA, Vec2 worldB)
    : Joint(a, b), length((worldB - worldA).length()), localA_(a->rot.applyT(worldA - a->position)),
      localB_(b->rot.applyT(worldB - b->position)) {}

void DistanceJoint::prepare(float h) {
    rA_ = a->rot.apply(localA_);
    rB_ = b->rot.apply(localB_);
    Vec2 d = b->position + rB_ - a->position - rA_;
    float len = d.length();
    u_ = len > 1e-6f ? d / len : Vec2{};
    float C = len - length;

    float crA = cross(rA_, u_), crB = cross(rB_, u_);
    float invK = a->invMass + b->invMass + a->invInertia * crA * crA + b->invInertia * crB * crB;
    mass_ = invK > 0.0f ? 1.0f / invK : 0.0f;

    if (hertz > 0.0f) {
        Soft s = softParams(mass_, hertz, dampingRatio, h);
        gamma_ = s.gamma;
        bias_ = C * s.biasRate;
        mass_ = invK + gamma_ > 0.0f ? 1.0f / (invK + gamma_) : 0.0f;
    } else {
        gamma_ = 0.0f;
        slack_ = rope && C < 0.0f;
        bias_ = slack_ ? C / h : C;
        soft_ = Softness::make(stiffness(h), kDampingRatio, h);
    }

    Vec2 p = u_ * impulse_;
    a->applyImpulse(-p, rA_);
    b->applyImpulse(p, rB_);
}

void DistanceJoint::solve(bool useBias) {
    float cdot = dot(u_, relativeVelocity(a, rA_, b, rB_));
    float lambda;
    if (hertz > 0.0f || slack_) {
        lambda = -mass_ * (cdot + bias_ + gamma_ * impulse_);
    } else if (useBias) {
        lambda = -soft_.massScale * mass_ * (cdot + soft_.biasRate * bias_) - soft_.impulseScale * impulse_;
    } else {
        lambda = -mass_ * cdot;
    }
    if (rope) {
        float old = impulse_;
        impulse_ = std::min(0.0f, old + lambda);
        lambda = impulse_ - old;
    } else {
        impulse_ += lambda;
    }
    Vec2 p = u_ * lambda;
    a->applyImpulse(-p, rA_);
    b->applyImpulse(p, rB_);
}

RevoluteJoint::RevoluteJoint(Body* a, Body* b, Vec2 world)
    : Joint(a, b), localA_(a->rot.applyT(world - a->position)), localB_(b->rot.applyT(world - b->position)) {}

void RevoluteJoint::prepare(float h) {
    rA_ = a->rot.apply(localA_);
    rB_ = b->rot.apply(localB_);
    const float mA = a->invMass, mB = b->invMass, iA = a->invInertia, iB = b->invInertia;

    k11_ = mA + mB + iA * rA_.y * rA_.y + iB * rB_.y * rB_.y;
    k12_ = -iA * rA_.x * rA_.y - iB * rB_.x * rB_.y;
    k22_ = mA + mB + iA * rA_.x * rA_.x + iB * rB_.x * rB_.x;
    error_ = b->position + rB_ - a->position - rA_;
    soft_ = Softness::make(stiffness(h), kDampingRatio, h);

    motorMass_ = iA + iB > 0.0f ? 1.0f / (iA + iB) : 0.0f;
    maxMotorImpulse_ = maxMotorTorque * h;
    if (!enableMotor) motorImpulse_ = 0.0f;

    a->applyImpulse(-impulse_, rA_);
    b->applyImpulse(impulse_, rB_);
    a->angularVelocity -= iA * motorImpulse_;
    b->angularVelocity += iB * motorImpulse_;
}

void RevoluteJoint::solve(bool useBias) {
    if (enableMotor) {
        float cdot = b->angularVelocity - a->angularVelocity - motorSpeed;
        float old = motorImpulse_;
        motorImpulse_ = std::clamp(old - motorMass_ * cdot, -maxMotorImpulse_, maxMotorImpulse_);
        float lambda = motorImpulse_ - old;
        a->angularVelocity -= a->invInertia * lambda;
        b->angularVelocity += b->invInertia * lambda;
    }

    Vec2 cdot = relativeVelocity(a, rA_, b, rB_);
    Vec2 lambda;
    if (useBias) {
        lambda = solve22(k11_, k12_, k22_, -(cdot + error_ * soft_.biasRate)) * soft_.massScale -
                 impulse_ * soft_.impulseScale;
    } else {
        lambda = solve22(k11_, k12_, k22_, -cdot);
    }
    impulse_ += lambda;
    a->applyImpulse(-lambda, rA_);
    b->applyImpulse(lambda, rB_);
}

MouseJoint::MouseJoint(Body* body, Vec2 world)
    : Joint(nullptr, body), target(world), maxForce(1000.0f * body->mass),
      localB_(body->rot.applyT(world - body->position)) {}

void MouseJoint::prepare(float h) {
    rB_ = b->rot.apply(localB_);
    const float m = b->invMass, i = b->invInertia;
    Soft s = softParams(b->mass, hertz, dampingRatio, h);
    gamma_ = s.gamma;

    k11_ = m + i * rB_.y * rB_.y + gamma_;
    k12_ = -i * rB_.x * rB_.y;
    k22_ = m + i * rB_.x * rB_.x + gamma_;
    bias_ = (b->position + rB_ - target) * s.biasRate;
    maxImpulse_ = maxForce * h;

    b->applyImpulse(impulse_, rB_);
}

void MouseJoint::solve(bool) {
    Vec2 cdot = relativeVelocity(nullptr, {}, b, rB_);
    Vec2 lambda = solve22(k11_, k12_, k22_, -(cdot + bias_ + impulse_ * gamma_));
    Vec2 old = impulse_;
    impulse_ += lambda;
    float len = impulse_.length();
    if (len > maxImpulse_) impulse_ = impulse_ * (maxImpulse_ / len);
    b->applyImpulse(impulse_ - old, rB_);
}

} // namespace pe
