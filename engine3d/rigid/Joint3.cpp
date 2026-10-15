#include "rigid/Joint3.h"
#include <cmath>
#include <limits>

namespace pe3 {

namespace {

// Solve the 2x2 symmetric system K x = rhs.
void solve22(float k11, float k12, float k22, float r1, float r2, float& x1, float& x2) {
    float det = k11 * k22 - k12 * k12;
    if (det != 0.0f) det = 1.0f / det;
    x1 = det * (k22 * r1 - k12 * r2);
    x2 = det * (k11 * r2 - k12 * r1);
}

float inverseOrZero(float k) { return k > 0.0f ? 1.0f / k : 0.0f; }

} // namespace

Body* Joint::world() {
    static Body anchor = [] {
        BodyDef d;
        d.isStatic = true;
        Body body(Shape::sphere(0.01f), d);
        body.id = std::numeric_limits<uint32_t>::max();
        return body;
    }();
    return &anchor;
}

Joint::Joint(Body* a, Body* b) : a(a ? a : world()), b(b ? b : world()) {}

void Joint::setAnchors(Vec3 worldA, Vec3 worldB) {
    localA_ = a->toLocal(worldA);
    localB_ = b->toLocal(worldB);
}

Vec3 Joint::pointVelocityError() const {
    return b->velocity + cross(b->angularVelocity, rB_) - a->velocity - cross(a->angularVelocity, rA_);
}

Mat3 Joint::pointMass() const {
    const float m = a->invMass + b->invMass;
    const Mat3 sA = Mat3::skew(rA_), sB = Mat3::skew(rB_);
    return Mat3::diagonal({m, m, m}) + (sA * a->invInertiaWorld * sA + sB * b->invInertiaWorld * sB) * -1.0f;
}

void Joint::applyLinear(Vec3 impulse) {
    a->applyImpulse(-impulse, rA_);
    b->applyImpulse(impulse, rB_);
}

void Joint::applyAngular(Vec3 impulse) {
    a->angularVelocity -= a->invInertiaWorld * impulse;
    b->angularVelocity += b->invInertiaWorld * impulse;
}

// ---------------------------------------------------------------- ball

BallJoint::BallJoint(Body* a, Body* b, Vec3 anchor) : Joint(a, b) { setAnchors(anchor, anchor); }

void BallJoint::prepare(float h) {
    computeArms();
    error_ = b->position + rB_ - a->position - rA_;
    k_ = pointMass();
    soft_ = Softness::make(stiffness(h), kDampingRatio, h);
    kAng_ = a->invInertiaWorld + b->invInertiaWorld;
    maxFrictionImpulse_ = maxFrictionTorque * h;
    if (maxFrictionTorque <= 0.0f) frictionImpulse_ = {};
    applyLinear(impulse_);
    applyAngular(frictionImpulse_);
}

void BallJoint::solve(bool useBias) {
    if (maxFrictionTorque > 0.0f) {
        const Vec3 old = frictionImpulse_;
        frictionImpulse_ += -kAng_.solve(b->angularVelocity - a->angularVelocity);
        if (const float len = frictionImpulse_.length(); len > maxFrictionImpulse_) frictionImpulse_ *= maxFrictionImpulse_ / len;
        applyAngular(frictionImpulse_ - old);
    }
    const Vec3 cdot = pointVelocityError();
    Vec3 lambda = useBias ? k_.solve(cdot + error_ * soft_.biasRate) * -soft_.massScale - impulse_ * soft_.impulseScale
                          : -k_.solve(cdot);
    impulse_ += lambda;
    applyLinear(lambda);
}

// ---------------------------------------------------------------- hinge

HingeJoint::HingeJoint(Body* a, Body* b, Vec3 anchor, Vec3 worldAxis) : Joint(a, b) {
    setAnchors(anchor, anchor);
    const Vec3 axis = worldAxis.normalized();
    Vec3 t1, t2;
    basis(axis, t1, t2);
    axisA_ = this->a->rotation.mulT(axis);
    axisB_ = this->b->rotation.mulT(axis);
    refA_ = this->a->rotation.mulT(t1);
    refB_ = this->b->rotation.mulT(t1);
}

float HingeJoint::angle() const {
    const Vec3 ra = a->rotation * refA_, rb = b->rotation * refB_;
    return std::atan2(dot(cross(ra, rb), axis()), dot(ra, rb));
}

void HingeJoint::prepare(float h) {
    computeArms();
    invH_ = 1.0f / h;
    pointError_ = b->position + rB_ - a->position - rA_;
    k_ = pointMass();
    soft_ = Softness::make(stiffness(h), kDampingRatio, h);

    axis_ = a->rotation * axisA_;
    const Vec3 axisB = b->rotation * axisB_;
    Vec3 p, q;
    basis(axis_, p, q);
    row1_ = cross(axisB, p);
    row2_ = cross(axisB, q);
    err1_ = dot(axisB, p);
    err2_ = dot(axisB, q);

    const Mat3 I = a->invInertiaWorld + b->invInertiaWorld;
    k11_ = dot(row1_, I * row1_);
    k12_ = dot(row1_, I * row2_);
    k22_ = dot(row2_, I * row2_);
    axialMass_ = inverseOrZero(dot(axis_, I * axis_));
    angle_ = angle();

    maxMotorImpulse_ = maxMotorTorque * h;
    if (!enableMotor) motorImpulse_ = 0.0f;
    if (!enableLimit) lowerImpulse_ = upperImpulse_ = 0.0f;

    applyLinear(pointImpulse_);
    applyAngular(row1_ * ang1_ + row2_ * ang2_ + axis_ * (motorImpulse_ + lowerImpulse_ - upperImpulse_));
}

void HingeJoint::solveLimit(float C, float sign, float& impulse, bool useBias) {
    const float cdot = sign * dot(b->angularVelocity - a->angularVelocity, axis_);
    float bias = 0.0f, massScale = 1.0f, impulseScale = 0.0f;
    if (C > 0.0f) {
        bias = C * invH_; // not yet at the stop: may close the gap, not cross it
    } else if (useBias) {
        bias = soft_.biasRate * C;
        massScale = soft_.massScale;
        impulseScale = soft_.impulseScale;
    }
    const float lambda = -axialMass_ * massScale * (cdot + bias) - impulseScale * impulse;
    const float next = std::max(impulse + lambda, 0.0f);
    applyAngular(axis_ * (sign * (next - impulse)));
    impulse = next;
}

void HingeJoint::solve(bool useBias) {
    if (enableMotor) {
        const float cdot = dot(b->angularVelocity - a->angularVelocity, axis_) - motorSpeed;
        const float old = motorImpulse_;
        motorImpulse_ = std::clamp(old - axialMass_ * cdot, -maxMotorImpulse_, maxMotorImpulse_);
        applyAngular(axis_ * (motorImpulse_ - old));
    }
    if (enableLimit) {
        solveLimit(angle_ - lowerAngle, 1.0f, lowerImpulse_, useBias);
        solveLimit(upperAngle - angle_, -1.0f, upperImpulse_, useBias);
    }

    const Vec3 dw = b->angularVelocity - a->angularVelocity;
    float r1 = dot(dw, row1_), r2 = dot(dw, row2_);
    if (useBias) r1 += soft_.biasRate * err1_, r2 += soft_.biasRate * err2_;
    float l1, l2;
    solve22(k11_, k12_, k22_, r1, r2, l1, l2);
    if (useBias) {
        l1 = -soft_.massScale * l1 - soft_.impulseScale * ang1_;
        l2 = -soft_.massScale * l2 - soft_.impulseScale * ang2_;
    } else {
        l1 = -l1, l2 = -l2;
    }
    ang1_ += l1;
    ang2_ += l2;
    applyAngular(row1_ * l1 + row2_ * l2);

    const Vec3 cdot = pointVelocityError();
    Vec3 lambda = useBias ? k_.solve(cdot + pointError_ * soft_.biasRate) * -soft_.massScale - pointImpulse_ * soft_.impulseScale
                          : -k_.solve(cdot);
    pointImpulse_ += lambda;
    applyLinear(lambda);
}

// ---------------------------------------------------------------- distance

DistanceJoint::DistanceJoint(Body* a, Body* b, Vec3 worldA, Vec3 worldB)
    : Joint(a, b), length((worldB - worldA).length()) {
    setAnchors(worldA, worldB);
}

void DistanceJoint::prepare(float h) {
    computeArms();
    const Vec3 d = b->position + rB_ - a->position - rA_;
    const float len = d.length();
    u_ = len > 1e-6f ? d / len : Vec3{};
    const float C = len - length;

    const Vec3 crA = cross(rA_, u_), crB = cross(rB_, u_);
    const float invK = a->invMass + b->invMass + dot(crA, a->invInertiaWorld * crA) + dot(crB, b->invInertiaWorld * crB);
    mass_ = inverseOrZero(invK);

    if (hertz > 0.0f) {
        const float omega = 2.0f * std::numbers::pi_v<float> * hertz;
        const float k = mass_ * omega * omega, c = 2.0f * mass_ * dampingRatio * omega;
        const float g = h * (c + h * k);
        gamma_ = g > 0.0f ? 1.0f / g : 0.0f;
        bias_ = C * h * k * gamma_;
        mass_ = inverseOrZero(invK + gamma_);
    } else {
        gamma_ = 0.0f;
        slack_ = rope && C < 0.0f;
        bias_ = slack_ ? C / h : C;
        soft_ = Softness::make(stiffness(h), kDampingRatio, h);
    }
    applyLinear(u_ * impulse_);
}

void DistanceJoint::solve(bool useBias) {
    const float cdot = dot(u_, pointVelocityError());
    float lambda;
    if (hertz > 0.0f || slack_)
        lambda = -mass_ * (cdot + bias_ + gamma_ * impulse_);
    else if (useBias)
        lambda = -soft_.massScale * mass_ * (cdot + soft_.biasRate * bias_) - soft_.impulseScale * impulse_;
    else
        lambda = -mass_ * cdot;
    if (rope) {
        const float old = impulse_;
        impulse_ = std::min(0.0f, old + lambda);
        lambda = impulse_ - old;
    } else {
        impulse_ += lambda;
    }
    applyLinear(u_ * lambda);
}

// ---------------------------------------------------------------- weld

WeldJoint::WeldJoint(Body* a, Body* b, Vec3 anchor) : Joint(a, b) {
    setAnchors(anchor, anchor);
    relative_ = this->a->orientation.conjugate() * this->b->orientation;
}

void WeldJoint::prepare(float h) {
    computeArms();
    pointError_ = b->position + rB_ - a->position - rA_;
    k_ = pointMass();
    kAng_ = a->invInertiaWorld + b->invInertiaWorld;
    const Quat e = b->orientation * (a->orientation * relative_).conjugate();
    angleError_ = Vec3{e.x, e.y, e.z} * (e.w < 0.0f ? -2.0f : 2.0f);
    soft_ = Softness::make(stiffness(h), kDampingRatio, h);
    applyLinear(pointImpulse_);
    applyAngular(angleImpulse_);
}

void WeldJoint::solve(bool useBias) {
    const Vec3 dw = b->angularVelocity - a->angularVelocity;
    Vec3 lambda = useBias ? kAng_.solve(dw + angleError_ * soft_.biasRate) * -soft_.massScale - angleImpulse_ * soft_.impulseScale
                          : -kAng_.solve(dw);
    angleImpulse_ += lambda;
    applyAngular(lambda);

    const Vec3 cdot = pointVelocityError();
    lambda = useBias ? k_.solve(cdot + pointError_ * soft_.biasRate) * -soft_.massScale - pointImpulse_ * soft_.impulseScale
                     : -k_.solve(cdot);
    pointImpulse_ += lambda;
    applyLinear(lambda);
}

// ---------------------------------------------------------------- mouse

MouseJoint::MouseJoint(Body* body, Vec3 anchor) : Joint(nullptr, body), target(anchor), maxForce(1000.0f * body->mass) {
    setAnchors(anchor, anchor);
}

void MouseJoint::prepare(float h) {
    computeArms();
    const float m = b->mass;
    const float omega = 2.0f * std::numbers::pi_v<float> * hertz;
    const float g = h * (2.0f * m * dampingRatio * omega + h * m * omega * omega);
    gamma_ = g > 0.0f ? 1.0f / g : 0.0f;
    bias_ = (b->position + rB_ - target) * (h * m * omega * omega * gamma_);
    k_ = pointMass() + Mat3::diagonal({gamma_, gamma_, gamma_});
    maxImpulse_ = maxForce * h;
    b->angularVelocity *= 0.98f;
    b->applyImpulse(impulse_, rB_);
}

void MouseJoint::solve(bool) {
    const Vec3 cdot = b->velocity + cross(b->angularVelocity, rB_);
    const Vec3 old = impulse_;
    impulse_ += -k_.solve(cdot + bias_ + impulse_ * gamma_);
    if (const float len = impulse_.length(); len > maxImpulse_) impulse_ *= maxImpulse_ / len;
    b->applyImpulse(impulse_ - old, rB_);
}

} // namespace pe3
