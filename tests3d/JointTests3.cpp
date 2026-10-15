#include "Check.h"
#include "Scenes3.h"
#include "core/World3.h"
#include <cmath>
#include <cstdio>
#include <numbers>

using namespace pe3;

namespace {

constexpr float kDt = 1.0f / 60.0f;
constexpr float kPi = std::numbers::pi_v<float>;

void run(World& w, float seconds) {
    for (long i = 0, n = std::lround(seconds / kDt); i < n; ++i) w.step(kDt);
}

Body* ball(World& w, Vec3 p, float r, float density = 1.0f) {
    BodyDef d;
    d.position = p;
    d.density = density;
    return w.createBody(Shape::sphere(r), d);
}

Body* box(World& w, Vec3 p, Vec3 half, Quat q = {}) {
    BodyDef d;
    d.position = p;
    d.orientation = q;
    return w.createBody(Shape::box(half), d);
}

World quiet() {
    World w;
    w.settings.angularDamping = 0.0f;
    return w;
}

// time between successive upward zero crossings of f, averaged over the run
template <typename F>
float measurePeriod(World& w, float seconds, F f) {
    float prev = f(), t = 0.0f, first = -1.0f, last = -1.0f;
    int crossings = 0;
    for (long i = 0, n = std::lround(seconds / kDt); i < n; ++i) {
        w.step(kDt);
        t += kDt;
        const float cur = f();
        if (prev < 0.0f && cur >= 0.0f) {
            const float tc = t - kDt * cur / (cur - prev);
            if (first < 0.0f) first = tc;
            else ++crossings;
            last = tc;
        }
        prev = cur;
    }
    return crossings ? (last - first) / crossings : 0.0f;
}

float energy(const World& w) {
    float e = 0.0f;
    for (const auto& b : w.bodies()) {
        if (b->isStatic) continue;
        const Vec3 wl = b->rotation.mulT(b->angularVelocity);
        const Vec3 inv = b->invInertiaLocal;
        e += 0.5f * b->mass * b->velocity.lengthSq() + b->mass * 9.81f * b->position.y;
        e += 0.5f * (wl.x * wl.x / inv.x + wl.y * wl.y / inv.y + wl.z * wl.z / inv.z);
    }
    return e;
}

} // namespace

TEST(SimplePendulumPeriod) {
    World w = quiet();
    const float L = 2.0f, theta = 0.1f;
    Body* bob = ball(w, {L * std::sin(theta), 5 - L * std::cos(theta), 0}, 0.05f);
    w.createJoint<DistanceJoint>(nullptr, bob, Vec3{0, 5, 0}, bob->position);
    const float T = measurePeriod(w, 12.0f, [&] { return bob->position.x; });
    CHECK_REL(T, 2 * kPi * std::sqrt(L / 9.81f), 0.01);
}

// The velocity solver pins the velocity to the tangent at the start of each substep, so a swinging
// joint loses a little energy every substep: about 13% per 10 s at 4 substeps, halving each time the
// substeps double. It must never gain energy.
TEST(BallJointPendulumEnergy) {
    for (int substeps : {4, 16}) {
        World w = quiet();
        w.settings.substeps = substeps;
        Body* rod = box(w, {1.0f, 5, 0}, {1.0f, 0.05f, 0.05f}); // horizontal bar pinned at one end
        w.createJoint<BallJoint>(nullptr, rod, Vec3{0, 5, 0});
        const float e0 = energy(w), swing = rod->mass * 9.81f * 1.0f;
        float worstGap = 0.0f, gain = 0.0f;
        for (int i = 0; i < 600; ++i) {
            w.step(kDt);
            worstGap = std::max(worstGap, (rod->toWorld({-1.0f, 0, 0}) - Vec3{0, 5, 0}).length());
            gain = std::max(gain, energy(w) - e0);
        }
        const float loss = (e0 - energy(w)) / swing;
        CHECK(worstGap < 0.01f);
        CHECK(gain < 0.002f * swing);
        CHECK(loss < (substeps == 4 ? 0.15f : 0.045f));
    }
}

// physical pendulum: a uniform rod pivoted at its end swings with T = 2 pi sqrt(2L / 3g)
TEST(HingedRodPeriod) {
    World w = quiet();
    const float L = 2.0f, theta = 0.1f;
    Body* rod = box(w, {0.5f * L * std::sin(theta), 5 - 0.5f * L * std::cos(theta), 0}, {0.03f, 0.5f * L, 0.03f},
                    Quat::axisAngle({0, 0, 1}, theta));
    w.createJoint<HingeJoint>(nullptr, rod, Vec3{0, 5, 0}, Vec3{0, 0, 1});
    const float T = measurePeriod(w, 12.0f, [&] { return rod->position.x; });
    CHECK_REL(T, 2 * kPi * std::sqrt(2 * L / (3 * 9.81f)), 0.015);
}

TEST(HingeKeepsAxis) {
    World w = quiet();
    w.settings.gravity = {};
    Body* door = box(w, {0.5f, 1, 0}, {0.5f, 1.0f, 0.05f});
    auto* hinge = w.createJoint<HingeJoint>(nullptr, door, Vec3{0, 1, 0}, Vec3{0, 1, 0});
    door->angularVelocity = {3, 2, 3}; // off-axis spin must be removed, on-axis kept
    run(w, 2.0f);
    CHECK(std::abs(dot(door->rotation.c[1], Vec3{0, 1, 0})) > 0.999f);
    CHECK((door->toWorld({-0.5f, 0, 0}) - Vec3{0, 1, 0}).length() < 0.01f);
    CHECK(std::abs(door->angularVelocity.y) > 0.5f);
    CHECK(std::abs(door->angularVelocity.x) < 0.01f && std::abs(door->angularVelocity.z) < 0.01f);
    (void)hinge;
}

TEST(HingeLimitHolds) {
    World w;
    Body* flap = box(w, {1.0f, 5, 0}, {1.0f, 0.05f, 0.3f});
    auto* h = w.createJoint<HingeJoint>(nullptr, flap, Vec3{0, 5, 0}, Vec3{0, 0, 1});
    h->enableLimit = true;
    h->lowerAngle = -0.5f;
    h->upperAngle = 0.5f;
    float lowest = 0.0f;
    for (int i = 0; i < 300; ++i) {
        w.step(kDt);
        lowest = std::min(lowest, h->angle());
    }
    CHECK(lowest > -0.5f - 0.03f);
    CHECK_NEAR(h->angle(), -0.5, 0.02);
}

TEST(HingeMotorReachesSpeed) {
    World w = quiet();
    Body* wheel = box(w, {0, 5, 0}, {1.0f, 0.1f, 0.1f});
    auto* h = w.createJoint<HingeJoint>(nullptr, wheel, Vec3{0, 5, 0}, Vec3{0, 0, 1});
    h->enableMotor = true;
    h->motorSpeed = 4.0f;
    h->maxMotorTorque = 100.0f;
    run(w, 1.0f);
    CHECK_NEAR(wheel->angularVelocity.z, 4.0, 0.05);

    // a weak motor can't hold up an unbalanced arm
    World w2 = quiet();
    Body* arm = box(w2, {1, 5, 0}, {1.0f, 0.1f, 0.1f});
    auto* h2 = w2.createJoint<HingeJoint>(nullptr, arm, Vec3{0, 5, 0}, Vec3{0, 0, 1});
    h2->enableMotor = true;
    h2->motorSpeed = 0.0f;
    h2->maxMotorTorque = 0.5f * arm->mass * 9.81f * 1.0f;
    run(w2, 1.0f);
    CHECK(h2->angle() < -0.5f);
}

TEST(RopeGoesSlackButNotLonger) {
    World w = quiet();
    Body* b = ball(w, {0, 3, 0}, 0.1f);
    auto* rope = w.createJoint<DistanceJoint>(nullptr, b, Vec3{0, 5, 0}, b->position);
    rope->rope = true;
    b->velocity = {0, 4, 0}; // thrown up: the rope must not push
    w.step(kDt);
    CHECK_NEAR(b->velocity.y, 4 - 9.81 * kDt, 0.02);
    run(w, 3.0f);
    CHECK((b->position - Vec3{0, 5, 0}).length() < 2.0f + 0.01f);
}

TEST(SpringJointFrequency) {
    World w = quiet();
    w.settings.gravity = {};
    Body* b = ball(w, {0, 0, 0}, 0.1f);
    auto* spring = w.createJoint<DistanceJoint>(nullptr, b, Vec3{0, 2, 0}, b->position);
    spring->hertz = 2.0f;
    b->position.y -= 0.2f;
    b->updateTransform();
    const float T = measurePeriod(w, 5.0f, [&] { return b->position.y; });
    CHECK_REL(T, 0.5, 0.02);
}

TEST(WeldedBodiesMoveAsOne) {
    World w = quiet();
    scenes3::addGround(w);
    Body* a = box(w, {0, 2, 0}, {0.5f, 0.2f, 0.2f});
    Body* b = box(w, {0.8f, 2.3f, 0}, {0.3f, 0.3f, 0.3f}, Quat::axisAngle({0, 0, 1}, 0.4f));
    w.createJoint<WeldJoint>(a, b, Vec3{0.5f, 2.1f, 0});
    a->velocity = {1, 3, 0};
    a->angularVelocity = {0, 2, 4};
    const Vec3 rel = a->toLocal(b->position);
    const Quat relQ = a->orientation.conjugate() * b->orientation;
    float worst = 0.0f;
    for (int i = 0; i < 300; ++i) {
        w.step(kDt);
        worst = std::max(worst, (a->toLocal(b->position) - rel).length());
    }
    const Quat q = (a->orientation.conjugate() * b->orientation) * relQ.conjugate();
    CHECK(worst < 0.02f);
    CHECK(std::abs(q.w) > 0.9995f);
}

TEST(JoinedBodiesDontCollide) {
    World w = quiet();
    w.settings.gravity = {};
    Body* a = box(w, {0, 0, 0}, {0.5f, 0.5f, 0.5f});
    Body* b = box(w, {0.6f, 0, 0}, {0.5f, 0.5f, 0.5f}); // overlapping
    auto* j = w.createJoint<BallJoint>(a, b, Vec3{0.3f, 0, 0});
    auto touching = [&] {
        for (const auto& [key, m] : w.manifolds())
            if ((m.a == a && m.b == b) || (m.a == b && m.b == a)) return true;
        return false;
    };
    run(w, 1.0f);
    CHECK(!touching());
    CHECK(a->velocity.length() < 1e-3f && b->velocity.length() < 1e-3f);
    j->collideConnected = true; // even after both fell asleep
    w.rebuildJointPairs();
    w.step(kDt);
    CHECK(touching());
}

TEST(GroupFiltersCollisions) {
    World w = quiet();
    w.settings.gravity = {};
    BodyDef d;
    d.group = 3;
    Body* a = w.createBody(Shape::box({0.5f, 0.5f, 0.5f}), d);
    d.position = {0.5f, 0, 0};
    Body* b = w.createBody(Shape::box({0.5f, 0.5f, 0.5f}), d);
    run(w, 0.5f);
    CHECK(a->velocity.length() < 1e-4f && b->velocity.length() < 1e-4f);
}

TEST(HangingChainSleepsAndWakes) {
    World w;
    scenes3::addGround(w);
    Body* prev = nullptr;
    std::vector<Body*> links;
    for (int i = 0; i < 10; ++i) {
        Body* link = ball(w, {0, 8.0f - 0.3f * (i + 1), 0}, 0.1f);
        w.createJoint<BallJoint>(prev, link, Vec3{0, 8.0f - 0.3f * i - 0.15f, 0} + Vec3{0, -0.15f, 0} * 0);
        prev = link;
        links.push_back(link);
    }
    run(w, 5.0f);
    CHECK(w.awakeCount() == 0);
    float gap = 0.0f;
    for (size_t i = 1; i < links.size(); ++i) gap = std::max(gap, std::abs((links[i]->position - links[i - 1]->position).length() - 0.3f));
    CHECK(gap < 0.01f);
    links.back()->velocity = {3, 0, 0};
    w.step(kDt);
    CHECK(w.awakeCount() == 10);
}

TEST(MouseJointDragsAndKeepsAwake) {
    World w;
    scenes3::addGround(w);
    Body* b = box(w, {0, 0.5f, 0}, {0.5f, 0.5f, 0.5f});
    run(w, 2.0f);
    CHECK(!b->awake);
    auto* m = w.createJoint<MouseJoint>(b, b->position);
    m->target = {2, 3, 0};
    run(w, 3.0f);
    CHECK((b->position - m->target).length() < 0.1f);
    CHECK(b->awake);
    w.destroyJoint(m);
    run(w, 3.0f);
    CHECK_NEAR(b->position.y, 0.5, 0.05);
}

TEST(DestroyingBodyRemovesItsJoints) {
    World w;
    Body* a = ball(w, {0, 5, 0}, 0.2f);
    Body* b = ball(w, {1, 5, 0}, 0.2f);
    w.createJoint<BallJoint>(nullptr, a, a->position);
    w.createJoint<DistanceJoint>(a, b, a->position, b->position);
    w.destroyBody(a);
    CHECK(w.joints().empty());
    run(w, 0.5f);
    CHECK(b->position.y < 5.0f - 1.0f);
}

TEST(DoublePendulumStaysBounded) {
    World w = quiet();
    Body* a = ball(w, {1, 5, 0}, 0.1f);
    Body* b = ball(w, {2, 5, 0}, 0.1f);
    w.createJoint<DistanceJoint>(nullptr, a, Vec3{0, 5, 0}, a->position);
    w.createJoint<DistanceJoint>(a, b, a->position, b->position);
    const float e0 = energy(w);
    float worst = 0.0f;
    for (int i = 0; i < 1200; ++i) {
        w.step(kDt);
        worst = std::max(worst, energy(w) - e0);
    }
    CHECK(worst < 0.02f * (a->mass + b->mass) * 9.81f); // never gains energy
    CHECK(std::isfinite(b->position.x));
}

TEST(BallJointFrictionStopsSwing) {
    World w = quiet();
    Body* bob = box(w, {0.0f, 4.0f, 0.0f}, {0.05f, 0.5f, 0.05f}, Quat::axisAngle({0, 0, 1}, 0.4f));
    bob->position = Vec3{0, 4.5f, 0} + bob->rotation * Vec3{0, -0.5f, 0};
    bob->updateTransform();
    auto* j = w.createJoint<BallJoint>(nullptr, bob, Vec3{0, 4.5f, 0});
    run(w, 3.0f);
    CHECK(bob->angularVelocity.length() > 0.1f); // frictionless: still swinging

    j->maxFrictionTorque = 0.3f * bob->mass * 9.81f * 0.5f;
    run(w, 4.0f);
    CHECK(bob->angularVelocity.length() < 0.01f);
    CHECK(std::abs(bob->rotation.c[1].x) > 0.01f); // friction holds it off-vertical rather than letting it settle straight down
}

// momentum and energy pass straight through the row: the struck end ball leaves, the rest stay put
TEST(NewtonsCradleTransfersThroughRow) {
    World w;
    for (const auto& s : scenes3::all())
        if (s.name == "Newton's cradle") s.build(w);
    std::vector<Body*> balls;
    for (auto& b : w.bodies())
        if (!b->isStatic) balls.push_back(b.get());
    CHECK(balls.size() == 5);
    float incoming = 0.0f;
    for (int f = 0; f < 60; ++f) {
        w.step(kDt);
        incoming = std::max(incoming, balls[0]->velocity.x);
        if (balls[4]->velocity.x > 0.5f) break;
    }
    run(w, 0.1f);
    CHECK(balls[4]->velocity.x > 0.9f * incoming);
    CHECK(std::abs(balls[0]->velocity.x) < 0.1f);
    for (int i = 1; i < 4; ++i) CHECK(std::abs(balls[i]->velocity.x) < 0.1f);
}
