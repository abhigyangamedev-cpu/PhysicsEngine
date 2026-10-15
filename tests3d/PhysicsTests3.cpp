#include "Check.h"
#include "Scenes3.h"
#include "core/World3.h"
#include <cmath>
#include <random>

using namespace pe3;

namespace {

constexpr float kDt = 1.0f / 60.0f;

void run(World& w, float seconds) {
    for (long i = 0, n = std::lround(seconds / kDt); i < n; ++i) w.step(kDt);
}

Body* ball(World& w, Vec3 p, float r, Vec3 v = {}) {
    BodyDef d;
    d.position = p;
    d.velocity = v;
    return w.createBody(Shape::sphere(r), d);
}

Body* box(World& w, Vec3 p, Vec3 half, Quat q = {}) {
    BodyDef d;
    d.position = p;
    d.orientation = q;
    return w.createBody(Shape::box(half), d);
}

bool finite(Vec3 v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

Vec3 angularMomentum(const Body& b) {
    const Vec3 inv = b.invInertiaLocal;
    Vec3 wl = b.rotation.mulT(b.angularVelocity);
    return b.rotation * Vec3{wl.x / inv.x, wl.y / inv.y, wl.z / inv.z};
}

} // namespace

TEST(FreeFallMatchesKinematics) {
    World w;
    Body* b = ball(w, {0, 100, 0}, 0.5f, {3, 0, 0});
    run(w, 2.0f);
    CHECK_NEAR(b->position.y, 100 - 0.5 * 9.81 * 4, 0.15);
    CHECK_NEAR(b->position.x, 6.0, 1e-3);
}

TEST(SphereRestsOnGround) {
    World w;
    scenes3::addGround(w);
    Body* b = ball(w, {0, 3, 0}, 0.5f);
    b->restitution = 0;
    run(w, 3.0f);
    CHECK_NEAR(b->position.y, 0.5, 0.02);
    CHECK(b->velocity.length() < 0.01f);
}

TEST(TiltedBoxSettlesFlat) {
    World w;
    scenes3::addGround(w);
    Body* b = box(w, {0, 2, 0}, {0.5f, 0.5f, 0.5f}, Quat::axisAngle({1, 0, 1}, 0.6f));
    run(w, 4.0f);
    CHECK_NEAR(b->position.y, 0.5, 0.02);
    CHECK(b->velocity.length() < 0.01f);
    CHECK(b->angularVelocity.length() < 0.01f);
    // a face points straight up
    float up = 0;
    for (int i = 0; i < 3; ++i) up = std::max(up, std::abs(b->rotation.c[i].y));
    CHECK_NEAR(up, 1.0, 0.01);
}

TEST(BoxStackStaysUp) {
    World w;
    scenes3::all()[0].build(w);
    std::vector<Vec3> start;
    for (auto& b : w.bodies()) start.push_back(b->position);
    run(w, 10.0f);
    for (size_t i = 1; i < start.size(); ++i) {
        const Vec3 d = w.bodies()[i]->position - start[i];
        CHECK(std::hypot(d.x, d.z) < 0.05f);
        CHECK(std::abs(d.y) < 0.01f * i); // each layer settles into the penetration slop
    }
}

TEST(PyramidStaysUp) {
    World w;
    scenes3::all()[1].build(w);
    std::vector<Vec3> start;
    for (auto& b : w.bodies()) start.push_back(b->position);
    run(w, 8.0f);
    float worst = 0;
    for (size_t i = 1; i < start.size(); ++i) worst = std::max(worst, (w.bodies()[i]->position - start[i]).length());
    CHECK(worst < 0.08f);
}

TEST(ElasticHeadOnSwapsVelocities) {
    World w;
    w.settings.gravity = {};
    w.settings.restitutionThreshold = 0.0f;
    Body* a = ball(w, {-2, 0, 0}, 0.5f, {4, 0, 0});
    Body* b = ball(w, {2, 0, 0}, 0.5f);
    a->restitution = b->restitution = 1.0f;
    a->friction = b->friction = 0.0f;
    run(w, 1.5f);
    CHECK_NEAR(a->velocity.x, 0.0, 0.05);
    CHECK_NEAR(b->velocity.x, 4.0, 0.05);
    CHECK_NEAR(a->velocity.x * a->mass + b->velocity.x * b->mass, 4.0 * a->mass, 1e-3);
}

TEST(SlidingBoxStopsAtFrictionDistance) {
    World w;
    scenes3::addGround(w)->friction = 0.4f;
    Body* b = box(w, {0, 0.5f, 0}, {0.5f, 0.5f, 0.5f});
    b->friction = 0.4f;
    run(w, 0.5f);
    const float x0 = b->position.x, v0 = 5.0f;
    b->velocity = {v0, 0, 0};
    run(w, 3.0f);
    CHECK_REL(b->position.x - x0, v0 * v0 / (2 * 0.4 * 9.81), 0.04);
}

// Kinetic friction converts sliding into rolling at v = 5/7 v0 for a solid sphere.
TEST(SphereStartsRollingAtFiveSevenths) {
    World w;
    w.settings.angularDamping = 0;
    scenes3::addGround(w);
    Body* b = ball(w, {0, 0.5f, 0}, 0.5f);
    b->restitution = 0;
    run(w, 0.5f);
    b->velocity = {7, 0, 0};
    run(w, 3.0f);
    CHECK_REL(b->velocity.x, 5.0, 0.02);
    CHECK_NEAR(b->angularVelocity.z * 0.5f, -b->velocity.x, 0.05);
}

TEST(TorqueFreeSpinConservesAngularMomentum) {
    World w;
    w.settings.gravity = {};
    w.settings.angularDamping = 0;
    Body* b = box(w, {0, 0, 0}, {0.2f, 0.5f, 1.0f});
    b->angularVelocity = {0.1f, 6.0f, 0.1f}; // near the intermediate axis: tumbles
    const Vec3 L0 = angularMomentum(*b);
    run(w, 5.0f);
    const Vec3 L1 = angularMomentum(*b);
    CHECK((L1 - L0).length() < 0.05f * L0.length()); // implicit gyroscopic step is slightly dissipative
    CHECK(std::abs(b->rotation.mulT(b->angularVelocity).y) < 5.9f); // it did flip
}

// finite state and nothing tunnels through the ground
TEST(AllScenesStayFiniteAndAboveGround) {
    for (const auto& s : scenes3::all()) {
        World w;
        s.build(w);
        run(w, 8.0f);
        int bad = 0;
        for (auto& b : w.bodies()) {
            if (b->isStatic) continue;
            const Vec3 p = b->position;
            const bool tunneled = p.y < -0.05f && std::abs(p.x) < 29 && std::abs(p.z) < 29;
            if (!finite(p) || !finite(b->velocity) || !finite(b->angularVelocity) || tunneled) ++bad;
        }
        if (bad) std::printf("    scene %s: %d bad bodies\n", s.name.c_str(), bad);
        CHECK(bad == 0);
    }
}

TEST(RaycastHitsNearestBody) {
    World w;
    scenes3::addGround(w);
    Body* b = box(w, {0, 2, 0}, {0.5f, 0.5f, 0.5f});
    ball(w, {0, 5, 0}, 0.5f);
    RayHit hit = w.raycast({0, 2, 10}, {0, 0, -1});
    CHECK(hit.body == b);
    CHECK_NEAR(hit.t, 9.5, 1e-4);
    CHECK_NEAR(hit.normal.z, 1.0, 1e-4);
}

TEST(RollingResistanceStopsSphere) {
    World w;
    scenes3::addGround(w);
    BodyDef d;
    d.position = {0, 0.5f, 0};
    d.velocity = {3, 0, 0};
    d.rollingResistance = 0.05f;
    Body* b = w.createBody(Shape::sphere(0.5f), d);
    run(w, 10.0f);
    CHECK(b->velocity.length() < 0.01f);
    CHECK(b->position.x < 25.0f);
}

TEST(DominoRingAllFall) {
    World w;
    scenes3::all()[3].build(w);
    run(w, 12.0f);
    int standing = 0;
    for (auto& b : w.bodies())
        if (!b->isStatic && b->rotation.c[1].y > 0.9f) ++standing;
    CHECK(standing == 0);
}

// crossed boxes meet edge to edge, then settle face down
TEST(EdgeOnEdgeDropSettles) {
    World w;
    scenes3::addGround(w);
    Body* low = box(w, {0, 0.5f, 0}, {0.5f, 0.5f, 0.5f});
    Body* top = box(w, {0, 2.5f, 0}, {1.0f, 0.2f, 0.2f}, Quat::axisAngle({0, 1, 0}, 0.7f) * Quat::axisAngle({1, 0, 0}, 0.785f));
    run(w, 6.0f);
    CHECK(finite(top->position));
    CHECK(top->velocity.length() < 0.02f && top->angularVelocity.length() < 0.05f);
    CHECK(low->velocity.length() < 0.02f);
    CHECK(top->position.y > 0.15f);
}

namespace {

int awakeDynamic(const World& w) { return w.awakeCount(); }

} // namespace

TEST(RestingStackFallsAsleepAndStays) {
    World w;
    scenes3::all()[0].build(w);
    run(w, 4.0f);
    CHECK(awakeDynamic(w) == 0);
    std::vector<Vec3> rest;
    for (auto& b : w.bodies()) rest.push_back(b->position);
    run(w, 10.0f);
    for (size_t i = 0; i < rest.size(); ++i) CHECK((w.bodies()[i]->position - rest[i]).lengthSq() == 0.0f);
}

TEST(TossedBallDoesNotSleepAtApex) {
    World w;
    scenes3::addGround(w);
    Body* b = ball(w, {0, 0.5f, 0}, 0.5f, {0, 3, 0});
    b->restitution = 0;
    run(w, 0.4f); // apex at ~0.31 s
    CHECK(b->awake);
    run(w, 0.5f);
    CHECK_NEAR(b->position.y, 0.5, 0.02);
}

TEST(ImpactWakesWholeIsland) {
    World w;
    scenes3::all()[1].build(w);
    run(w, 4.0f);
    CHECK(awakeDynamic(w) == 0);
    Body* corner = w.bodies()[1].get(); // left end of the base row, first in the ball's path
    const Vec3 cornerStart = corner->position;
    BodyDef d;
    d.position = {-8, 1, 0};
    d.velocity = {25, 0, 0};
    d.density = 8;
    w.createBody(Shape::sphere(0.5f), d);
    run(w, 0.6f);
    CHECK(awakeDynamic(w) > 40);
    run(w, 3.0f);
    CHECK((corner->position - cornerStart).length() > 0.5f);
}

TEST(ForceAndVelocityWakeSleepers) {
    World w;
    scenes3::addGround(w);
    Body* a = box(w, {-2, 0.5f, 0}, {0.5f, 0.5f, 0.5f});
    Body* b = box(w, {2, 0.5f, 0}, {0.5f, 0.5f, 0.5f});
    run(w, 2.0f);
    CHECK(!a->awake && !b->awake);
    a->applyForce({0, 50 * a->mass, 0});
    b->velocity = {0, 3, 0};
    w.step(kDt);
    CHECK(a->awake && b->awake);
    CHECK(a->position.y > 0.5f && b->position.y > 0.5f);
}

TEST(RemovingSupportWakesSleepers) {
    World w;
    scenes3::addGround(w);
    BodyDef d;
    d.isStatic = true;
    d.position = {0, 3, 0};
    Body* shelf = w.createBody(Shape::box({2, 0.1f, 2}), d);
    Body* b = box(w, {0, 3.6f, 0}, {0.5f, 0.5f, 0.5f});
    run(w, 2.0f);
    CHECK(!b->awake);
    w.destroyBody(shelf);
    run(w, 2.0f);
    CHECK_NEAR(b->position.y, 0.5, 0.02);
}

TEST(SleepCanBeDisabled) {
    World w;
    w.settings.enableSleep = false;
    scenes3::all()[0].build(w);
    run(w, 4.0f);
    CHECK(awakeDynamic(w) == 10);
}

TEST(AllScenesEventuallySleep) {
    for (const auto& s : scenes3::all()) {
        if (!s.settles) continue;
        World w;
        s.build(w);
        run(w, 20.0f);
        int awake = 0;
        for (auto& b : w.bodies())
            if (!b->isStatic && b->awake && std::abs(b->position.x) < 29 && std::abs(b->position.z) < 29) ++awake;
        if (awake) std::printf("    scene %s: %d still awake\n", s.name.c_str(), awake);
        CHECK(awake == 0);
    }
}

// Random piles of mismatched boxes and spheres spawned overlapping, at random angles and speeds.
// Guards the collision code against degenerate configurations (e.g. clip polygons growing past their buffer).
TEST(RandomPilesStayFinite) {
    for (unsigned seed = 1; seed <= 12; ++seed) {
        World w;
        scenes3::addGround(w);
        std::mt19937 rng(seed);
        std::uniform_real_distribution<float> u(-1.0f, 1.0f), size(0.02f, 1.2f), ang(0.0f, 6.2831853f);
        for (int i = 0; i < 80; ++i) {
            BodyDef d;
            d.position = {u(rng) * 2.0f, 0.5f + i * 0.12f, u(rng) * 2.0f};
            d.orientation = Quat::axisAngle(Vec3{u(rng), u(rng), u(rng)} + Vec3{0, 0, 1e-3f}, ang(rng));
            d.velocity = Vec3{u(rng), u(rng), u(rng)} * (i % 7 == 0 ? 40.0f : 3.0f);
            d.angularVelocity = Vec3{u(rng), u(rng), u(rng)} * 10.0f;
            d.density = 0.2f + 5.0f * (u(rng) + 1.0f);
            const bool flat = i % 5 == 0;
            Shape s = i % 3 == 0 ? Shape::sphere(size(rng) * 0.5f)
                                 : Shape::box({size(rng), flat ? 0.02f : size(rng), flat ? size(rng) * 2 : size(rng)});
            w.createBody(s, d);
        }
        // exact face-on-face alignments as well, where clip vertices land on clip planes
        for (int i = 0; i < 20; ++i) box(w, {0.0f, 3.0f + i * 0.4f, 0.0f}, {0.5f, 0.2f, 0.5f});
        run(w, 6.0f);
        int bad = 0;
        for (auto& b : w.bodies())
            if (!finite(b->position) || !finite(b->velocity) || !finite(b->angularVelocity)) ++bad;
        if (bad) std::printf("    seed %u: %d bad bodies\n", seed, bad);
        CHECK(bad == 0);
    }
}
