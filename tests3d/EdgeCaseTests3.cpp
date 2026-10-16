// Edge cases for the 3D engine and its editor scenes: bad time steps, zero settings, degenerate
// shapes and joints, and hostile scenes the inspector or a generated problem could hand it.
#include "Check.h"
#include "EditorScene3.h"
#include "core/World3.h"
#include <cmath>
#include <limits>
#include <random>

using namespace pe3;

namespace {

constexpr float kDt = 1.0f / 60.0f;

bool finite(Vec3 v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

int badBodies(const World& w) {
    int bad = 0;
    for (const auto& b : w.bodies())
        if (!finite(b->position) || !finite(b->velocity) || !finite(b->angularVelocity) || !std::isfinite(b->orientation.w)) ++bad;
    return bad;
}

Body* dynamic(World& w, const Shape& s, Vec3 p) {
    BodyDef d;
    d.position = p;
    return w.createBody(s, d);
}

void busyWorld(World& w) {
    BodyDef g;
    g.isStatic = true;
    g.position = {0, -0.5f, 0};
    w.createBody(Shape::box({20, 0.5f, 20}), g);
    for (int i = 0; i < 4; ++i) dynamic(w, Shape::box({0.5f, 0.5f, 0.5f}), {0, 0.5f + i * 1.0f, 0});
    Body* bob = dynamic(w, Shape::sphere(0.2f), {3, 3, 0});
    w.createJoint<DistanceJoint>(nullptr, bob, Vec3{3, 6, 0}, Vec3{3, 3, 0});
}

} // namespace

TEST(Edge3StepWithZeroNegativeOrNanDtChangesNothing) {
    for (float dt : {0.0f, -kDt, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
        World w;
        busyWorld(w);
        for (int i = 0; i < 20; ++i) w.step(kDt);
        std::vector<Vec3> before;
        for (const auto& b : w.bodies()) before.push_back(b->position);
        w.step(dt);
        CHECK(badBodies(w) == 0);
        for (size_t i = 0; i < before.size(); ++i) CHECK((w.bodies()[i]->position - before[i]).lengthSq() == 0.0f);
        for (int i = 0; i < 20; ++i) w.step(kDt);
        CHECK(badBodies(w) == 0);
    }
}

TEST(Edge3SettingsOfZeroStillStep) {
    World w;
    busyWorld(w);
    w.settings.substeps = 0;
    w.settings.velocityIterations = 0;
    w.settings.relaxIterations = 0;
    for (int i = 0; i < 60; ++i) w.step(kDt);
    CHECK(badBodies(w) == 0);
    w.settings.substeps = -2;
    w.settings.timeToSleep = 0.0f;
    w.settings.sleepThreshold = 0.0f;
    for (int i = 0; i < 60; ++i) w.step(kDt);
    CHECK(badBodies(w) == 0);
}

TEST(Edge3CoincidentAndNestedShapes) {
    World w;
    BodyDef g;
    g.isStatic = true;
    g.position = {0, -0.5f, 0};
    w.createBody(Shape::box({10, 0.5f, 10}), g);
    // exactly on top of each other, centred inside one another, and a sphere at a box's centre
    dynamic(w, Shape::sphere(0.5f), {0, 1, 0});
    dynamic(w, Shape::sphere(0.5f), {0, 1, 0});
    dynamic(w, Shape::box({0.5f, 0.5f, 0.5f}), {3, 1, 0});
    dynamic(w, Shape::box({0.5f, 0.5f, 0.5f}), {3, 1, 0});
    dynamic(w, Shape::box({1, 1, 1}), {-3, 1, 0});
    dynamic(w, Shape::sphere(0.3f), {-3, 1, 0});
    dynamic(w, Shape::sphere(0.3f), {6, -0.5f, 0}); // at the ground's centre plane
    for (int i = 0; i < 240; ++i) w.step(kDt);
    CHECK(badBodies(w) == 0);
    for (const auto& b : w.bodies()) CHECK(b->isStatic || b->position.y > -2.0f); // pushed out, not through
}

TEST(Edge3DegenerateShapesStayFinite) {
    World w;
    BodyDef g;
    g.isStatic = true;
    g.position = {0, -0.5f, 0};
    w.createBody(Shape::box({10, 0.5f, 10}), g);
    dynamic(w, Shape::box({1e-4f, 1e-4f, 1e-4f}), {0, 1, 0});
    dynamic(w, Shape::box({5, 1e-4f, 5}), {0, 2, 0}); // a sheet
    dynamic(w, Shape::box({1e-4f, 3, 1e-4f}), {2, 4, 0}); // a needle
    dynamic(w, Shape::sphere(1e-4f), {-2, 1, 0});
    for (int i = 0; i < 240; ++i) w.step(kDt);
    CHECK(badBodies(w) == 0);
}

TEST(Edge3DegenerateJointsStayFinite) {
    World w;
    Body* a = dynamic(w, Shape::sphere(0.2f), {0, 3, 0});
    Body* b = dynamic(w, Shape::sphere(0.2f), {0, 3, 0});
    w.createJoint<DistanceJoint>(a, b, Vec3{0, 3, 0}, Vec3{0, 3, 0}); // zero length between coincident anchors
    Body* c = dynamic(w, Shape::box({0.3f, 0.3f, 0.3f}), {2, 3, 0});
    w.createJoint<HingeJoint>(nullptr, c, Vec3{2, 4, 0}, Vec3{0, 0, 0}); // no axis
    Body* d = dynamic(w, Shape::box({0.3f, 0.3f, 0.3f}), {4, 3, 0});
    w.createJoint<BallJoint>(d, d, Vec3{4, 3, 0}); // a body to itself
    w.createJoint<DistanceJoint>(nullptr, nullptr, Vec3{}, Vec3{1, 0, 0}); // nothing to itself
    for (int i = 0; i < 240; ++i) w.step(kDt);
    CHECK(badBodies(w) == 0);
}

TEST(Edge3DestroyingJointedBodiesAndClearing) {
    World w;
    busyWorld(w);
    for (int i = 0; i < 30; ++i) w.step(kDt);
    w.destroyBody(w.bodies().back().get()); // the bob still has a joint on it
    for (int i = 0; i < 30; ++i) w.step(kDt);
    CHECK(badBodies(w) == 0);
    w.clear();
    for (int i = 0; i < 5; ++i) w.step(kDt);
    CHECK(w.raycast({0, 5, 0}, {0, -1, 0}).body == nullptr);
    CHECK(w.raycast({0, 5, 0}, {0, 0, 0}).body == nullptr); // zero direction
}

TEST(Edge3HostileEditorScenesRunOrFlagABlowUp) {
    std::mt19937 rng(31);
    const float nan = std::numeric_limits<float>::quiet_NaN(), inf = std::numeric_limits<float>::infinity();
    const float wild[] = {0.0f, -1.0f, 1e-30f, 1e30f, nan, inf, -inf, 0.01f, 200.0f, 1e7f};
    auto pickWild = [&] { return wild[rng() % std::size(wild)]; };
    auto pickVec = [&] { return Vec3{pickWild(), pickWild(), pickWild()}; };
    for (int trial = 0; trial < 120; ++trial) {
        editor3::Scene s;
        s.world.g = rng() % 4 == 0 ? pickWild() : 9.81f;
        s.world.substeps = int(rng() % 80) - 20;
        s.world.timeScale = pickWild();
        const int n = 1 + int(rng() % 6);
        for (int i = 0; i < n; ++i) {
            editor3::ObjectSpec& o = s.add(("O" + std::to_string(i)).c_str(), rng() % 2 ? editor3::ObjKind::Box : editor3::ObjKind::Ball,
                                           rng() % 3 ? Vec3{float(i), 1.0f + i, 0.0f} : pickVec());
            if (rng() % 2) o.size = pickVec();
            if (rng() % 2) o.radius = pickWild();
            if (rng() % 2) o.mass = pickWild();
            if (rng() % 3 == 0) o.velocity = pickVec();
            if (rng() % 3 == 0) o.spin = pickVec();
            if (rng() % 3 == 0) o.tiltDeg = pickVec();
            if (rng() % 4 == 0) o.friction = pickWild();
            if (rng() % 4 == 0) o.restitution = pickWild();
            if (rng() % 4 == 0) o.rolling = pickWild();
            o.fixed = rng() % 6 == 0;
        }
        for (int i = 0, links = int(rng() % 4); i < links; ++i) {
            const int a = rng() % 3 ? s.objects[rng() % s.objects.size()].id : -1;
            const int b = rng() % 5 ? s.objects[rng() % s.objects.size()].id : 999; // sometimes dangling
            editor3::LinkSpec& l = s.link(editor3::LinkKind(rng() % 5), {a, rng() % 2 ? pickVec() : Vec3{}}, {b, {}});
            if (rng() % 2) l.length = pickWild();
            if (rng() % 2) l.k = pickWild();
            if (rng() % 2) l.dampingRatio = pickWild();
            if (rng() % 2) l.axis = pickVec();
            if (rng() % 2) l.swingLimitDeg = pickWild();
        }
        editor3::Simulation sim;
        sim.load(s);
        while (sim.time() < 1.0f && !sim.blewUp()) sim.step();
        if (!sim.blewUp() && badBodies(sim.world()) != 0) check::fail(__FILE__, __LINE__, "trial " + std::to_string(trial) + " went NaN unnoticed");
    }
}

TEST(Edge3ABlownUpWorldDoesNotPoisonTheNextOne) {
    // joints pinned to the world share one static anchor body across every world
    {
        World w;
        Body* bob = dynamic(w, Shape::sphere(0.2f), {0, 3, 0});
        w.createJoint<BallJoint>(nullptr, bob, Vec3{0, 4, 0});
        w.step(kDt);
        bob->velocity = {std::numeric_limits<float>::quiet_NaN(), 0, 0}; // what a blow-up leaves behind
        for (int i = 0; i < 5; ++i) w.step(kDt);
    }
    World w;
    Body* bob = dynamic(w, Shape::sphere(0.2f), {1, 3, 0});
    auto* j = w.createJoint<HingeJoint>(nullptr, bob, Vec3{0, 3, 0}, Vec3{0, 0, 1});
    CHECK(finite(j->a->velocity) && finite(j->a->angularVelocity) && finite(j->a->position));
    for (int i = 0; i < 120; ++i) w.step(kDt);
    CHECK(badBodies(w) == 0);
    CHECK_NEAR((bob->position - Vec3{0, 3, 0}).length(), 1.0, 0.02);
}
