// The 3D editor's scene model, and every 3D real-life problem against its textbook answer.
#include "Check.h"
#include "Problems3.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace editor3;
using pe3::Vec3;

namespace {

ProblemRun solve(const Problem& p, Simulation& sim) {
    sim.load(p.setup());
    ProblemRun r;
    while (sim.time() < 30.0f)
        if (sim.step(), p.monitor(sim, r)) break;
    return r;
}

} // namespace

TEST(Problems3AreWellFormed) {
    CHECK(!problems().empty());
    for (const Problem& p : problems()) {
        CHECK(p.name && *p.name && p.topic && *p.topic && p.question && *p.question && p.answer && *p.answer);
        CHECK(p.setup && p.monitor && p.cameraDistance > 0.0f);
        const Scene s = p.setup();
        bool focus = false;
        for (const auto& o : s.objects) focus |= o.name == p.focus;
        CHECK(focus);
    }
}

TEST(Problems3MatchTextbookAnswers) {
    for (const Problem& p : problems()) {
        Simulation sim;
        const ProblemRun r = solve(p, sim);
        std::printf("    %-28s %.4f (textbook %.4f %s) at t = %.2f s\n", p.name, r.result, p.expected, p.unit, sim.time());
        CHECK(r.done);
        CHECK(!r.lines.empty());
        CHECK(!sim.blewUp());
        CHECK_REL(r.result, p.expected, 0.01);
    }
}

TEST(Problems3RestartCleanly) {
    for (const Problem& p : problems()) {
        Simulation a, b;
        CHECK_NEAR(solve(p, a).result, solve(p, b).result, 1e-4);
    }
}

TEST(Editor3BuildMatchesSpec) {
    Scene s;
    auto& box = s.add("A", ObjKind::Box, {1, 2, 3});
    box.size = {2, 1, 0.5f};
    box.mass = 4.0f;
    box.velocity = {1, 0, 0};
    auto& ball = s.add("B", ObjKind::Ball, {0, 5, 0});
    ball.radius = 0.25f;
    ball.mass = 0.3f;
    ball.fixed = true;
    const int ia = s.objects[0].id, ib = s.objects[1].id;
    s.link(LinkKind::Rod, {ia, {}}, {ib, {}});
    s.link(LinkKind::Spring, {-1, {5, 5, 5}}, {ia, {}}).k = 100.0f;

    Simulation sim;
    sim.load(s);
    CHECK(sim.size() == 2);
    CHECK(sim.built().ground != nullptr);
    CHECK(sim.built().joints.size() == 2);
    CHECK_REL(sim.body(0)->mass, 4.0, 1e-4);
    CHECK(sim.body(1)->isStatic);
    CHECK_NEAR(sim.body(0)->shape.halfExtents.x, 1.0, 1e-6);
    CHECK_NEAR(sim.body(0)->velocity.x, 1.0, 1e-6);
}

TEST(Editor3SpringUsesStiffness) {
    // a 2 kg ball on a k = 200 N/m spring, no gravity: period 2 pi sqrt(m / k)
    Scene s;
    s.world.g = 0.0f;
    s.world.ground = false;
    auto& ball = s.add("M", ObjKind::Ball, {1.5f, 0, 0});
    ball.mass = 2.0f;
    const int id = ball.id;
    auto& spring = s.link(LinkKind::Spring, {-1, {0, 0, 0}}, {id, {}});
    spring.k = 200.0f;
    spring.length = 1.0f;
    Simulation sim;
    sim.load(s);
    float prev = 0.5f, first = -1.0f, last = -1.0f;
    int crossings = 0;
    while (sim.time() < 3.0f) {
        sim.step();
        const float x = sim.body(0)->position.x - 1.0f;
        if (prev > 0.0f && x <= 0.0f) {
            const float t = sim.time();
            if (first < 0.0f) first = t;
            else ++crossings, last = t;
        }
        prev = x;
    }
    CHECK(crossings >= 2);
    CHECK_REL((last - first) / crossings, 2.0 * 3.14159265 * std::sqrt(2.0 / 200.0), 0.02);
}

TEST(Editor3RemovingObjectDropsItsLinks) {
    Scene s = defaultScene();
    const int a = s.objects[0].id;
    const int b = s.add("B", ObjKind::Ball, {2, 1, 0}).id;
    s.link(LinkKind::Rope, {a, {}}, {b, {}});
    s.link(LinkKind::Hinge, {-1, {0, 2, 0}}, {b, {}});
    s.removeObject(b);
    CHECK(s.links.empty());
    CHECK(s.objects.size() == 1);
}

TEST(Editor3HostileValuesStayFinite) {
    Scene s;
    auto& o = s.add("X", ObjKind::Box, {0, 1, 0});
    o.size = {-5, 0, 1e30f};
    o.mass = -3.0f;
    o.velocity = {NAN, 1, 2};
    o.friction = INFINITY;
    auto& l = s.link(LinkKind::Spring, {-1, {0, 3, 0}}, {o.id, {}});
    l.k = -10.0f;
    l.axis = {0, 0, 0};
    s.link(LinkKind::Rod, {-1, {}}, {-1, {1, 1, 1}}); // world to world: dropped
    Simulation sim;
    sim.load(s);
    CHECK(sim.scene().links.size() == 1);
    for (int i = 0; i < 240; ++i) sim.step();
    const Vec3 p = sim.body(0)->position;
    CHECK(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z));
}

TEST(Editor3EndpointsRoundTripThroughTilt) {
    Scene s;
    auto& o = s.add("T", ObjKind::Box, {1, 2, 3});
    o.tiltDeg = {20, -35, 50};
    const Vec3 p{1.4f, 2.7f, 2.2f};
    const Endpoint e = s.endpointAt(o.id, p);
    const Vec3 back = s.endpointWorld(e);
    CHECK_NEAR(back.x, p.x, 1e-5);
    CHECK_NEAR(back.y, p.y, 1e-5);
    CHECK_NEAR(back.z, p.z, 1e-5);
    CHECK(s.endpointAt(-1, p).obj == -1); // world points stay world points
}

TEST(Editor3FreeFallWithoutGround) {
    Scene s;
    s.world.ground = false;
    s.world.g = 9.81f;
    auto& o = s.add("F", ObjKind::Ball, {0, 50, 0});
    o.velocity = {2, 3, -1};
    Simulation sim;
    sim.load(s);
    CHECK(sim.built().ground == nullptr);
    while (sim.time() < 2.0f - Simulation::kDt * 0.5f) sim.step();
    const Vec3 p = sim.body(0)->position;
    CHECK_NEAR(p.x, 4.0, 1e-3);
    CHECK_NEAR(p.z, -2.0, 1e-3);
    CHECK_NEAR(p.y, 50.0 + 3.0 * 2.0 - 0.5 * 9.81 * 4.0, 0.05);
}

TEST(Editor3RodPendulumPeriod) {
    // a small heavy bob on a 2 m rod from a fixed point, 5 degrees out: T = 2 pi sqrt(L / g)
    Scene s;
    s.world.ground = false;
    const float L = 2.0f, a = 5.0f * 3.14159265f / 180.0f;
    auto& bob = s.add("Bob", ObjKind::Ball, {L * std::sin(a), 5.0f - L * std::cos(a), 0});
    bob.radius = 0.02f;
    const int id = bob.id;
    s.link(LinkKind::Rod, {-1, {0, 5, 0}}, {id, {}});
    Simulation sim;
    sim.load(s);
    float prev = sim.body(0)->position.x, first = -1, last = -1;
    int crossings = 0;
    while (sim.time() < 10.0f) {
        sim.step();
        const float x = sim.body(0)->position.x;
        if (prev > 0.0f && x <= 0.0f) {
            const float t = sim.time() - Simulation::kDt * x / (x - prev);
            if (first < 0) first = t;
            else ++crossings, last = t;
        }
        prev = x;
    }
    CHECK(crossings >= 3);
    CHECK_REL((last - first) / crossings, 2.0 * 3.14159265 * std::sqrt(L / 9.81), 0.01);
    CHECK_NEAR((sim.body(0)->position - Vec3{0, 5, 0}).length(), L, 0.01);
}

TEST(Editor3HingeKeepsItsAxis) {
    // a door on a vertical hinge at its edge, pushed: it swings about y and its hinge edge stays put
    Scene s;
    s.world.g = 0.0f;
    s.world.ground = false;
    auto& door = s.add("Door", ObjKind::Box, {0.5f, 1, 0});
    door.size = {1.0f, 2.0f, 0.05f};
    door.spin = {0, 1.5f, 0};
    door.velocity = {0, 0, -0.75f}; // w x r about the hinge: the door turns about its edge from the start
    const int id = door.id;
    auto& h = s.link(LinkKind::Hinge, {-1, {0, 1, 0}}, {id, {}});
    h.axis = {0, 1, 0};
    Simulation sim;
    sim.load(s);
    for (int i = 0; i < 240; ++i) sim.step();
    const pe3::Body* b = sim.body(0);
    const Vec3 edge = b->toWorld({-0.5f, 0, 0});
    CHECK_NEAR(edge.x, 0.0, 0.01);
    CHECK_NEAR(edge.z, 0.0, 0.01);
    CHECK_NEAR(b->angularVelocity.x, 0.0, 0.01);
    CHECK_NEAR(b->angularVelocity.z, 0.0, 0.01);
    CHECK(std::abs(b->angularVelocity.y) > 1.0f);
}

TEST(Editor3BallJointSwingLimitFromSpec) {
    Scene s;
    s.world.ground = false;
    auto& bob = s.add("Bob", ObjKind::Ball, {0, 4, 0});
    bob.radius = 0.1f;
    bob.velocity = {6, 0, 0};
    const int id = bob.id;
    auto& j = s.link(LinkKind::BallJoint, {-1, {0, 5, 0}}, {id, {}});
    j.swingLimitDeg = 30.0f;
    Simulation sim;
    sim.load(s);
    float worst = 0.0f;
    for (int i = 0; i < 480; ++i) {
        sim.step();
        const Vec3 d = sim.body(0)->position - Vec3{0, 5, 0};
        worst = std::max(worst, std::acos(std::clamp(-d.y / d.length(), -1.0f, 1.0f)) * 180.0f / 3.14159265f);
    }
    CHECK(worst < 31.0f); // a bob 1 m out at 6 m/s: the limit must hold, not just slow it
    CHECK(worst > 25.0f);
}

TEST(Editor3NamesSkipUsedOnes) {
    Scene s;
    s.add("A", ObjKind::Box, {});
    s.add("C", ObjKind::Box, {});
    CHECK(s.nextName() == "B");
    s.add("B", ObjKind::Box, {});
    CHECK(s.nextName() == "D");
}

TEST(Problems3HoldUpAtFewerSubsteps) {
    // the problems ask for 8 substeps; at the default 4 the answers should still be close
    for (const Problem& p : problems()) {
        Scene s = p.setup();
        s.world.substeps = 4;
        Simulation sim;
        sim.load(s);
        ProblemRun r;
        while (sim.time() < 30.0f)
            if (sim.step(), p.monitor(sim, r)) break;
        CHECK(r.done);
        CHECK_REL(r.result, p.expected, 0.02);
    }
}
