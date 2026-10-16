// Edge cases at the boundaries of the engine and the editor's scene model: bad time steps, settings of
// zero, values at the limits of a float, exact round trips, and scene files that lie about their ids.
#include "Check.h"
#include "Helpers.h"
#include "soft/Builders.h"
#include <climits>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>

using namespace th;
using namespace pe;

namespace {

bool finite(Vec2 v) { return std::isfinite(v.x) && std::isfinite(v.y); }

int badBodies(const World& w) {
    int bad = 0;
    for (const auto& b : w.bodies())
        if (!finite(b->position) || !finite(b->velocity) || !std::isfinite(b->angle) || !std::isfinite(b->angularVelocity)) ++bad;
    for (const auto& p : w.particles.particles)
        if (!finite(p.position) || !finite(p.velocity)) ++bad;
    return bad;
}

// A little of everything: a stack, a rope with a box on it and a ball.
void busyWorld(World& w) {
    BodyDef ground;
    ground.isStatic = true;
    ground.position = {0.0f, -0.5f};
    w.createBody(Shape::box(10.0f, 0.5f), ground);
    BodyDef d;
    d.position = {0.0f, 0.5f};
    w.createBody(Shape::box(0.5f, 0.5f), d);
    d.position = {0.0f, 1.5f};
    w.createBody(Shape::circle(0.4f), d);
    BodyDef hook;
    hook.isStatic = true;
    hook.position = {3.0f, 6.0f};
    Body* h = w.createBody(Shape::circle(0.1f), hook);
    auto rope = makeRope(w.particles, {3.0f, 6.0f}, {3.0f, 3.0f}, 8, 0.5f);
    w.particles.attach(rope.front(), h);
    d.position = {3.0f, 2.8f};
    Body* weight = w.createBody(Shape::box(0.3f, 0.3f), d);
    w.particles.attach(rope.back(), weight);
}

// Centre-of-mass velocity of every dynamic body and particle.
Vec2 comVelocity(const World& w) {
    float m = 0.0f;
    Vec2 p;
    for (const auto& b : w.bodies())
        if (!b->isStatic) m += b->mass, p += b->velocity * b->mass;
    for (const auto& q : w.particles.particles)
        if (q.invMass > 0.0f) m += 1.0f / q.invMass, p += q.velocity * (1.0f / q.invMass);
    return p / m;
}

std::string tempPath(const char* name) { return (std::filesystem::temp_directory_path() / name).string(); }

} // namespace

// ---- the world's step ----

TEST(edge_world_step_with_zero_negative_or_nan_dt_changes_nothing) {
    const float bad[] = {0.0f, -1.0f / 60.0f, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()};
    for (float dt : bad) {
        World w;
        busyWorld(w);
        for (int i = 0; i < 30; ++i) w.step(1.0f / 60.0f);
        std::vector<Vec2> before;
        for (const auto& b : w.bodies()) before.push_back(b->position);
        w.step(dt);
        CHECK(badBodies(w) == 0);
        for (size_t i = 0; i < before.size(); ++i) CHECK(w.bodies()[i]->position.x == before[i].x && w.bodies()[i]->position.y == before[i].y);
        for (int i = 0; i < 30; ++i) w.step(1.0f / 60.0f); // and carries on normally afterwards
        CHECK(badBodies(w) == 0);
    }
}

TEST(edge_world_settings_of_zero_still_step) {
    World w;
    busyWorld(w);
    w.settings.substeps = 0;
    w.settings.velocityIterations = 0;
    w.settings.relaxIterations = 0;
    w.settings.particleIterations = 0;
    w.fluid.stepsPerFrame = 0;
    for (int i = 0; i < 60; ++i) w.step(1.0f / 60.0f);
    CHECK(badBodies(w) == 0);
    w.settings.substeps = -3;
    w.settings.particleIterations = -1;
    for (int i = 0; i < 60; ++i) w.step(1.0f / 60.0f);
    CHECK(badBodies(w) == 0);
}

TEST(edge_empty_world_steps) {
    World w;
    for (int i = 0; i < 10; ++i) w.step(1.0f / 60.0f);
    w.particles.addParticle({0.0f, 0.0f}, 0.0f, 0.1f, 0); // a pinned particle and nothing else
    for (int i = 0; i < 10; ++i) w.step(1.0f / 60.0f);
    CHECK(badBodies(w) == 0);
    CHECK(w.particles.particles[0].position.y == 0.0f);
}

// ---- particles ----

TEST(edge_particles_fall_freely_at_any_iteration_count) {
    // re-deriving velocity from positions rounded the gravity increment away once the rope was far
    // from the origin: with fine steps it stalled at a "terminal velocity" of about 20 m/s
    for (int iters : {1, 4, 16}) {
        World w;
        w.settings.particleIterations = iters;
        w.particles.damping = 0.0f;
        makeRope(w.particles, {0.0f, 3.0f}, {5.0f, 3.0f}, 10, 1.0f);
        for (int i = 0; i < 300; ++i) w.step(1.0f / 60.0f);
        CHECK_NEAR(comVelocity(w).y, -9.81 * 5.0, 0.05);
    }
}

TEST(edge_rope_attachment_conserves_momentum_in_free_fall) {
    // a light body on a heavy rope: inside the system the rope pulls the body as hard as the body
    // pulls the rope, so together they fall exactly like anything else
    for (int iters : {1, 4, 16}) {
        World w;
        w.settings.particleIterations = iters;
        w.particles.damping = 0.0f;
        Body* disc = w.createBody(Shape::circle(0.2f), BodyDef{});
        auto rope = makeRope(w.particles, {0.0f, 3.0f}, {5.0f, 3.0f}, 10, 1.0f);
        w.particles.attach(rope.front(), disc);
        for (int i = 1; i <= 300; ++i) {
            w.step(1.0f / 60.0f);
            if (i % 60 == 0) CHECK_NEAR(comVelocity(w).y, -9.81 * i / 60.0, 0.6);
        }
        CHECK(badBodies(w) == 0);
    }
}

TEST(edge_destroying_an_attached_body_mid_step_sequence) {
    World w;
    busyWorld(w);
    for (int i = 0; i < 20; ++i) w.step(1.0f / 60.0f);
    Body* weight = w.bodies().back().get();
    w.destroyBody(weight);
    CHECK(w.particles.attachments.size() == 1); // only the hook's
    for (int i = 0; i < 60; ++i) w.step(1.0f / 60.0f);
    CHECK(badBodies(w) == 0);
    w.clear();
    for (int i = 0; i < 5; ++i) w.step(1.0f / 60.0f);
}

// ---- scene values at the limits ----

TEST(edge_sanitize_turns_non_finite_and_absurd_values_into_usable_ones) {
    const float inf = std::numeric_limits<float>::infinity(), nan = std::numeric_limits<float>::quiet_NaN();
    editor::Scene s = empty(9.81f);
    s.world.g = nan;
    s.world.drag = inf;
    s.world.E = {nan, 1.0f};
    s.world.B = inf;
    s.world.coulombK = -inf;
    s.world.G = nan;
    s.world.timeScale = nan;
    s.world.stopAt = inf;
    ObjectSpec a = block({nan, inf}, 1e30f, -5.0f, 1e30f);
    a.angleDeg = inf;
    a.velocity = {inf, -inf};
    a.omega = nan;
    a.force = {1e38f, nan};
    a.friction = inf;
    a.restitution = nan;
    a.charge = -inf;
    ObjectSpec b = ball({0.0f, 0.0f}, 1e-30f, 0.0f);
    b.radius = nan;
    ObjectSpec c;
    c.kind = ObjKind::Wedge;
    c.base = inf;
    c.slopeDeg = nan;
    const int ia = add(s, a), ib = add(s, b);
    add(s, c);
    LinkSpec l = link(LinkKind::Spring, at(ia), at(ib));
    l.k = inf;
    l.damping = nan;
    l.length = nan;
    l.a.local = {nan, 0.0f};
    l.wheel = {inf, inf};
    l.wheelRadius = nan;
    l.ratio = inf;
    add(s, l);
    s.sanitize();

    const WorldSpec& w = s.world;
    for (float v : {w.g, w.drag, w.E.x, w.E.y, w.B, w.coulombK, w.G, w.timeScale, w.stopAt}) CHECK(std::isfinite(v));
    for (const auto& o : s.objects) {
        for (float v : {o.pos.x, o.pos.y, o.angleDeg, o.width, o.height, o.radius, o.base, o.slopeDeg, o.mass, o.velocity.x,
                        o.velocity.y, o.omega, o.force.x, o.force.y, o.friction, o.restitution, o.charge})
            CHECK(std::isfinite(v));
        CHECK(o.width > 0.0f && o.width <= 1e4f && o.height > 0.0f && o.height <= 1e4f);
        CHECK(o.radius > 0.0f && o.radius <= 1e4f && o.base > 0.0f && o.base <= 1e4f);
        CHECK(o.mass > 0.0f && o.mass <= 1e9f);
        CHECK(o.restitution >= 0.0f && o.restitution <= 1.0f);
    }
    for (const auto& k : s.links)
        for (float v : {k.k, k.damping, k.length, k.a.local.x, k.wheel.x, k.wheel.y, k.wheelRadius, k.ratio}) CHECK(std::isfinite(v));

    Simulation sim;
    sim.load(s);
    while (sim.time() < 2.0f && !sim.blewUp()) sim.step();
    CHECK(sim.blewUp() || badBodies(sim.world()) == 0);
}

TEST(edge_huge_and_tiny_objects_run_or_flag_a_blow_up) {
    // the extremes the inspector accepts, against each other and the ground
    for (float size : {0.01f, 1e4f})
        for (float mass : {1e-3f, 1e9f}) {
            editor::Scene s = withGround();
            add(s, block({0.0f, size * 0.5f + 0.01f}, size, size, mass));
            add(s, ball({0.0f, size + 2.0f}, 0.5f, 1e9f / mass));
            Simulation sim;
            sim.load(s);
            while (sim.time() < 2.0f && !sim.blewUp()) sim.step();
            CHECK(sim.blewUp() || badBodies(sim.world()) == 0);
        }
}

// ---- scene files ----

TEST(edge_scene_save_load_round_trip_is_exact) {
    editor::Scene s = withGround();
    ObjectSpec o = block({0.1234567f, -98765.43f}, 0.3333333f, 1.0f / 7.0f, 2.7182817f);
    o.angleDeg = 12.345679f;
    o.velocity = {1e-7f, 3.1415927f};
    o.friction = 0.30000001f;
    o.charge = -1.234567e-6f;
    const int id = add(s, o);
    LinkSpec l = link(LinkKind::Spring, fixedPoint({0.7777777f, 5.0f}), at(id));
    l.k = 1234.5678f;
    l.length = 0.98765432f;
    add(s, l);
    s.world.g = 9.80665f;
    s.world.coulombK = 8.9875517e9f;
    const std::string path = tempPath("pe_edge_exact.txt");
    CHECK(s.save(path.c_str()));
    editor::Scene t;
    CHECK(t.load(path.c_str()));
    std::filesystem::remove(path);
    CHECK(t.objects.size() == s.objects.size() && t.links.size() == 1);
    if (t.objects.size() != s.objects.size() || t.links.size() != 1) return;
    const ObjectSpec& r = t.objects.back();
    CHECK(r.pos.x == o.pos.x && r.pos.y == o.pos.y);
    CHECK(r.width == o.width && r.height == o.height && r.mass == o.mass);
    CHECK(r.angleDeg == o.angleDeg && r.velocity.x == o.velocity.x && r.velocity.y == o.velocity.y);
    CHECK(r.friction == o.friction && r.charge == o.charge);
    CHECK(t.links[0].k == l.k && t.links[0].length == l.length && t.links[0].a.local.x == l.a.local.x);
    CHECK(t.world.g == s.world.g && t.world.coulombK == s.world.coulombK);
}

TEST(edge_scene_file_with_duplicate_huge_and_dangling_ids) {
    const std::string path = tempPath("pe_edge_ids.txt");
    {
        std::ofstream f(path);
        f << "world 9.81 1 0 0 0 0 0 0 8.99e9 0 1\n";
        // two objects claim id 5, one claims the largest int, and links name ids that are or aren't there
        f << "obj 5 \"A\" 0 0 1 0 1 1 0.5 4 30 0 1 0 0 0 0 0 0 0 0 0 0 0\n";
        f << "obj 5 \"B\" 1 3 1 0 1 1 0.5 4 30 0 1 0 0 0 0 0 0 0 0 0 0 0\n";
        f << "obj 2147483647 \"C\" 0 6 1 0 1 1 0.5 4 30 0 1 0 0 0 0 0 0 0 0 0 0 0\n";
        f << "obj -4 \"D\" 0 9 1 0 1 1 0.5 4 30 0 1 0 0 0 0 0 0 0 0 0 0 0\n";
        f << "link 7 0 5 0 0 2147483647 0 0 50 0 -1 0 0 0.3 1\n";
        f << "link 8 1 42 0 0 5 0 0 50 0 -1 0 0 0.3 1\n";  // object 42 doesn't exist
        f << "link 9 1 -1 0 0 -1 0 0 50 0 -1 0 0 0.3 1\n";  // two fixed points
        f << "link 5 2 -1 0 9 5 0 0 50 0 -1 0 0 0.3 1\n";   // shares its id with an object
    }
    editor::Scene s;
    CHECK(s.load(path.c_str()));
    std::filesystem::remove(path);
    CHECK(s.objects.size() == 4);
    std::vector<int> ids;
    for (const auto& o : s.objects) ids.push_back(o.id);
    for (const auto& l : s.links) ids.push_back(l.id);
    std::sort(ids.begin(), ids.end());
    CHECK(std::adjacent_find(ids.begin(), ids.end()) == ids.end()); // every id unique
    for (int id : ids) CHECK(id >= 0 && id < s.nextId);
    for (const auto& l : s.links) {
        CHECK(l.a.obj < 0 || s.object(l.a.obj));
        CHECK(l.b.obj < 0 || s.object(l.b.obj));
    }
    // 8's object is missing; 7 still joins A to C, whose id was out of range, and 9 joins two fixed points
    CHECK(s.links.size() == 3);
    if (s.links.size() == 3) {
        const ObjectSpec* a = s.object(s.links[0].a.obj);
        const ObjectSpec* c = s.object(s.links[0].b.obj);
        CHECK(a && a->name == "A" && c && c->name == "C");
        CHECK(s.links[2].b.obj >= 0 && s.object(s.links[2].b.obj)->name == "A"); // the first object with id 5
    }
    // adding to the loaded scene gives fresh ids without overflowing
    const int fresh = add(s, ball({0.0f, 12.0f}, 0.5f, 1.0f));
    CHECK(fresh > 0 && std::count(ids.begin(), ids.end(), fresh) == 0);
    Simulation sim;
    sim.load(s);
    sim.runUntil(0.5f);
    CHECK(badBodies(sim.world()) == 0);
}

TEST(edge_scene_load_of_missing_empty_and_binary_files) {
    editor::Scene s = withGround();
    const std::string missing = tempPath("pe_edge_does_not_exist.txt");
    std::filesystem::remove(missing);
    CHECK(!s.load(missing.c_str()));
    CHECK(s.objects.size() == 1); // a failed load leaves the scene alone

    const std::string path = tempPath("pe_edge_binary.txt");
    {
        std::ofstream f(path, std::ios::binary);
    }
    CHECK(s.load(path.c_str()));
    CHECK(s.objects.empty());
    {
        std::ofstream f(path, std::ios::binary);
        for (int i = 0; i < 4096; ++i) f.put(char((i * 131 + 7) & 0xff));
        f << "\nobj 1 \"unterminated 0 0 0\n";
    }
    CHECK(s.load(path.c_str()));
    std::filesystem::remove(path);
    Simulation sim;
    sim.load(s);
    sim.runUntil(0.1f);
}

TEST(edge_scene_save_to_an_unwritable_path_fails_cleanly) {
    editor::Scene s = withGround();
    CHECK(!s.save(""));
    const std::string dir = tempPath("pe_edge_dir_as_file");
    std::filesystem::create_directories(dir);
    CHECK(!s.save(dir.c_str()));
    std::filesystem::remove(dir);
}
