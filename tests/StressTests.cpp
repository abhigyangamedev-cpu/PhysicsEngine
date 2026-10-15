// Stress tests: every demo scene run headless, fuzzed editor scenes, files and expressions, and the
// engine pushed with big piles, degenerate shapes and bodies removed mid-run.
#include "Check.h"
#include "Expr.h"
#include "Helpers.h"
#include "Scenes.h"
#include "core/Parallel.h"
#include "editor/Examples.h"
#include "field/Fields.h"
#include "soft/Builders.h"
#include <atomic>
#include <cctype>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <random>

using namespace th;
using pe::Body;
using pe::BodyDef;
using pe::Shape;
using pe::World;

namespace {

bool finite(Vec2 v) { return std::isfinite(v.x) && std::isfinite(v.y); }

// Number of non-finite or runaway values in a world: bodies, soft-body particles and fluid.
int badState(const World& w, float limit = 1e5f) {
    int bad = 0;
    auto ok = [&](Vec2 p) { return finite(p) && std::abs(p.x) < limit && std::abs(p.y) < limit; };
    for (const auto& b : w.bodies())
        if (!ok(b->position) || !finite(b->velocity) || !std::isfinite(b->angle) || !std::isfinite(b->angularVelocity))
            ++bad;
    for (const auto& p : w.particles.particles)
        if (!ok(p.position) || !finite(p.velocity)) ++bad;
    for (size_t i = 0; i < w.fluid.size(); ++i)
        if (!ok(w.fluid.position[i]) || !finite(w.fluid.velocity[i])) ++bad;
    return bad;
}

void loadDemo(World& w, const ::Scene& scene) {
    w.clear();
    w.settings = {};
    w.particles.damping = 0.5f;
    scene.setup(w);
}

// "nan" or "inf" as a word of their own, as printf writes them (not inside "Resonance")
bool printsNonFinite(const std::string& text) {
    for (const char* word : {"nan", "inf"})
        for (size_t at = text.find(word); at != std::string::npos; at = text.find(word, at + 1)) {
            bool startOk = at == 0 || !std::isalpha(static_cast<unsigned char>(text[at - 1]));
            bool endOk = at + 3 >= text.size() || !std::isalpha(static_cast<unsigned char>(text[at + 3]));
            if (startOk && endOk) return true;
        }
    return false;
}

std::string tempPath(const char* name) { return (std::filesystem::temp_directory_path() / name).string(); }

} // namespace

// ---- Demo scenes ----

TEST(stress_every_demo_scene_runs_and_stays_finite) {
    constexpr float dt = 1.0f / 60.0f;
    for (const ::Scene& scene : scenes()) {
        World w;
        loadDemo(w, scene);
        for (int i = 0; i < 600; ++i) {
            w.step(dt);
            if (scene.update) scene.update(w, dt);
        }
        int bad = badState(w);
        if (bad) std::printf("    %s: %d bad values\n", scene.name, bad);
        CHECK(bad == 0);
        if (scene.readout) {
            ::Readout r;
            scene.readout(w, r);
            for (const auto& t : r.text)
                if (printsNonFinite(t)) {
                    std::printf("    %s: \"%s\"\n", scene.name, t.c_str());
                    CHECK(false);
                }
            for (const auto& l : r.lines) CHECK(finite(l.a) && finite(l.b));
        }
    }
}

TEST(stress_demo_scenes_survive_clicks_spawns_and_reloads) {
    constexpr float dt = 1.0f / 60.0f;
    std::mt19937 rng(7);
    for (const ::Scene& scene : scenes()) {
        World w;
        for (int reload = 0; reload < 2; ++reload) {
            loadDemo(w, scene);
            std::uniform_real_distribution<float> dx(-12.0f, 12.0f), dy(-6.0f, 6.0f);
            for (int i = 0; i < 360; ++i) {
                if (i % 20 == 0) {
                    // what App does on a click: let the scene handle it, otherwise drop a body there
                    Vec2 p = scene.cameraCenter + Vec2{dx(rng), dy(rng)};
                    const int button = (i / 20) % 2 ? 3 : 1;
                    if (!(scene.click && scene.click(w, p, button))) {
                        BodyDef d;
                        d.position = p;
                        w.createBody(button == 1 ? Shape::box(0.5f, 0.5f) : Shape::circle(0.5f), d);
                    }
                    if (scene.stir) w.fluid.stir(p, 1.5f, {20.0f, -10.0f});
                }
                w.step(dt);
                if (scene.update) scene.update(w, dt);
                if (scene.readout && i % 30 == 0) {
                    ::Readout r;
                    scene.readout(w, r);
                }
            }
            int bad = badState(w);
            if (bad) std::printf("    %s (reload %d): %d bad values\n", scene.name, reload, bad);
            CHECK(bad == 0);
        }
    }
}

TEST(stress_switching_between_all_demo_scenes_in_one_world) {
    // App keeps one World and reloads scenes into it; scene statics must not leak between them
    World w;
    for (int round = 0; round < 2; ++round)
        for (const ::Scene& scene : scenes()) {
            loadDemo(w, scene);
            for (int i = 0; i < 30; ++i) {
                w.step(1.0f / 60.0f);
                if (scene.update) scene.update(w, 1.0f / 60.0f);
            }
            if (scene.readout) {
                ::Readout r;
                scene.readout(w, r);
            }
            CHECK(badState(w) == 0);
        }
}

// ---- Engine ----

TEST(stress_large_pile_stays_in_its_box_and_settles) {
    World w;
    BodyDef wall;
    wall.isStatic = true;
    wall.position = {0.0f, -0.5f};
    w.createBody(Shape::box(10.0f, 0.5f), wall);
    wall.position = {-10.5f, 10.0f};
    w.createBody(Shape::box(0.5f, 11.0f), wall);
    wall.position = {10.5f, 10.0f};
    w.createBody(Shape::box(0.5f, 11.0f), wall);
    std::mt19937 rng(3);
    std::uniform_real_distribution<float> size(0.2f, 0.45f);
    for (int i = 0; i < 400; ++i) {
        BodyDef d;
        d.position = {-9.0f + (i % 20) * 0.9f, 1.0f + (i / 20) * 0.9f};
        d.angle = size(rng) * 7.0f;
        float s = size(rng);
        w.createBody(i % 3 == 0 ? Shape::circle(s) : i % 3 == 1 ? Shape::box(s, s * 0.8f)
                                                                : Shape::polygon({{-s, -s}, {s, -s}, {0, s}}), d);
    }
    for (int i = 0; i < 900; ++i) w.step(1.0f / 60.0f);
    CHECK(badState(w) == 0);
    float ke = 0.0f;
    int escaped = 0;
    for (const auto& b : w.bodies()) {
        if (b->isStatic) continue;
        ke += 0.5f * b->mass * b->velocity.lengthSq();
        if (b->position.y < 0.0f || std::abs(b->position.x) > 10.0f) ++escaped;
    }
    CHECK(escaped == 0);
    CHECK(ke < 5.0f); // at rest after 15 s
}

TEST(stress_box_pyramid_stands) {
    World w;
    BodyDef g;
    g.isStatic = true;
    g.position = {0.0f, -0.5f};
    w.createBody(Shape::box(20.0f, 0.5f), g);
    std::vector<Body*> boxes;
    const int rows = 12;
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < rows - r; ++c) {
            BodyDef d;
            d.position = {(c - (rows - r) * 0.5f) * 1.02f + 0.5f, 0.5f + r * 1.0f};
            boxes.push_back(w.createBody(Shape::box(0.5f, 0.5f), d));
        }
    std::vector<Vec2> start;
    for (Body* b : boxes) start.push_back(b->position);
    for (int i = 0; i < 600; ++i) w.step(1.0f / 60.0f);
    float drift = 0.0f;
    for (size_t i = 0; i < boxes.size(); ++i) drift = std::max(drift, (boxes[i]->position - start[i]).length());
    CHECK(drift < 0.15f);
}

TEST(stress_fast_body_does_not_tunnel_through_thick_floor) {
    World w;
    BodyDef g;
    g.isStatic = true;
    g.position = {0.0f, -1.0f};
    w.createBody(Shape::box(20.0f, 1.0f), g);
    BodyDef d;
    d.position = {0.0f, 5.0f};
    d.velocity = {0.0f, -60.0f};
    Body* b = w.createBody(Shape::circle(0.3f), d);
    for (int i = 0; i < 120; ++i) w.step(1.0f / 60.0f);
    CHECK(b->position.y > 0.0f);
}

TEST(stress_degenerate_polygons_do_not_poison_the_world) {
    // collinear, repeated and too-few points: none of these may turn into NaN bodies
    std::vector<std::vector<Vec2>> bad = {
        {{0, 0}, {1, 0}, {2, 0}},
        {{1, 1}, {1, 1}, {1, 1}},
        {{0, 0}, {1, 1}},
        {{0, 0}},
        {},
        {{0, 0}, {1, 0}, {1, 0}, {0, 0}},
    };
    for (const auto& pts : bad) {
        Shape s = Shape::polygon(pts);
        for (Vec2 v : s.vertices) CHECK(finite(v));
        for (Vec2 n : s.normals) CHECK(finite(n));
        World w;
        BodyDef g;
        g.isStatic = true;
        g.position = {0.0f, -0.5f};
        w.createBody(Shape::box(5.0f, 0.5f), g);
        BodyDef d;
        d.position = {0.0f, 0.5f};
        w.createBody(s, d);
        d.position = {0.2f, 1.5f};
        w.createBody(Shape::box(0.5f, 0.5f), d);
        for (int i = 0; i < 120; ++i) w.step(1.0f / 60.0f);
        CHECK(badState(w) == 0);
    }
    Shape zero = Shape::box(0.0f, 0.0f);
    for (Vec2 v : zero.vertices) CHECK(finite(v));
    World w;
    BodyDef d;
    w.createBody(Shape::circle(0.0f), d);
    w.createBody(zero, d);
    for (int i = 0; i < 60; ++i) w.step(1.0f / 60.0f);
    CHECK(badState(w) == 0);
}

TEST(stress_destroying_bodies_mid_run_leaves_no_dangling_springs) {
    World w;
    auto* springs = w.addField<pe::SpringField>();
    BodyDef d;
    std::vector<Body*> bodies;
    for (int i = 0; i < 20; ++i) {
        d.position = {float(i), 0.0f};
        bodies.push_back(w.createBody(Shape::circle(0.2f), d));
    }
    for (int i = 0; i + 1 < 20; ++i)
        springs->add(bodies[i], bodies[i]->position, bodies[i + 1], bodies[i + 1]->position, 10.0f);
    auto ids = makeRope(w.particles, {0, 3}, {5, 3}, 10, 1.0f);
    w.particles.attach(ids.front(), bodies[0]);
    w.fluid.addBlock({2, 2}, {4, 4});
    for (int i = 0; i < 30; ++i) w.step(1.0f / 60.0f);
    for (int i = 0; i < 20; i += 2) w.destroyBody(bodies[i]);
    for (const auto& s : springs->springs) {
        bool aliveA = false, aliveB = s.b == nullptr;
        for (const auto& b : w.bodies()) aliveA |= b.get() == s.a, aliveB |= b.get() == s.b;
        CHECK(aliveA && aliveB);
    }
    for (int i = 0; i < 60; ++i) w.step(1.0f / 60.0f);
    CHECK(badState(w) == 0);
}

TEST(stress_joints_survive_extreme_mass_ratios) {
    World w;
    BodyDef a;
    a.isStatic = true;
    Body* anchor = w.createBody(Shape::circle(0.1f), a);
    Body* prev = anchor;
    for (int i = 1; i <= 20; ++i) {
        BodyDef d;
        d.position = {float(i) * 0.5f, 0.0f};
        d.density = i == 20 ? 1000.0f : 0.01f;
        Body* b = w.createBody(Shape::circle(0.1f), d);
        w.createJoint<pe::DistanceJoint>(prev, b, prev->position, b->position);
        prev = b;
    }
    for (int i = 0; i < 600; ++i) w.step(1.0f / 60.0f);
    CHECK(badState(w) == 0);
}

TEST(stress_soft_bodies_and_fluid_together) {
    World w;
    BodyDef g;
    g.isStatic = true;
    g.position = {0.0f, -0.5f};
    w.createBody(Shape::box(10.0f, 0.5f), g);
    makeCloth(w.particles, {{-4, 8}, 15, 10, 0.3f, 0.05f, 0, 1e-4f, 1e-2f, 1.5f});
    makeBlob(w.particles, {{3, 6}, 1.0f, 24, 1.0f, 1.0f, 1e-5f, 1e-4f});
    makeJelly(w.particles, {{-3, 3}, 2.0f, 2.0f, 6, 6, 2.0f, 1e-4f});
    w.fluid.addBlock({-2, 0.2f}, {2, 3});
    w.fluid.bounds = {{-12, -2}, {12, 20}};
    for (int i = 0; i < 600; ++i) {
        if (i % 50 == 0) w.fluid.stir({0, 1}, 2.0f, {30, 30});
        w.step(1.0f / 60.0f);
    }
    CHECK(badState(w) == 0);
}

TEST(stress_nbody_gravity_with_coincident_bodies) {
    World w;
    w.settings.gravity = {};
    auto* n = w.addField<pe::NBodyGravity>();
    n->G = 1.0f;
    BodyDef d;
    d.collides = false;
    for (int i = 0; i < 300; ++i) {
        d.position = {float(i % 3) * 1e-7f, 0.0f}; // nearly on top of each other
        w.createBody(Shape::circle(0.1f), d);
    }
    for (int i = 0; i < 60; ++i) w.step(1.0f / 60.0f);
    CHECK(badState(w, 1e9f) == 0);
}

TEST(stress_parallel_for_covers_every_index_once) {
    for (size_t count : {0u, 1u, 7u, 127u, 128u, 129u, 1000u, 100000u}) {
        for (int rep = 0; rep < 20; ++rep) {
            std::vector<std::atomic<int>> hits(count);
            pe::parallelFor(count, [&](size_t b, size_t e) {
                for (size_t i = b; i < e; ++i) hits[i]++;
            }, 16);
            int wrong = 0;
            for (auto& h : hits) wrong += h != 1;
            CHECK(wrong == 0);
        }
    }
}

// ---- Editor ----

namespace {

editor::Scene randomScene(std::mt19937& rng, bool sane) {
    std::uniform_real_distribution<float> u(0.0f, 1.0f);
    auto range = [&](float lo, float hi) { return lo + (hi - lo) * u(rng); };
    auto pick = [&](std::initializer_list<float> v) { return *(v.begin() + rng() % v.size()); };
    editor::Scene s = defaultScene();
    s.world.g = sane ? range(0.0f, 20.0f) : pick({0.0f, -50.0f, 1e4f, 9.81f});
    s.world.drag = sane ? range(0.0f, 1.0f) : pick({0.0f, 100.0f, -1.0f});
    s.world.E = {range(-5, 5), range(-5, 5)};
    s.world.B = u(rng) < 0.3f ? range(-3, 3) : 0.0f;
    s.world.coulomb = u(rng) < 0.3f;
    s.world.coulombK = sane ? 1.0f : 8.99e9f;
    s.world.mutualGravity = u(rng) < 0.3f;
    const int n = 1 + rng() % 12;
    for (int i = 0; i < n; ++i) {
        ObjectSpec o;
        o.kind = ObjKind(rng() % 3);
        o.pos = {range(-8, 8), range(0.5f, 10)};
        o.angleDeg = range(-180, 180);
        if (sane) {
            o.width = range(0.2f, 3);
            o.height = range(0.2f, 3);
            o.radius = range(0.1f, 1.5f);
            o.base = range(0.5f, 4);
            o.slopeDeg = range(5, 70);
            o.mass = range(0.1f, 20);
            o.velocity = {range(-10, 10), range(-10, 10)};
            o.force = {range(-50, 50), range(-50, 50)};
            o.charge = u(rng) < 0.3f ? range(-1e-3f, 1e-3f) : 0.0f;
        } else {
            o.width = pick({0.0f, -1.0f, 1e-6f, 50.0f});
            o.height = pick({0.0f, -1.0f, 1e-6f, 50.0f});
            o.radius = pick({0.0f, -2.0f, 1e-6f, 30.0f});
            o.base = pick({0.0f, -1.0f, 100.0f});
            o.slopeDeg = pick({0.0f, 90.0f, -30.0f, 180.0f});
            o.mass = pick({0.0f, -5.0f, 1e-9f, 1e9f});
            o.velocity = {pick({0.0f, 1e4f, -300.0f}), pick({0.0f, 1e4f})};
            o.force = {pick({0.0f, 1e7f}), pick({0.0f, -1e7f})};
            o.charge = pick({0.0f, 1.0f, -1.0f});
        }
        o.omega = range(-5, 5);
        o.friction = sane ? range(0, 1) : pick({0.0f, 5.0f, -1.0f});
        o.restitution = sane ? range(0, 1) : pick({0.0f, 1.0f, 3.0f, -1.0f});
        o.fixed = u(rng) < 0.2f;
        o.noSpin = u(rng) < 0.5f;
        o.flip = u(rng) < 0.5f;
        o.holdVelocity = u(rng) < 0.15f;
        add(s, o);
    }
    const int links = rng() % 6;
    for (int i = 0; i < links; ++i) {
        auto endpoint = [&]() -> Endpoint {
            if (u(rng) < 0.2f) return fixedPoint({range(-8, 8), range(0, 12)});
            const auto& o = s.objects[rng() % s.objects.size()];
            return {o.id, sane ? Vec2{} : Vec2{range(-1, 1), range(-1, 1)}};
        };
        LinkSpec l = link(LinkKind(rng() % 5), endpoint(), endpoint());
        l.k = sane ? range(1, 200) : pick({0.0f, -10.0f, 1e7f});
        l.damping = sane ? range(0, 2) : pick({0.0f, -1.0f, 1e5f});
        l.length = u(rng) < 0.5f ? -1.0f : sane ? range(0.5f, 5) : pick({0.0f, 1e-6f, 100.0f});
        l.wheel = {range(-5, 5), range(8, 14)};
        l.wheelRadius = sane ? range(0.1f, 1) : pick({0.0f, -1.0f, 20.0f});
        l.ratio = sane ? pick({1.0f, 2.0f}) : pick({0.0f, -1.0f, 1e3f});
        add(s, l);
    }
    return s;
}

} // namespace

TEST(stress_random_sane_editor_scenes_stay_finite) {
    std::mt19937 rng(11);
    int failures = 0;
    for (int trial = 0; trial < 150; ++trial) {
        editor::Scene s = randomScene(rng, true);
        Simulation sim;
        sim.load(s);
        sim.runUntil(3.0f);
        float energy = sim.totalEnergy();
        if (badState(sim.world(), 1e6f) || !std::isfinite(energy) || !std::isfinite(sim.driveForce(0))) {
            if (failures++ < 3) {
                char path[64];
                std::snprintf(path, sizeof path, "stress_scene_%d.txt", trial);
                s.save(tempPath(path).c_str());
                std::printf("    trial %d went non-finite, saved %s\n", trial, tempPath(path).c_str());
            }
        }
        for (size_t i = 0; i < sim.size(); ++i) {
            CHECK(finite(sim.acceleration(int(i))));
            CHECK(std::isfinite(sim.driveForce(int(i))));
        }
    }
    CHECK(failures == 0);
}

TEST(stress_random_hostile_editor_scenes_never_crash_or_go_nan) {
    std::mt19937 rng(23);
    int failures = 0;
    for (int trial = 0; trial < 200; ++trial) {
        editor::Scene s = randomScene(rng, false);
        Simulation sim;
        sim.load(s);
        while (sim.time() < 1.0f && !sim.blewUp()) sim.step();
        sim.totalEnergy();
        int bad = 0;
        for (const auto& b : sim.world().bodies())
            if (!finite(b->position) || !finite(b->velocity) || !std::isfinite(b->angle)) ++bad;
        // extreme inputs may legitimately explode; the simulation must notice before it goes NaN
        if (bad && !sim.blewUp() && failures++ < 3) {
            char path[64];
            std::snprintf(path, sizeof path, "stress_hostile_%d.txt", trial);
            s.save(tempPath(path).c_str());
            std::printf("    trial %d: %d NaN bodies, saved %s\n", trial, bad, tempPath(path).c_str());
        }
    }
    CHECK(failures == 0);
}

TEST(stress_random_scene_save_load_round_trip) {
    std::mt19937 rng(5);
    const std::string path = tempPath("pe_stress_roundtrip.txt");
    for (int trial = 0; trial < 100; ++trial) {
        editor::Scene s = randomScene(rng, trial % 2 == 0);
        if (trial % 3 == 0) s.objects.front().name = "Two words";
        if (trial % 5 == 0) s.objects.back().name = "";
        CHECK(s.save(path.c_str()));
        s.sanitize(); // loading clamps values the same way the inspector does
        editor::Scene t;
        CHECK(t.load(path.c_str()));
        CHECK(t.objects.size() == s.objects.size());
        CHECK(t.links.size() == s.links.size());
        if (t.objects.size() != s.objects.size() || t.links.size() != s.links.size()) continue;
        for (size_t i = 0; i < s.objects.size(); ++i) {
            CHECK(t.objects[i].name == s.objects[i].name);
            CHECK(t.objects[i].id == s.objects[i].id);
            CHECK(t.objects[i].holdVelocity == s.objects[i].holdVelocity);
            CHECK_NEAR(t.objects[i].mass, s.objects[i].mass, std::abs(s.objects[i].mass) * 1e-5);
            CHECK_NEAR(t.objects[i].pos.x, s.objects[i].pos.x, 1e-4);
        }
        CHECK(t.nextId >= s.nextId);
    }
    std::filesystem::remove(path);
}

TEST(stress_garbage_scene_files_load_without_crashing) {
    std::mt19937 rng(9);
    const std::string path = tempPath("pe_stress_garbage.txt");
    const char* tokens[] = {"world", "obj", "link", "1", "-1", "0", "nan", "inf", "1e40", "abc", "99", "-99999",
                            "2147483647", "\n", "\n", " ", "3.5", "A"};
    for (int trial = 0; trial < 300; ++trial) {
        {
            std::ofstream f(path);
            const int n = rng() % 200;
            for (int i = 0; i < n; ++i) f << tokens[rng() % std::size(tokens)] << ' ';
        }
        editor::Scene s;
        s.load(path.c_str());
        // whatever loaded must still build and run
        Simulation sim;
        sim.load(s);
        sim.runUntil(0.1f);
        for (const auto& o : s.objects) {
            CHECK(int(o.kind) >= 0 && int(o.kind) <= 2);
            CHECK(o.id < s.nextId);
        }
        for (const auto& l : s.links) {
            CHECK(int(l.kind) >= 0 && int(l.kind) <= 4);
            CHECK(l.id < s.nextId);
        }
    }
    std::filesystem::remove(path);
}

TEST(stress_editor_scene_mutations_keep_links_consistent) {
    std::mt19937 rng(17);
    for (int trial = 0; trial < 100; ++trial) {
        editor::Scene s = randomScene(rng, true);
        while (!s.objects.empty()) {
            if (!s.links.empty() && rng() % 3 == 0) s.removeLink(s.links[rng() % s.links.size()].id);
            else s.removeObject(s.objects[rng() % s.objects.size()].id);
            for (const auto& l : s.links) {
                CHECK(l.a.obj < 0 || s.object(l.a.obj));
                CHECK(l.b.obj < 0 || s.object(l.b.obj));
            }
            Simulation sim;
            sim.load(s);
            sim.runUntil(0.05f);
        }
    }
}

TEST(stress_examples_reload_in_one_simulation) {
    Simulation sim;
    for (int round = 0; round < 2; ++round)
        for (const Example& ex : examples()) {
            sim.load(ex.setup());
            ExampleRun r;
            while (sim.time() < 30.0f)
                if (sim.step(), ex.monitor(sim, r)) break;
            CHECK(r.done);
            CHECK_REL(r.result, ex.expected, 0.005);
        }
}

// ---- Expressions ----

TEST(stress_expression_fuzz_never_crashes) {
    std::mt19937 rng(13);
    const char* pieces[] = {"1", "2.5", "1e3", "1e400", ".", "+", "-", "*", "/", "^", "(", ")", "sqrt", "sin", "cos",
                            "tan", "asin", "acos", "atan", "pi", "g", " ", "x", "e", "0", "9999999999999"};
    for (int trial = 0; trial < 20000; ++trial) {
        std::string text;
        const int n = rng() % 12;
        for (int i = 0; i < n; ++i) text += pieces[rng() % std::size(pieces)];
        auto v = evaluate(text);
        if (v) CHECK(!std::isnan(*v));
    }
}

TEST(stress_expression_deep_nesting_and_long_input) {
    std::string deep(200000, '(');
    deep += "1";
    deep += std::string(200000, ')');
    evaluate(deep); // must not overflow the stack
    std::string minus(200000, '-');
    evaluate(minus + "1");
    std::string pow = "2";
    for (int i = 0; i < 100000; ++i) pow += "^1";
    evaluate(pow);
    std::string sum = "1";
    for (int i = 0; i < 100000; ++i) sum += "+1";
    auto v = evaluate(sum);
    CHECK(v && std::abs(*v - 100001.0) < 1e-6);
    CHECK(!evaluate(std::string(100000, ')')));
}

TEST(stress_box_hangs_still_from_a_particle_rope) {
    World w;
    w.particles.damping = 0.5f;
    BodyDef a;
    a.isStatic = true;
    a.position = {0.0f, 18.0f};
    Body* hook = w.createBody(Shape::circle(0.08f), a);
    auto rope = makeRope(w.particles, {0.0f, 18.0f}, {0.0f, 12.0f}, 20, 0.5f);
    w.particles.attach(rope.front(), hook);
    BodyDef d;
    d.position = {0.0f, 11.5f};
    Body* box = w.createBody(Shape::box(0.5f, 0.5f), d);
    w.particles.attach(rope.back(), box);
    for (int i = 0; i < 600; ++i) w.step(1.0f / 60.0f);
    CHECK(badState(w) == 0);
    CHECK_NEAR(box->position.x, 0.0, 0.05);
    CHECK_NEAR(box->position.y, 11.5, 0.1);
    CHECK(box->velocity.length() < 0.05f);
}

TEST(stress_light_body_on_heavy_particle_rope_stays_bounded) {
    World w;
    BodyDef d;
    d.position = {0.0f, 0.0f};
    w.particles.damping = 0.0f;
    Body* disc = w.createBody(Shape::circle(0.2f), d); // 0.13 kg holding a 1 kg rope 3 m away
    auto rope = makeRope(w.particles, {0, 3}, {5, 3}, 10, 1.0f);
    w.particles.attach(rope.front(), disc);
    for (int i = 0; i < 300; ++i) w.step(1.0f / 60.0f);
    CHECK(badState(w) == 0);
    // nothing to hold it up: everything falls together, freely
    CHECK_NEAR(disc->velocity.y, -9.81 * 5.0, 2.0);
}

TEST(stress_simulation_flags_a_blow_up_instead_of_running_on_nan) {
    // two 1 C charges on 1 g objects with the real Coulomb constant: nothing survives that
    editor::Scene s = empty();
    s.world.coulomb = true;
    s.world.coulombK = 8.99e9f;
    ObjectSpec a = ball({0.0f, 0.0f}, 0.1f, 1e-3f), b = ball({0.3f, 0.0f}, 0.1f, 1e-3f);
    a.charge = b.charge = 1.0f;
    add(s, a);
    add(s, b);
    Simulation sim;
    sim.load(s);
    sim.runUntil(1.0f);
    CHECK(sim.blewUp());
    sim.load(s);
    CHECK(!sim.blewUp()); // reloading clears it
}

TEST(stress_stiff_spring_on_a_light_object_stays_bounded) {
    editor::Scene s = empty();
    const int id = add(s, ball({1.0f, 0.0f}, 0.1f, 1e-3f));
    LinkSpec l = link(LinkKind::Spring, fixedPoint({0.0f, 0.0f}), at(id));
    l.k = 1e7f;
    l.length = 0.5f;
    add(s, l);
    Simulation sim;
    sim.load(s);
    sim.runUntil(2.0f);
    CHECK(!sim.blewUp());
    CHECK(std::abs(sim.body(id - 1)->position.x) < 2.0f);
}
