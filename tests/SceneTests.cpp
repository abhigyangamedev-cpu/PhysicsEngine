#include "Check.h"
#include "Helpers.h"
#include <cstdio>
#include <fstream>

using namespace th;

namespace {

std::string tempPath(const char* name) { return std::string("editor_test_") + name + ".txt"; }

} // namespace

TEST(scene_default_has_ground_at_zero) {
    Scene s = defaultScene();
    CHECK(s.objects.size() == 1);
    const ObjectSpec& g = s.objects[0];
    CHECK(g.fixed);
    CHECK_NEAR(g.pos.y + g.height * 0.5f, 0.0, 1e-6);
    CHECK_NEAR(g.friction, 1.0, 1e-6);
}

TEST(scene_names_skip_used_letters) {
    Scene s = empty();
    CHECK(s.nextName() == "A");
    add(s, block({}, 1, 1, 1));
    CHECK(s.nextName() == "B");
    ObjectSpec c = block({}, 1, 1, 1);
    c.name = "C";
    add(s, c);
    CHECK(s.nextName() == "B");
    for (int i = 0; i < 30; ++i) add(s, block({}, 1, 1, 1));
    CHECK(s.object(s.objects.back().id)->name.size() >= 2); // past Z the names become A1, A2, ...
}

TEST(scene_lookup_by_id) {
    Scene s = empty();
    int a = add(s, block({1, 2}, 1, 1, 1));
    int b = add(s, link(LinkKind::Spring, at(a), fixedPoint({0, 0})));
    CHECK(s.object(a) && s.object(a)->pos.x == 1.0f);
    CHECK(s.link(b) && s.link(b)->kind == LinkKind::Spring);
    CHECK(s.object(999) == nullptr);
    CHECK(s.link(999) == nullptr);
}

TEST(scene_remove_object_removes_its_links) {
    Scene s = empty();
    int a = add(s, block({0, 0}, 1, 1, 1));
    int b = add(s, block({3, 0}, 1, 1, 1));
    add(s, link(LinkKind::Spring, at(a), at(b)));
    add(s, link(LinkKind::Rope, at(b), fixedPoint({3, 5})));
    add(s, link(LinkKind::Rod, at(a), fixedPoint({0, 5})));
    s.removeObject(b);
    CHECK(s.objects.size() == 1);
    CHECK(s.links.size() == 1);
    CHECK(s.links[0].kind == LinkKind::Rod);
    s.removeLink(s.links[0].id);
    CHECK(s.links.empty());
}

TEST(scene_endpoint_frames_round_trip) {
    Scene s = empty();
    ObjectSpec o = block({2, 3}, 2, 1, 1);
    o.angleDeg = 90.0f;
    int id = add(s, o);
    Endpoint e = s.endpointAt(id, {2, 4}); // one metre above the centre = +x in the rotated frame
    CHECK_NEAR(e.local.x, 1.0, 1e-5);
    CHECK_NEAR(e.local.y, 0.0, 1e-5);
    Vec2 back = s.endpointWorld(e);
    CHECK_NEAR(back.x, 2.0, 1e-5);
    CHECK_NEAR(back.y, 4.0, 1e-5);
    Endpoint w = s.endpointAt(-1, {7, 8});
    CHECK(w.obj == -1 && s.endpointWorld(w).x == 7.0f);
}

TEST(scene_shapes_have_the_right_area) {
    ObjectSpec b = block({}, 2, 3, 1);
    CHECK_NEAR(b.shape().computeMass(1.0f).mass, 6.0, 1e-4);
    ObjectSpec c = ball({}, 0.5f, 1);
    CHECK_NEAR(c.shape().computeMass(1.0f).mass, kPi * 0.25, 1e-3);
    ObjectSpec w = wedge({}, 3, 45);
    CHECK_NEAR(w.shape().computeMass(1.0f).mass, 4.5, 1e-3);
    w.flip = true;
    CHECK_NEAR(w.shape().computeMass(1.0f).mass, 4.5, 1e-3);
}

TEST(scene_build_honours_mass_and_flags) {
    Scene s = empty(9.81f);
    ObjectSpec a = block({0, 0}, 2, 0.5f, 7.0f);
    a.noSpin = true;
    add(s, a);
    ObjectSpec b = ball({5, 0}, 0.3f, 2.5f);
    b.noSpin = false;
    b.velocity = {1, 2};
    b.charge = -3.0f;
    add(s, b);
    ObjectSpec c = wedge({10, 0}, 4, 30, false);
    c.mass = 11.0f;
    add(s, c);
    ObjectSpec d = block({-5, 0}, 1, 1, 1);
    d.fixed = true;
    add(s, d);

    Simulation sim;
    sim.load(s);
    CHECK_REL(sim.body(0)->mass, 7.0, 1e-4);
    CHECK(sim.body(0)->invInertia == 0.0f);
    CHECK_REL(sim.body(1)->mass, 2.5, 1e-4);
    CHECK(sim.body(1)->invInertia > 0.0f);
    CHECK(sim.body(1)->velocity.y == 2.0f);
    CHECK(sim.body(1)->charge == -3.0f);
    CHECK_REL(sim.body(2)->mass, 11.0, 1e-4);
    CHECK(sim.body(3)->isStatic);
    CHECK_NEAR(sim.world().settings.gravity.y, -9.81, 1e-6);
    CHECK(sim.world().settings.minFriction);
}

TEST(scene_build_creates_fields) {
    Scene s = empty();
    Simulation sim;
    sim.load(s);
    const size_t base = sim.world().fields().size(); // the spring field is always there
    s.world.E = {1, 0};
    s.world.B = 2.0f;
    s.world.coulomb = true;
    s.world.mutualGravity = true;
    sim.load(s);
    CHECK(sim.world().fields().size() == base + 4);
    CHECK_NEAR(sim.world().magneticFieldAt({3, 3}), 2.0, 1e-6);
}

TEST(scene_spring_rest_length) {
    Scene s = empty();
    int a = add(s, block({0, 0}, 1, 1, 1));
    LinkSpec auto_ = link(LinkKind::Spring, fixedPoint({-3, 0}), at(a)); // world end first: gets swapped
    add(s, auto_);
    LinkSpec set = link(LinkKind::Spring, at(a), fixedPoint({0, 2}));
    set.length = 0.5f;
    add(s, set);
    Simulation sim;
    sim.load(s);
    const auto& springs = sim.built().springs->springs;
    CHECK(springs.size() == 2);
    CHECK_NEAR(springs[0].rest, 3.0, 1e-5);
    CHECK_NEAR(springs[1].rest, 0.5, 1e-6);
    CHECK(sim.built().springIndex[0] == 0 && sim.built().springIndex[1] == 1);
    CHECK_NEAR(s.initialLength(s.links[0]), 3.0, 1e-5);
}

TEST(scene_links_between_fixed_points_are_skipped) {
    Scene s = empty();
    add(s, link(LinkKind::Spring, fixedPoint({0, 0}), fixedPoint({1, 0})));
    Simulation sim;
    sim.load(s);
    CHECK(sim.built().springs->springs.empty());
    CHECK(sim.built().springIndex[0] == -1);
}

TEST(scene_wheel_contacts_atwood_wraps_over_the_top) {
    auto [a, b] = wheelContacts({0, 8}, 0.5f, {-0.5f, 5}, {0.5f, 5});
    CHECK_NEAR(a.x, -0.5, 1e-4);
    CHECK_NEAR(a.y, 8.0, 1e-3);
    CHECK_NEAR(b.x, 0.5, 1e-4);
    CHECK_NEAR(b.y, 8.0, 1e-3);
}

TEST(scene_wheel_contacts_rope_back_under) {
    // both strands leave to the left: one over the top, one back under the bottom
    auto [a, b] = wheelContacts({10, 0.95f}, 0.45f, {-5, 1.4f}, {0, 0.5f});
    CHECK_NEAR(a.x, 10.0, 1e-3);
    CHECK_NEAR(a.y, 1.4, 1e-3);
    CHECK_NEAR(b.x, 10.0, 1e-3);
    CHECK_NEAR(b.y, 0.5, 1e-3);
}

TEST(scene_wheel_contacts_table_and_hanging) {
    auto [a, b] = wheelContacts({0.3f, 4.95f}, 0.3f, {-3, 5.25f}, {0.6f, 3});
    CHECK_NEAR(a.y, 5.25, 1e-3); // horizontal strand leaves the top
    CHECK_NEAR(b.x, 0.6, 1e-3);  // vertical strand leaves the right side
    CHECK_NEAR(b.y, 4.95, 1e-2);
}

TEST(scene_pulley_initial_length_is_both_strands) {
    Scene s = empty();
    int a = add(s, block({-0.5f, 5}, 0.4f, 0.4f, 1));
    int b = add(s, block({0.5f, 4}, 0.4f, 0.4f, 1));
    LinkSpec p = link(LinkKind::Pulley, at(a), at(b));
    p.wheel = {0, 8};
    p.wheelRadius = 0.5f;
    add(s, p);
    CHECK_NEAR(s.initialLength(s.links[0]), 3.0 + 4.0, 1e-3);
    s.links[0].ratio = 2.0f;
    CHECK_NEAR(s.initialLength(s.links[0]), 3.0 + 2.0 * 4.0, 1e-3);
}

TEST(scene_save_load_round_trip) {
    Scene s = withGround(10.0f);
    s.world.timeScale = 0.5f;
    s.world.stopAt = 2.5f;
    s.world.drag = 0.1f;
    s.world.E = {1, -2};
    s.world.B = 0.3f;
    s.world.coulomb = true;
    s.world.coulombK = 8.99e9f;
    s.world.mutualGravity = true;
    s.world.G = 2.0f;
    ObjectSpec o = wedge({1, 0}, 3, 37, false);
    o.flip = true;
    o.mass = 4.5f;
    o.velocity = {1.5f, -2};
    o.omega = 0.7f;
    o.force = {3, 4};
    o.friction = 0.25f;
    o.restitution = 0.6f;
    o.charge = 2e-6f;
    o.holdVelocity = true;
    o.angleDeg = 12;
    int id = add(s, o);
    LinkSpec l = link(LinkKind::Pulley, at(id), fixedPoint({1, 2}));
    l.k = 30;
    l.damping = 0.4f;
    l.length = 2.2f;
    l.wheel = {5, 6};
    l.wheelRadius = 0.35f;
    l.ratio = 2;
    add(s, l);

    const std::string path = tempPath("roundtrip");
    CHECK(s.save(path.c_str()));
    Scene r;
    CHECK(r.load(path.c_str()));
    std::remove(path.c_str());

    CHECK(r.objects.size() == s.objects.size());
    CHECK(r.links.size() == 1);
    CHECK(r.nextId == s.nextId);
    CHECK_NEAR(r.world.g, 10.0, 1e-6);
    CHECK_NEAR(r.world.stopAt, 2.5, 1e-6);
    CHECK_NEAR(r.world.E.y, -2.0, 1e-6);
    CHECK(r.world.coulomb && r.world.mutualGravity);
    CHECK_REL(r.world.coulombK, 8.99e9, 1e-5);
    const ObjectSpec& q = r.objects.back();
    CHECK(q.kind == ObjKind::Wedge && q.flip && !q.fixed && q.holdVelocity);
    CHECK(q.name == s.objects.back().name);
    CHECK_NEAR(q.slopeDeg, 37.0, 1e-5);
    CHECK_NEAR(q.velocity.x, 1.5, 1e-6);
    CHECK_NEAR(q.omega, 0.7, 1e-6);
    CHECK_NEAR(q.force.y, 4.0, 1e-6);
    CHECK_REL(q.charge, 2e-6, 1e-5);
    CHECK_NEAR(q.angleDeg, 12.0, 1e-5);
    const LinkSpec& m = r.links[0];
    CHECK(m.kind == LinkKind::Pulley && m.a.obj == id && m.b.obj == -1);
    CHECK_NEAR(m.b.local.y, 2.0, 1e-6);
    CHECK_NEAR(m.length, 2.2, 1e-6);
    CHECK_NEAR(m.wheelRadius, 0.35, 1e-6);
    CHECK_NEAR(m.ratio, 2.0, 1e-6);
}

TEST(scene_loads_files_without_newer_fields) {
    const std::string path = tempPath("old");
    {
        std::ofstream f(path);
        f << "world 9.81 1 0 0 0 0 0 0 8.99e9 0 1\n";
        f << "obj 1 Ground 0 0 -0.5 0 60 1 0.5 4 30 0 1 0 0 0 0 0 1 0 0 1 0\n"; // no holdVelocity column
        f << "obj 2 A 0 0 0.5 0 1 1 0.5 4 30 0 2 0 0 0 0 0 0.3 0 0 0 1\n";
        f << "garbage line\n";
    }
    Scene s;
    CHECK(s.load(path.c_str()));
    std::remove(path.c_str());
    CHECK(s.objects.size() == 2);
    CHECK(!s.objects[1].holdVelocity);
    CHECK(s.objects[1].noSpin);
    CHECK(s.nextId == 3);
}

TEST(scene_load_missing_file_keeps_scene) {
    Scene s = withGround();
    add(s, block({}, 1, 1, 1));
    CHECK(!s.load("definitely_missing_editor_file.txt"));
    CHECK(s.objects.size() == 2);
}
