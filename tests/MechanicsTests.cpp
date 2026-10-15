// Kinematics, friction, pulleys and wedges against their textbook answers.
#include "Check.h"
#include "Helpers.h"

using namespace th;

// ---- kinematics ----

TEST(free_fall_position_and_velocity) {
    Scene s = empty(9.81f);
    add(s, ball({0, 100}, 0.1f, 1));
    Simulation sim;
    sim.load(s);
    sim.runUntil(2.0f);
    CHECK_NEAR(sim.body(0)->velocity.y, -19.62, 0.01);
    CHECK_NEAR(sim.body(0)->position.y, 100.0 - 0.5 * 9.81 * 4.0, 0.05);
}

TEST(free_fall_is_independent_of_mass) {
    Scene s = empty(10.0f);
    add(s, ball({0, 50}, 0.2f, 0.1f));
    add(s, ball({5, 50}, 0.2f, 100.0f));
    Simulation sim;
    sim.load(s);
    sim.runUntil(1.5f);
    CHECK_NEAR(sim.body(0)->position.y, sim.body(1)->position.y, 1e-4);
}

TEST(projectile_range_height_and_time) {
    Scene s = withGround(9.81f);
    ObjectSpec b = ball({0, 0.1f}, 0.1f, 1);
    const float v = 20.0f, a = 45.0f * kDeg;
    b.velocity = Vec2{std::cos(a), std::sin(a)} * v;
    add(s, b);
    Simulation sim;
    sim.load(s);
    float peak = 0.0f;
    while (sim.time() < 0.5f || sim.body(1)->position.y > 0.1001f) {
        sim.step();
        peak = std::max(peak, sim.body(1)->position.y - 0.1f);
        if (sim.time() > 5.0f) break;
    }
    CHECK_REL(sim.body(1)->position.x, v * v / 9.81f, 0.003);         // R = v^2 sin 2a / g
    CHECK_REL(peak, v * v * 0.5f / (2.0f * 9.81f), 0.005);            // H = v^2 sin^2 a / 2g
    CHECK_NEAR(sim.time(), 2.0f * v * std::sin(a) / 9.81f, 0.01);     // T = 2 v sin a / g
}

TEST(horizontal_launch_from_a_height) {
    Scene s = withGround(10.0f);
    ObjectSpec b = ball({0, 20.1f}, 0.1f, 1);
    b.velocity = {5, 0};
    add(s, b);
    Simulation sim;
    sim.load(s);
    sim.runUntil(1.9f);
    // lands at t = sqrt(2h/g) = 2 s, x = 10 m
    while (sim.body(1)->velocity.y < -0.5f && sim.time() < 3.0f) sim.step();
    CHECK_NEAR(sim.time(), 2.0, 0.02);
    CHECK_NEAR(sim.body(1)->position.x, 10.0, 0.1);
}

TEST(applied_force_gives_f_over_m) {
    Scene s = empty(0.0f);
    ObjectSpec b = block({0, 0}, 1, 1, 2);
    b.force = {6, 0};
    add(s, b);
    Simulation sim;
    sim.load(s);
    sim.runUntil(2.0f);
    CHECK_NEAR(sim.body(0)->velocity.x, 6.0, 1e-3);
    CHECK_NEAR(sim.body(0)->position.x, 6.0, 0.03);
}

TEST(balanced_force_keeps_constant_velocity) {
    Scene s = empty(10.0f);
    ObjectSpec b = ball({0, 0}, 0.5f, 3);
    b.velocity = {0, 4};
    b.force = {0, 30}; // = mg
    add(s, b);
    Simulation sim;
    sim.load(s);
    sim.runUntil(3.0f);
    CHECK_NEAR(sim.body(0)->velocity.y, 4.0, 1e-3);
    CHECK_NEAR(sim.body(0)->position.y, 12.0, 0.01);
}

TEST(linear_drag_reaches_terminal_velocity) {
    Scene s = empty(10.0f);
    s.world.drag = 0.5f;
    add(s, ball({0, 0}, 0.1f, 1));
    Simulation sim;
    sim.load(s);
    sim.runUntil(20.0f);
    CHECK_REL(sim.body(0)->velocity.y, -20.0, 0.005); // v_t = g / c
}

TEST(no_spin_keeps_orientation) {
    Scene s = withGround();
    ObjectSpec b = block({0, 3}, 1, 1, 1);
    b.angleDeg = 20.0f;
    b.noSpin = true;
    add(s, b);
    Simulation sim;
    sim.load(s);
    sim.runUntil(2.0f);
    CHECK_NEAR(sim.body(1)->angle, 20.0f * kDeg, 1e-5);
}

// ---- friction ----

TEST(smooth_incline_acceleration_several_angles) {
    for (float deg : {15.0f, 30.0f, 45.0f, 60.0f}) {
        Scene s = withGround(9.81f);
        add(s, wedge({0, 0}, 10, deg));
        int b = add(s, onSlope({0, 0}, deg, 7.0f, 0.4f, 1));
        s.object(b)->friction = 0.0f;
        Simulation sim;
        sim.load(s);
        Vec2 a = accelBetween(sim, sim.indexOf(b), 0.2f, 0.5f);
        CHECK_REL(a.length(), 9.81 * std::sin(deg * kDeg), 0.005);
    }
}

TEST(rough_incline_acceleration) {
    Scene s = withGround(10.0f);
    add(s, wedge({0, 0}, 10, 37)); // fixed wedge: mu 0, so the block's mu decides via the min rule
    s.objects.back().friction = 1.0f;
    int b = add(s, onSlope({0, 0}, 37, 7.0f, 0.4f, 2));
    s.object(b)->friction = 0.25f;
    Simulation sim;
    sim.load(s);
    Vec2 a = accelBetween(sim, sim.indexOf(b), 0.2f, 0.6f);
    CHECK_REL(a.length(), 10.0 * (std::sin(37 * kDeg) - 0.25 * std::cos(37 * kDeg)), 0.01);
}

TEST(static_friction_holds_block_on_incline) {
    Scene s = withGround(9.81f);
    add(s, wedge({0, 0}, 10, 30));
    s.objects.back().friction = 1.0f;
    int b = add(s, onSlope({0, 0}, 30, 5.0f, 0.5f, 1));
    s.object(b)->friction = 0.7f; // > tan 30 = 0.577
    Simulation sim;
    sim.load(s);
    const Vec2 start = s.object(b)->pos;
    sim.runUntil(2.0f);
    const pe::Body* body = sim.body(sim.indexOf(b));
    CHECK(body->velocity.length() < 0.01f);
    CHECK((body->position - start).length() < 0.02f);
}

TEST(sliding_block_stopping_distance) {
    Scene s = withGround(10.0f);
    ObjectSpec b = block({0, 0.5f}, 1, 1, 3);
    b.friction = 0.4f;
    b.velocity = {4, 0};
    add(s, b);
    Simulation sim;
    sim.load(s);
    sim.runUntil(1.2f); // stops at t = v / (mu g) = 1 s
    CHECK(std::abs(sim.body(1)->velocity.x) < 0.01f);
    CHECK_NEAR(sim.body(1)->position.x, 2.0, 0.02); // d = v^2 / (2 mu g)
}

TEST(force_below_limiting_friction_does_not_move) {
    Scene s = withGround(10.0f);
    ObjectSpec b = block({0, 0.5f}, 1, 1, 5);
    b.friction = 0.5f;
    b.force = {20, 0}; // limiting friction is 25 N
    add(s, b);
    Simulation sim;
    sim.load(s);
    sim.runUntil(2.0f);
    CHECK(std::abs(sim.body(1)->position.x) < 0.005f);
}

TEST(force_above_limiting_friction_accelerates) {
    Scene s = withGround(10.0f);
    ObjectSpec b = block({0, 0.5f}, 1, 1, 5);
    b.friction = 0.5f;
    b.force = {40, 0};
    add(s, b);
    Simulation sim;
    sim.load(s);
    Vec2 a = accelBetween(sim, 1, 0.3f, 1.0f);
    CHECK_REL(a.x, (40.0 - 25.0) / 5.0, 0.01);
}

TEST(two_blocks_move_together_below_slipping_force) {
    // A (1 kg) on B (4 kg), mu 0.5 between them, smooth floor, F on B. Slips above F = 25 N.
    Scene s = withGround(10.0f);
    s.objects[0].friction = 0.0f;
    ObjectSpec b = block({0, 0.5f}, 3, 1, 4);
    b.friction = 1.0f;
    b.force = {15, 0};
    add(s, b);
    ObjectSpec a = block({0, 1.25f}, 1, 0.5f, 1);
    a.friction = 0.5f;
    int ia = add(s, a);
    Simulation sim;
    sim.load(s);
    CHECK_REL(accelBetween(sim, sim.indexOf(ia), 0.3f, 1.0f).x, 3.0, 0.01);
}

TEST(two_blocks_slip_above_slipping_force) {
    Scene s = withGround(10.0f);
    s.objects[0].friction = 0.0f;
    ObjectSpec b = block({0, 0.5f}, 6, 1, 4);
    b.friction = 1.0f;
    b.force = {40, 0};
    int ib = add(s, b);
    ObjectSpec a = block({0, 1.25f}, 1, 0.5f, 1);
    a.friction = 0.5f;
    int ia = add(s, a);
    Simulation sim;
    sim.load(s);
    sim.runUntil(0.2f);
    Vec2 va = sim.body(sim.indexOf(ia))->velocity, vb = sim.body(sim.indexOf(ib))->velocity;
    sim.runUntil(0.8f);
    CHECK_REL((sim.body(sim.indexOf(ia))->velocity.x - va.x) / 0.6f, 5.0, 0.01);          // mu g
    CHECK_REL((sim.body(sim.indexOf(ib))->velocity.x - vb.x) / 0.6f, (40.0 - 5.0) / 4.0, 0.01);
}

TEST(constant_velocity_drive_measures_friction) {
    Scene s = withGround(9.81f);
    ObjectSpec b = block({0, 0.5f}, 1, 1, 2);
    b.friction = 0.3f;
    b.velocity = {1, 0};
    b.holdVelocity = true;
    add(s, b);
    Simulation sim;
    sim.load(s);
    sim.runUntil(1.0f);
    double sum = 0;
    int n = 0;
    while (sim.time() < 2.0f) {
        sim.step();
        sum += sim.driveForce(1);
        ++n;
    }
    CHECK_REL(sum / n, 0.3 * 2 * 9.81, 0.005);
    CHECK_NEAR(sim.body(1)->velocity.x, 1.0, 0.005);
}

TEST(constant_velocity_drive_on_smooth_floor_needs_no_force) {
    Scene s = withGround(9.81f);
    ObjectSpec b = block({0, 0.5f}, 1, 1, 2);
    b.velocity = {2, 0};
    b.holdVelocity = true;
    add(s, b);
    Simulation sim;
    sim.load(s);
    sim.runUntil(1.0f);
    CHECK_NEAR(sim.driveForce(1), 0.0, 0.01);
}

// ---- pulleys ----

namespace {

Scene atwood(float m1, float m2, float ratio = 1.0f) {
    Scene s = empty(9.81f);
    int a = add(s, block({-0.5f, 5}, 0.4f, 0.4f, m1));
    int b = add(s, block({0.5f, 5}, 0.4f, 0.4f, m2));
    LinkSpec p = link(LinkKind::Pulley, at(a), at(b));
    p.wheel = {0, 8};
    p.wheelRadius = 0.5f;
    p.ratio = ratio;
    add(s, p);
    return s;
}

const pe::PulleyJoint* pulley(const Simulation& sim) {
    for (const pe::Joint* j : sim.built().joints)
        if (auto* p = dynamic_cast<const pe::PulleyJoint*>(j)) return p;
    return nullptr;
}

} // namespace

TEST(atwood_acceleration_and_tension) {
    Simulation sim;
    sim.load(atwood(2, 1));
    CHECK_REL(accelBetween(sim, 0, 0.2f, 0.7f).y, -9.81 / 3.0, 0.003);
    CHECK_REL(pulley(sim)->tension(), 4.0 * 9.81 / 3.0, 0.003); // 2 m1 m2 g / (m1 + m2)
}

TEST(atwood_equal_masses_stay_put) {
    Simulation sim;
    sim.load(atwood(3, 3));
    sim.runUntil(2.0f);
    CHECK(std::abs(sim.body(0)->velocity.y) < 1e-3f);
    CHECK_REL(pulley(sim)->tension(), 3.0 * 9.81, 0.003);
}

TEST(atwood_rope_length_is_conserved) {
    Simulation sim;
    sim.load(atwood(5, 1));
    const float before = pulley(sim)->lengthA() + pulley(sim)->lengthB();
    sim.runUntil(0.8f);
    CHECK_NEAR(pulley(sim)->lengthA() + pulley(sim)->lengthB(), before, 0.01);
}

TEST(block_and_tackle_ratio_two) {
    // b hangs from 2 strands: a_b = (2 m_a - m_b) g / (4 m_a + m_b), a_a = 2 a_b
    Simulation sim;
    sim.load(atwood(1, 1, 2.0f));
    const float ab = (2.0f - 1.0f) * 9.81f / (4.0f + 1.0f);
    CHECK_REL(accelBetween(sim, 1, 0.2f, 0.6f).y, ab, 0.01);
    CHECK_REL(sim.acceleration(0).y, -2.0f * ab, 0.02);
}

TEST(table_block_pulled_by_hanging_mass) {
    // smooth / rough table: a = (m2 - mu m1) g / (m1 + m2)
    for (float mu : {0.0f, 0.2f}) {
        Scene s = empty(10.0f);
        ObjectSpec table = block({-5, 4.5f}, 10, 1, 1);
        table.fixed = true;
        table.friction = 1.0f;
        add(s, table);
        ObjectSpec a = block({-3, 5.25f}, 0.5f, 0.5f, 3);
        a.friction = mu;
        int ia = add(s, a);
        int ib = add(s, block({0.6f, 3}, 0.4f, 0.4f, 2));
        LinkSpec p = link(LinkKind::Pulley, at(ia), at(ib));
        p.wheel = {0.3f, 4.95f};
        p.wheelRadius = 0.3f;
        add(s, p);
        Simulation sim;
        sim.load(s);
        const float expected = (2.0f - mu * 3.0f) * 10.0f / 5.0f;
        CHECK_REL(accelBetween(sim, sim.indexOf(ia), 0.2f, 0.6f).x, expected, 0.01);
    }
}

// ---- wedges ----

TEST(movable_wedge_on_smooth_floor) {
    // block m on smooth wedge M: A = m g sin cos / (M + m sin^2), horizontal momentum stays zero
    const float m = 1, M = 4, deg = 30, g = 10;
    Scene s = withGround(g);
    s.objects[0].friction = 0.0f;
    ObjectSpec w = wedge({-4, 0}, 8, deg, false);
    w.mass = M;
    w.noSpin = true;
    int iw = add(s, w);
    int ib = add(s, onSlope({-4, 0}, deg, 5.0f, 0.4f, m));
    Simulation sim;
    sim.load(s);
    const int wi = sim.indexOf(iw), bi = sim.indexOf(ib);
    const float th = deg * kDeg;
    const float A = m * g * std::sin(th) * std::cos(th) / (M + m * std::sin(th) * std::sin(th));
    CHECK_REL(accelBetween(sim, wi, 0.2f, 0.6f).x, A, 0.02); // the block slides left, the wedge recoils right
    const float px = M * sim.body(wi)->velocity.x + m * sim.body(bi)->velocity.x;
    CHECK_NEAR(px, 0.0, 0.02);
}
