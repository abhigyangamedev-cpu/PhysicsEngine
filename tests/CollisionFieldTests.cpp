// Collisions, restitution, and electric / magnetic / gravitational fields.
#include "Check.h"
#include "Helpers.h"

using namespace th;

namespace {

// Two smooth balls heading at each other in zero gravity; returns their velocities afterwards.
std::pair<float, float> collide(float m1, float v1, float m2, float v2, float e) {
    Scene s = empty(0.0f);
    ObjectSpec a = ball({-2, 0}, 0.5f, m1);
    a.velocity = {v1, 0};
    a.restitution = e;
    add(s, a);
    ObjectSpec b = ball({2, 0}, 0.5f, m2);
    b.velocity = {v2, 0};
    b.restitution = e;
    add(s, b);
    Simulation sim;
    sim.load(s);
    sim.runUntil(3.0f);
    return {sim.body(0)->velocity.x, sim.body(1)->velocity.x};
}

} // namespace

TEST(elastic_equal_masses_swap_velocities) {
    auto [a, b] = collide(1, 3, 1, 0, 1.0f);
    CHECK_NEAR(a, 0.0, 0.01);
    CHECK_NEAR(b, 3.0, 0.01);
}

TEST(elastic_unequal_masses) {
    auto [a, b] = collide(1, 4, 3, 0, 1.0f);
    CHECK_NEAR(a, -2.0, 0.02); // (m1 - m2) / (m1 + m2) v
    CHECK_NEAR(b, 2.0, 0.02);  // 2 m1 / (m1 + m2) v
}

TEST(head_on_elastic_collision) {
    auto [a, b] = collide(2, 3, 2, -1, 1.0f);
    CHECK_NEAR(a, -1.0, 0.02);
    CHECK_NEAR(b, 3.0, 0.02);
}

TEST(perfectly_inelastic_move_together) {
    auto [a, b] = collide(1, 6, 2, 0, 0.0f);
    CHECK_NEAR(a, 2.0, 0.02);
    CHECK_NEAR(b, 2.0, 0.02);
}

TEST(partially_elastic_collision) {
    const float e = 0.5f;
    auto [a, b] = collide(1, 4, 1, 0, e);
    CHECK_NEAR(a, (1 - e) * 4 / 2, 0.02);
    CHECK_NEAR(b, (1 + e) * 4 / 2, 0.02);
    CHECK_NEAR(b - a, e * 4, 0.02); // separation speed = e * approach speed
}

TEST(collisions_conserve_momentum) {
    for (float e : {0.0f, 0.3f, 0.7f, 1.0f}) {
        auto [a, b] = collide(2, 5, 3, -2, e);
        CHECK_NEAR(2 * a + 3 * b, 2 * 5 + 3 * -2, 0.02);
    }
}

TEST(ball_bounce_height_scales_with_e_squared) {
    for (float e : {0.5f, 0.8f}) {
        Scene s = withGround(9.81f);
        s.objects[0].restitution = 0.0f;
        ObjectSpec b = ball({0, 5.1f}, 0.1f, 1);
        b.restitution = e;
        int id = add(s, b);
        Simulation sim;
        sim.load(s);
        // fall, bounce, then track the peak of the first rebound
        while (sim.body(sim.indexOf(id))->velocity.y <= 0.0f && sim.time() < 3.0f) sim.step();
        float peak = 0;
        while (sim.body(sim.indexOf(id))->velocity.y > 0.0f && sim.time() < 6.0f) {
            sim.step();
            peak = std::max(peak, sim.body(sim.indexOf(id))->position.y - 0.1f);
        }
        CHECK_REL(peak, e * e * 5.0f, 0.03);
    }
}

TEST(uniform_electric_field_acceleration) {
    Scene s = empty(0.0f);
    s.world.E = {3, 0};
    ObjectSpec p = ball({0, 0}, 0.1f, 1);
    p.charge = 2.0f;
    add(s, p);
    ObjectSpec n = ball({0, 5}, 0.1f, 2);
    n.charge = -1.0f;
    add(s, n);
    add(s, ball({0, 10}, 0.1f, 1)); // neutral: unaffected
    Simulation sim;
    sim.load(s);
    sim.runUntil(1.0f);
    CHECK_NEAR(sim.body(0)->position.x, 0.5 * 6 * 1, 0.02);    // a = qE / m = 6
    CHECK_NEAR(sim.body(1)->position.x, -0.5 * 1.5 * 1, 0.01); // a = -1.5
    CHECK_NEAR(sim.body(2)->position.x, 0.0, 1e-6);
}

TEST(charge_in_electric_field_and_gravity) {
    // E field balancing gravity: qE = mg, the charge floats
    Scene s = empty(10.0f);
    s.world.E = {0, 5};
    ObjectSpec p = ball({0, 0}, 0.1f, 1);
    p.charge = 2.0f;
    add(s, p);
    Simulation sim;
    sim.load(s);
    sim.runUntil(2.0f);
    CHECK_NEAR(sim.body(0)->position.y, 0.0, 1e-3);
}

TEST(magnetic_field_circular_orbit) {
    // r = m v / (q B), T = 2 pi m / (q B)
    const float m = 1, q = 1, B = 1, v = 2;
    Scene s = empty(0.0f);
    s.world.B = B;
    ObjectSpec p = ball({0, 0}, 0.05f, m);
    p.charge = q;
    p.velocity = {v, 0};
    add(s, p);
    Simulation sim;
    sim.load(s);
    const Vec2 center{0, -m * v / (q * B)}; // B out of the screen curves a positive charge clockwise
    float worst = 0;
    while (sim.time() < 2 * kPi * m / (q * B)) {
        sim.step();
        worst = std::max(worst, std::abs((sim.body(0)->position - center).length() - m * v / (q * B)));
    }
    CHECK(worst < 0.01f);
    sim.runUntil(2 * kPi * m / (q * B));
    CHECK((sim.body(0)->position).length() < 0.03f); // back where it started after one period
    CHECK_REL(sim.body(0)->velocity.length(), v, 1e-4); // B does no work
}

TEST(magnetic_field_ignores_neutral_bodies) {
    Scene s = empty(0.0f);
    s.world.B = 5;
    ObjectSpec p = ball({0, 0}, 0.1f, 1);
    p.velocity = {1, 0};
    add(s, p);
    Simulation sim;
    sim.load(s);
    sim.runUntil(1.0f);
    CHECK_NEAR(sim.body(0)->position.x, 1.0, 1e-3);
    CHECK_NEAR(sim.body(0)->position.y, 0.0, 1e-6);
}

TEST(coulomb_like_charges_repel) {
    Scene s = empty(0.0f);
    s.world.coulomb = true;
    s.world.coulombK = 1.0f;
    ObjectSpec a = ball({-1, 0}, 0.1f, 1);
    a.charge = 1;
    add(s, a);
    ObjectSpec b = ball({1, 0}, 0.1f, 2);
    b.charge = 2;
    add(s, b);
    Simulation sim;
    sim.load(s);
    sim.runUntil(0.05f);
    // F = k q1 q2 / r^2 = 0.5 N: a1 = 0.5, a2 = 0.25 apart
    CHECK_REL(-sim.body(0)->velocity.x / 0.05f, 0.5, 0.01);
    CHECK_REL(sim.body(1)->velocity.x / 0.05f, 0.25, 0.01);
    CHECK_NEAR(sim.body(0)->velocity.x * 1 + sim.body(1)->velocity.x * 2, 0.0, 1e-5);
}

TEST(coulomb_opposite_charges_attract) {
    Scene s = empty(0.0f);
    s.world.coulomb = true;
    s.world.coulombK = 2.0f;
    ObjectSpec a = ball({-1.5f, 0}, 0.1f, 1);
    a.charge = 1;
    add(s, a);
    ObjectSpec b = ball({1.5f, 0}, 0.1f, 1);
    b.charge = -1;
    add(s, b);
    Simulation sim;
    sim.load(s);
    sim.runUntil(0.05f);
    CHECK_REL(sim.body(0)->velocity.x / 0.05f, 2.0 / 9.0, 0.01);
}

TEST(mutual_gravity_pulls_bodies_together) {
    Scene s = empty(0.0f);
    s.world.mutualGravity = true;
    s.world.G = 1.0f;
    add(s, ball({-1, 0}, 0.1f, 4));
    add(s, ball({1, 0}, 0.1f, 1));
    Simulation sim;
    sim.load(s);
    sim.runUntil(0.05f);
    // a1 = G m2 / r^2 = 0.25, a2 = G m1 / r^2 = 1 (softening 0.1 m changes this by 0.4%)
    CHECK_REL(sim.body(0)->velocity.x / 0.05f, 0.25, 0.01);
    CHECK_REL(-sim.body(1)->velocity.x / 0.05f, 1.0, 0.01);
}

TEST(mutual_gravity_circular_orbit) {
    // light satellite around a heavy fixed... here a heavy body: v = sqrt(G M / r)
    Scene s = empty(0.0f);
    s.world.mutualGravity = true;
    s.world.G = 1.0f;
    ObjectSpec sun = ball({0, 0}, 0.2f, 1000);
    sun.fixed = true;
    add(s, sun);
    ObjectSpec planet = ball({10, 0}, 0.1f, 0.001f);
    planet.velocity = {0, std::sqrt(1000.0f / 10.0f)};
    add(s, planet);
    Simulation sim;
    sim.load(s);
    float worst = 0;
    while (sim.time() < 3.0f) {
        sim.step();
        worst = std::max(worst, std::abs(sim.body(1)->position.length() - 10.0f));
    }
    CHECK(worst < 0.05f);
}

TEST(fixed_charge_attracts_free_charge) {
    Scene s = empty(0.0f);
    s.world.coulomb = true;
    s.world.coulombK = 1.0f;
    ObjectSpec src = ball({0, 0}, 0.2f, 1);
    src.fixed = true;
    src.charge = 4;
    add(s, src);
    ObjectSpec p = ball({2, 0}, 0.1f, 1);
    p.charge = -1;
    add(s, p);
    Simulation sim;
    sim.load(s);
    sim.runUntil(0.05f);
    CHECK_REL(-sim.body(1)->velocity.x / 0.05f, 1.0, 0.01); // k q Q / r^2 = 4 / 4
    CHECK(sim.body(0)->position.x == 0.0f);
}
