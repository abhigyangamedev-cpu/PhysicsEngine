// Springs, pendulums, rods, ropes, hinges and energy.
#include "Check.h"
#include "Helpers.h"

using namespace th;

TEST(horizontal_spring_period) {
    for (float k : {20.0f, 50.0f, 200.0f}) {
        Scene s = empty(0.0f);
        int a = add(s, block({1.0f, 0}, 0.5f, 0.5f, 2));
        LinkSpec sp = link(LinkKind::Spring, fixedPoint({-3, 0}), at(a));
        sp.k = k;
        sp.length = 3.5f; // released from 0.5 m stretch
        add(s, sp);
        Simulation sim;
        sim.load(s);
        float T = measurePeriod(sim, 8.0f, [&] { return sim.body(0)->position.x - 0.5f; });
        CHECK_REL(T, 2.0f * kPi * std::sqrt(2.0f / k), 0.003);
    }
}

TEST(spring_amplitude_and_max_speed) {
    Scene s = empty(0.0f);
    int a = add(s, block({0.4f, 0}, 0.5f, 0.5f, 1));
    LinkSpec sp = link(LinkKind::Spring, fixedPoint({-2, 0}), at(a));
    sp.k = 100;
    sp.length = 2.0f; // amplitude 0.4 m
    add(s, sp);
    Simulation sim;
    sim.load(s);
    float vmax = 0, xmin = 1e9f;
    while (sim.time() < 2.0f) {
        sim.step();
        vmax = std::max(vmax, std::abs(sim.body(0)->velocity.x));
        xmin = std::min(xmin, sim.body(0)->position.x);
    }
    CHECK_REL(vmax, 0.4f * 10.0f, 0.005); // A omega
    CHECK_NEAR(xmin, -0.4, 0.005);
}

TEST(block_between_two_springs_period) {
    // springs on both sides act in parallel: T = 2 pi sqrt(m / (k1 + k2))
    Scene s = empty(0.0f);
    int a = add(s, block({0.2f, 0}, 0.5f, 0.5f, 3));
    LinkSpec left = link(LinkKind::Spring, fixedPoint({-2, 0}), at(a));
    left.k = 40;
    left.length = 2.0f;
    add(s, left);
    LinkSpec right = link(LinkKind::Spring, at(a), fixedPoint({2, 0}));
    right.k = 80;
    right.length = 2.0f;
    add(s, right);
    Simulation sim;
    sim.load(s);
    float T = measurePeriod(sim, 8.0f, [&] { return sim.body(0)->position.x; });
    CHECK_REL(T, 2.0f * kPi * std::sqrt(3.0f / 120.0f), 0.003);
}

TEST(vertical_spring_equilibrium_extension) {
    Scene s = empty(10.0f);
    int a = add(s, ball({0, 8}, 0.2f, 2));
    LinkSpec sp = link(LinkKind::Spring, fixedPoint({0, 10}), at(a));
    sp.k = 100;
    sp.damping = 6;
    sp.length = 2.0f;
    add(s, sp);
    Simulation sim;
    sim.load(s);
    sim.runUntil(12.0f);
    CHECK_NEAR(sim.body(0)->position.y, 10.0 - 2.0 - 2.0 * 10.0 / 100.0, 0.002); // x = mg / k
}

TEST(vertical_spring_period_ignores_gravity) {
    Scene s = empty(9.81f);
    int a = add(s, ball({0, 7}, 0.2f, 0.5f));
    LinkSpec sp = link(LinkKind::Spring, fixedPoint({0, 10}), at(a));
    sp.k = 50;
    sp.length = 2.5f;
    add(s, sp);
    Simulation sim;
    sim.load(s);
    const float eq = 10.0f - 2.5f - 0.5f * 9.81f / 50.0f;
    float T = measurePeriod(sim, 6.0f, [&] { return sim.body(0)->position.y - eq; });
    CHECK_REL(T, 2.0f * kPi * std::sqrt(0.5f / 50.0f), 0.003);
}

TEST(spring_energy_is_conserved) {
    Scene s = empty(9.81f);
    int a = add(s, ball({0, 6}, 0.2f, 1));
    LinkSpec sp = link(LinkKind::Spring, fixedPoint({0, 10}), at(a));
    sp.k = 30;
    sp.length = 3.0f;
    add(s, sp);
    Simulation sim;
    sim.load(s);
    const float e0 = sim.totalEnergy();
    float worst = 0;
    while (sim.time() < 5.0f) {
        sim.step();
        worst = std::max(worst, std::abs(sim.totalEnergy() - e0));
    }
    CHECK(worst < 0.01f * std::abs(e0) + 0.05f);
}

TEST(damped_spring_loses_energy) {
    Scene s = empty(0.0f);
    int a = add(s, block({0.5f, 0}, 0.5f, 0.5f, 1));
    LinkSpec sp = link(LinkKind::Spring, fixedPoint({-2, 0}), at(a));
    sp.k = 40;
    sp.damping = 1;
    sp.length = 2.0f;
    add(s, sp);
    Simulation sim;
    sim.load(s);
    const float e0 = sim.totalEnergy();
    sim.runUntil(3.0f);
    // amplitude decays as exp(-c t / 2m): energy as exp(-c t / m)
    CHECK_REL(sim.totalEnergy() / e0, std::exp(-3.0f), 0.1);
}

namespace {

Scene pendulum(LinkKind kind, float length, float startDeg) {
    Scene s = empty(9.81f);
    const Vec2 pivot{0, 10};
    const float a = startDeg * kDeg;
    int b = add(s, ball(pivot + Vec2{std::sin(a), -std::cos(a)} * length, 0.05f, 1));
    add(s, link(kind, fixedPoint(pivot), at(b)));
    return s;
}

} // namespace

TEST(rod_pendulum_small_angle_period) {
    for (float L : {1.0f, 2.0f, 4.0f}) {
        Simulation sim;
        sim.load(pendulum(LinkKind::Rod, L, 5.0f));
        float T = measurePeriod(sim, 12.0f, [&] { return sim.body(0)->position.x; });
        CHECK_REL(T, 2.0f * kPi * std::sqrt(L / 9.81f), 0.005);
    }
}

TEST(rope_pendulum_matches_rod_when_taut) {
    Simulation sim;
    sim.load(pendulum(LinkKind::Rope, 2.0f, 5.0f));
    float T = measurePeriod(sim, 10.0f, [&] { return sim.body(0)->position.x; });
    CHECK_REL(T, 2.0f * kPi * std::sqrt(2.0f / 9.81f), 0.005);
}

TEST(rod_keeps_its_length_and_energy) {
    Simulation sim;
    sim.load(pendulum(LinkKind::Rod, 3.0f, 60.0f));
    const float e0 = sim.totalEnergy();
    float worstLen = 0, worstE = 0;
    while (sim.time() < 6.0f) {
        sim.step();
        worstLen = std::max(worstLen, std::abs((sim.body(0)->position - Vec2{0, 10}).length() - 3.0f));
        worstE = std::max(worstE, std::abs(sim.totalEnergy() - e0));
    }
    CHECK(worstLen < 0.01f);
    CHECK(worstE < 0.01f * std::abs(e0));
}

TEST(pendulum_speed_at_bottom) {
    // released from 60 deg: v = sqrt(2 g L (1 - cos 60)) = sqrt(g L)
    Simulation sim;
    sim.load(pendulum(LinkKind::Rod, 2.0f, 60.0f));
    float vmax = 0;
    while (sim.time() < 1.5f) {
        sim.step();
        vmax = std::max(vmax, sim.body(0)->velocity.length());
    }
    CHECK_REL(vmax, std::sqrt(9.81f * 2.0f), 0.005);
}

TEST(rope_is_slack_until_stretched) {
    Scene s = empty(10.0f);
    int b = add(s, ball({0, 9}, 0.1f, 1));
    LinkSpec r = link(LinkKind::Rope, fixedPoint({0, 10}), at(b));
    r.length = 3.0f;
    add(s, r);
    Simulation sim;
    sim.load(s);
    sim.runUntil(0.5f); // falls freely for the first 2 m (0.63 s)
    CHECK_NEAR(sim.body(0)->position.y, 9.0 - 0.5 * 10 * 0.25, 0.01);
    sim.runUntil(3.0f);
    CHECK(sim.body(0)->position.y > 10.0f - 3.0f - 0.03f);
}

TEST(hinge_keeps_pivot_fixed) {
    Scene s = empty(9.81f);
    ObjectSpec bar = block({1, 5}, 2, 0.2f, 1);
    bar.noSpin = false;
    int id = add(s, bar);
    LinkSpec h = link(LinkKind::Hinge, fixedPoint({0, 5}), s.endpointAt(id, {0, 5}));
    add(s, h);
    Simulation sim;
    sim.load(s);
    float worst = 0;
    while (sim.time() < 3.0f) {
        sim.step();
        const pe::Body* b = sim.body(0);
        Vec2 pin = b->position + b->rot.apply(h.b.local);
        worst = std::max(worst, (pin - Vec2{0, 5}).length());
    }
    CHECK(worst < 0.01f);
    // a uniform bar pivoted at its end: T = 2 pi sqrt(2 L / 3 g) for small swings; here it just has to swing
    CHECK(sim.body(0)->angle != 0.0f);
}
