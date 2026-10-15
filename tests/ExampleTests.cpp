// Every real-life problem must reach its asked-for moment and agree with its textbook value.
#include "Check.h"
#include "editor/Examples.h"
#include <string>

using namespace editor;

namespace {

ExampleRun run(const Example& ex, Simulation& sim) {
    sim.load(ex.setup());
    ExampleRun r;
    while (sim.time() < 30.0f)
        if (sim.step(), ex.monitor(sim, r)) break;
    return r;
}

const Example& named(const char* name) {
    for (const Example& ex : examples())
        if (std::string(ex.name) == name) return ex;
    check::fail(__FILE__, __LINE__, std::string("no example named ") + name);
    return examples().front();
}

} // namespace

TEST(examples_are_well_formed) {
    CHECK(!examples().empty());
    for (const Example& ex : examples()) {
        CHECK(ex.name && *ex.name);
        CHECK(ex.topic && *ex.topic);
        CHECK(ex.question && *ex.question);
        CHECK(ex.answer && *ex.answer);
        CHECK(ex.setup && ex.monitor);
        CHECK(ex.cameraScale > 0.0f);
        CHECK(ex.plot >= 0 && ex.plot <= 9);
        Scene s = ex.setup();
        bool focusFound = false;
        for (const auto& o : s.objects) focusFound |= o.name == ex.focus;
        CHECK(focusFound);
        for (const auto& l : s.links) {
            CHECK(l.a.obj < 0 || s.object(l.a.obj));
            CHECK(l.b.obj < 0 || s.object(l.b.obj));
        }
    }
}

TEST(examples_match_their_textbook_answers) {
    for (const Example& ex : examples()) {
        Simulation sim;
        ExampleRun r = run(ex, sim);
        CHECK(r.done);
        CHECK(!r.lines.empty());
        CHECK_REL(r.result, ex.expected, 0.005);
    }
}

TEST(example_balloon_stone_lands_at_five_seconds) {
    Simulation sim;
    ExampleRun r = run(named("Balloon and stone"), sim);
    CHECK_NEAR(sim.time(), 5.0, 0.02);
    CHECK_NEAR(r.result, 125.0, 0.1);
}

TEST(example_blocks_free_a_rides_with_b) {
    Simulation sim;
    run(named("Blocks A, B, C (as drawn)"), sim);
    CHECK_NEAR(sim.body("A")->velocity.x, sim.body("B")->velocity.x, 0.01);
}

TEST(example_blocks_held_a_stays_still) {
    Simulation sim;
    run(named("Blocks A, B, C (A held)"), sim);
    CHECK_NEAR(sim.body("A")->velocity.x, 0.0, 0.01);
    CHECK_NEAR(sim.body("C")->velocity.x, -0.5, 0.01);
}

TEST(example_monitor_restarts_cleanly) {
    // running the same example twice gives the same answer (no state leaks between runs)
    Simulation a, b;
    ExampleRun r1 = run(named("Blocks A, B, C (as drawn)"), a);
    ExampleRun r2 = run(named("Blocks A, B, C (as drawn)"), b);
    CHECK_NEAR(r1.result, r2.result, 1e-4);
}

TEST(example_piston_gas_stays_at_atmospheric_pressure) {
    Simulation sim;
    ExampleRun r = run(named("Gas heated under a piston"), sim);
    CHECK_NEAR(sim.driveForce(sim.indexNamed("Piston")), 40.0, 0.05);
    CHECK_NEAR(r.result, 25.9, 0.02);
}

TEST(example_car_skid_matches_v_squared_over_2_mu_g) {
    Simulation sim;
    ExampleRun r = run(named("Skidding car"), sim);
    CHECK_NEAR(r.result, 400.0 / 14.0, 0.15);
    CHECK_NEAR(sim.time(), 20.0 / 7.0, 0.05);
}

TEST(example_football_lands_at_projectile_range) {
    Simulation sim;
    ExampleRun r = run(named("Football kick"), sim);
    CHECK_NEAR(r.result, 34.641, 0.1);
    CHECK_NEAR(sim.time(), 2.0, 0.02);
}
