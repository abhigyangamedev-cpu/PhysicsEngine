// Every Standard Example must reach its asked-for moment and agree with its textbook value.
#include "Check.h"
#include "editor/Examples.h"

using namespace editor;

namespace {

ExampleRun run(const Example& ex, Simulation& sim) {
    sim.load(ex.setup());
    ExampleRun r;
    while (sim.time() < 30.0f)
        if (sim.step(), ex.monitor(sim, r)) break;
    return r;
}

} // namespace

TEST(examples_are_well_formed) {
    CHECK(!examples().empty());
    for (const Example& ex : examples()) {
        CHECK(ex.name && *ex.name);
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
    ExampleRun r = run(examples()[0], sim);
    CHECK_NEAR(sim.time(), 5.0, 0.02);
    CHECK_NEAR(r.result, 125.0, 0.1);
}

TEST(example_blocks_free_a_rides_with_b) {
    Simulation sim;
    run(examples()[1], sim);
    CHECK_NEAR(sim.body("A")->velocity.x, sim.body("B")->velocity.x, 0.01);
}

TEST(example_blocks_held_a_stays_still) {
    Simulation sim;
    run(examples()[2], sim);
    CHECK_NEAR(sim.body("A")->velocity.x, 0.0, 0.01);
    CHECK_NEAR(sim.body("C")->velocity.x, -0.5, 0.01);
}

TEST(example_monitor_restarts_cleanly) {
    // running the same example twice gives the same answer (no state leaks between runs)
    Simulation a, b;
    ExampleRun r1 = run(examples()[1], a);
    ExampleRun r2 = run(examples()[1], b);
    CHECK_NEAR(r1.result, r2.result, 1e-4);
}

TEST(example_piston_gas_stays_at_atmospheric_pressure) {
    Simulation sim;
    ExampleRun r = run(examples()[3], sim);
    CHECK_NEAR(sim.driveForce(sim.indexNamed("Piston")), 40.0, 0.05);
    CHECK_NEAR(r.result, 25.9, 0.02);
}
