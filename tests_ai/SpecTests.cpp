// Reading generated scenes and measuring them, checked against textbook answers.
#include "Check.h"
#include "QuestionSpec.h"
#include "SpecHelpers.h"
#include <cmath>
#include <cstdio>
#include <functional>
#include <numbers>

using namespace spec_json;

namespace {

std::shared_ptr<ai::Spec> read(const std::string& text) {
    std::string error;
    auto spec = ai::readSpec(parsed(text), error);
    if (!spec) std::printf("    readSpec: %s\n", error.c_str());
    return spec;
}

std::string readError(const std::string& text) {
    std::string error;
    auto spec = ai::readSpec(parsed(text), error);
    CHECK(!spec);
    return error;
}

// A walk over every schema node that is an object.
void eachObjectSchema(const json::Value& v, const std::function<void(const json::Value&)>& fn) {
    if (v.isObject()) {
        if (v.str("type") == "object") fn(v);
        for (const auto& [k, child] : v.members()) eachObjectSchema(child, fn);
    } else if (v.isArray()) {
        for (const auto& child : v.items()) eachObjectSchema(child, fn);
    }
}

} // namespace

TEST(SchemaIsStrict) {
    int objects = 0;
    eachObjectSchema(ai::outputSchema(), [&](const json::Value& o) {
        ++objects;
        CHECK(o["additionalProperties"].isBool() && !o["additionalProperties"].asBool(true));
        CHECK(o["required"].size() == o["properties"].size());
        for (const auto& name : o["required"].items()) CHECK(o["properties"].has(name.asString()));
    });
    CHECK(objects >= 10);
    CHECK(std::string(ai::systemPrompt()).find("scene_2d") != std::string::npos);
}

TEST(SpecReadsA2DScene) {
    auto spec = read(answer2(object2("Ball", "ball", R"("position":[1,2],"radius":0.25,"velocity":[3,4],"mass":2)"), "",
                             measure("apex", "Ball", 0, "max_height"), 1.0));
    CHECK(spec);
    if (!spec) return;
    CHECK(spec->space == ai::Space::TwoD);
    CHECK(spec->scene2.objects.size() == 2); // the ground, then the ball
    CHECK(spec->scene2.objects[0].name == "Ground" && spec->scene2.objects[0].fixed);
    const auto& ball = spec->scene2.objects[1];
    CHECK(ball.kind == editor::ObjKind::Ball && ball.radius == 0.25f && ball.mass == 2.0f);
    CHECK(ball.velocity.x == 3.0f && ball.velocity.y == 4.0f);
    CHECK(spec->focus == "Ball"); // empty focus falls back to the measured object
}

TEST(SpecPlacesWedgesByTheirCorner) {
    auto spec = read(answer2(object2("Ramp", "wedge", R"("position":[2,0],"wedge_base":3,"wedge_slope_deg":45,"fixed":true)") + "," +
                                 object2("Ball", "ball", R"("position":[0,5],"radius":0.2)"),
                             "", measure("lands", "Ball", 0, "time"), 1.0));
    CHECK(spec);
    if (!spec) return;
    const auto& w = spec->scene2.objects[1];
    CHECK_NEAR(w.pos.x, 2.0 + 2.0, 1e-4); // unflipped centroid at 2/3 of the base
    CHECK_NEAR(w.pos.y, 1.0, 1e-4);       // and a third of the height (3 m at 45 degrees)
}

TEST(SpecErrorsNameTheProblem) {
    const std::string ball = object2("Ball", "ball", R"("position":[0,5])");
    CHECK(readError(answer2(ball, link2("rope", R"("a":"","b":"Bob")"), measure("apex", "Ball", 0, "y"), 1)).find("\"Bob\"") !=
          std::string::npos);
    CHECK(readError(answer2(ball + "," + ball, "", measure("apex", "Ball", 0, "y"), 1)).find("unique") != std::string::npos);
    CHECK(readError(answer2(ball, "", measure("apex", "Nobody", 0, "y"), 1)).find("Nobody") != std::string::npos);
    CHECK(readError(answer2(ball, "", measure("apex", "Ball", 0, "period"), 1)).find("reversals") != std::string::npos);
    CHECK(readError(answer2(object2("Box", "cube", ""), "", measure("apex", "Box", 0, "y"), 1)).find("cube") != std::string::npos);
    CHECK(readError(R"({"cannot_simulate":"It is about circuits.","space":"2d"})").rfind("cannot simulate", 0) == 0);
}

TEST(SpecCleansTextToAscii) {
    std::string text = answer2(object2("Ball", "ball", R"("position":[0,5])"), "", measure("apex", "Ball", 0, "y"), 1);
    json::Value v = parsed(text);
    v.set("question", "A ramp at 30\xc2\xb0 with \xce\xbc = 0.2 \xe2\x80\x94 find v\xc2\xb2.");
    std::string error;
    auto spec = ai::readSpec(v, error);
    CHECK(spec);
    if (spec) CHECK(spec->question == "A ramp at 30 deg with mu = 0.2 - find v^2.");
}

// A ball leaves a 20 m drop horizontally at 15 m/s: it lands after sqrt(2h/g) and travels 15 m/s times that.
TEST(MeasureProjectileRangeOnLanding) {
    const double t = std::sqrt(2.0 * 20.0 / 9.81), range = 15.0 * t;
    auto spec = read(answer2(object2("Ball", "ball", R"("position":[0,20.1],"radius":0.1,"velocity":[15,0],"no_spin":true)"), "",
                             measure("lands", "Ball", 0, "range"), range));
    CHECK(spec);
    if (!spec) return;
    const ai::Check c = ai::verify(*spec);
    std::printf("    range %.4f m (textbook %.4f) at t = %.3f s\n", c.result, range, c.time);
    CHECK(c.reached && c.problem.empty());
    CHECK_REL(c.result, range, 0.005);
}

TEST(MeasureTimeOfFlight) {
    const double t = std::sqrt(2.0 * 20.0 / 9.81);
    auto spec = read(answer2(object2("Ball", "ball", R"("position":[0,20.1],"radius":0.1,"no_spin":true)"), "",
                             measure("lands", "Ball", 0, "time"), t));
    CHECK(spec);
    if (!spec) return;
    const ai::Check c = ai::verify(*spec);
    CHECK(c.reached);
    CHECK_NEAR(c.result, t, 0.005);
}

TEST(MeasureMaxHeightAtApex) {
    const double h = 1.0 + 10.0 * 10.0 / (2.0 * 9.81);
    auto spec = read(answer2(object2("Ball", "ball", R"("position":[0,1],"radius":0.1,"velocity":[2,10],"no_spin":true)"), "",
                             measure("apex", "Ball", 0, "max_height"), h));
    CHECK(spec);
    if (!spec) return;
    const ai::Check c = ai::verify(*spec);
    CHECK(c.reached);
    CHECK_REL(c.result, h, 0.002);
}

// A 1 m pendulum swinging through 5 degrees: T = 2 pi sqrt(L/g), averaged over two and a half swings.
TEST(MeasurePendulumPeriodFromReversals) {
    const double L = 1.0, a = 5.0 * std::numbers::pi / 180.0, T = 2.0 * std::numbers::pi * std::sqrt(L / 9.81);
    char pos[64];
    std::snprintf(pos, sizeof pos, R"("position":[%.6f,%.6f],"radius":0.05,"no_spin":true)", L * std::sin(a), 3.0 - L * std::cos(a));
    auto spec = read(answer2(object2("Bob", "ball", pos), link2("rod", R"("a":"","a_offset":[0,3],"b":"Bob")"),
                             measure("reversals", "Bob", 5, "period", "x"), T));
    CHECK(spec);
    if (!spec) return;
    const ai::Check c = ai::verify(*spec);
    std::printf("    period %.4f s (small-angle %.4f)\n", c.result, T);
    CHECK(c.reached);
    CHECK_REL(c.result, T, 0.01);
}

// Frictionless block sliding 2 m down a 30 degree wedge: v = sqrt(2 g d sin 30).
TEST(MeasureSpeedDownAWedge) {
    const double q = 30.0 * std::numbers::pi / 180.0, d = 2.0, v = std::sqrt(2.0 * 9.81 * d * std::sin(q));
    const double h = 0.4, start = 4.0; // block centre placed 4 m up the slope
    char block[160];
    std::snprintf(block, sizeof block, R"("position":[%.6f,%.6f],"size":[0.4,%.2f],"angle_deg":30,"no_spin":true,"mass":2)",
                  start * std::cos(q) - h / 2 * std::sin(q), start * std::sin(q) + h / 2 * std::cos(q) + 0.001, h);
    auto spec = read(answer2(object2("Ramp", "wedge", R"("position":[0,0],"wedge_base":5,"wedge_slope_deg":30,"fixed":true)") + "," +
                                 object2("Block", "block", block),
                             "", measure("distance", "Block", d, "speed"), v));
    CHECK(spec);
    if (!spec) return;
    const ai::Check c = ai::verify(*spec);
    std::printf("    speed %.4f m/s (textbook %.4f)\n", c.result, v);
    CHECK(c.reached);
    CHECK_REL(c.result, v, 0.02);
}

TEST(Measure3DBallDropTime) {
    const double t = std::sqrt(2.0 * 5.0 / 9.81);
    auto spec = read(answer3(object3("Ball", "ball", R"("position":[0,5.2,0],"radius":0.2)"), "",
                             measure("lands", "Ball", 0, "time"), t));
    CHECK(spec);
    if (!spec) return;
    CHECK(spec->space == ai::Space::ThreeD && spec->scene3.world.ground);
    const ai::Check c = ai::verify(*spec);
    CHECK(c.reached);
    CHECK_NEAR(c.result, t, 0.01);
}

TEST(VerifyReportsAnEventThatNeverHappens) {
    auto spec = read(answer2(object2("Ball", "ball", R"("position":[0,0.5],"radius":0.5)"), "", measure("apex", "Ball", 0, "y"), 1));
    CHECK(spec);
    if (!spec) return;
    spec->measure.timeout = 2.0f;
    const ai::Check c = ai::verify(*spec);
    CHECK(!c.reached);
    CHECK(c.problem.find("never happened") != std::string::npos);
}

// The editors restart a run by handing the monitor a fresh run; it must measure the same again.
TEST(EditorMonitorRestartsCleanly) {
    auto spec = read(answer2(object2("Ball", "ball", R"("position":[0,10.1],"radius":0.1,"no_spin":true)"), "",
                             measure("lands", "Ball", 0, "speed"), std::sqrt(2 * 9.81 * 10)));
    CHECK(spec);
    if (!spec) return;
    auto ex = ai::toExample(spec);
    CHECK(std::string(ex->example.name) == "Test" && std::string(ex->example.unit) == "m");
    float results[2] = {};
    for (float& result : results) {
        editor::Simulation sim;
        sim.load(ex->example.setup());
        editor::ExampleRun run;
        while (sim.time() < 5.0f)
            if (sim.step(), ex->example.monitor(sim, run)) break;
        CHECK(run.done && !run.lines.empty());
        result = run.result;
    }
    CHECK_NEAR(results[0], results[1], 1e-6);
    CHECK_REL(results[0], std::sqrt(2 * 9.81 * 10), 0.005);
}

TEST(ProblemFor3DPointsIntoItsOwnText) {
    auto spec = read(answer3(object3("Ball", "ball", R"("position":[0,3,0],"radius":0.2)"), "", measure("lands", "Ball", 0, "time"), 0.7));
    CHECK(spec);
    if (!spec) return;
    auto p = ai::toProblem(spec);
    CHECK(std::string(p->problem.name) == "Test 3D" && std::string(p->problem.focus) == "Ball");
    CHECK(p->problem.setup().objects.size() == 1 && p->problem.cameraDistance > 0.0f);
}
