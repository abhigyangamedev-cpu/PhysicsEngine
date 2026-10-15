#include "Problems3.h"
#include <cmath>
#include <cstdio>
#include <numbers>

namespace editor3 {

using namespace pe3;

namespace {

constexpr float kPi = std::numbers::pi_v<float>;
constexpr float kDeg = kPi / 180.0f;

std::string format(const char* fmt, double a, double b = 0.0, double c = 0.0) {
    char buf[112];
    std::snprintf(buf, sizeof buf, fmt, a, b, c);
    return buf;
}

// ---- Tetherball: a conical pendulum ----

constexpr float kTetherL = 1.5f, kTetherAngle = 30.0f * kDeg, kTetherG = 9.8f;
const Vec3 kTetherPivot{0.0f, 3.5f, 0.0f};

Scene tetherballSetup() {
    Scene s;
    s.world.g = kTetherG;
    s.world.substeps = 8;
    auto& pole = s.add("Pole", ObjKind::Box, {0.0f, kTetherPivot.y * 0.5f, 0.0f});
    pole.size = {0.06f, kTetherPivot.y, 0.06f};
    pole.fixed = true;
    const float r = kTetherL * std::sin(kTetherAngle);
    auto& ball = s.add("Ball", ObjKind::Ball, kTetherPivot + Vec3{r, -kTetherL * std::cos(kTetherAngle), 0.0f});
    ball.radius = 0.11f;
    ball.mass = 0.5f;
    ball.velocity = {0.0f, 0.0f, -std::sqrt(kTetherG * r * std::tan(kTetherAngle))}; // just fast enough to circle level
    ball.color = 0xf1c453ff;
    const int id = ball.id;
    s.link(LinkKind::Rope, {-1, kTetherPivot}, {id, {}});
    return s;
}

// azimuth of the ball round the pole, unwrapped; two full turns give the period
bool tetherballMonitor(const Simulation& sim, ProblemRun& run) {
    const Body* ball = sim.body("Ball");
    if (!ball || run.done) return false;
    const Vec3 d = ball->position - kTetherPivot;
    const float phi = std::atan2(-d.z, d.x);
    if (!run.primed) run.prev[0] = phi, run.prev[1] = 0.0f, run.primed = true;
    float step = phi - run.prev[0];
    if (step > kPi) step -= 2 * kPi;
    if (step < -kPi) step += 2 * kPi;
    const float before = run.prev[1], total = before + step;
    run.prev[0] = phi;
    run.prev[1] = total;
    const float angle = std::acos(std::clamp(-d.y / d.length(), -1.0f, 1.0f)) / kDeg;
    const float t = sim.time();
    if (total >= 4 * kPi) {
        const float tCross = t - Simulation::kDt * (total - 4 * kPi) / (total - before);
        run.result = tCross / 2.0f;
        run.lines = {format("Two turns took %.3f s", tCross), format("Period T = %.3f s", run.result),
                     format("string at %.1f deg to the vertical", angle)};
        run.done = true;
        return true;
    }
    run.lines = {format("t = %.3f s   turns %.2f", t, total / (2 * kPi)),
                 format("string angle %.2f deg, radius %.3f m", angle, std::hypot(d.x, d.z)),
                 format("ball speed %.3f m/s", ball->velocity.length())};
    return false;
}

// ---- Bowling ball rolling down a ramp ----

constexpr float kSlope = 30.0f * kDeg;
const Vec3 kRampCenter{0.0f, 2.4f, 0.0f};
const Vec3 kRampNormal{std::sin(kSlope), std::cos(kSlope), 0.0f}; // the top face, tilted to face +x
const Vec3 kDownhill{std::cos(kSlope), -std::sin(kSlope), 0.0f};
constexpr float kRampHalfThick = 0.2f;

ObjectSpec& addRamp(Scene& s, float friction) {
    auto& ramp = s.add("Ramp", ObjKind::Box, kRampCenter);
    ramp.size = {8.0f, 2 * kRampHalfThick, 2.0f};
    ramp.tiltDeg = {0.0f, 0.0f, -kSlope / kDeg};
    ramp.fixed = true;
    ramp.friction = friction;
    ramp.restitution = 0.0f;
    return ramp;
}

// a point on the ramp's top face, `along` metres downhill of its middle
Vec3 onRamp(float along, float height) { return kRampCenter + kRampNormal * (kRampHalfThick + height) + kDownhill * along; }

constexpr float kBowlR = 0.109f, kRollStart = -3.0f, kRollDistance = 2.0f;

Scene bowlingSetup() {
    Scene s;
    s.world.g = 9.8f;
    s.world.substeps = 8;
    addRamp(s, 0.6f);
    auto& ball = s.add("Bowling ball", ObjKind::Ball, onRamp(kRollStart, kBowlR));
    ball.radius = kBowlR;
    ball.mass = 6.0f;
    ball.friction = 0.6f;
    ball.restitution = 0.0f;
    ball.color = 0x30343bff;
    return s;
}

bool bowlingMonitor(const Simulation& sim, ProblemRun& run) {
    const Body* ball = sim.body("Bowling ball");
    if (!ball || run.done) return false;
    const float s = dot(ball->position - onRamp(kRollStart, kBowlR), kDownhill);
    const float v = ball->velocity.length(), spin = ball->angularVelocity.length() * kBowlR;
    if (s >= kRollDistance && run.primed) {
        const float f = (kRollDistance - run.prev[0]) / std::max(s - run.prev[0], 1e-6f);
        run.result = run.prev[1] + (v - run.prev[1]) * f;
        run.lines = {format("After %.2f m: speed v = %.3f m/s", kRollDistance, run.result),
                     format("spin x radius = %.3f m/s (= v: rolling)", spin),
                     format("at t = %.3f s", sim.time())};
        run.done = true;
        return true;
    }
    run.prev[0] = s;
    run.prev[1] = v;
    run.primed = true;
    run.lines = {format("t = %.3f s   rolled %.3f m", sim.time(), s), format("speed %.3f m/s", v),
                 format("spin x radius %.3f m/s", spin)};
    return false;
}

// ---- Glancing billiard collision ----

constexpr float kCueR = 0.0286f, kCueSpeed = 2.0f;

Scene billiardsSetup() {
    Scene s;
    s.world.g = 0.0f; // seen from above, ignoring the cloth: only the collision matters
    s.world.ground = false;
    s.world.substeps = 8;
    auto ball = [&](const char* name, Vec3 pos, Vec3 vel, uint32_t color) {
        auto& b = s.add(name, ObjKind::Ball, pos);
        b.radius = kCueR;
        b.mass = 0.17f;
        b.velocity = vel;
        b.friction = 0.0f;
        b.restitution = 1.0f;
        b.color = color;
    };
    ball("Cue ball", {-0.4f, 1.0f, 0.0f}, {kCueSpeed, 0.0f, 0.0f}, 0xf5f5f0ff);
    ball("Red ball", {0.0f, 1.0f, kCueR}, {}, 0xd62828ff); // aimed half a ball off centre
    return s;
}

bool billiardsMonitor(const Simulation& sim, ProblemRun& run) {
    const Body* cue = sim.body("Cue ball");
    const Body* red = sim.body("Red ball");
    if (!cue || !red || run.done) return false;
    const float gap = (red->position - cue->position).length() - 2 * kCueR;
    const Vec3 vc = cue->velocity, vr = red->velocity;
    const float sc = vc.length(), sr = vr.length();
    if (sr > 0.05f) run.primed = true;
    if (run.primed && gap > 0.02f) {
        run.result = std::acos(std::clamp(dot(vc, vr) / std::max(sc * sr, 1e-9f), -1.0f, 1.0f)) / kDeg;
        const float ke0 = 0.5f * cue->mass * kCueSpeed * kCueSpeed;
        const float ke1 = 0.5f * cue->mass * sc * sc + 0.5f * red->mass * sr * sr;
        run.lines = {format("Angle between the paths = %.1f deg", run.result),
                     format("cue %.3f m/s at %.1f deg, red %.3f m/s", sc, std::atan2(-vc.z, vc.x) / kDeg, sr),
                     format("kinetic energy kept: %.1f %%", 100.0 * ke1 / ke0)};
        run.done = true;
        return true;
    }
    run.lines = {format("t = %.3f s   gap %.4f m", sim.time(), gap), format("cue ball %.3f m/s", sc),
                 format("red ball %.3f m/s", sr)};
    return false;
}

// ---- Crate sliding down a loading ramp ----

constexpr float kCrateMu = 0.3f, kCrateStart = -3.2f, kCrateDistance = 3.0f;
const Vec3 kCrateSize{0.5f, 0.4f, 0.5f};

Scene crateSetup() {
    Scene s;
    s.world.g = 9.8f;
    s.world.substeps = 8;
    addRamp(s, kCrateMu);
    auto& crate = s.add("Crate", ObjKind::Box, onRamp(kCrateStart, kCrateSize.y * 0.5f));
    crate.size = kCrateSize;
    crate.tiltDeg = {0.0f, 0.0f, -kSlope / kDeg};
    crate.mass = 20.0f;
    crate.friction = kCrateMu;
    crate.restitution = 0.0f;
    crate.color = 0xb07d48ff;
    return s;
}

bool crateMonitor(const Simulation& sim, ProblemRun& run) {
    const Body* crate = sim.body("Crate");
    if (!crate || run.done) return false;
    const float s = dot(crate->position - onRamp(kCrateStart, kCrateSize.y * 0.5f), kDownhill);
    const float t = sim.time(), v = crate->velocity.length();
    if (s >= kCrateDistance && run.primed) {
        const float f = (kCrateDistance - run.prev[0]) / std::max(s - run.prev[0], 1e-6f);
        run.result = t - Simulation::kDt * (1.0f - f);
        run.lines = {format("Slid %.1f m in t = %.3f s", kCrateDistance, run.result),
                     format("speed then %.3f m/s", v),
                     format("acceleration 2s/t^2 = %.3f m/s^2", 2.0 * kCrateDistance / (run.result * run.result))};
        run.done = true;
        return true;
    }
    run.prev[0] = s;
    run.primed = true;
    run.lines = {format("t = %.3f s   slid %.3f m", t, s), format("speed %.3f m/s", v)};
    return false;
}

} // namespace

const std::vector<Problem>& problems() {
    static const std::vector<Problem> list = {
        {"Tetherball period", "Circular motion",
         "A 0.5 kg tetherball hangs from the top of a pole on a 1.5 m rope. It is hit so that it swings round the pole "
         "in a level circle with the rope at 30 degrees to the vertical. How long does one trip round the pole take? "
         "(g = 9.8 m/s^2)",
         "2.29 s. Tension T balances weight vertically and supplies the centripetal force: T cos(30) = m g and "
         "T sin(30) = m v^2 / r with r = L sin(30) = 0.75 m. So v^2 = g r tan(30) and the period is "
         "2 pi r / v = 2 pi sqrt(L cos(30) / g) = 2 pi sqrt(1.299 / 9.8) = 2.29 s. The mass doesn't matter.",
         2.2876f, "s", tetherballSetup, {0.0f, 2.4f, 0.0f}, 7.0f, 0.6f, 0.45f, "Ball", tetherballMonitor},
        {"Bowling ball down a ramp", "Rolling motion",
         "A 6 kg bowling ball is let go from rest on a 30 degree ramp and rolls down it without slipping. How fast is "
         "it moving after rolling 2 m along the ramp? (g = 9.8 m/s^2, treat the ball as a uniform solid sphere)",
         "3.74 m/s. It drops h = 2 sin(30) = 1 m. Energy: m g h = 1/2 m v^2 + 1/2 I w^2 with I = 2/5 m R^2 and "
         "w = v / R, so m g h = 7/10 m v^2 and v = sqrt(10 g h / 7) = sqrt(14) = 3.74 m/s. A sliding block would reach "
         "4.43 m/s: the rest of the energy goes into spin.",
         3.7417f, "m/s", bowlingSetup, {-1.0f, 2.2f, 0.0f}, 8.0f, 0.15f, 0.3f, "Bowling ball", bowlingMonitor},
        {"Glancing billiard shot", "Collisions",
         "A cue ball rolling at 2 m/s strikes an identical, resting red ball half a ball off centre. The collision is "
         "elastic and the balls are smooth (ignore spin and the cloth). At what angle to each other do the two balls "
         "move off?",
         "90 degrees. Equal masses, elastic: momentum gives v = v1 + v2 and energy gives v^2 = v1^2 + v2^2, so "
         "v1 . v2 = 0. The red ball leaves along the line of centres, 30 degrees from the cue line (sin = R / 2R), at "
         "2 cos(30) = 1.73 m/s; the cue ball goes 60 degrees the other way at 2 sin(30) = 1 m/s.",
         90.0f, "deg", billiardsSetup, {-0.1f, 1.0f, 0.0f}, 1.1f, 0.0f, 1.2f, "Red ball", billiardsMonitor},
        {"Crate down a loading ramp", "Friction",
         "A 20 kg crate is set down at rest on a loading ramp inclined at 30 degrees. The coefficient of kinetic "
         "friction between crate and ramp is 0.3. How long does the crate take to slide 3 m down the ramp? "
         "(g = 9.8 m/s^2)",
         "1.60 s. Along the ramp: m a = m g sin(30) - 0.3 m g cos(30), so a = 9.8 (0.5 - 0.26) = 2.35 m/s^2, the same "
         "for any mass. From rest s = 1/2 a t^2, so t = sqrt(2 x 3 / 2.35) = 1.60 s. It does slide: tan(30) = 0.58 "
         "is more than 0.3.",
         1.5965f, "s", crateSetup, {-1.5f, 2.4f, 0.0f}, 7.0f, 0.2f, 0.35f, "Crate", crateMonitor},
    };
    return list;
}

} // namespace editor3
