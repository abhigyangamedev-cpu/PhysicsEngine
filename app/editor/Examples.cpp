#include "editor/Examples.h"
#include <cmath>
#include <cstdio>

namespace editor {

using namespace pe;

namespace {

ObjectSpec& add(Scene& s, const char* name, ObjKind kind, Vec2 pos);

LinkSpec& link(Scene& s, LinkKind kind, Endpoint a, Endpoint b) {
    LinkSpec l;
    l.id = s.nextId++;
    l.kind = kind;
    l.a = a;
    l.b = b;
    s.links.push_back(l);
    return s.links.back();
}

std::string format(const char* fmt, double a, double b = 0.0) {
    char buf[96];
    std::snprintf(buf, sizeof buf, fmt, a, b);
    return buf;
}

ObjectSpec& add(Scene& s, const char* name, ObjKind kind, Vec2 pos) {
    ObjectSpec o;
    o.id = s.nextId++;
    o.name = name;
    o.kind = kind;
    o.pos = pos;
    s.objects.push_back(o);
    return s.objects.back();
}

// ---- Balloon and stone ----

Scene balloonSetup() {
    Scene s = defaultScene();
    s.world.g = 10.0f;
    auto& balloon = add(s, "Balloon", ObjKind::Ball, {-4.0f, 75.0f});
    balloon.radius = 2.0f;
    balloon.mass = 1.0f;
    balloon.velocity = {0.0f, 10.0f};
    balloon.force = {0.0f, 10.0f}; // buoyancy balances mg: constant velocity
    balloon.noSpin = true;
    auto& stone = add(s, "Stone", ObjKind::Ball, {0.0f, 75.5f}); // bottom of the stone at 75 m
    stone.radius = 0.5f;
    stone.mass = 0.5f;
    stone.velocity = {0.0f, 10.0f}; // dropped: it leaves with the balloon's velocity
    stone.noSpin = true;
    return s;
}

bool balloonMonitor(const Simulation& sim, ExampleRun& run) {
    const Body* stone = sim.body("Stone");
    const Body* balloon = sim.body("Balloon");
    if (!stone || !balloon || run.done) return false;
    const float g = sim.scene().world.g, t = sim.time(), dt = Simulation::kDt;
    const float height = stone->aabb.min.y;
    const bool landed = run.primed && (height <= 0.01f || stone->velocity.y > run.prev[1] + 1.0f);
    if (landed) {
        // exact touchdown inside the last step, from the stone's state at its start
        const float h0 = run.prev[0], v0 = run.prev[1];
        const float tau = (v0 + std::sqrt(v0 * v0 + 2.0f * g * h0)) / g;
        const float tHit = t - dt + tau;
        const float balloonY = run.prev[2] + run.prev[3] * tau;
        run.result = balloonY;
        run.lines = {format("Stone hits the ground at t = %.2f s", tHit),
                     format("Balloon height then = %.1f m", balloonY)};
        run.done = true;
        return true;
    }
    run.prev[0] = height;
    run.prev[1] = stone->velocity.y;
    run.prev[2] = balloon->position.y;
    run.prev[3] = balloon->velocity.y;
    run.primed = true;
    run.lines = {format("t = %.3f s", t), format("stone height   %.2f m  (vy %.2f m/s)", height, stone->velocity.y),
                 format("balloon height %.2f m", balloon->position.y)};
    return false;
}

// ---- Three stacked blocks, rope round a wall pulley ----

constexpr float kSlide = 0.5f; // the constant speed C is pulled at; any value gives the same force

Scene blocksSetup(bool holdA) {
    Scene s = defaultScene();
    s.world.g = 10.0f;
    auto block = [&](const char* name, Vec2 pos, float w, float h, float m, float vx) -> ObjectSpec& {
        auto& o = add(s, name, ObjKind::Block, pos);
        o.width = w;
        o.height = h;
        o.mass = m;
        o.friction = 0.5f;
        o.velocity = {vx, 0.0f};
        o.noSpin = true;
        return o;
    };
    // ids are taken right away: adding objects invalidates the references
    auto& c = block("C", {0.0f, 0.5f}, 16.0f, 1.0f, 8.0f, -kSlide);
    c.holdVelocity = true; // F is whatever keeps C at constant speed
    const int ic = c.id;
    const int ib = block("B", {-5.0f, 1.4f}, 3.0f, 0.8f, 6.0f, kSlide).id;
    const int ia = block("A", {holdA ? -4.0f : -5.0f, 2.1f}, 1.2f, 0.6f, 4.0f, holdA ? 0.0f : kSlide).id;
    auto& wall = add(s, "Wall", ObjKind::Block, {11.5f, 2.0f});
    wall.width = 1.0f;
    wall.height = 4.0f;
    wall.fixed = true;
    // the rope leaves B, runs over the wheel on the wall and back under it to C
    auto& rope = link(s, LinkKind::Pulley, {ib, {}}, {ic, {}});
    rope.wheel = {10.0f, 0.95f};
    rope.wheelRadius = 0.45f;
    if (holdA) link(s, LinkKind::Rope, {-1, {-11.0f, 2.1f}}, {ia, {}});
    return s;
}

Scene blocksFree() { return blocksSetup(false); }
Scene blocksHeld() { return blocksSetup(true); }

bool blocksMonitor(const Simulation& sim, ExampleRun& run) {
    const int ic = sim.indexNamed("C");
    const Body* a = sim.body("A");
    const Body* bb = sim.body("B");
    if (ic < 0 || !a || !bb || run.done) return false;
    const float F = sim.driveForce(ic), t = sim.time();
    float tension = 0.0f;
    for (const Joint* j : sim.built().joints)
        if (auto* p = dynamic_cast<const PulleyJoint*>(j)) tension = p->tension();

    // average once the drive has settled
    if (t >= 2.0f) {
        run.prev[0] += F;
        run.prev[1] += tension;
        run.prev[2] += 1.0f;
    }
    if (t >= 3.5f) {
        const float n = run.prev[2];
        run.result = run.prev[0] / n;
        run.lines = {format("Force needed on C = %.1f N", run.prev[0] / n),
                     format("rope tension T = %.1f N", run.prev[1] / n),
                     format("A moves at %.2f m/s, B at %.2f m/s", a->velocity.x, bb->velocity.x)};
        run.done = true;
        return true;
    }
    run.lines = {format("t = %.2f s  (averaging from 2 s to 3.5 s)", t), format("drive force on C  %.1f N", F),
                 format("rope tension      %.1f N", tension),
                 format("A vx %.2f   B vx %.2f m/s", a->velocity.x, bb->velocity.x)};
    return false;
}

} // namespace

const std::vector<Example>& examples() {
    static const std::vector<Example> list = {
        {"Balloon and stone",
         "A gas balloon is going up with a constant velocity of 10 m/s. When this balloon reached a height of 75 m, a "
         "stone is dropped from it and the balloon keeps moving up with the same velocity. The height of the balloon "
         "when the stone hits the ground is ____ m. (Take g = 10 m/s^2)",
         "125 m. The dropped stone starts with the balloon's velocity, 10 m/s upward. Taking up as positive: "
         "-75 = 10t - 5t^2, so t^2 - 2t - 15 = 0 and t = 5 s. The balloon rises 10 x 5 = 50 m more: 75 + 50 = 125 m.",
         125.0f, balloonSetup, {0.0f, 66.0f}, 3.0f, "Stone", 1, balloonMonitor},
        {"Blocks A, B, C (as drawn)",
         "In the given figure the blocks A, B and C weigh 4 kg, 6 kg and 8 kg respectively. The coefficient of sliding "
         "friction between any two surfaces is 0.5. The force F required to slide the block C with constant speed is "
         "____ N. (Use g = 10 m/s^2)  [JEE Main 2026, 24 Jan]  Here A rests freely on B, exactly as in the figure.",
         "190 N for the figure as drawn. At constant speed nothing pushes A sideways, so A rides along with B and the "
         "A-B friction is zero. B: T = f(BC) = 0.5 x (4+6) x 10 = 50 N. C: F = f(BC) + T + f(ground) = 50 + 50 + "
         "0.5 x 18 x 10 = 190 N. The official key says 210 N: it puts kinetic friction 20 N between A and B, which "
         "only happens if A is held still (see the next example).",
         190.0f, blocksFree, {4.0f, 3.0f}, 30.0f, "C", 4, blocksMonitor},
        {"Blocks A, B, C (A held)",
         "Same problem, but A is tied to a fixed point so it stays at rest while B slides under it. This is the "
         "situation the official answer key assumes.",
         "210 N, the official answer. A-B friction is now kinetic: 0.5 x 4 x 10 = 20 N. B: T = 20 + 50 = 70 N. "
         "C: F = f(BC) + T + f(ground) = 50 + 70 + 90 = 210 N.",
         210.0f, blocksHeld, {4.0f, 3.0f}, 30.0f, "C", 4, blocksMonitor},
    };
    return list;
}

} // namespace editor
