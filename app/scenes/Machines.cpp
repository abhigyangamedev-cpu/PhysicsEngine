// Pulleys & Machines: Atwood machines, block and tackle, an incline with a pulley, levers and gears.
#include "SceneKit.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace pe;
using namespace kit;

namespace {

float g_time = 0.0f;

// ---- Atwood machine ----

struct Atwood {
    Body* heavy;
    Body* light;
    PulleyJoint* rope;
    Vec2 wheel;
    float radius;
    AccelMeter accel;
    float tension = 0.0f; // sampled while the masses move; the rope goes slack once one lands
};
std::vector<Atwood> g_atwoods;

void addAtwood(World& w, float x, float heavyMass, float lightMass) {
    const Vec2 wheel{x, 12.0f};
    const float r = 0.6f;
    addPulleyWheel(w, wheel, r);
    // each box hangs straight below its side of the wheel
    float sh = 0.3f + 0.05f * heavyMass, sl = 0.3f + 0.05f * lightMass;
    Body* heavy = addBody(w, Shape::box(sh, sh), {x - r, 7.0f}, heavyMass, 0xe76f51ff);
    Body* light = addBody(w, Shape::box(sl, sl), {x + r, 3.0f}, lightMass, 0x2a9d8fff);
    heavy->group = light->group = 1;
    auto* rope = w.createJoint<PulleyJoint>(heavy, light, wheel - Vec2{r, 0.0f}, wheel + Vec2{r, 0.0f},
                                            heavy->position + Vec2{0.0f, sh}, light->position + Vec2{0.0f, sl});
    g_atwoods.push_back({heavy, light, rope, wheel, r, {}, 0.0f});
}

void atwood(World& w) {
    g_time = 0.0f;
    g_atwoods.clear();
    addStaticBox(w, {0.0f, -0.5f}, 20.0f, 0.5f);
    addStaticBox(w, {0.0f, 12.9f}, 12.0f, 0.15f); // beam the pulleys hang from
    addAtwood(w, -7.0f, 3.0f, 2.0f);
    addAtwood(w, 0.0f, 5.0f, 1.0f);
    addAtwood(w, 7.0f, 2.2f, 2.0f);
}

void atwoodUpdate(World&, float dt) {
    g_time += dt;
    for (auto& a : g_atwoods) {
        if (g_time > 0.2f && a.heavy->position.y > 1.2f && a.heavy->velocity.y < -1e-3f) {
            a.accel.add(g_time, -a.heavy->velocity.y);
            a.tension = a.rope->tension();
        }
    }
}

void atwoodReadout(const World&, Readout& r) {
    r.add("Atwood machine   a = (m1 - m2) g / (m1 + m2)   T = 2 m1 m2 g / (m1 + m2)");
    for (const auto& a : g_atwoods) {
        float m1 = a.heavy->mass, m2 = a.light->mass;
        float aTheory = (m1 - m2) / (m1 + m2) * kG, tTheory = 2.0f * m1 * m2 * kG / (m1 + m2);
        r.add("%.1f kg vs %.1f kg   a: theory %.2f  measured %.2f m/s^2   T: theory %.1f  measured %.1f N", m1, m2,
              aTheory, a.accel.ready() ? a.accel.value() : 0.0f, tTheory, a.tension);
        arc(r, a.wheel, a.radius, 0.0f, kPi, kRope);
    }
}

// ---- Block and tackle ----

struct Tackle {
    Body* effort;
    Body* load;
    PulleyJoint* rope;
    Vec2 effortStart, loadStart;
    Vec2 top, side;
};
std::vector<Tackle> g_tackles;

void addTackle(World& w, float x, int advantage, float loadMass) {
    const Vec2 top{x, 13.0f}, side{x + 2.0f, 13.0f};
    addPulleyWheel(w, top, 0.55f);
    addPulleyWheel(w, side, 0.35f);
    // the load hangs from a moving block carried by `advantage` strands; the effort pulls one strand
    Body* load = addBody(w, Shape::circle(0.55f), {x, 4.0f}, loadMass, 0x5e81acff);
    float effortMass = 1.15f * loadMass / advantage; // a little more than balance, so it slowly lifts
    float s = 0.25f + 0.08f * effortMass;
    Body* effort = addBody(w, Shape::box(s, s), {x + 2.35f, 8.0f}, effortMass, 0xe76f51ff);
    effort->group = load->group = 1;
    auto* rope = w.createJoint<PulleyJoint>(effort, load, side + Vec2{0.35f, 0.0f}, top, effort->position + Vec2{0.0f, s},
                                            load->position, float(advantage));
    g_tackles.push_back({effort, load, rope, effort->position, load->position, top, side});
}

void blockAndTackle(World& w) {
    g_tackles.clear();
    addStaticBox(w, {0.0f, -0.5f}, 20.0f, 0.5f);
    addStaticBox(w, {0.0f, 13.9f}, 14.0f, 0.15f);
    addTackle(w, -10.0f, 1, 4.0f);
    addTackle(w, -3.0f, 2, 4.0f);
    addTackle(w, 4.0f, 4, 4.0f);
}

void blockAndTackleReadout(const World&, Readout& r) {
    r.add("Block and tackle: n strands hold the load, so it needs 1/n of the force but moves 1/n as far");
    for (const auto& t : g_tackles) {
        float de = t.effortStart.y - t.effort->position.y, dl = t.load->position.y - t.loadStart.y;
        r.add("MA %.0f: %.2f kg lifts %.1f kg   effort down %.2f m, load up %.2f m   tension %.1f N", t.rope->ratio,
              t.effort->mass, t.load->mass, de, dl, t.rope->tension());
        r.line(t.top + Vec2{0.55f, 0.0f}, t.side + Vec2{0.0f, 0.35f}, kRope);
        arc(r, t.side, 0.35f, 0.0f, kPi * 0.5f, kRope);
    }
}

// ---- Incline with a pulley ----

constexpr float kInclineAngle = 30.0f * kPi / 180.0f;
constexpr float kMu = 0.2f;
Body* g_slider = nullptr;
Body* g_hanger = nullptr;
PulleyJoint* g_inclineRope = nullptr;
Vec2 g_inclineWheel;
AccelMeter g_inclineAccel;

void inclinePulley(World& w) {
    g_time = 0.0f;
    g_inclineAccel.reset();
    addStaticBox(w, {0.0f, -0.5f}, 20.0f, 0.5f);

    const float run = 12.0f, rise = run * std::tan(kInclineAngle);
    const Vec2 foot{-10.0f, 0.0f}, top{foot.x + run, rise};
    addStaticPolygon(w, {foot, {top.x, 0.0f}, top}, kMu);

    const Vec2 dir{std::cos(kInclineAngle), std::sin(kInclineAngle)}, n = dir.perp();
    const float hw = 0.5f, hh = 0.3f, r = 0.4f;
    // the rope leaves the block parallel to the slope and meets the wheel at a tangent
    const Vec2 ropeTop = top + n * hh + dir * 0.25f;
    g_inclineWheel = ropeTop - n * r;
    addPulleyWheel(w, g_inclineWheel, r);

    BodyDef d;
    d.position = top - dir * 6.0f + n * hh;
    d.angle = kInclineAngle;
    d.friction = kMu;
    d.color = 0xe9c46aff;
    d.density = 2.0f / (4.0f * hw * hh);
    g_slider = w.createBody(Shape::box(hw, hh), d);

    const Vec2 groundB = g_inclineWheel + Vec2{r, 0.0f};
    g_hanger = addBody(w, Shape::box(0.32f, 0.32f), {groundB.x, 3.5f}, 1.5f, 0xe76f51ff);
    g_inclineRope = w.createJoint<PulleyJoint>(g_slider, g_hanger, ropeTop, groundB, g_slider->position + dir * hw,
                                               g_hanger->position + Vec2{0.0f, 0.32f});
}

void inclinePulleyUpdate(World&, float dt) {
    g_time += dt;
    if (g_time > 0.2f && g_hanger->position.y > 0.8f) g_inclineAccel.add(g_time, -g_hanger->velocity.y);
}

void inclinePulleyReadout(const World&, Readout& r) {
    const float m1 = g_slider->mass, m2 = g_hanger->mass, s = std::sin(kInclineAngle), c = std::cos(kInclineAngle);
    const float a = kG * (m2 - m1 * s - kMu * m1 * c) / (m1 + m2);
    r.add("Incline + pulley   a = g (m2 - m1 sin(t) - mu m1 cos(t)) / (m1 + m2)");
    r.add("m1 = %.1f kg on 30 deg slope, mu = %.1f   m2 = %.1f kg hanging", m1, kMu, m2);
    r.add("a: theory %.2f  measured %.2f m/s^2   tension %.1f N", a, g_inclineAccel.ready() ? g_inclineAccel.value() : 0.0f,
          g_inclineRope->tension());
    arc(r, g_inclineWheel, 0.4f, 0.0f, kInclineAngle + kPi * 0.5f, kRope);
}

// ---- Lever and torque ----

struct Lever {
    Body* plank;
    Vec2 pivot;
};
struct Weight {
    int lever;
    Body* body;
    DistanceJoint* rope;
};
std::vector<Lever> g_levers;
std::vector<Weight> g_weights;

void hangWeight(World& w, int lever, float arm, float mass) {
    const Lever& l = g_levers[lever];
    Vec2 attach = l.plank->position + l.plank->rot.apply({arm, -0.12f});
    float s = 0.2f + 0.07f * mass;
    Body* b = addBody(w, Shape::box(s, s), attach - Vec2{0.0f, 1.2f + s}, mass, mass >= 3.0f ? 0xe76f51ff : 0x2a9d8fff);
    b->group = 2;
    auto* rope = w.createJoint<DistanceJoint>(l.plank, b, attach, b->position + Vec2{0.0f, s});
    rope->rope = true;
    g_weights.push_back({lever, b, rope});
}

void addLever(World& w, Vec2 pivot) {
    Body* fulcrum = addStaticPolygon(w, {{pivot.x - 0.8f, 0.0f}, {pivot.x + 0.8f, 0.0f}, pivot});
    Body* plank = addBody(w, Shape::box(5.0f, 0.12f), pivot, 0.6f, 0xb48eadff);
    w.createJoint<RevoluteJoint>(fulcrum, plank, pivot);
    g_levers.push_back({plank, pivot});
}

void lever(World& w) {
    g_levers.clear();
    g_weights.clear();
    addStaticBox(w, {0.0f, -0.5f}, 20.0f, 0.5f);
    addLever(w, {-7.5f, 4.0f});
    addLever(w, {7.5f, 4.0f});
    hangWeight(w, 0, -2.0f, 3.0f); // 3 kg x 2 m = 2 kg x 3 m: balanced
    hangWeight(w, 0, 3.0f, 2.0f);
    hangWeight(w, 1, -4.0f, 2.0f); // 2 kg x 4 m > 3 kg x 2 m: tips left
    hangWeight(w, 1, 2.0f, 3.0f);
}

bool leverClick(World& w, Vec2 p, int button) {
    if (button != 3) return false;
    // hang 1 kg from the nearest lever, directly above the click
    int best = p.x < 0.0f ? 0 : 1;
    const Lever& l = g_levers[best];
    Vec2 local = l.plank->rot.applyT(p - l.plank->position);
    hangWeight(w, best, std::clamp(local.x, -4.8f, 4.8f), 1.0f);
    return true;
}

void leverReadout(const World&, Readout& r) {
    r.add("Lever: balanced when the turning moments match, sum of m g d = 0   (right click: hang 1 kg)");
    for (size_t i = 0; i < g_levers.size(); ++i) {
        float left = 0.0f, right = 0.0f;
        std::string parts;
        for (const auto& wt : g_weights) {
            if (wt.lever != int(i)) continue;
            float d = wt.rope->anchorA().x - g_levers[i].pivot.x;
            float moment = wt.body->mass * kG * d;
            (d < 0.0f ? left : right) += std::abs(moment);
            char buf[48];
            std::snprintf(buf, sizeof buf, "%s%.0fkg@%.1fm", parts.empty() ? "" : " ", wt.body->mass, d);
            parts += buf;
        }
        float tilt = g_levers[i].plank->angle * 180.0f / kPi;
        r.add("Lever %zu: %s   moments L %.1f / R %.1f N m   tilt %.1f deg", i + 1, parts.c_str(), left, right, tilt);
    }
}

// ---- Gears and belts ----

struct Gear {
    Body* body;
    float radius;
    int teeth;
};
std::vector<Gear> g_gears;
constexpr float kBeltHub = 1.5f; // the belt runs on a hub on the big gear's axle

RevoluteJoint* addGear(World& w, Vec2 center, float radius, uint32_t color, float phase = 0.0f) {
    Body* hub = addAnchor(w, center);
    BodyDef d;
    d.position = center;
    d.angle = phase;
    d.collides = false;
    d.color = color;
    Body* g = w.createBody(Shape::circle(radius), d);
    g_gears.push_back({g, radius, std::max(8, int(std::lround(8.0f * radius)))});
    return w.createJoint<RevoluteJoint>(hub, g, center);
}

void gears(World& w) {
    g_gears.clear();
    w.settings.gravity = {};
    w.settings.angularDamping = 0.0f;

    RevoluteJoint* motor = addGear(w, {-9.0f, 7.0f}, 2.0f, 0xe76f51ff);
    motor->enableMotor = true;
    motor->motorSpeed = 1.0f;
    motor->maxMotorTorque = 1e4f;
    addGear(w, {-6.0f, 7.0f}, 1.0f, 0xe9c46aff, kPi / 8.0f);
    addGear(w, {-2.0f, 7.0f}, 3.0f, 0x2a9d8fff); // phases chosen so teeth fall into gaps
    addGear(w, {7.0f, 7.0f}, 0.75f, 0x5e81acff);
    const Gear &driver = g_gears[0], &small = g_gears[1], &big = g_gears[2], &pulley = g_gears[3];
    w.createJoint<GearJoint>(driver.body, small.body, driver.radius / small.radius);
    w.createJoint<GearJoint>(small.body, big.body, small.radius / big.radius);
    w.createJoint<GearJoint>(big.body, pulley.body, -kBeltHub / pulley.radius); // a belt keeps the direction
}

void gearsReadout(const World&, Readout& r) {
    r.add("Gears: meshing gears turn opposite ways, w2 = -w1 r1 / r2; a belt keeps the direction");
    const char* names[] = {"driver r=2", "gear r=1", "gear r=3", "belt pulley r=0.75"};
    float expected = g_gears[0].body->angularVelocity;
    for (size_t i = 0; i < g_gears.size(); ++i) {
        const Gear& g = g_gears[i];
        if (i == 1 || i == 2) expected *= -g_gears[i - 1].radius / g.radius;
        if (i == 3) expected *= kBeltHub / g.radius;
        r.add("%-18s w = %+.2f rad/s   predicted %+.2f", names[i], g.body->angularVelocity, expected);
        if (i == 3) continue; // a belt pulley has no teeth

        // teeth
        const uint32_t c = 0xdcdcdcff;
        for (int k = 0; k < g.teeth; ++k) {
            float a = g.body->angle + 2.0f * kPi * k / g.teeth, half = kPi / g.teeth * 0.45f;
            Vec2 c0{std::cos(a - half), std::sin(a - half)}, c1{std::cos(a + half), std::sin(a + half)};
            Vec2 p = g.body->position;
            float rin = g.radius - 0.12f, rout = g.radius + 0.12f; // teeth straddle the pitch circle
            r.line(p + c0 * rin, p + c0 * rout, c);
            r.line(p + c0 * rout, p + c1 * rout, c);
            r.line(p + c1 * rout, p + c1 * rin, c);
        }
    }
    // belt: the two outer tangents between the big gear's hub circle and the pulley
    const Gear& a = g_gears[2];
    const Gear& b = g_gears[3];
    const float ra = kBeltHub, rb = b.radius;
    Vec2 d = b.body->position - a.body->position;
    float len = d.length();
    Vec2 u = d / len;
    float s = (ra - rb) / len, c = std::sqrt(std::max(0.0f, 1.0f - s * s));
    for (float side : {1.0f, -1.0f}) {
        Vec2 n = u * s + u.perp() * (c * side);
        r.line(a.body->position + n * ra, b.body->position + n * rb, kRope);
    }
    ring(r, a.body->position, ra, kRope);
}

} // namespace

std::vector<Scene> machineScenes(int c) {
    return {
        {.name = "Atwood", .category = c, .setup = atwood, .update = atwoodUpdate, .readout = atwoodReadout,
         .cameraCenter = {0.0f, 8.0f}, .cameraScale = 36.0f},
        {.name = "Block & Tackle", .category = c, .setup = blockAndTackle, .readout = blockAndTackleReadout,
         .cameraCenter = {0.0f, 8.0f}, .cameraScale = 36.0f},
        {.name = "Incline & Pulley", .category = c, .setup = inclinePulley, .update = inclinePulleyUpdate,
         .readout = inclinePulleyReadout, .cameraCenter = {-2.0f, 5.0f}},
        {.name = "Lever", .category = c, .setup = lever, .click = leverClick, .readout = leverReadout,
         .cameraCenter = {0.0f, 4.5f}},
        {.name = "Gears & Belt", .category = c, .setup = gears, .readout = gearsReadout},
    };
}
