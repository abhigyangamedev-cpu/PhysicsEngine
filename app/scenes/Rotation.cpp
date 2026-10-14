// Rotation & Rolling: a rolling race, sliding into rolling, and conservation of angular momentum.
#include "SceneKit.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace pe;
using namespace kit;

namespace {

float g_time = 0.0f;

// ---- Rolling race ----

constexpr float kSlope = 20.0f * kPi / 180.0f, kRaceDistance = 10.0f;
struct Racer {
    Body* body;
    const char* name;
    float inertiaFactor; // I / (m r^2); a frictionless slider has none to spin up
    Vec2 start;
    float finish = 0.0f;
    AccelMeter accel;
    bool hoop = false;
};
std::vector<Racer> g_racers;

void rollingRace(World& w) {
    g_time = 0.0f;
    g_racers.clear();
    w.settings.angularDamping = 0.0f;
    const Vec2 n{-std::sin(kSlope), std::cos(kSlope)}; // out of the slope, which runs downhill to the left
    const float run = 11.0f, rise = run * std::tan(kSlope); // 11.7 m of slope, 1.5 m clear of the ramp above
    struct Spec { const char* name; float factor; };
    const Spec specs[] = {{"block (sliding, no friction)", 0.0f}, {"solid disc  I = 1/2 m r^2", 0.5f}, {"hoop  I = m r^2", 1.0f}};
    for (int i = 0; i < 3; ++i) {
        const float baseY = 11.0f - 5.5f * i, x0 = -5.0f;
        addStaticPolygon(w, {{x0, baseY}, {x0 + run, baseY}, {x0 + run, baseY + rise}}, 0.9f);
        addStaticBox(w, {x0 - 4.0f, baseY - 0.25f}, 4.0f, 0.25f)->friction = 0.9f; // run-out at the foot
        addStaticBox(w, {x0 - 8.2f, baseY + 0.6f}, 0.2f, 0.6f);
        const Vec2 top{x0 + run - 0.8f, baseY + rise - 0.8f * std::tan(kSlope)};
        Body* b;
        if (specs[i].factor == 0.0f) {
            b = addBody(w, Shape::box(0.45f, 0.45f), top + n * 0.45f, 1.0f, 0xe9c46aff);
            b->angle = kSlope;
            b->friction = 0.0f;
            b->updateTransform();
        } else {
            b = addBody(w, Shape::circle(0.5f), top + n * 0.5f, 1.0f, specs[i].factor < 1.0f ? 0x2a9d8fff : 0xe76f51ff);
            b->friction = 0.9f;
            if (specs[i].factor == 1.0f) {
                b->inertia = b->mass * 0.25f;
                b->invInertia = 1.0f / b->inertia;
            }
        }
        g_racers.push_back({b, specs[i].name, specs[i].factor, b->position, 0.0f, {}, specs[i].factor == 1.0f});
    }
}

void rollingRaceUpdate(World&, float dt) {
    g_time += dt;
    for (auto& r : g_racers) {
        float travelled = (r.body->position - r.start).length();
        if (r.finish == 0.0f && travelled >= kRaceDistance) r.finish = g_time;
        if (r.finish == 0.0f && g_time > 0.1f) r.accel.add(g_time, r.body->velocity.length());
    }
}

void rollingRaceReadout(const World&, Readout& r) {
    r.add("Rolling race down a 20 deg slope: a = g sin(t) / (1 + I / m r^2)");
    r.add("Energy spent spinning up is not available for speed, so the hoop comes last");
    for (const auto& racer : g_racers) {
        float a = kG * std::sin(kSlope) / (1.0f + racer.inertiaFactor);
        char finish[16] = "...";
        if (racer.finish > 0.0f) std::snprintf(finish, sizeof finish, "%.2f", racer.finish);
        r.add("%-30s a: theory %.2f measured %.2f   %.0f m in %4s s (theory %.2f)", racer.name, a,
              racer.accel.ready() ? racer.accel.value() : 0.0f, kRaceDistance, finish, std::sqrt(2.0f * kRaceDistance / a));
        if (racer.hoop) ring(r, racer.body->position, 0.38f, 0x12141aff);
    }
}

// ---- Slip to roll ----

constexpr float kDiscRadius = 0.5f, kSlipMu = 0.3f;
struct Slider {
    Body* body;
    float v0, spin0; // spin0 = -w0 r: positive is topspin when moving right
    const char* label;
};
std::vector<Slider> g_sliders;

void slipToRoll(World& w) {
    g_sliders.clear();
    w.settings.angularDamping = 0.0f;
    struct Spec { float v0, spin, y; const char* label; };
    for (const Spec& s : {Spec{8.0f, 0.0f, 10.5f, "no spin"}, Spec{8.0f, -12.0f, 7.0f, "backspin"},
                          Spec{8.0f, -24.0f, 3.5f, "heavy backspin"}, Spec{8.0f, 16.0f, 0.0f, "topspin"}}) {
        Body* floor = addStaticBox(w, {22.0f, s.y - 0.25f}, 40.0f, 0.25f);
        floor->friction = kSlipMu;
        Body* b = addBody(w, Shape::circle(kDiscRadius), {-12.0f, s.y + kDiscRadius}, 1.0f,
                          s.spin < 0 ? 0xe76f51ff : (s.spin > 0 ? 0x2a9d8fff : 0xe9c46aff));
        b->friction = kSlipMu;
        b->restitution = 0.0f;
        b->velocity = {s.v0, 0.0f};
        b->angularVelocity = -s.spin / kDiscRadius;
        g_sliders.push_back({b, s.v0, s.spin, s.label});
    }
}

void slipToRollReadout(const World&, Readout& r) {
    r.add("Sliding into rolling: friction acts until v = w r. For a disc, angular momentum about");
    r.add("the contact point is conserved, so the final speed is 2/3 (v0 + w0 r / 2)");
    for (const auto& s : g_sliders) {
        float v = s.body->velocity.x, wr = -s.body->angularVelocity * kDiscRadius;
        bool rolling = std::abs(v - wr) < 0.05f;
        r.add("%-15s v0 = %.0f  w0 r = %+3.0f   v = %+5.2f  w r = %+5.2f  %s   final: theory %+.2f", s.label, s.v0, s.spin0, v,
              wr, rolling ? "rolling" : "sliding", 2.0f / 3.0f * (s.v0 + 0.5f * s.spin0));
    }
}

// ---- Angular momentum ----

constexpr float kStartSeparation = 8.0f, kMinSeparation = 2.0f, kSpinMass = 1.0f, kOmega0 = 1.0f;
const Vec2 kSpinCenter{0.0f, 6.0f};
Body* g_spinA = nullptr;
Body* g_spinB = nullptr;

// Separation over a 16 s cycle: hold, pull in over 5 s, hold, let out over 5 s.
float separation(float time) {
    float t = std::fmod(time, 16.0f), f;
    if (t < 3.0f) f = 0.0f;
    else if (t < 8.0f) f = (t - 3.0f) / 5.0f;
    else if (t < 11.0f) f = 1.0f;
    else f = 1.0f - std::min(1.0f, (t - 11.0f) / 5.0f);
    f = f * f * (3.0f - 2.0f * f);
    return kStartSeparation + (kMinSeparation - kStartSeparation) * f;
}

void angularMomentum(World& w) {
    g_time = 0.0f;
    w.settings.gravity = {};
    w.settings.angularDamping = 0.0f;
    const float r = kStartSeparation * 0.5f;
    g_spinA = addBody(w, Shape::circle(0.4f), kSpinCenter - Vec2{r, 0.0f}, kSpinMass, 0xe76f51ff);
    g_spinB = addBody(w, Shape::circle(0.4f), kSpinCenter + Vec2{r, 0.0f}, kSpinMass, 0x2a9d8fff);
    g_spinA->velocity = {0.0f, -kOmega0 * r};
    g_spinB->velocity = {0.0f, kOmega0 * r};
    g_spinA->group = g_spinB->group = 1;
}

void angularMomentumUpdate(World&, float dt) {
    g_time += dt;
    // The string pulls each mass straight toward the centre. Shortening a joint wouldn't do: the
    // solver moves bodies to fix the length and then discards that velocity, so the masses would
    // slide inward without the radial motion that spins them up. Here each mass gets a purely
    // radial velocity that brings it to the target radius over the next step; central impulses
    // can't change r x v, so angular momentum is conserved exactly.
    const float target = 0.5f * separation(g_time + dt);
    for (Body* b : {g_spinA, g_spinB}) {
        Vec2 r = b->position - kSpinCenter;
        float len = r.length();
        Vec2 u = r / len;
        float vt = cross(u, b->velocity); // tangential speed, left alone
        float along = std::sqrt(std::max(0.0f, target * target - vt * vt * dt * dt)); // straight-line drift adds vt dt
        b->velocity = u * ((along - len) / dt) + u.perp() * vt;
    }
}

void angularMomentumReadout(const World&, Readout& r) {
    const Vec2 c = kSpinCenter;
    float L = 0.0f, ke = 0.0f;
    for (Body* b : {g_spinA, g_spinB}) {
        L += cross(b->position - c, b->velocity) * b->mass + b->inertia * b->angularVelocity;
        ke += kineticEnergy(*b);
    }
    float d = (g_spinB->position - g_spinA->position).length();
    float omega = cross(g_spinB->position - g_spinA->position, g_spinB->velocity - g_spinA->velocity) / (d * d);
    float predicted = kOmega0 * (kStartSeparation * kStartSeparation) / (d * d);
    r.add("Angular momentum L = I w is conserved: pull the masses in and they spin faster");
    r.add("separation %.2f m   w = %.2f rad/s   predicted w0 (d0 / d)^2 = %.2f   (point masses)", d, omega, predicted);
    r.add("L = %.2f kg m^2/s (started at %.2f)   kinetic energy %.1f J: the string does work pulling them in", L,
          2.0f * kSpinMass * kOmega0 * 0.25f * kStartSeparation * kStartSeparation, ke);
    ring(r, c, d * 0.5f, 0x3a3f4eff, 48);
    r.line(g_spinA->position, g_spinB->position, 0xc8965aff);
}

} // namespace

std::vector<Scene> rotationScenes(int c) {
    return {
        {.name = "Rolling Race", .category = c, .setup = rollingRace, .update = rollingRaceUpdate,
         .readout = rollingRaceReadout, .cameraCenter = {-3.0f, 8.5f}, .cameraScale = 32.0f},
        {.name = "Slip to Roll", .category = c, .setup = slipToRoll, .readout = slipToRollReadout,
         .cameraCenter = {0.0f, 5.5f}, .cameraScale = 36.0f},
        {.name = "Angular Momentum", .category = c, .setup = angularMomentum, .update = angularMomentumUpdate,
         .readout = angularMomentumReadout, .cameraCenter = {0.0f, 6.0f}, .trails = true},
    };
}
