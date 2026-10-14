// Oscillations & Waves: spring-mass SHM, a pendulum wave, coupled pendulums, resonance and standing waves.
#include "SceneKit.h"
#include "field/Fields.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace pe;
using namespace kit;

namespace {

float g_time = 0.0f;
SpringField* g_springs = nullptr;

// ---- Springs and simple harmonic motion ----

struct Oscillator {
    Body* mass;
    float k, restY; // equilibrium height
    PeriodMeter period;
};
std::vector<Oscillator> g_oscillators;

void springsShm(World& w) {
    g_time = 0.0f;
    g_oscillators.clear();
    g_springs = w.addField<SpringField>();
    addStaticBox(w, {0.0f, 13.6f}, 12.0f, 0.15f);
    struct Spec { float x, m, k; };
    for (const Spec& s : {Spec{-7.0f, 1.0f, 10.0f}, Spec{0.0f, 4.0f, 10.0f}, Spec{7.0f, 1.0f, 40.0f}}) {
        const Vec2 top{s.x, 13.45f};
        const float rest = 3.0f, eq = top.y - rest - s.m * kG / s.k;
        float size = 0.3f + 0.1f * s.m;
        Body* b = addBody(w, Shape::box(size, size), {s.x, eq}, s.m, s.m > 2.0f ? 0xe76f51ff : 0x2a9d8fff);
        g_springs->add(b, b->position, nullptr, top, s.k).rest = rest;
        b->position.y -= 1.2f; // start 1.2 m below equilibrium
        b->updateTransform();
        g_oscillators.push_back({b, s.k, eq, {}});
    }
}

void springsShmUpdate(World&, float dt) {
    g_time += dt;
    for (auto& o : g_oscillators) o.period.add(g_time, o.mass->velocity.y, dt);
}

void springsShmReadout(const World&, Readout& r) {
    r.add("Simple harmonic motion: T = 2 pi sqrt(m / k), independent of amplitude");
    for (const auto& o : g_oscillators) {
        float m = o.mass->mass, x = o.mass->position.y - o.restY;
        float energy = 0.5f * m * o.mass->velocity.lengthSq() + 0.5f * o.k * x * x;
        char measured[16] = "...";
        if (o.period.period > 0.0f) std::snprintf(measured, sizeof measured, "%.2f", o.period.period);
        r.add("m = %.0f kg  k = %.0f N/m   T: theory %.2f s  measured %4s s   energy %.2f J", m, o.k,
              2.0f * kPi * std::sqrt(m / o.k), measured, energy);
    }
}

// ---- Pendulum wave ----

constexpr int kPendulums = 15;
constexpr float kWaveCycle = 60.0f; // pendulum n swings 15 + n times per cycle

void pendulumWave(World& w) {
    g_time = 0.0f;
    w.settings.angularDamping = 0.0f;
    addStaticBox(w, {0.0f, 13.3f}, 12.0f, 0.12f);
    const float amplitude = 8.0f * kPi / 180.0f;
    for (int n = 0; n < kPendulums; ++n) {
        float period = kWaveCycle / (15 + n);
        float length = kG * (period / (2.0f * kPi)) * (period / (2.0f * kPi));
        Vec2 pivot{-10.5f + 1.5f * n, 13.18f};
        Body* top = addAnchor(w, pivot);
        Vec2 bob = pivot + Vec2{std::sin(amplitude), -std::cos(amplitude)} * length;
        Body* b = addBody(w, Shape::circle(0.2f), bob, 1.0f, n % 2 ? 0x48aadcff : 0xe9c46aff);
        b->group = 1;
        w.createJoint<DistanceJoint>(top, b, pivot, bob);
    }
}

void pendulumWaveUpdate(World&, float dt) { g_time += dt; }

void pendulumWaveReadout(const World&, Readout& r) {
    r.add("Pendulum wave: lengths chosen so pendulum n swings 15 + n times every %.0f s", kWaveCycle);
    float phase = std::fmod(g_time, kWaveCycle);
    r.add("t = %.1f s   back in step in %.1f s   (half way, %.0f s: alternating)", g_time, kWaveCycle - phase,
          kWaveCycle * 0.5f);
}

// ---- Coupled pendulums ----

constexpr float kCoupledLength = 6.0f;
constexpr float kCoupling = 0.4f; // N/m
struct Pair {
    Body* a;
    Body* b;
    Vec2 pivotA, pivotB;
    const char* label;
};
std::vector<Pair> g_pairs;

float pendulumEnergy(const Body& b, Vec2 pivot) {
    return 0.5f * b.mass * b.velocity.lengthSq() + b.mass * kG * (b.position.y - (pivot.y - kCoupledLength));
}

void coupledPendulums(World& w) {
    g_time = 0.0f;
    g_pairs.clear();
    w.settings.angularDamping = 0.0f;
    g_springs = w.addField<SpringField>();
    addStaticBox(w, {0.0f, 13.3f}, 14.0f, 0.12f);
    const float swing = 10.0f * kPi / 180.0f;
    struct Spec { float x, angleA, angleB; const char* label; };
    for (const Spec& s : {Spec{-9.0f, swing, 0.0f, "one pushed: energy beats"}, Spec{0.0f, swing, swing, "in phase: no transfer"},
                          Spec{9.0f, swing, -swing, "opposite: no transfer"}}) {
        Vec2 pa{s.x - 1.5f, 13.18f}, pb{s.x + 1.5f, 13.18f};
        Body* bodies[2];
        for (int i = 0; i < 2; ++i) {
            Vec2 pivot = i ? pb : pa;
            float angle = i ? s.angleB : s.angleA;
            Vec2 bob = pivot + Vec2{std::sin(angle), -std::cos(angle)} * kCoupledLength;
            bodies[i] = addBody(w, Shape::circle(0.3f), bob, 1.0f, i ? 0x2a9d8fff : 0xe76f51ff);
            bodies[i]->group = 1;
            w.createJoint<DistanceJoint>(addAnchor(w, pivot), bodies[i], pivot, bob);
        }
        // the coupling spring is relaxed when both hang straight down
        auto& spring = g_springs->add(bodies[0], bodies[0]->position, bodies[1], bodies[1]->position, kCoupling);
        spring.rest = 3.0f;
        g_pairs.push_back({bodies[0], bodies[1], pa, pb, s.label});
    }
}

void coupledUpdate(World&, float dt) { g_time += dt; }

void coupledReadout(const World&, Readout& r) {
    const float w1 = std::sqrt(kG / kCoupledLength), w2 = std::sqrt(kG / kCoupledLength + 2.0f * kCoupling);
    r.add("Coupled pendulums: modes at %.3f and %.3f rad/s; energy swaps sides every %.1f s", w1, w2, kPi / (w2 - w1));
    for (const auto& p : g_pairs) {
        float ea = pendulumEnergy(*p.a, p.pivotA), eb = pendulumEnergy(*p.b, p.pivotB), total = std::max(ea + eb, 1e-6f);
        r.add("%-26s left %3.0f%%  right %3.0f%%", p.label, 100.0f * ea / total, 100.0f * eb / total);
    }
    r.add("t = %.1f s", g_time);
}

// ---- Driven oscillators and resonance ----

constexpr float kDriveAmplitude = 0.15f;
constexpr float kResonanceK = 20.0f;
struct Driven {
    Body* mass;
    float eqY, amplitude;
};
std::vector<Driven> g_driven;
int g_driveIndex = 2;
const float kResonanceMasses[] = {2.0f, 1.25f, 0.8f, 0.55f, 0.4f};

float naturalHz(float m) { return std::sqrt(kResonanceK / m) / (2.0f * kPi); }

void resonance(World& w) {
    g_time = 0.0f;
    g_driven.clear();
    g_springs = w.addField<SpringField>();
    const float driveHz = naturalHz(kResonanceMasses[g_driveIndex]);
    for (int i = 0; i < 5; ++i) {
        float m = kResonanceMasses[i], x = -8.0f + 4.0f * i;
        const Vec2 top{x, 13.0f};
        float eq = top.y - 3.0f - m * kG / kResonanceK;
        Body* b = addBody(w, Shape::box(0.25f + 0.12f * m, 0.25f + 0.12f * m), {x, eq}, m,
                          i == g_driveIndex ? 0xe76f51ff : 0x5e81acff);
        auto& s = g_springs->add(b, b->position, nullptr, top, kResonanceK, 0.4f);
        s.rest = 3.0f;
        s.drive = {0.0f, kDriveAmplitude};
        s.driveHz = driveHz;
        g_driven.push_back({b, eq, 0.0f});
    }
}

void resonanceUpdate(World&, float dt) {
    g_time += dt;
    for (auto& d : g_driven) d.amplitude = std::max(d.amplitude * (1.0f - 0.15f * dt), std::abs(d.mass->position.y - d.eqY));
}

bool resonanceClick(World& w, Vec2, int button) {
    if (button != 3) return false;
    g_driveIndex = (g_driveIndex + 1) % 5;
    w.clear();
    resonance(w);
    return true;
}

void resonanceReadout(const World&, Readout& r) {
    const float driveHz = g_springs->springs[0].driveHz;
    r.add("Resonance: the support shakes at %.2f Hz; the mass with that natural frequency builds up", driveHz);
    for (size_t i = 0; i < g_driven.size(); ++i) {
        float m = g_driven[i].mass->mass;
        r.add("m = %.2f kg   natural %.2f Hz   amplitude %.2f m%s", m, naturalHz(m), g_driven[i].amplitude,
              int(i) == g_driveIndex ? "   <- driven at resonance" : "");
    }
    r.add("Right click: drive at the next natural frequency");
    // the shaking support bar
    float y = 13.0f + kDriveAmplitude * std::sin(2.0f * kPi * driveHz * g_springs->time);
    r.line({-9.5f, y}, {9.5f, y}, 0x8a93a6ff);
}

// ---- Standing waves on a string ----

constexpr int kSegments = 40;
constexpr float kStringLength = 24.0f, kBeadMass = 0.05f, kStringK = 200.0f, kStretch = 0.1f;
int g_harmonic = 3;

float tension() { return kStringK * kStretch; }
float harmonicHz(int n) {
    // beaded string: w_n = 2 sqrt(T / (m a)) sin(n pi / 2N)
    const float a = kStringLength / kSegments;
    return 2.0f * std::sqrt(tension() / (kBeadMass * a)) * std::sin(n * kPi / (2.0f * kSegments)) / (2.0f * kPi);
}

void standingWaves(World& w) {
    g_time = 0.0f;
    w.settings.gravity = {};
    w.settings.linearDamping = 0.08f;
    g_springs = w.addField<SpringField>();
    const float a = kStringLength / kSegments, y = 7.0f, x0 = -kStringLength * 0.5f;
    Body* prev = nullptr;
    Vec2 prevPoint{x0, y};
    for (int i = 1; i <= kSegments; ++i) {
        Vec2 p{x0 + a * i, y};
        Body* bead = nullptr;
        if (i < kSegments) {
            BodyDef d;
            d.position = p;
            d.collides = false;
            d.color = 0xe9c46aff;
            d.density = kBeadMass / (kPi * 0.12f * 0.12f);
            bead = w.createBody(Shape::circle(0.12f), d);
        }
        // springs are pre-stretched so the string carries tension
        SpringField::Spring* s;
        if (!prev) {
            s = &g_springs->add(bead, p, nullptr, prevPoint, kStringK); // driven left end
            s->drive = {0.0f, 0.06f};
            s->driveHz = harmonicHz(g_harmonic);
        } else if (!bead) {
            s = &g_springs->add(prev, prevPoint, nullptr, p, kStringK); // fixed right end
        } else {
            s = &g_springs->add(prev, prevPoint, bead, p, kStringK);
        }
        s->rest = a - kStretch;
        s->coil = false;
        prev = bead;
        prevPoint = p;
    }
    addStaticBox(w, {x0 - 0.2f, y}, 0.2f, 1.0f)->collides = false;
    addStaticBox(w, {-x0 + 0.2f, y}, 0.2f, 1.0f)->collides = false;
}

void standingWavesUpdate(World&, float dt) { g_time += dt; }

bool standingWavesClick(World& w, Vec2, int button) {
    if (button != 3) return false;
    g_harmonic = g_harmonic % 6 + 1;
    w.clear();
    w.settings = {};
    standingWaves(w);
    return true;
}

void standingWavesReadout(const World&, Readout& r) {
    const float mu = kBeadMass / (kStringLength / kSegments), v = std::sqrt(tension() / mu);
    r.add("Standing waves: driving one end at a harmonic, f_n = n v / 2L, builds n loops");
    r.add("harmonic n = %d   f = %.2f Hz   wavelength %.1f m   wave speed v = sqrt(T / mu) = %.1f m/s", g_harmonic,
          harmonicHz(g_harmonic), 2.0f * kStringLength / g_harmonic, v);
    r.add("Right click: next harmonic   drag a bead to pluck the string");
    // nodes
    for (int k = 0; k <= g_harmonic; ++k) {
        float x = -kStringLength * 0.5f + kStringLength * k / g_harmonic;
        r.line({x, 7.0f - 0.25f}, {x, 7.0f + 0.25f}, 0x8a93a6ff);
    }
}

} // namespace

std::vector<Scene> oscillationScenes(int c) {
    return {
        {.name = "Springs & SHM", .category = c, .setup = springsShm, .update = springsShmUpdate,
         .readout = springsShmReadout, .cameraCenter = {0.0f, 8.5f}, .cameraScale = 36.0f},
        {.name = "Pendulum Wave", .category = c, .setup = pendulumWave, .update = pendulumWaveUpdate,
         .readout = pendulumWaveReadout, .cameraCenter = {0.0f, 9.0f}},
        {.name = "Coupled Pendulums", .category = c, .setup = coupledPendulums, .update = coupledUpdate,
         .readout = coupledReadout, .cameraCenter = {0.0f, 8.0f}, .cameraScale = 36.0f},
        {.name = "Resonance", .category = c, .setup = resonance, .update = resonanceUpdate, .click = resonanceClick,
         .readout = resonanceReadout, .cameraCenter = {0.0f, 8.5f}, .cameraScale = 32.0f},
        {.name = "Standing Waves", .category = c, .setup = standingWaves, .update = standingWavesUpdate,
         .click = standingWavesClick, .readout = standingWavesReadout},
    };
}
