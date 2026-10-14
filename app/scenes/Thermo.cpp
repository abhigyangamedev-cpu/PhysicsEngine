// Thermodynamics: kinetic theory with hard disks. Gas speeds relax to Maxwell-Boltzmann, a heavy
// grain jitters in Brownian motion, two gases diffuse, and a piston obeys the ideal gas law.
//
// The molecules are bodies that fly freely but skip the rigid contact solver: its soft contacts
// and sequential restitution slowly pump energy into a dense gas. Instead every frame resolves
// overlapping, approaching pairs with the exact two-body elastic impulse and reflects molecules
// off the walls, so kinetic energy is conserved exactly.
#include "SceneKit.h"
#include <algorithm>
#include <cmath>
#include <deque>

using namespace pe;
using namespace kit;

namespace {

float g_time = 0.0f;

struct HardDiskGas {
    std::vector<Body*> disks;
    Aabb box;
    bool divider = false; // a thin wall at x = 0
    Body* piston = nullptr; // a weighted lid: the box's top is the piston's underside
    float pistonHalfHeight = 0.0f;

    void collide() {
        const float top = piston ? piston->position.y - pistonHalfHeight : box.max.y;
        for (Body* b : disks) {
            const float r = b->shape.radius;
            Vec2& p = b->position;
            Vec2& v = b->velocity;
            if (p.x - r < box.min.x && v.x < 0.0f) v.x = -v.x, p.x = box.min.x + r;
            if (p.x + r > box.max.x && v.x > 0.0f) v.x = -v.x, p.x = box.max.x - r;
            if (p.y - r < box.min.y && v.y < 0.0f) v.y = -v.y, p.y = box.min.y + r;
            if (divider && std::abs(p.x) < r + 0.12f && p.x * v.x < 0.0f) v.x = -v.x;
            if (p.y + r > top) {
                if (!piston) {
                    if (v.y > 0.0f) v.y = -v.y;
                } else if (v.y > piston->velocity.y) {
                    // 1D elastic collision with the piston
                    float m = b->mass, M = piston->mass, u = v.y, U = piston->velocity.y;
                    v.y = ((m - M) * u + 2.0f * M * U) / (m + M);
                    piston->velocity.y = ((M - m) * U + 2.0f * m * u) / (m + M);
                }
                p.y = top - r;
            }
        }
        // a few hundred disks: checking every pair is cheap enough
        for (size_t i = 0; i < disks.size(); ++i) {
            Body* a = disks[i];
            for (size_t j = i + 1; j < disks.size(); ++j) {
                Body* b = disks[j];
                Vec2 d = b->position - a->position;
                float reach = a->shape.radius + b->shape.radius, distSq = d.lengthSq();
                if (distSq >= reach * reach || distSq < 1e-12f) continue;
                float dist = std::sqrt(distSq);
                Vec2 n = d / dist;
                float wa = a->invMass / (a->invMass + b->invMass);
                float overlap = reach - dist;
                a->position -= n * (overlap * wa);
                b->position += n * (overlap * (1.0f - wa));
                float vn = dot(b->velocity - a->velocity, n);
                if (vn >= 0.0f) continue; // already separating
                float impulse = 2.0f * vn / (a->invMass + b->invMass);
                a->velocity += n * (impulse * a->invMass);
                b->velocity -= n * (impulse * b->invMass);
            }
        }
        for (Body* b : disks) b->updateTransform();
    }

    // Average kinetic energy per molecule, which is kT for a 2D gas. The grain is excluded.
    float kT(size_t count) const {
        float ke = 0.0f;
        for (size_t i = 0; i < count; ++i) ke += 0.5f * disks[i]->mass * disks[i]->velocity.lengthSq();
        return count ? ke / count : 0.0f;
    }
};
HardDiskGas g_gas;

// A box from (-hw, 7 - hh) to (hw, 7 + hh), drawn with static bodies that the gas never touches.
void gasBox(World& w, float hw, float hh) {
    g_gas = {};
    g_gas.box = {{-hw, 7.0f - hh}, {hw, 7.0f + hh}};
    w.settings.gravity = {};
    for (auto [pos, half] : {std::pair{Vec2{0.0f, -hh - 0.25f}, Vec2{hw + 0.5f, 0.25f}}, std::pair{Vec2{0.0f, hh + 0.25f}, Vec2{hw + 0.5f, 0.25f}},
                             std::pair{Vec2{-hw - 0.25f, 0.0f}, Vec2{0.25f, hh}}, std::pair{Vec2{hw + 0.25f, 0.0f}, Vec2{0.25f, hh}}})
        addStaticBox(w, pos + Vec2{0.0f, 7.0f}, half.x, half.y);
}

Body* addMolecule(World& w, Vec2 pos, float radius, float mass, float speed, uint32_t color) {
    Body* b = addBody(w, Shape::circle(radius), pos, mass, color);
    float a = randf(0.0f, 2.0f * kPi);
    b->velocity = Vec2{std::cos(a), std::sin(a)} * speed;
    b->collides = false;
    g_gas.disks.push_back(b);
    return b;
}

void gasUpdate(World&, float dt) {
    g_time += dt;
    g_gas.collide();
}

// ---- Ideal gas: Maxwell-Boltzmann speeds ----

void idealGas(World& w) {
    g_time = 0.0f;
    gasBox(w, 10.0f, 5.0f);
    // every molecule starts at the same speed; collisions spread them out
    for (int i = 0; i < 360; ++i)
        addMolecule(w, {-9.6f + (i % 30) * 0.66f, 2.4f + (i / 30) * 0.8f}, 0.16f, 1.0f, 4.0f, 0x48aadcff);
}

void idealGasReadout(const World&, Readout& r) {
    const auto& gas = g_gas.disks;
    const float kT = g_gas.kT(gas.size()), m = 1.0f;
    const float sigma2 = kT / m; // 2D Maxwell-Boltzmann: f(v) = v / s^2 exp(-v^2 / 2 s^2)
    r.add("Ideal gas: every molecule starts at 4 m/s");
    r.add("and collisions spread the speeds into the");
    r.add("Maxwell-Boltzmann distribution,");
    r.add("f(v) = (m v / kT) exp(-m v^2 / 2 kT)");
    r.add("kT = <KE> = %.3f J (conserved)", kT);
    r.add("t = %.1f s", g_time);
    constexpr int kBins = 24;
    const float vmax = 3.5f * std::sqrt(sigma2), bin = vmax / kBins;
    r.histogram.assign(kBins, 0.0f);
    for (Body* b : gas) {
        int k = int(b->velocity.length() / bin);
        if (k < kBins) r.histogram[k] += 1.0f;
    }
    for (int k = 0; k < kBins; ++k) {
        float v = (k + 0.5f) * bin;
        r.histogramCurve.push_back(gas.size() * bin * v / sigma2 * std::exp(-v * v / (2.0f * sigma2)));
    }
    r.histogramLabel = "speed histogram (bars) vs Maxwell-Boltzmann (line)";
}

// ---- Brownian motion ----

Body* g_grain = nullptr;
std::deque<Vec2> g_path;

void brownian(World& w) {
    g_time = 0.0f;
    g_path.clear();
    gasBox(w, 10.0f, 5.0f);
    for (int i = 0; i < 420; ++i) {
        Vec2 p{-9.7f + (i % 35) * 0.57f, 2.4f + (i / 35) * 0.8f};
        if ((p - Vec2{0.0f, 7.0f}).length() < 1.3f) continue;
        addMolecule(w, p, 0.1f, 0.25f, 6.0f, 0x5e81acff);
    }
    g_grain = addMolecule(w, {0.0f, 7.0f}, 0.9f, 25.0f, 0.0f, 0xe9c46aff); // last, so kT() can skip it
}

void brownianUpdate(World& w, float dt) {
    gasUpdate(w, dt);
    if (g_path.empty() || (g_path.back() - g_grain->position).lengthSq() > 0.0025f) g_path.push_back(g_grain->position);
    if (g_path.size() > 1500) g_path.pop_front();
}

void brownianReadout(const World&, Readout& r) {
    const float kT = g_gas.kT(g_gas.disks.size() - 1);
    r.add("Brownian motion: a heavy grain is kicked at random by much lighter, fast molecules");
    r.add("equipartition: on average the grain's KE settles to the molecules' kT = %.2f J", kT);
    r.add("grain KE now %.2f J   (a molecule weighs 1/%.0f of the grain)", 0.5f * g_grain->mass * g_grain->velocity.lengthSq(),
          g_grain->mass / 0.25f);
    for (size_t i = 1; i < g_path.size(); ++i) r.line(g_path[i - 1], g_path[i], 0xf4a261ff);
}

// ---- Diffusion ----

Body* g_divider = nullptr;
std::vector<Body*> g_red, g_blue;

void diffusion(World& w) {
    g_time = 0.0f;
    g_red.clear();
    g_blue.clear();
    gasBox(w, 10.0f, 5.0f);
    g_gas.divider = true;
    g_divider = addStaticBox(w, {0.0f, 7.0f}, 0.12f, 5.0f);
    g_divider->collides = false;
    for (int i = 0; i < 180; ++i) {
        Vec2 p{-9.6f + (i % 15) * 0.63f, 2.4f + (i / 15) * 0.8f};
        g_red.push_back(addMolecule(w, p, 0.15f, 1.0f, 4.0f, 0xe76f51ff));
        g_blue.push_back(addMolecule(w, {-p.x, p.y}, 0.15f, 1.0f, 4.0f, 0x48aadcff));
    }
}

void diffusionUpdate(World& w, float dt) {
    gasUpdate(w, dt);
    if (g_divider && g_time > 1.5f) {
        w.destroyBody(g_divider);
        g_divider = nullptr;
        g_gas.divider = false;
    }
}

void diffusionReadout(const World&, Readout& r) {
    int redRight = 0, blueLeft = 0;
    for (Body* b : g_red) redRight += b->position.x > 0.0f;
    for (Body* b : g_blue) blueLeft += b->position.x < 0.0f;
    // each crossed fraction tends to 1/2 when fully mixed
    float mixed = 0.5f * (float(redRight) / g_red.size() + float(blueLeft) / g_blue.size());
    auto h = [](float p) { return p <= 0.0f || p >= 1.0f ? 0.0f : -(p * std::log(p) + (1 - p) * std::log(1 - p)); };
    r.add("Diffusion: %s", g_divider ? "the divider lifts at t = 1.5 s" : "random motion mixes the gases; they never unmix");
    r.add("red on the right %3d / %zu   blue on the left %3d / %zu", redRight, g_red.size(), blueLeft, g_blue.size());
    r.add("mixing %.0f%%   entropy of mixing %.2f of the maximum   t = %.1f s", 200.0f * std::min(mixed, 0.5f),
          h(mixed) / std::log(2.0f), g_time);
}

// ---- Piston: the ideal gas law ----

constexpr float kCylinderHalf = 4.0f, kPistonMass = 30.0f, kPistonHalf = 0.3f, kMoleculeR = 0.12f;
Body* g_piston = nullptr;
float g_avgHeight = 0.0f, g_avgKT = 0.0f; // the piston bobs, so the readout uses ~3 s running averages

void piston(World& w) {
    g_time = 0.0f;
    g_avgHeight = g_avgKT = 0.0f;
    g_gas = {};
    g_gas.box = {{-kCylinderHalf, 0.0f}, {kCylinderHalf, 1e6f}};
    w.settings.gravity = {}; // only the piston has weight, so the gas has no density gradient
    addStaticBox(w, {0.0f, -0.25f}, kCylinderHalf + 0.5f, 0.25f);
    addStaticBox(w, {-kCylinderHalf - 0.25f, 8.0f}, 0.25f, 8.0f);
    addStaticBox(w, {kCylinderHalf + 0.25f, 8.0f}, 0.25f, 8.0f);
    g_piston = addBody(w, Shape::box(kCylinderHalf - 0.02f, kPistonHalf), {0.0f, 10.0f}, kPistonMass, 0x8a93a6ff);
    g_piston->friction = 0.0f;
    g_gas.piston = g_piston;
    g_gas.pistonHalfHeight = kPistonHalf;
    for (int i = 0; i < 150; ++i)
        addMolecule(w, {-3.6f + (i % 10) * 0.8f, 0.5f + (i / 10) * 0.6f}, kMoleculeR, 1.0f, 6.0f, 0xe76f51ff);
}

void pistonUpdate(World& w, float dt) {
    gasUpdate(w, dt);
    // weight, plus a little friction in the seal so the piston settles instead of bouncing forever
    g_piston->applyForce({0.0f, -kPistonMass * kG - 2.0f * g_piston->velocity.y});
    g_piston->velocity.x = g_piston->angularVelocity = 0.0f;
    const float k = std::min(1.0f, dt / 3.0f), height = g_piston->position.y - kPistonHalf, kT = g_gas.kT(g_gas.disks.size());
    g_avgHeight = g_avgHeight == 0.0f ? height : g_avgHeight + (height - g_avgHeight) * k;
    g_avgKT = g_avgKT == 0.0f ? kT : g_avgKT + (kT - g_avgKT) * k;
}

bool pistonClick(World&, Vec2 p, int button) {
    if (button != 3) return false;
    const float scale = p.x > 0.0f ? 1.15f : 1.0f / 1.15f; // right half heats, left half cools
    for (Body* b : g_gas.disks) b->velocity *= scale;
    return true;
}

void pistonReadout(const World&, Readout& r) {
    const size_t N = g_gas.disks.size();
    const float kT = g_avgKT;
    const float width = 2.0f * kCylinderHalf, height = g_avgHeight;
    const float P = kPistonMass * kG / width, V = width * height;
    // hard disks take up room: Henderson's equation of state, Z = (1 + eta^2 / 8) / (1 - eta)^2
    const float eta = N * kPi * kMoleculeR * kMoleculeR / V, Z = (1.0f + eta * eta / 8.0f) / ((1.0f - eta) * (1.0f - eta));
    r.add("Ideal gas law (2D): P V = N k T. The weighted piston sets the pressure, so");
    r.add("heating the gas raises the piston (V grows with T)   right click: right half heats, left cools");
    if (kT <= 0.0f) return;
    r.add("N = %zu   kT = %.1f J   P = Mg / width = %.1f N/m   V = %.1f m^2   (3 s averages)", N, kT, P, V);
    r.add("P V / N k T = %.2f   (ideal gas 1; hard disks of this size %.2f)", P * V / (N * kT), Z);
}

} // namespace

std::vector<Scene> thermoScenes(int c) {
    return {
        {.name = "Ideal Gas", .category = c, .setup = idealGas, .update = gasUpdate, .readout = idealGasReadout,
         .cameraCenter = {-7.0f, 6.5f}, .cameraScale = 30.0f},
        {.name = "Brownian Motion", .category = c, .setup = brownian, .update = brownianUpdate, .readout = brownianReadout,
         .cameraCenter = {-5.0f, 6.5f}, .cameraScale = 30.0f},
        {.name = "Diffusion", .category = c, .setup = diffusion, .update = diffusionUpdate, .readout = diffusionReadout,
         .cameraCenter = {-5.0f, 6.5f}, .cameraScale = 30.0f},
        {.name = "Piston", .category = c, .setup = piston, .update = pistonUpdate, .click = pistonClick,
         .readout = pistonReadout, .cameraCenter = {0.0f, 7.5f}},
    };
}
