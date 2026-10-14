#include "Scenes.h"
#include "SceneKit.h"
#include "field/Fields.h"
#include "soft/Builders.h"
#include <cstdarg>
#include <cstdio>
#include <deque>
#include <unordered_map>

using namespace pe;
using kit::addAnchor;
using kit::addStaticBox;
using kit::randf;
using kit::rng;

void Readout::add(const char* fmt, ...) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buf, sizeof buf, fmt, args);
    va_end(args);
    text.emplace_back(buf);
}

namespace {

void addBounds(World& w) {
    addStaticBox(w, {0.0f, -0.5f}, 20.0f, 0.5f);
    addStaticBox(w, {-15.5f, 10.0f}, 0.5f, 10.0f);
    addStaticBox(w, {15.5f, 10.0f}, 0.5f, 10.0f);
}

void pyramid(World& w) {
    addStaticBox(w, {0.0f, -0.5f}, 20.0f, 0.5f);
    constexpr int rows = 14;
    constexpr float h = 0.5f;
    Shape box = Shape::box(h, h);
    for (int row = 0; row < rows; ++row) {
        int count = rows - row;
        float x0 = -count * h + h;
        for (int i = 0; i < count; ++i) {
            BodyDef d;
            d.position = {x0 + i * 2.0f * h * 1.02f, h + row * 2.0f * h};
            d.friction = 0.6f;
            d.restitution = 0.0f;
            w.createBody(box, d);
        }
    }
}

void shapeRain(World& w) {
    addBounds(w);
    addStaticBox(w, {-6.0f, 5.0f}, 4.0f, 0.2f, -0.35f);
    addStaticBox(w, {6.0f, 9.0f}, 4.0f, 0.2f, 0.35f);
    for (int i = 0; i < 120; ++i) {
        BodyDef d;
        d.position = {-13.0f + (i % 15) * 1.8f + randf(-0.2f, 0.2f), 12.0f + (i / 15) * 1.8f};
        d.angle = randf(0.0f, 6.28f);
        w.createBody(randomShape(randf(0.3f, 0.7f)), d);
    }
}

void frictionAndBounce(World& w) {
    addBounds(w);
    for (int i = 0; i < 5; ++i) {
        float y = 13.0f - i * 2.5f;
        addStaticBox(w, {-7.0f, y}, 6.0f, 0.15f, -0.3f);
        BodyDef d;
        d.position = {-11.0f, y + 1.6f};
        d.angle = -0.3f;
        d.friction = i * 0.1f;
        d.restitution = 0.0f;
        w.createBody(Shape::box(0.4f, 0.4f), d);
    }
    for (int i = 0; i < 6; ++i) {
        BodyDef d;
        d.position = {3.0f + i * 2.0f, 14.0f};
        d.restitution = i * 0.18f;
        w.createBody(Shape::circle(0.45f), d);
    }
}

void dominoes(World& w) {
    addStaticBox(w, {0.0f, -0.5f}, 20.0f, 0.5f);
    for (int i = 0; i < 25; ++i) {
        BodyDef d;
        d.position = {-12.0f + i * 1.0f, 1.0f};
        d.friction = 0.4f;
        d.restitution = 0.0f;
        w.createBody(Shape::box(0.1f, 1.0f), d);
    }
    BodyDef ball;
    ball.position = {-14.5f, 1.5f};
    ball.velocity = {4.0f, 0.0f};
    w.createBody(Shape::circle(0.4f), ball);
}

void pendulums(World& w) {
    addStaticBox(w, {0.0f, -0.5f}, 20.0f, 0.5f);

    // Newton's cradle
    constexpr float r = 0.5f;
    for (int i = 0; i < 5; ++i) {
        float x = -10.0f + i * (2.0f * r + 0.02f); // tiny gap keeps impacts pairwise
        Body* top = addAnchor(w, {x, 12.0f});
        BodyDef d;
        d.position = i == 0 ? Vec2{x - 5.0f, 12.0f} : Vec2{x, 7.0f};
        d.restitution = 1.0f;
        d.friction = 0.0f;
        Body* ball = w.createBody(Shape::circle(r), d);
        w.createJoint<DistanceJoint>(top, ball, top->position, ball->position)->length = 5.0f;
    }

    // double pendulum (chaotic)
    Body* pivot = addAnchor(w, {3.0f, 12.0f});
    BodyDef d;
    d.position = {6.0f, 12.0f};
    Body* m1 = w.createBody(Shape::circle(0.3f), d);
    d.position = {9.0f, 12.0f};
    Body* m2 = w.createBody(Shape::circle(0.3f), d);
    w.createJoint<DistanceJoint>(pivot, m1, pivot->position, m1->position);
    w.createJoint<DistanceJoint>(m1, m2, m1->position, m2->position);

    // spring-mass oscillators with different damping
    for (int i = 0; i < 3; ++i) {
        Body* top = addAnchor(w, {11.0f + i * 1.5f, 12.0f});
        BodyDef bd;
        bd.position = {11.0f + i * 1.5f, 7.0f};
        Body* box = w.createBody(Shape::box(0.4f, 0.4f), bd);
        auto* s = w.createJoint<DistanceJoint>(top, box, top->position, box->position);
        s->length = 3.0f;
        s->hertz = 1.0f;
        s->dampingRatio = i * 0.15f;
    }
}

void bridge(World& w) {
    addStaticBox(w, {0.0f, -0.5f}, 20.0f, 0.5f);
    Body* left = addStaticBox(w, {-12.0f, 3.0f}, 1.0f, 3.0f);
    Body* right = addStaticBox(w, {12.0f, 3.0f}, 1.0f, 3.0f);

    // planks follow a sagging curve so the chain has slack instead of being pulled taut
    constexpr int planks = 22;
    constexpr float sag = 1.2f;
    auto curve = [](float x) { return 5.85f - sag * (1.0f - (x / 11.0f) * (x / 11.0f)); };
    Body* prev = left;
    for (int i = 0; i < planks; ++i) {
        float x0 = -11.0f + 22.0f * i / planks, x1 = -11.0f + 22.0f * (i + 1) / planks;
        Vec2 p0{x0, curve(x0)}, p1{x1, curve(x1)};
        BodyDef d;
        d.position = (p0 + p1) * 0.5f;
        d.angle = std::atan2(p1.y - p0.y, p1.x - p0.x);
        d.friction = 0.6f;
        Body* plank = w.createBody(Shape::box((p1 - p0).length() * 0.5f, 0.15f), d);
        w.createJoint<RevoluteJoint>(prev, plank, p0);
        prev = plank;
    }
    w.createJoint<RevoluteJoint>(prev, right, Vec2{11.0f, curve(11.0f)});

    for (int i = 0; i < 12; ++i) {
        BodyDef d;
        d.position = {-8.0f + i * 1.4f, 9.0f + (i % 3)};
        w.createBody(randomShape(0.45f), d);
    }
}

void machines(World& w) {
    addBounds(w);

    // motorized windmill
    Body* hub = addAnchor(w, {-7.0f, 6.0f});
    BodyDef d;
    d.position = {-7.0f, 6.0f};
    d.density = 2.0f;
    Body* blade = w.createBody(Shape::box(4.0f, 0.2f), d);
    auto* motor = w.createJoint<RevoluteJoint>(hub, blade, hub->position);
    motor->enableMotor = true;
    motor->motorSpeed = 1.5f;
    motor->maxMotorTorque = 5000.0f;
    for (int i = 0; i < 40; ++i) {
        BodyDef bd;
        bd.position = {-12.0f + (i % 10) * 1.0f, 1.0f + (i / 10) * 1.0f};
        w.createBody(Shape::circle(0.35f), bd);
    }

    // spring trampoline
    Body* base1 = addAnchor(w, {4.0f, 0.5f});
    Body* base2 = addAnchor(w, {8.0f, 0.5f});
    d = {};
    d.position = {6.0f, 3.0f};
    Body* platform = w.createBody(Shape::box(2.5f, 0.2f), d);
    for (Body* base : {base1, base2}) {
        auto* s = w.createJoint<DistanceJoint>(base, platform, base->position, base->position + Vec2{0.0f, 2.5f});
        s->hertz = 2.0f;
        s->dampingRatio = 0.1f;
    }
    for (int i = 0; i < 4; ++i) {
        BodyDef bd;
        bd.position = {5.0f + i * 0.7f, 6.0f + i * 1.5f};
        w.createBody(Shape::box(0.3f, 0.3f), bd);
    }

    // rope swing
    Body* top = addAnchor(w, {11.0f, 14.0f});
    BodyDef bd;
    bd.position = {14.0f, 12.0f};
    Body* weight = w.createBody(Shape::box(0.5f, 0.5f), bd);
    auto* rope = w.createJoint<DistanceJoint>(top, weight, top->position, weight->position);
    rope->rope = true;
}

void cloth(World& w) {
    addBounds(w);
    ClothDef cd;
    cd.topLeft = {-6.0f, 14.0f};
    cd.columns = 40;
    cd.rows = 28;
    cd.spacing = 0.3f;
    cd.tearRatio = 1.8f;
    auto idx = makeCloth(w.particles, cd);
    for (int c = 0; c < cd.columns; c += 3) w.particles.particles[idx[c]].invMass = 0.0f;

    BodyDef d;
    d.position = {-1.0f, 18.0f};
    d.density = 3.0f;
    w.createBody(Shape::circle(1.0f), d);
    d.position = {3.0f, 22.0f};
    w.createBody(Shape::box(0.8f, 0.8f), d);
    addStaticBox(w, {9.5f, 3.0f}, 0.4f, 3.0f);
}

void softBodies(World& w) {
    addBounds(w);
    addStaticBox(w, {-7.0f, 4.0f}, 5.0f, 0.2f, -0.3f);

    for (int i = 0; i < 4; ++i) {
        BlobDef bd;
        bd.center = {-10.0f + i * 2.6f, 9.0f + i * 1.5f};
        bd.radius = 1.0f;
        bd.pressure = 1.0f + i * 0.15f;
        makeBlob(w.particles, bd);
    }
    for (int i = 0; i < 3; ++i) {
        JellyDef jd;
        jd.center = {3.0f + i * 3.0f, 3.0f + i * 3.0f};
        jd.width = jd.height = 2.0f;
        jd.compliance = 1e-5f * (1 + i * 10);
        makeJelly(w.particles, jd);
    }

    // particle rope carrying a rigid box
    Body* hook = addAnchor(w, {12.0f, 18.0f});
    auto rope = makeRope(w.particles, {12.0f, 18.0f}, {12.0f, 12.0f}, 20, 0.5f);
    w.particles.attach(rope.front(), hook);
    BodyDef d;
    d.position = {12.0f, 11.5f};
    Body* box = w.createBody(Shape::box(0.5f, 0.5f), d);
    w.particles.attach(rope.back(), box);
}

// ---- Gravity & Orbits ----

Body* addMass(World& w, Vec2 pos, Vec2 vel, float radius, float mass, bool collides = true) {
    BodyDef d;
    d.position = pos;
    d.velocity = vel;
    d.density = mass / (std::numbers::pi_v<float> * radius * radius);
    d.collides = collides;
    d.restitution = 0.2f;
    return w.createBody(Shape::circle(radius), d);
}

Vec2 circularVelocity(const Body& center, Vec2 pos, float G, bool clockwise = false) {
    Vec2 r = pos - center.position;
    float v = orbitalSpeed(G, center.mass, r.length());
    Vec2 dir = r.normalized().perp();
    return center.velocity + (clockwise ? -dir : dir) * v;
}

void zeroMomentum(World& w) {
    Vec2 p;
    float m = 0.0f;
    for (const auto& b : w.bodies()) {
        p += b->velocity * b->mass;
        m += b->mass;
    }
    if (m == 0.0f) return;
    for (const auto& b : w.bodies()) b->velocity -= p / m;
}

void solarSystem(World& w) {
    w.settings.gravity = {};
    w.particles.damping = 0.0f;
    auto* g = w.addField<NBodyGravity>();
    g->softening = 0.05f;
    g->theta = 0.5f;

    Body* sun = addMass(w, {}, {}, 1.2f, 2000.0f);
    struct Planet { float r, radius, mass; };
    const Planet planets[] = {{4.5f, 0.22f, 0.4f}, {7.5f, 0.35f, 2.0f}, {11.0f, 0.4f, 6.0f}, {15.0f, 0.3f, 1.0f},
                              {26.0f, 0.75f, 12.0f}};
    Body* earthLike = nullptr;
    for (const auto& pl : planets) {
        float a = randf(0.0f, 6.28f);
        Vec2 pos = Vec2{std::cos(a), std::sin(a)} * pl.r;
        Body* b = addMass(w, pos, circularVelocity(*sun, pos, g->G), pl.radius, pl.mass);
        if (pl.r == 11.0f) earthLike = b;
    }
    Vec2 moonPos = earthLike->position + Vec2{0.6f, 0.0f};
    addMass(w, moonPos, circularVelocity(*earthLike, moonPos, g->G), 0.12f, 0.02f);

    // asteroid belt between the 4th and 5th planet
    w.particles.selfCollision = false;
    const int group = w.particles.newGroup();
    for (int i = 0; i < 600; ++i) {
        float a = randf(0.0f, 6.28f), r = randf(19.0f, 21.0f);
        Vec2 pos = Vec2{std::cos(a), std::sin(a)} * r;
        uint32_t id = w.particles.addParticle(pos, 1e-4f, 0.04f, group);
        auto& p = w.particles.particles[id];
        p.velocity = circularVelocity(*sun, pos, g->G);
        p.color = 0xa89f91ff;
    }
    zeroMomentum(w);
}

void threeBody(World& w) {
    w.settings.gravity = {};
    auto* g = w.addField<NBodyGravity>();
    g->softening = 0.01f;
    g->theta = 0.0f;

    // Chenciner-Montgomery figure-eight choreography, scaled by L
    constexpr float L = 3.0f;
    const float vs = 1.0f / std::sqrt(L);
    const Vec2 x1{-0.97000436f, 0.24308753f}, v3{-0.93240737f, -0.86473146f};
    addMass(w, x1 * L, v3 * (-0.5f * vs), 0.12f, 1.0f, false);
    addMass(w, -x1 * L, v3 * (-0.5f * vs), 0.12f, 1.0f, false);
    addMass(w, {}, v3 * vs, 0.12f, 1.0f, false);
}

void addGalaxy(World& w, Vec2 center, Vec2 velocity, float coreMass, int stars, float radius, bool clockwise,
               uint32_t color, int group) {
    Body* core = addMass(w, center, velocity, 0.6f, coreMass, false);
    constexpr float starMass = 0.05f;
    for (int i = 0; i < stars; ++i) {
        float r = 2.0f + radius * std::pow(randf(0.0f, 1.0f), 0.8f);
        float a = randf(0.0f, 6.28f);
        Vec2 pos = center + Vec2{std::cos(a), std::sin(a)} * r;
        float enclosed = coreMass + starMass * stars * (r / (radius + 2.0f));
        float v = std::sqrt(enclosed / r);
        Vec2 dir = (pos - center).normalized().perp();
        uint32_t id = w.particles.addParticle(pos, starMass, 0.05f, group);
        auto& p = w.particles.particles[id];
        p.velocity = core->velocity + (clockwise ? -dir : dir) * v;
        p.color = color;
    }
}

void galaxies(World& w) {
    w.settings.gravity = {};
    w.settings.substeps = 2;
    w.settings.particleIterations = 1;
    w.particles.damping = 0.0f;
    w.particles.selfCollision = false;
    auto* g = w.addField<NBodyGravity>();
    g->softening = 0.5f;
    g->theta = 0.8f;
    const int group = w.particles.newGroup();
    addGalaxy(w, {-28.0f, -10.0f}, {2.4f, 0.6f}, 1000.0f, 1600, 14.0f, false, 0x7fb8ffff, group);
    addGalaxy(w, {28.0f, 10.0f}, {-2.4f, -0.6f}, 800.0f, 1300, 12.0f, true, 0xffb36bff, group);
}

void orbitSandbox(World& w) {
    w.settings.gravity = {};
    auto* g = w.addField<NBodyGravity>();
    g->softening = 0.05f;
    Body* sun = addMass(w, {}, {}, 1.5f, 3000.0f);
    for (float r : {6.0f, 10.0f, 15.0f}) {
        Vec2 pos{r, 0.0f};
        addMass(w, pos, circularVelocity(*sun, pos, g->G), 0.35f, 1.0f);
    }
}

bool orbitSandboxClick(World& w, Vec2 p, int button) {
    const Body* heaviest = nullptr;
    for (const auto& b : w.bodies())
        if (!heaviest || b->mass > heaviest->mass) heaviest = b.get();
    if (!heaviest || (p - heaviest->position).length() < 2.0f) return false;
    Vec2 v = circularVelocity(*heaviest, p, 1.0f);
    if (button == 3) v = heaviest->velocity + (v - heaviest->velocity) * 0.55f; // eccentric comet
    addMass(w, p, v, button == 3 ? 0.2f : randf(0.25f, 0.5f), button == 3 ? 0.05f : 1.0f);
    return true;
}

// ---- Electromagnetism ----

Body* addCharge(World& w, Vec2 pos, Vec2 vel, float charge, float radius, float mass, bool collides = true) {
    BodyDef d;
    d.position = pos;
    d.velocity = vel;
    d.charge = charge;
    d.collides = collides;
    d.density = mass / (std::numbers::pi_v<float> * radius * radius);
    d.restitution = 0.1f;
    d.friction = 0.2f;
    return w.createBody(Shape::circle(radius), d);
}

void addBox(World& w, float hw, float hh) {
    addStaticBox(w, {0.0f, -hh - 0.5f}, hw + 1.0f, 0.5f);
    addStaticBox(w, {0.0f, hh + 0.5f}, hw + 1.0f, 0.5f);
    addStaticBox(w, {-hw - 0.5f, 0.0f}, 0.5f, hh);
    addStaticBox(w, {hw + 0.5f, 0.0f}, 0.5f, hh);
}

void coulombCrystal(World& w) {
    w.settings.gravity = {};
    w.settings.linearDamping = 0.8f;
    auto* c = w.addField<CoulombField>();
    c->k = 3.0f;
    c->softening = 0.05f;
    addBox(w, 18.0f, 10.0f);
    for (int i = 0; i < 96; ++i) {
        Vec2 pos{-16.0f + (i % 16) * 2.1f + randf(-0.3f, 0.3f), -8.0f + (i / 16) * 3.0f + randf(-0.3f, 0.3f)};
        bool positive = (i + i / 16) % 2 == 0;
        addCharge(w, pos, {randf(-1.0f, 1.0f), randf(-1.0f, 1.0f)}, positive ? 1.0f : -1.0f,
                  positive ? 0.32f : 0.42f, 1.0f);
    }
}

void dipole(World& w) {
    w.settings.gravity = {};
    auto* c = w.addField<CoulombField>();
    c->k = 4.0f;
    c->softening = 0.2f;
    for (auto [x, q] : {std::pair{-4.0f, 3.0f}, std::pair{4.0f, -3.0f}}) {
        BodyDef d;
        d.position = {x, 0.0f};
        d.isStatic = true;
        d.draggable = true;
        d.charge = q;
        w.createBody(Shape::circle(0.5f), d);
    }
    for (int i = 0; i < 6; ++i) {
        float y = -5.0f + i * 2.0f;
        if (std::abs(y) < 0.5f) y = 6.0f;
        addCharge(w, {-12.0f, y}, {2.0f, 0.0f}, 0.15f, 0.12f, 0.05f);
    }
}

bool dipoleClick(World& w, Vec2 p, int button) {
    addCharge(w, p, {}, button == 3 ? -0.15f : 0.15f, 0.12f, 0.05f);
    return true;
}

std::unordered_map<uint32_t, float> g_birth;
std::deque<Body*> g_landed;
float g_clock = 0.0f, g_emitTimer = 0.0f;
int g_species = 0;

void spectrometer(World& w) {
    w.settings.gravity = {};
    g_birth.clear();
    g_landed.clear();
    g_clock = g_emitTimer = 0.0f;
    g_species = 0;

    auto* B = w.addField<UniformMagneticField>();
    B->B = 1.0f;
    B->region.box = {{0.0f, -20.0f}, {30.0f, 20.0f}};

    addStaticBox(w, {-8.5f, 0.0f}, 0.6f, 0.8f)->collides = false;  // ion source
    addStaticBox(w, {-0.3f, -8.0f}, 0.2f, 7.4f)->collides = false; // detector below the slit
    addStaticBox(w, {-0.3f, 5.6f}, 0.2f, 5.0f)->collides = false;  // detector above the slit
}

void spectrometerUpdate(World& w, float dt) {
    g_clock += dt;
    g_emitTimer += dt;
    struct Ion { float charge, mass; };
    // radius = m v / (|q| B), so the ion lands at y = -2r; equal q/m lands in the same spot
    static constexpr Ion species[] = {{1.0f, 0.25f}, {1.0f, 0.5f}, {1.0f, 1.0f}, {1.0f, 1.5f}, {2.0f, 1.0f}, {-1.0f, 0.5f}};
    if (g_emitTimer > 0.35f) {
        g_emitTimer = 0.0f;
        const Ion& ion = species[g_species++ % std::size(species)];
        Body* b = addCharge(w, {-7.5f, 0.0f}, {4.0f, 0.0f}, ion.charge, 0.1f + 0.08f * std::cbrt(ion.mass), ion.mass, false);
        g_birth[b->id] = g_clock;
    }

    std::vector<Body*> expired;
    for (const auto& b : w.bodies()) {
        auto it = g_birth.find(b->id);
        if (it == g_birth.end()) continue;
        if (b->position.x < -0.1f && b->velocity.x < 0.0f && g_clock - it->second > 2.0f) {
            b->velocity = {};
            g_birth.erase(it);
            g_landed.push_back(b.get());
        } else if (g_clock - it->second > 14.0f) {
            expired.push_back(b.get());
        }
    }
    while (g_landed.size() > 120) {
        expired.push_back(g_landed.front());
        g_landed.pop_front();
    }
    for (Body* b : expired) {
        g_birth.erase(b->id);
        w.destroyBody(b);
    }
}

void exbDrift(World& w) {
    w.settings.gravity = {};
    auto* B = w.addField<UniformMagneticField>();
    B->B = 1.5f;
    auto* E = w.addField<UniformElectricField>();
    E->E = {0.0f, -1.5f};
    // drift velocity E x B / B^2 is the same for every charge and mass
    struct P { float y, charge, mass; };
    for (const P& p : {P{6.0f, 1.0f, 1.0f}, P{3.0f, 1.0f, 3.0f}, P{0.0f, -1.0f, 1.0f}, P{-3.0f, 2.0f, 1.0f},
                       P{-6.0f, -1.0f, 2.5f}})
        addCharge(w, {12.0f, p.y}, {}, p.charge, 0.15f, p.mass, false);
}

void exbDriftUpdate(World& w, float) {
    for (const auto& b : w.bodies()) {
        if (b->position.x < -20.0f) b->position.x += 40.0f;
        if (b->position.x > 20.0f) b->position.x -= 40.0f;
    }
}

// ---- Fluids ----

void tank(World& w, float halfWidth, float height, bool lid = false) {
    addStaticBox(w, {0.0f, -0.5f}, halfWidth + 1.0f, 0.5f);
    addStaticBox(w, {-halfWidth - 0.5f, height * 0.5f}, 0.5f, height * 0.5f);
    addStaticBox(w, {halfWidth + 0.5f, height * 0.5f}, 0.5f, height * 0.5f);
    if (lid) addStaticBox(w, {0.0f, height + 0.5f}, halfWidth + 1.0f, 0.5f);
    // anything that still splashes out falls past the floor and is dropped
    w.fluid.bounds = {{-halfWidth - 10.0f, -3.0f}, {halfWidth + 10.0f, height + 40.0f}};
}

Body* floater(World& w, Vec2 pos, Shape shape, float density, float angle = 0.0f) {
    BodyDef d;
    d.position = pos;
    d.angle = angle;
    d.density = density;
    d.friction = 0.3f;
    return w.createBody(shape, d);
}

void damBreak(World& w) {
    // closed like the classic dam-break experiment: the jet off the obstacle reaches well past 15 m
    tank(w, 16.0f, 15.0f, true);
    w.fluid.configure(0.25f);
    w.fluid.addBlock({-16.0f, 0.0f}, {-6.0f, 12.0f});
    addStaticBox(w, {4.0f, 1.0f}, 0.4f, 1.0f);
    addStaticBox(w, {10.0f, 2.0f}, 2.5f, 0.15f, 0.5f);
    floater(w, {7.0f, 0.6f}, Shape::box(0.6f, 0.6f), 0.6f);
}

void buoyancy(World& w) {
    tank(w, 16.0f, 12.0f);
    w.fluid.configure(0.25f);
    w.fluid.addBlock({-16.0f, 0.0f}, {16.0f, 4.5f});
    floater(w, {-12.0f, 7.0f}, Shape::box(0.8f, 0.8f), 0.25f);
    floater(w, {-7.0f, 7.0f}, Shape::box(0.8f, 0.8f), 0.5f);
    floater(w, {-2.0f, 7.0f}, Shape::box(0.8f, 0.8f), 0.85f);
    floater(w, {3.0f, 7.0f}, Shape::circle(0.7f), 2.5f);
    floater(w, {9.0f, 7.0f}, Shape::box(3.0f, 0.3f), 0.4f);
    floater(w, {13.5f, 8.0f}, Shape::polygon({{-1.0f, 0.6f}, {1.0f, 0.6f}, {0.6f, -0.4f}, {-0.6f, -0.4f}}), 0.3f);
}

bool buoyancyClick(World& w, Vec2 p, int button) {
    if (button != 3) return false;
    floater(w, p, randomShape(randf(0.4f, 0.8f)), randf(0.2f, 1.5f), randf(0.0f, 3.0f));
    return true;
}

float g_emit = 0.0f;

void waterWheel(World& w) {
    g_emit = 0.0f;
    // basin with a drain in the middle of the floor
    addStaticBox(w, {-9.0f, -0.5f}, 8.0f, 0.5f);
    addStaticBox(w, {9.0f, -0.5f}, 8.0f, 0.5f);
    addStaticBox(w, {-17.5f, 6.0f}, 0.5f, 7.0f);
    addStaticBox(w, {17.5f, 6.0f}, 0.5f, 7.0f);
    addStaticBox(w, {-9.0f, 11.0f}, 3.5f, 0.15f, -0.25f); // chute

    w.fluid.configure(0.25f);
    w.fluid.bounds = {{-30.0f, -3.0f}, {30.0f, 40.0f}}; // water through the drain is dropped
    w.fluid.addBlock({-17.0f, 0.0f}, {-11.0f, 3.0f});

    // six paddles pinned to an axle, tied at their tips so they turn as one wheel
    const Vec2 axle{-2.0f, 6.0f};
    Body* hub = addAnchor(w, axle);
    hub->collides = false;
    constexpr int kPaddles = 6;
    std::vector<Body*> paddles;
    for (int i = 0; i < kPaddles; ++i) {
        float a = 2.0f * std::numbers::pi_v<float> * i / kPaddles;
        Vec2 dir{std::cos(a), std::sin(a)};
        BodyDef d;
        d.position = axle + dir * 2.0f;
        d.angle = a;
        d.density = 0.6f;
        d.group = 1;
        Body* paddle = w.createBody(Shape::box(1.5f, 0.12f), d);
        w.createJoint<RevoluteJoint>(hub, paddle, axle);
        paddles.push_back(paddle);
    }
    for (int i = 0; i < kPaddles; ++i) {
        Body* a = paddles[i];
        Body* b = paddles[(i + 1) % kPaddles];
        w.createJoint<DistanceJoint>(a, b, a->position + a->rot.apply({1.5f, 0.0f}), b->position + b->rot.apply({1.5f, 0.0f}));
        w.createJoint<DistanceJoint>(a, b, a->position + a->rot.apply({-1.0f, 0.0f}), b->position + b->rot.apply({-1.0f, 0.0f}));
    }
}

void waterWheelUpdate(World& w, float dt) {
    g_emit += dt;
    if (g_emit > 0.05f && w.fluid.size() < 2600) {
        g_emit = 0.0f;
        for (int i = 0; i < 4; ++i)
            w.fluid.addParticle({-12.0f, 12.5f + i * 0.25f}, {5.0f, -1.0f});
    }
}

void viscosity(World& w) {
    addStaticBox(w, {0.0f, -0.5f}, 17.0f, 0.5f);
    for (float x : {-16.5f, -5.5f, 5.5f, 16.5f}) addStaticBox(w, {x, 4.5f}, 0.25f, 4.5f);
    // even the thickest fluid splashes past the walls off its dam, so the lanes are closed to keep them apart
    addStaticBox(w, {0.0f, 9.25f}, 16.75f, 0.25f);
    w.fluid.configure(0.22f);
    w.fluid.bounds = {{-27.0f, -3.0f}, {27.0f, 50.0f}};
    struct Lane { float x0, visc; uint32_t color; };
    for (const Lane& l : {Lane{-16.25f, 0.01f, 0x3c8ce7ff}, Lane{-5.25f, 0.12f, 0xd8c14aff}, Lane{5.75f, 0.45f, 0xd9822bff}}) {
        w.fluid.addBlock({l.x0, 0.0f}, {l.x0 + 3.5f, 7.0f}, l.visc, l.color);
        addStaticBox(w, {l.x0 + 6.0f, 1.0f}, 0.15f, 1.0f);
    }
}

} // namespace

Shape randomShape(float size) {
    switch (std::uniform_int_distribution<int>(0, 3)(rng())) {
    case 0: return Shape::circle(size);
    case 1: return Shape::box(size, size * randf(0.5f, 1.2f));
    default: {
        int n = std::uniform_int_distribution<int>(3, 7)(rng());
        std::vector<Vec2> pts;
        for (int i = 0; i < n; ++i) {
            float a = 2.0f * std::numbers::pi_v<float> * (i + randf(-0.2f, 0.2f)) / n;
            pts.push_back(Vec2{std::cos(a), std::sin(a)} * size * randf(0.8f, 1.2f));
        }
        return Shape::polygon(std::move(pts));
    }
    }
}

const std::vector<Category>& categories() {
    static const std::vector<Category> list = {
        {"Rigid Bodies", "Stacking, collisions, friction and bounce", nullptr},
        {"Joints & Springs", "Pendulums, bridges, motors and ropes", nullptr},
        {"Soft Bodies & Cloth", "Tearable cloth, pressure blobs and jelly", nullptr},
        {"Gravity & Orbits", "N-body gravity, orbits and colliding galaxies", nullptr},
        {"Electromagnetism", "Charges in electric and magnetic fields", nullptr},
        {"Fluids", "Liquids, splashes, buoyancy and viscosity", nullptr},
        {"Pulleys & Machines", "Atwood machines, block and tackle, levers and gears", nullptr},
        {"Oscillations & Waves", "Springs, pendulums, resonance and standing waves", nullptr},
        {"Collisions & Momentum", "Elastic and inelastic impacts, recoil and billiards", nullptr},
        {"Projectiles", "Launch angles, air resistance and the monkey and hunter", nullptr},
        {"Rotation & Rolling", "Rolling races, spin, slip and angular momentum", nullptr},
        {"Thermodynamics", "Kinetic theory: gas speeds, Brownian motion and diffusion", nullptr},
        {"Optics", "Ray optics: refraction, prisms, lenses and mirrors", nullptr},
    };
    return list;
}

static std::vector<Scene> coreScenes() {
    return {
        {.name = "Pyramid", .category = 0, .setup = pyramid},
        {.name = "Shape Rain", .category = 0, .setup = shapeRain},
        {.name = "Friction & Bounce", .category = 0, .setup = frictionAndBounce},
        {.name = "Dominoes", .category = 0, .setup = dominoes},
        {.name = "Pendulums & Springs", .category = 1, .setup = pendulums},
        {.name = "Bridge", .category = 1, .setup = bridge},
        {.name = "Machines", .category = 1, .setup = machines},
        {.name = "Cloth", .category = 2, .setup = cloth},
        {.name = "Soft Bodies", .category = 2, .setup = softBodies},
        {.name = "Solar System", .category = 3, .setup = solarSystem, .cameraCenter = {}, .cameraScale = 14.0f,
         .trails = true},
        {.name = "Three-Body", .category = 3, .setup = threeBody, .cameraCenter = {}, .cameraScale = 60.0f,
         .trails = true},
        {.name = "Galaxy Collision", .category = 3, .setup = galaxies, .cameraCenter = {}, .cameraScale = 9.0f},
        {.name = "Orbit Sandbox", .category = 3, .setup = orbitSandbox, .click = orbitSandboxClick,
         .cameraCenter = {}, .cameraScale = 18.0f, .trails = true},
        {.name = "Coulomb Crystal", .category = 4, .setup = coulombCrystal, .cameraCenter = {}, .cameraScale = 27.0f},
        {.name = "Dipole Field", .category = 4, .setup = dipole, .click = dipoleClick, .cameraCenter = {},
         .cameraScale = 40.0f,
         .trails = true, .fieldLines = true},
        {.name = "Mass Spectrometer", .category = 4, .setup = spectrometer, .update = spectrometerUpdate,
         .cameraCenter = {5.0f, -3.0f}, .cameraScale = 32.0f, .trails = true},
        {.name = "ExB Drift", .category = 4, .setup = exbDrift, .update = exbDriftUpdate, .cameraCenter = {},
         .cameraScale = 34.0f, .trails = true},
        {.name = "Dam Break", .category = 5, .setup = damBreak, .cameraCenter = {0.0f, 7.0f}, .cameraScale = 30.0f,
         .stir = true},
        {.name = "Buoyancy", .category = 5, .setup = buoyancy, .click = buoyancyClick, .cameraCenter = {0.0f, 6.0f},
         .cameraScale = 30.0f, .stir = true},
        {.name = "Water Wheel", .category = 5, .setup = waterWheel, .update = waterWheelUpdate,
         .cameraCenter = {0.0f, 7.0f}, .cameraScale = 30.0f, .stir = true},
        {.name = "Viscosity", .category = 5, .setup = viscosity, .cameraCenter = {0.0f, 5.0f}, .cameraScale = 30.0f,
         .stir = true},
    };
}

const std::vector<Scene>& scenes() {
    static const std::vector<Scene> list = [] {
        std::vector<Scene> all = coreScenes();
        // categories 6+ live in app/scenes/, in the same order as categories()
        int category = 6;
        for (auto make : {machineScenes, oscillationScenes, collisionScenes, projectileScenes, rotationScenes,
                          thermoScenes, opticsScenes}) {
            auto more = make(category++);
            all.insert(all.end(), more.begin(), more.end());
        }
        return all;
    }();
    return list;
}

std::vector<int> scenesIn(int category) {
    std::vector<int> out;
    for (int i = 0; i < static_cast<int>(scenes().size()); ++i)
        if (scenes()[i].category == category) out.push_back(i);
    return out;
}
