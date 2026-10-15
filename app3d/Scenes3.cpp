#include "Scenes3.h"
#include <algorithm>
#include <cmath>
#include <numbers>
#include <random>

using namespace pe3;

namespace scenes3 {

namespace {

constexpr float kPi = std::numbers::pi_v<float>;

Body* box(World& w, Vec3 pos, Vec3 half, uint32_t color, Quat q = {}) {
    BodyDef d;
    d.position = pos;
    d.orientation = q;
    d.color = color;
    return w.createBody(Shape::box(half), d);
}

Body* ball(World& w, Vec3 pos, float r, uint32_t color, Vec3 vel = {}) {
    BodyDef d;
    d.position = pos;
    d.velocity = vel;
    d.color = color;
    d.rollingResistance = 0.05f;
    return w.createBody(Shape::sphere(r), d);
}

void stack(World& w) {
    addGround(w);
    for (int i = 0; i < 10; ++i) box(w, {0, 0.5f + i * 1.0f, 0}, {0.5f, 0.5f, 0.5f}, i % 2 ? 0xe07a5fff : 0x3d85c6ff);
}

void pyramid(World& w) {
    addGround(w);
    const int base = 10;
    const float s = 0.5f;
    for (int row = 0; row < base; ++row)
        for (int i = 0; i < base - row; ++i)
            box(w, {(i - (base - row - 1) * 0.5f) * 2 * s, s + row * 2 * s, 0}, {s, s, s}, 0xf2cc8fff);
}

void wall(World& w) {
    addGround(w);
    for (int row = 0; row < 8; ++row)
        for (int i = 0; i < 8; ++i)
            box(w, {(i - 3.5f + (row % 2) * 0.5f) * 1.0f, 0.25f + row * 0.5f, 0}, {0.5f, 0.25f, 0.25f},
                row % 2 ? 0xc0504dff : 0xd9822bff);
    ball(w, {0, 1.5f, 12}, 0.7f, 0x444444ff, {0, 2, -18});
}

void dominoes(World& w) {
    addGround(w);
    const int n = 40;
    const float R = 6.0f;
    Body* last = nullptr;
    float lastA = 0.0f;
    for (int i = 0; i < n; ++i) {
        float a = 2 * kPi * i / n;
        last = box(w, {R * std::cos(a), 0.6f, R * std::sin(a)}, {0.05f, 0.6f, 0.3f}, 0xeeeeeeff,
                   Quat::axisAngle({0, 1, 0}, -a - kPi / 2));
        lastA = a;
    }
    last->angularVelocity = Vec3{std::cos(lastA), 0, std::sin(lastA)} * 2.0f;
}

void walls(World& w, float half, float height, uint32_t color) {
    for (int i = 0; i < 4; ++i) {
        float a = i * kPi / 2;
        BodyDef d;
        d.isStatic = true;
        d.position = {half * std::cos(a), height, half * std::sin(a)};
        d.orientation = Quat::axisAngle({0, 1, 0}, -a);
        d.color = color;
        w.createBody(Shape::box({0.2f, height, half + 0.2f}), d);
    }
}

void ballPit(World& w) {
    addGround(w);
    walls(w, 6, 1, 0x888888ff);
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> u(-4.5f, 4.5f);
    const uint32_t colors[] = {0xe63946ff, 0xf1c453ff, 0x2a9d8fff, 0x457b9dff, 0xf4a261ff};
    for (int i = 0; i < 300; ++i) ball(w, {u(rng), 2.0f + i * 0.08f, u(rng)}, 0.35f, colors[i % 5]);
}

void rain(World& w) {
    addGround(w);
    walls(w, 8, 1.5f, 0x666b75ff);
    std::mt19937 rng(3);
    std::uniform_real_distribution<float> u(-3.0f, 3.0f), s(0.25f, 0.6f), ang(0.0f, 2 * kPi);
    for (int i = 0; i < 150; ++i) {
        Vec3 p{u(rng), 2.0f + i * 0.15f, u(rng)};
        Quat q = Quat::axisAngle(Vec3{u(rng), u(rng), u(rng)}.normalized() + Vec3{0, 1e-3f, 0}, ang(rng));
        if (i % 3 == 0)
            ball(w, p, s(rng), 0x6a994eff);
        else
            box(w, p, {s(rng), s(rng), s(rng)}, i % 2 ? 0x9b5de5ff : 0x00bbf9ff, q);
    }
}

void billiards(World& w) {
    addGround(w);
    walls(w, 7, 0.4f, 0x2d6a4fff);
    for (auto& b : w.bodies()) b->restitution = 0.8f;
    w.settings.restitutionThreshold = 0.2f;
    const float r = 0.3f;
    int k = 0;
    for (int row = 0; row < 5; ++row)
        for (int i = 0; i <= row; ++i) {
            auto* b = ball(w, {(i - row * 0.5f) * 2.02f * r, r, -row * 1.75f * r}, r, k++ % 2 ? 0xd62828ff : 0xfcbf49ff);
            b->restitution = 0.9f;
            b->friction = 0.2f;
        }
    auto* cue = ball(w, {0, r, 5}, r, 0xffffffff, {0, 0, -14});
    cue->restitution = 0.9f;
    cue->friction = 0.2f;
}


Body* fixedBox(World& w, Vec3 pos, Vec3 half, uint32_t color, Quat q = {}) {
    BodyDef d;
    d.isStatic = true;
    d.position = pos;
    d.orientation = q;
    d.color = color;
    return w.createBody(Shape::box(half), d);
}

Body* rod(World& w, Body* a, Body* b, Vec3 pa, Vec3 pb) {
    w.createJoint<DistanceJoint>(a, b, pa, pb);
    return b;
}

// JEE classics side by side: simple, conical, double and elastic pendulums
void pendulums(World& w) {
    addGround(w);
    w.settings.substeps = 12; // joints lose less swing energy with smaller substeps
    w.settings.angularDamping = 0.0f;
    const float top = 6.0f;
    fixedBox(w, {0, top + 0.4f, 0}, {8.0f, 0.1f, 0.3f}, 0x5c6370ff);
    for (float x : {-6.0f, -2.0f, 2.0f, 6.0f}) fixedBox(w, {x, top + 0.2f, 0}, {0.04f, 0.2f, 0.04f}, 0x5c6370ff);

    const float L = 3.0f, simple = 0.5f;
    Vec3 p{-6, top, 0};
    rod(w, nullptr, ball(w, p + Vec3{L * std::sin(simple), -L * std::cos(simple), 0}, 0.18f, 0xe63946ff), p,
        p + Vec3{L * std::sin(simple), -L * std::cos(simple), 0});

    // conical: v^2 = g r tan(theta) keeps the bob on a horizontal circle
    const float cone = 0.6f, r = L * std::sin(cone);
    p = {-2, top, 0};
    Body* c = ball(w, p + Vec3{r, -L * std::cos(cone), 0}, 0.18f, 0xf1c453ff,
                   {0, 0, std::sqrt(9.81f * r * std::tan(cone))});
    rod(w, nullptr, c, p, c->position);

    p = {2, top, 0};
    Body* d1 = ball(w, p + Vec3{1.5f, 0, 0}, 0.16f, 0x2a9d8fff);
    Body* d2 = ball(w, p + Vec3{3.0f, 0, 0}, 0.16f, 0x2a9d8fff);
    rod(w, nullptr, d1, p, d1->position);
    rod(w, d1, d2, d1->position, d2->position);
    d1->group = d2->group = 1;

    p = {6, top, 0};
    Body* s = ball(w, p + Vec3{1.2f, -2.2f, 0.4f}, 0.2f, 0x457b9dff);
    auto* spring = w.createJoint<DistanceJoint>(nullptr, s, p, s->position);
    spring->length = 2.0f;
    spring->hertz = 1.2f;
    spring->dampingRatio = 0.01f;
    for (auto& b : w.bodies()) b->rollingResistance = 0.0f;
}

void newtonsCradle(World& w) {
    addGround(w);
    w.settings.substeps = 12;
    w.settings.angularDamping = 0.0f;
    w.settings.restitutionThreshold = 0.05f;
    const float r = 0.25f, top = 4.0f, L = 2.5f, spread = 0.7f, gap = 0.03f;
    fixedBox(w, {0, top, spread}, {2.0f, 0.04f, 0.04f}, 0x8d99aeff);
    fixedBox(w, {0, top, -spread}, {2.0f, 0.04f, 0.04f}, 0x8d99aeff);
    for (int i = 0; i < 5; ++i) {
        const float x = (i - 2) * (2 * r + gap);
        const float a = i == 0 ? -0.7f : 0.0f; // the first ball is pulled back
        const float drop = std::sqrt(L * L - spread * spread);
        BodyDef d;
        d.position = {x + drop * std::sin(a), top - drop * std::cos(a), 0};
        d.restitution = 1.0f;
        d.friction = 0.0f;
        d.color = 0xc0c5ceff;
        Body* b = w.createBody(Shape::sphere(r), d);
        for (float z : {spread, -spread}) w.createJoint<DistanceJoint>(nullptr, b, Vec3{x, top, z}, b->position);
    }
}

void wreckingBall(World& w) {
    addGround(w);
    for (int y = 0; y < 8; ++y)
        for (int x = 0; x < 3; ++x)
            for (int z = 0; z < 3; ++z)
                box(w, {(x - 1) * 1.0f, 0.5f + y * 1.0f, (z - 1) * 1.0f}, {0.5f, 0.5f, 0.5f}, (x + y + z) % 2 ? 0xd9822bff : 0xc0504dff);
    const Vec3 pivot{-1.6f, 10.0f, 0}; // lowest point lands the ball on the tower's base
    const float L = 7.5f, a = 1.2f;
    BodyDef d;
    d.position = pivot + Vec3{-L * std::sin(a), -L * std::cos(a), 0};
    d.density = 6.0f;
    d.color = 0x30343bff;
    Body* ball = w.createBody(Shape::sphere(0.8f), d);
    w.createJoint<DistanceJoint>(nullptr, ball, pivot, ball->position)->rope = true;
}

// hinged planks hanging between two posts, with crates dropped on top
void ropeBridge(World& w) {
    addGround(w);
    const float half = 5.0f, height = 4.0f, sag = 1.0f;
    for (float x : {-half, half}) fixedBox(w, {x + (x > 0 ? 0.3f : -0.3f), height / 2, 0}, {0.3f, height / 2, 0.8f}, 0x6b4f3aff);

    auto curve = [&](float x) { return Vec3{x, height - sag * (1.0f - (x / half) * (x / half)), 0}; };
    std::vector<Vec3> samples;
    for (int i = 0; i <= 2000; ++i) samples.push_back(curve(-half + 2 * half * i / 2000.0f));
    std::vector<float> arc{0.0f};
    for (size_t i = 1; i < samples.size(); ++i) arc.push_back(arc.back() + (samples[i] - samples[i - 1]).length());
    const int planks = 16;
    std::vector<Vec3> links;
    for (int k = 0, s = 0; k <= planks; ++k) {
        const float target = arc.back() * k / planks;
        while (s + 1 < static_cast<int>(arc.size()) - 1 && arc[s + 1] < target) ++s;
        links.push_back(samples[s]);
    }
    links.front() = curve(-half);
    links.back() = curve(half);

    // a little friction in each hinge, like rope rubbing on wood, so the sway dies out
    auto hinge = [&](Body* a, Body* b, Vec3 at) {
        auto* h = w.createJoint<HingeJoint>(a, b, at, Vec3{0, 0, 1});
        h->enableMotor = true, h->motorSpeed = 0.0f, h->maxMotorTorque = 0.3f;
    };
    Body* prev = nullptr;
    for (int k = 0; k < planks; ++k) {
        const Vec3 a = links[k], b = links[k + 1], d = b - a;
        BodyDef def;
        def.position = (a + b) * 0.5f;
        def.orientation = Quat::axisAngle({0, 0, 1}, std::atan2(d.y, d.x));
        def.density = 0.6f;
        def.group = 7;
        def.color = k % 2 ? 0xa47148ff : 0x8d5b36ff;
        Body* plank = w.createBody(Shape::box({d.length() * 0.5f - 0.02f, 0.05f, 0.7f}), def);
        hinge(prev, plank, a);
        prev = plank;
    }
    hinge(prev, nullptr, links.back());

    for (int i = 0; i < 6; ++i) box(w, {-3.0f + i * 1.2f, 6.0f + i * 0.8f, 0}, {0.3f, 0.3f, 0.3f}, 0x3d85c6ff);
    ball(w, {0, 9, 0}, 0.4f, 0xe07a5fff);
}

// three motor-driven paddles stirring a bin of balls
void paddleWheel(World& w) {
    addGround(w);
    const float hx = 3.6f, hz = 1.5f, wall = 1.6f;
    fixedBox(w, {-hx - 0.2f, wall, 0}, {0.2f, wall, hz + 0.4f}, 0x888888ff);
    fixedBox(w, {hx + 0.2f, wall, 0}, {0.2f, wall, hz + 0.4f}, 0x888888ff);
    fixedBox(w, {0, wall, -hz - 0.2f}, {hx, wall, 0.2f}, 0x888888ff);
    fixedBox(w, {0, wall, hz + 0.2f}, {hx, wall, 0.2f}, 0x666b75ff);
    const Vec3 hub{0, 1.8f, 0};
    for (int i = 0; i < 3; ++i) {
        BodyDef d;
        d.position = hub;
        d.orientation = Quat::axisAngle({0, 0, 1}, i * kPi / 3);
        d.group = 3;
        d.density = 2.0f;
        d.color = 0xf4a261ff;
        Body* paddle = w.createBody(Shape::box({1.6f, 0.06f, hz - 0.1f}), d);
        auto* h = w.createJoint<HingeJoint>(nullptr, paddle, hub, Vec3{0, 0, 1});
        h->enableMotor = true;
        h->motorSpeed = -1.5f;
        h->maxMotorTorque = 2000.0f;
    }
    std::mt19937 rng(11);
    std::uniform_real_distribution<float> ux(-hx + 0.3f, hx - 0.3f), uz(-hz + 0.3f, hz - 0.3f);
    const uint32_t colors[] = {0xe63946ff, 0x2a9d8fff, 0x457b9dff, 0xf1c453ff};
    for (int i = 0; i < 110; ++i) ball(w, {ux(rng), 3.5f + i * 0.06f, uz(rng)}, 0.22f, colors[i % 4]);
}

// joints get a little friction, like real limbs, so a light forearm doesn't swing forever.
// Parts of one figure share a group and never collide, so the neck and hips carry swing cones:
// without them the head could fold straight into the chest and the thighs up through the torso.
void ragdoll(World& w, Vec3 p, int group, uint32_t color) {
    auto part = [&](Vec3 offset, Shape s) {
        BodyDef d;
        d.position = p + offset;
        d.group = group;
        d.color = color;
        return w.createBody(s, d);
    };
    auto ballJoint = [&](Body* a, Body* b, Vec3 at, float friction) {
        auto* j = w.createJoint<BallJoint>(a, b, p + at);
        j->maxFrictionTorque = friction;
        return j;
    };
    auto hinge = [&](Body* a, Body* b, Vec3 at, float lower, float upper, float friction) {
        auto* h = w.createJoint<HingeJoint>(a, b, p + at, Vec3{1, 0, 0});
        h->enableLimit = true, h->lowerAngle = lower, h->upperAngle = upper;
        h->enableMotor = true, h->motorSpeed = 0.0f, h->maxMotorTorque = friction;
    };
    Body* pelvis = part({0, 0, 0}, Shape::box({0.18f, 0.1f, 0.12f}));
    Body* torso = part({0, 0.34f, 0}, Shape::box({0.2f, 0.22f, 0.12f}));
    Body* head = part({0, 0.72f, 0}, Shape::sphere(0.13f));
    head->color = 0xf2cc8fff;
    head->rollingResistance = 0.1f;
    hinge(pelvis, torso, {0, 0.11f, 0}, -0.3f, 0.8f, 0.05f);
    ballJoint(torso, head, {0, 0.58f, 0}, 0.02f)->limitSwing({0, 1, 0}, 0.6f); // ~35 deg keeps the head clear of the torso
    for (float s : {-1.0f, 1.0f}) {
        Body* upper = part({s * 0.28f, 0.39f, 0}, Shape::box({0.06f, 0.15f, 0.06f}));
        Body* fore = part({s * 0.28f, 0.09f, 0}, Shape::box({0.05f, 0.14f, 0.05f}));
        ballJoint(torso, upper, {s * 0.28f, 0.54f, 0}, 0.02f);
        hinge(upper, fore, {s * 0.28f, 0.24f, 0}, -2.4f, 0.0f, 0.01f);

        Body* thigh = part({s * 0.1f, -0.31f, 0}, Shape::box({0.08f, 0.2f, 0.08f}));
        Body* shin = part({s * 0.1f, -0.73f, 0}, Shape::box({0.07f, 0.2f, 0.07f}));
        ballJoint(pelvis, thigh, {s * 0.1f, -0.1f, 0}, 0.04f)->limitSwing({0, -1, 0}, 1.6f);
        hinge(thigh, shin, {s * 0.1f, -0.52f, 0}, 0.0f, 2.4f, 0.02f);
    }
}

void ragdolls(World& w) {
    addGround(w);
    const int steps = 10;
    const float rise = 0.4f, run = 0.8f;
    for (int i = 0; i < steps; ++i) {
        const float h = 0.5f * rise * (i + 1); // each step is a solid block down to the ground
        fixedBox(w, {0, h, -i * run}, {2.5f, h, 0.5f * run}, i % 2 ? 0x6c757dff : 0x7d868fff);
    }
    const float topY = rise * steps, topZ = -(steps - 1) * run;
    const uint32_t colors[] = {0x3d85c6ff, 0xe07a5fff, 0x81b29aff, 0x9b5de5ff};
    for (int i = 0; i < 4; ++i) {
        const size_t first = w.bodies().size();
        ragdoll(w, {-1.5f + i * 1.0f, topY + 1.1f + i * 0.8f, topZ}, 100 + i, colors[i]);
        for (size_t k = first; k < w.bodies().size(); ++k) w.bodies()[k]->velocity = {0, 1.5f, 4.5f}; // thrown off the top
    }
}

} // namespace

Body* addGround(World& w, float half) {
    BodyDef d;
    d.isStatic = true;
    d.position = {0, -1, 0};
    d.color = 0x3a3f4bff;
    d.friction = 0.6f;
    return w.createBody(Shape::box({half, 1, half}), d);
}

const std::vector<const char*>& categories() {
    static const std::vector<const char*> list = {"Rigid bodies", "Joints & constraints"};
    return list;
}

const std::vector<Scene>& all() {
    // focus and distance frame the scene; settles = false for scenes that keep moving on purpose
    static const std::vector<Scene> list = {
        {"Box stack", stack, {0, 5, 0}, 20, true, 0, "Ten cubes balanced in a column"},
        {"Pyramid", pyramid, {0, 4, 0}, 20, true, 0, "55 boxes in a stepped pyramid"},
        {"Wall + wrecking ball", wall, {0, 2, 0}, 16, true, 0, "A heavy ball rolls into a brick wall"},
        {"Domino ring", dominoes, {0, 0, 0}, 18, true, 0, "40 dominoes topple round a circle"},
        {"Ball pit", ballPit, {0, 1, 0}, 18, true, 0, "300 balls poured into a walled pit"},
        {"Mixed rain", rain, {0, 2, 0}, 20, true, 0, "Boxes and balls of every size pile up"},
        {"Billiards break", billiards, {0, 0, 0}, 16, true, 0, "Cue ball breaks a rack inside cushions"},
        {"Pendulums", pendulums, {0, 4, 0}, 18, false, 1, "Simple, conical, double and spring pendulums"},
        {"Newton's cradle", newtonsCradle, {0, 3, 0}, 9, false, 1, "Momentum passes straight through the row"},
        {"Wrecking ball", wreckingBall, {-2, 4, 0}, 22, false, 1, "A ball on a rope swings into a tower"},
        {"Rope bridge", ropeBridge, {0, 3, 0}, 16, true, 1, "Hinged planks sag under falling crates"},
        {"Paddle wheel (motor)", paddleWheel, {0, 1.5f, 0}, 13, false, 1, "Motor-driven paddles stir a bin of balls"},
        {"Ragdolls on stairs", ragdolls, {0, 1.5f, -2.5f}, 11, true, 1, "Jointed figures tumble down a staircase"},
    };
    return list;
}

} // namespace scenes3
