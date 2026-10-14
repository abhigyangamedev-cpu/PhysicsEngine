#include "Scenes.h"
#include "soft/Builders.h"
#include <numbers>
#include <random>

using namespace pe;

namespace {

std::mt19937& rng() {
    static std::mt19937 gen{1337};
    return gen;
}

float randf(float lo, float hi) { return std::uniform_real_distribution<float>(lo, hi)(rng()); }

Body* addStaticBox(World& w, Vec2 pos, float hw, float hh, float angle = 0.0f) {
    BodyDef d;
    d.position = pos;
    d.angle = angle;
    d.isStatic = true;
    d.friction = 0.6f;
    return w.createBody(Shape::box(hw, hh), d);
}

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

Body* addAnchor(World& w, Vec2 pos) {
    BodyDef d;
    d.position = pos;
    d.isStatic = true;
    return w.createBody(Shape::circle(0.08f), d);
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
        {"Gravity & Orbits", "N-body gravity, orbits and slingshots", "Phase 3"},
        {"Electromagnetism", "Charges in electric and magnetic fields", "Phase 3"},
        {"Fluids", "SPH liquids, splashes and buoyancy", "Phase 4"},
    };
    return list;
}

const std::vector<Scene>& scenes() {
    static const std::vector<Scene> list = {
        {"Pyramid", 0, pyramid},
        {"Shape Rain", 0, shapeRain},
        {"Friction & Bounce", 0, frictionAndBounce},
        {"Dominoes", 0, dominoes},
        {"Pendulums & Springs", 1, pendulums},
        {"Bridge", 1, bridge},
        {"Machines", 1, machines},
        {"Cloth", 2, cloth},
        {"Soft Bodies", 2, softBodies},
    };
    return list;
}

std::vector<int> scenesIn(int category) {
    std::vector<int> out;
    for (int i = 0; i < static_cast<int>(scenes().size()); ++i)
        if (scenes()[i].category == category) out.push_back(i);
    return out;
}
