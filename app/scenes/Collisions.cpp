// Collisions & Momentum: elastic vs inelastic carts, recoil, billiards and the ballistic pendulum.
#include "SceneKit.h"
#include "field/Fields.h"
#include <algorithm>
#include <cmath>

using namespace pe;
using namespace kit;

namespace {

float g_time = 0.0f;

float cartHalfWidth(float mass) { return 0.35f + 0.12f * mass; }

Body* addCart(World& w, Vec2 pos, float mass, float velocity, float restitution, uint32_t color) {
    float hw = cartHalfWidth(mass), hh = 0.3f + 0.05f * mass;
    Body* b = addBody(w, Shape::box(hw, hh), pos, mass, color);
    b->velocity = {velocity, 0.0f};
    b->restitution = restitution;
    b->friction = 0.0f;
    return b;
}

void trackLine(Readout& r, float y) { r.line({-16.0f, y - 0.75f}, {16.0f, y - 0.75f}, 0x5a5f6eff); }

// ---- Elastic and inelastic ----

struct Track {
    Body* a;
    Body* b;
    float e, v0;
    const char* label;
};
std::vector<Track> g_tracks;

void elasticInelastic(World& w) {
    g_tracks.clear();
    w.settings.gravity = {};
    w.settings.restitutionThreshold = 0.0f;
    struct Spec { float y, ma, mb, e; const char* label; };
    for (const Spec& s : {Spec{9.0f, 1.0f, 1.0f, 1.0f, "elastic, equal masses"}, Spec{5.5f, 1.0f, 3.0f, 1.0f, "elastic, 1 kg into 3 kg"},
                          Spec{2.0f, 1.0f, 3.0f, 0.0f, "perfectly inelastic, 1 kg into 3 kg"}}) {
        Body* a = addCart(w, {-10.0f, s.y}, s.ma, 4.0f, s.e, 0xe76f51ff);
        Body* b = addCart(w, {0.0f, s.y}, s.mb, 0.0f, s.e, 0x2a9d8fff);
        g_tracks.push_back({a, b, s.e, 4.0f, s.label});
    }
}

void elasticInelasticReadout(const World&, Readout& r) {
    r.add("Momentum is always conserved; kinetic energy only in elastic collisions");
    for (const auto& t : g_tracks) {
        float ma = t.a->mass, mb = t.b->mass;
        float p0 = ma * t.v0, ke0 = 0.5f * ma * t.v0 * t.v0;
        float p = ma * t.a->velocity.x + mb * t.b->velocity.x;
        float ke = kineticEnergy(*t.a) + kineticEnergy(*t.b);
        // theory for after the collision
        float va = t.e > 0.5f ? (ma - mb) / (ma + mb) * t.v0 : ma * t.v0 / (ma + mb);
        float vb = t.e > 0.5f ? 2.0f * ma / (ma + mb) * t.v0 : va;
        r.add("%s", t.label);
        r.add("   p %.2f -> %.2f kg m/s   KE %.2f -> %.2f J   v after: %+.2f, %+.2f (theory %+.2f, %+.2f)", p0, p, ke0, ke,
              t.a->velocity.x, t.b->velocity.x, va, vb);
        trackLine(r, t.a->position.y);
    }
}

// ---- Recoil ----

struct Recoil {
    Body* a;
    Body* b;
};
std::vector<Recoil> g_recoils;

void recoil(World& w) {
    g_recoils.clear();
    w.settings.gravity = {};
    auto* springs = w.addField<SpringField>();
    for (auto [y, ma, mb] : {std::tuple{10.0f, 1.0f, 4.0f}, std::tuple{5.0f, 2.0f, 2.0f}}) {
        // 0.8 m apart, with a compressed spring between them that only pushes
        const float ha = cartHalfWidth(ma), hb = cartHalfWidth(mb);
        Body* a = addCart(w, {-0.4f - ha, y}, ma, 0.0f, 0.2f, 0xe76f51ff);
        Body* b = addCart(w, {0.4f + hb, y}, mb, 0.0f, 0.2f, 0x2a9d8fff);
        auto& s = springs->add(a, {-0.4f, y}, b, {0.4f, y}, 60.0f);
        s.rest = 1.5f;
        s.pushOnly = true;
        g_recoils.push_back({a, b});
    }
}

void recoilReadout(const World&, Readout& r) {
    r.add("Recoil: the carts start at rest, so total momentum stays 0 and v1 / v2 = -m2 / m1");
    for (const auto& c : g_recoils) {
        float pa = c.a->mass * c.a->velocity.x, pb = c.b->mass * c.b->velocity.x;
        r.add("%.0f kg: %+.2f m/s   %.0f kg: %+.2f m/s   total p = %+.3f   v1/v2 = %.2f (theory %.2f)", c.a->mass,
              c.a->velocity.x, c.b->mass, c.b->velocity.x, pa + pb,
              std::abs(c.b->velocity.x) > 1e-3f ? c.a->velocity.x / c.b->velocity.x : 0.0f, -c.b->mass / c.a->mass);
        trackLine(r, c.a->position.y);
    }
}

// ---- Billiards ----

Body* g_cue = nullptr;
std::vector<Body*> g_balls;

void billiards(World& w) {
    g_balls.clear();
    w.settings.gravity = {};
    w.settings.linearDamping = 0.25f; // rolling resistance of the cloth
    w.settings.restitutionThreshold = 0.05f;
    const float hw = 14.0f, hh = 6.0f;
    for (auto [pos, half] : {std::pair{Vec2{0.0f, -hh - 0.5f}, Vec2{hw + 1.0f, 0.5f}}, std::pair{Vec2{0.0f, hh + 0.5f}, Vec2{hw + 1.0f, 0.5f}},
                             std::pair{Vec2{-hw - 0.5f, 0.0f}, Vec2{0.5f, hh}}, std::pair{Vec2{hw + 0.5f, 0.0f}, Vec2{0.5f, hh}}}) {
        Body* cushion = addStaticBox(w, pos, half.x, half.y);
        cushion->restitution = 0.8f;
        cushion->color = 0x2f6b4fff;
    }
    const float r = 0.4f;
    static constexpr uint32_t colors[] = {0xe9c46aff, 0x5e81acff, 0xe76f51ff, 0xb48eadff, 0xf4a261ff, 0x2a9d8fff, 0x8b3a3aff, 0x222222ff};
    int k = 0;
    for (int row = 0; row < 5; ++row)
        for (int i = 0; i <= row; ++i) {
            Vec2 p{5.0f + row * r * 1.75f, (i - row * 0.5f) * r * 2.02f};
            Body* b = addBody(w, Shape::circle(r), p, 0.17f, colors[k++ % std::size(colors)]);
            b->restitution = 0.95f;
            b->friction = 0.05f;
            g_balls.push_back(b);
        }
    g_cue = addBody(w, Shape::circle(r), {-8.0f, 0.05f}, 0.17f, 0xf0f0f0ff);
    g_cue->restitution = 0.95f;
    g_cue->friction = 0.05f;
    g_cue->velocity = {16.0f, 0.0f};
    g_balls.push_back(g_cue);
}

bool billiardsClick(World&, Vec2 p, int button) {
    if (button != 3) return false;
    g_cue->velocity = (p - g_cue->position) * 2.5f; // shoot toward the click, harder from further away
    g_cue->angularVelocity = 0.0f;
    return true;
}

void billiardsReadout(const World&, Readout& r) {
    Vec2 p;
    float ke = 0.0f;
    for (Body* b : g_balls) {
        p += b->velocity * b->mass;
        ke += kineticEnergy(*b);
    }
    r.add("Billiards: ball-ball hits conserve momentum; cushions and the cloth do not");
    r.add("total momentum (%+.2f, %+.2f) kg m/s   kinetic energy %.2f J", p.x, p.y, ke);
    r.add("Right click: shoot the cue ball toward the cursor");
}

// ---- Ballistic pendulum ----

constexpr float kBulletMass = 0.25f, kBulletSpeed = 40.0f, kBlockMass = 4.0f, kRopeLength = 4.0f;
Body* g_block = nullptr;
Body* g_bullet = nullptr;
bool g_captured = false;
float g_blockStartY = 0.0f, g_maxRise = 0.0f, g_captureSpeed = 0.0f;

void ballisticPendulum(World& w) {
    g_time = 0.0f;
    g_captured = false;
    g_maxRise = g_captureSpeed = 0.0f;
    addStaticBox(w, {0.0f, -0.5f}, 20.0f, 0.5f);
    const Vec2 pivot{4.0f, 11.0f};
    Body* top = addAnchor(w, pivot);
    g_block = addBody(w, Shape::box(0.5f, 0.5f), pivot - Vec2{0.0f, kRopeLength + 0.5f}, kBlockMass, 0xb48eadff);
    g_block->restitution = 0.0f;
    g_blockStartY = g_block->position.y;
    // two ropes keep the block level
    for (float dx : {-0.45f, 0.45f})
        w.createJoint<DistanceJoint>(top, g_block, pivot + Vec2{dx, 0.0f}, g_block->position + Vec2{dx, 0.5f});

    // aimed a little high: the bullet drops about 0.4 m on its way
    const float aimY = g_block->position.y + 0.4f;
    Body* gun = addStaticBox(w, {-9.0f, aimY}, 1.0f, 0.18f);
    gun->collides = false;
    BodyDef d;
    d.position = {-7.8f, aimY};
    d.density = kBulletMass / (kPi * 0.08f * 0.08f);
    d.restitution = 0.0f;
    d.color = 0xe9c46aff;
    g_bullet = w.createBody(Shape::circle(0.08f), d);
    g_bullet->velocity = {kBulletSpeed, 0.0f};
}

void ballisticUpdate(World& w, float dt) {
    g_time += dt;
    // the contact has stopped the bullet against the block's face
    if (!g_captured && g_bullet->velocity.x < 0.5f * kBulletSpeed) {
        // the bullet embeds: pin it where it hit so the two swing as one
        w.createJoint<RevoluteJoint>(g_block, g_bullet, g_bullet->position);
        g_captured = true;
    }
    if (g_captured) {
        g_captureSpeed = std::max(g_captureSpeed, g_block->velocity.length());
        g_maxRise = std::max(g_maxRise, g_block->position.y - g_blockStartY);
    }
}

void ballisticReadout(const World&, Readout& r) {
    const float V = kBulletMass * kBulletSpeed / (kBulletMass + kBlockMass);
    r.add("Ballistic pendulum: momentum is conserved in the hit, then energy in the swing");
    r.add("bullet %.2f kg at %.0f m/s into a %.0f kg block", kBulletMass, kBulletSpeed, kBlockMass);
    r.add("speed after hit: theory m v / (m + M) = %.2f   measured %.2f m/s", V, g_captureSpeed);
    r.add("rise: theory V^2 / 2g = %.3f   measured %.3f m", V * V / (2.0f * kG), g_maxRise);
    r.add("kinetic energy lost in the hit: %.0f%%", 100.0f * kBlockMass / (kBulletMass + kBlockMass));
}

} // namespace

std::vector<Scene> collisionScenes(int c) {
    return {
        {.name = "Elastic vs Inelastic", .category = c, .setup = elasticInelastic, .readout = elasticInelasticReadout},
        {.name = "Recoil", .category = c, .setup = recoil, .readout = recoilReadout},
        {.name = "Billiards", .category = c, .setup = billiards, .click = billiardsClick, .readout = billiardsReadout,
         .cameraCenter = {0.0f, 0.9f}, .cameraScale = 38.0f},
        {.name = "Ballistic Pendulum", .category = c, .setup = ballisticPendulum, .update = ballisticUpdate,
         .readout = ballisticReadout, .cameraCenter = {0.0f, 7.5f}},
    };
}
