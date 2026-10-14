// Projectiles: launch angles, air resistance and the monkey and hunter.
#include "SceneKit.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

using namespace pe;
using namespace kit;

namespace {

constexpr float kBall = 0.2f;
float g_time = 0.0f;

struct Shot {
    Body* ball;
    float angle, speed;
    float landedX = NAN, peak = 0.0f;
    Vec2 start;
};
std::vector<Shot> g_shots;

Body* launch(World& w, Vec2 from, float angle, float speed, uint32_t color) {
    Body* b = addBody(w, Shape::circle(kBall), from, 0.5f, color);
    b->velocity = Vec2{std::cos(angle), std::sin(angle)} * speed;
    b->restitution = 0.0f;
    b->friction = 0.9f;
    b->group = 1; // shots pass through each other
    g_shots.push_back({b, angle, speed, NAN, from.y, from});
    return b;
}

// "12.34", or "..." while the shot is still in the air
std::string landed(const Shot& s) {
    if (std::isnan(s.landedX)) return "...";
    char buf[16];
    std::snprintf(buf, sizeof buf, "%.2f", s.landedX - s.start.x);
    return buf;
}

void addGround(World& w) { addStaticBox(w, {0.0f, -0.5f}, 30.0f, 0.5f)->restitution = 0.0f; }

void trackShots(float dt) {
    g_time += dt;
    for (auto& s : g_shots) {
        s.peak = std::max(s.peak, s.ball->position.y);
        if (std::isnan(s.landedX) && s.ball->velocity.y <= 0.0f && s.ball->position.y < s.start.y + 0.02f &&
            s.ball->position.x > s.start.x + 0.5f)
            s.landedX = s.ball->position.x;
    }
}

// ---- Launch angles ----

const Vec2 kCannon{-14.0f, kBall};
constexpr float kLaunchSpeed = 14.0f;

void launchAngles(World& w) {
    g_time = 0.0f;
    g_shots.clear();
    addGround(w);
    static constexpr uint32_t colors[] = {0xe76f51ff, 0xf4a261ff, 0xe9c46aff, 0x2a9d8fff, 0x5e81acff};
    for (int i = 0; i < 5; ++i) launch(w, kCannon, (15.0f + 15.0f * i) * kPi / 180.0f, kLaunchSpeed, colors[i]);
}

void launchUpdate(World&, float dt) { trackShots(dt); }

bool launchClick(World& w, Vec2 p, int button) {
    if (button != 3) return false;
    Vec2 d = p - kCannon;
    launch(w, kCannon, std::atan2(d.y, d.x), kLaunchSpeed, 0xb48eadff);
    return true;
}

void launchReadout(const World&, Readout& r) {
    r.add("Same speed, different angles: range R = v^2 sin(2a) / g is largest at 45 deg,");
    r.add("and complementary angles (30 / 60, 15 / 75) land together   (right click: fire toward cursor)");
    for (const auto& s : g_shots) {
        float a = s.angle, range = s.speed * s.speed * std::sin(2.0f * a) / kG;
        float height = std::pow(s.speed * std::sin(a), 2.0f) / (2.0f * kG);
        r.add("%2.0f deg   range: theory %5.2f  measured %5s m   peak: theory %4.2f  measured %4.2f m", a * 180.0f / kPi,
              range, landed(s).c_str(), height,
              s.peak - s.start.y);
    }
}

// ---- Air resistance ----

constexpr float kLinearDrag = 0.25f;     // F = -b v
constexpr float kQuadraticDrag = 0.025f; // F = -c |v| v

void airResistance(World& w) {
    g_time = 0.0f;
    g_shots.clear();
    addGround(w);
    const float angle = 45.0f * kPi / 180.0f;
    launch(w, kCannon, angle, 16.0f, 0xe9c46aff);
    launch(w, kCannon, angle, 16.0f, 0x2a9d8fff);
    launch(w, kCannon, angle, 16.0f, 0xe76f51ff);
}

void airUpdate(World&, float dt) {
    trackShots(dt);
    // forces are applied during the next step
    Body* lin = g_shots[1].ball;
    Body* quad = g_shots[2].ball;
    lin->applyForce(lin->velocity * -kLinearDrag);
    quad->applyForce(quad->velocity * (-kQuadraticDrag * quad->velocity.length()));
}

void airReadout(const World&, Readout& r) {
    r.add("Air resistance shortens the flight and makes it lopsided: steeper on the way down");
    const char* names[] = {"vacuum", "linear drag  F = -b v", "quadratic drag  F = -c |v| v"};
    for (size_t i = 0; i < g_shots.size(); ++i) {
        const Shot& s = g_shots[i];
        r.add("%-30s range %5s m   peak %.2f m   speed now %.1f m/s", names[i],
              landed(s).c_str(),
              s.peak - s.start.y, s.ball->velocity.length());
    }
    r.add("vacuum theory: range %.2f m, peak %.2f m", 16.0f * 16.0f / kG, 16.0f * 16.0f * 0.5f / (2.0f * kG));
}

// ---- Monkey and hunter ----

const Vec2 kGun{-13.0f, 0.6f}, kMonkey{8.0f, 10.0f};
float g_dartSpeed = 20.0f;
Body* g_dart = nullptr;
Body* g_monkey = nullptr;
bool g_hit = false;
float g_hitTime = 0.0f;

void monkeyHunter(World& w) {
    g_time = 0.0f;
    g_hit = false;
    g_shots.clear();
    addGround(w);
    addStaticBox(w, {kMonkey.x + 0.9f, 7.0f}, 0.25f, 7.0f)->collides = false; // the tree
    g_monkey = addBody(w, Shape::box(0.45f, 0.45f), kMonkey, 2.0f, 0xb5835aff);
    Vec2 aim = (kMonkey - kGun).normalized();
    g_dart = launch(w, kGun, std::atan2(aim.y, aim.x), g_dartSpeed, 0xe9c46aff);
    g_dart->group = 0;
    g_dart->restitution = 0.0f;
}

void monkeyUpdate(World&, float dt) {
    trackShots(dt);
    if (!g_hit && (g_dart->position - g_monkey->position).length() < 0.8f) {
        g_hit = true;
        g_hitTime = g_time;
    }
}

bool monkeyClick(World& w, Vec2, int button) {
    if (button != 3) return false;
    g_dartSpeed = randf(17.0f, 30.0f);
    w.clear();
    monkeyHunter(w);
    return true;
}

void monkeyReadout(const World&, Readout& r) {
    const Vec2 d = kMonkey - kGun;
    const float t = d.length() / g_dartSpeed;
    r.add("Monkey and hunter: aim straight at the monkey, who lets go the instant you fire.");
    r.add("Both fall 1/2 g t^2 below their straight-line paths, so the dart always hits.");
    r.add("dart speed %.1f m/s   predicted hit at t = %.2f s, %.2f m below the branch", g_dartSpeed, t, 0.5f * kG * t * t);
    r.add(g_hit ? "HIT at t = %.2f s" : "t = %.2f s", g_hit ? g_hitTime : g_time);
    r.add("Right click: fire again at a different speed");
    // the line of sight, dashed
    for (int i = 0; i < 40; i += 2) r.line(kGun + d * (i / 40.0f), kGun + d * ((i + 1) / 40.0f), 0x8a93a6ff);
}

} // namespace

std::vector<Scene> projectileScenes(int c) {
    return {
        {.name = "Launch Angles", .category = c, .setup = launchAngles, .update = launchUpdate, .click = launchClick,
         .readout = launchReadout, .cameraCenter = {-3.0f, 6.0f}, .trails = true},
        {.name = "Air Resistance", .category = c, .setup = airResistance, .update = airUpdate, .readout = airReadout,
         .cameraCenter = {-1.0f, 5.0f}, .trails = true},
        {.name = "Monkey & Hunter", .category = c, .setup = monkeyHunter, .update = monkeyUpdate, .click = monkeyClick,
         .readout = monkeyReadout, .cameraCenter = {-2.0f, 6.0f}, .trails = true},
    };
}
