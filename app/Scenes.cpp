#include "Scenes.h"
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

const std::vector<Scene>& scenes() {
    static const std::vector<Scene> list = {
        {"Pyramid", pyramid},
        {"Shape Rain", shapeRain},
        {"Friction & Restitution", frictionAndBounce},
        {"Dominoes", dominoes},
    };
    return list;
}
