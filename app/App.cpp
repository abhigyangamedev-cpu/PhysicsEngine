#include "App.h"
#include <algorithm>
#include <cstdio>
#include <string>

using namespace pe;

namespace {

constexpr float kFixedDt = 1.0f / 60.0f;
constexpr float kTopBar = 52.0f;

constexpr Color kCategoryColors[] = {
    {231, 111, 81}, {244, 162, 97}, {42, 157, 143}, {94, 129, 172}, {180, 142, 173}, {72, 170, 220},
};

Color bodyColor(const Body& b) {
    if (b.isStatic) return {70, 75, 85};
    static constexpr Color palette[] = {
        {231, 111, 81}, {244, 162, 97}, {233, 196, 106}, {42, 157, 143}, {94, 129, 172}, {180, 142, 173},
    };
    return palette[b.id % std::size(palette)];
}

Color darker(Color c) { return {Uint8(c.r * 0.6f), Uint8(c.g * 0.6f), Uint8(c.b * 0.6f), c.a}; }

void drawWorld(Renderer& gfx, const World& world, bool showContacts) {
    const auto& ps = world.particles;
    for (const auto& pc : ps.pressures) {
        thread_local std::vector<Vec2> ring;
        ring.clear();
        for (uint32_t i : pc.ring) ring.push_back(ps.particles[i].position);
        gfx.polygon(ring, {94, 200, 172, 200}, {40, 120, 100});
    }

    for (const auto& b : world.bodies()) {
        Color c = bodyColor(*b);
        if (b->shape.type == ShapeType::Circle)
            gfx.circle(b->position, b->shape.radius, b->angle, c, darker(c));
        else
            gfx.polygon(b->worldVertices, c, darker(c));
    }

    for (const auto& l : ps.links)
        if (!l.broken) gfx.line(ps.particles[l.a].position, ps.particles[l.b].position, {170, 190, 230, 160});
    for (const auto& p : ps.particles)
        gfx.point(p.position, 3.0f, p.invMass == 0.0f ? Color{255, 80, 80} : Color{220, 230, 255});
    for (const auto& a : ps.attachments)
        gfx.line(ps.particles[a.particle].position, a.body->position + a.body->rot.apply(a.localAnchor), {255, 200, 80});

    for (const auto& j : world.joints()) {
        Color c{230, 230, 230};
        if (auto* d = dynamic_cast<const DistanceJoint*>(j.get())) {
            if (d->rope) c = {200, 150, 90};
            else if (d->hertz > 0.0f) c = {250, 210, 80};
        }
        gfx.line(j->anchorA(), j->anchorB(), c);
        gfx.point(j->anchorA(), 4.0f, c);
        gfx.point(j->anchorB(), 4.0f, c);
    }

    if (showContacts) {
        for (const auto& [key, m] : world.manifolds()) {
            for (int i = 0; i < m.count; ++i) {
                gfx.point(m.contacts[i].point, 5.0f, {255, 60, 60});
                gfx.line(m.contacts[i].point, m.contacts[i].point + m.normal * 0.3f, {255, 255, 255});
            }
        }
    }
}

} // namespace

void Grab::begin(World& w, Vec2 p) {
    target = p;
    particle = w.particles.nearest(p, 0.4f);
    if (particle >= 0) {
        particleInvMass = w.particles.particles[particle].invMass;
        w.particles.particles[particle].invMass = 0.0f;
    } else if (Body* b = w.bodyAt(p)) {
        joint = w.createJoint<MouseJoint>(b, p);
    }
}

void Grab::update(World& w) {
    if (joint) joint->target = target;
    if (particle >= 0) w.particles.particles[particle].position = target;
}

void Grab::end(World& w) {
    if (joint) w.destroyJoint(joint);
    if (particle >= 0) w.particles.particles[particle].invMass = particleInvMass;
    joint = nullptr;
    particle = -1;
}

App::App(SDL_Renderer* sdl, int width, int height) : gfx_(sdl, cam_), ui_(gfx_) {
    cam_.width = width;
    cam_.height = height;
    resetBackdrop();
}

void App::resetBackdrop() {
    backdrop_.clear();
    scenes()[1].setup(backdrop_);
    backdropTime_ = 0.0f;
}

void App::loadScene(int index) {
    grab_ = {};
    sceneIndex_ = index;
    category_ = scenes()[index].category;
    world_.clear();
    scenes()[index].setup(world_);
    accumulator_ = 0.0f;
}

void App::openCategory(int category) {
    auto list = scenesIn(category);
    if (list.empty()) return;
    cam_.center = {0.0f, 7.0f};
    cam_.scale = 40.0f;
    paused_ = false;
    loadScene(list.front());
    screen_ = Screen::Simulation;
}

void App::handleEvent(const SDL_Event& e) {
    switch (e.type) {
    case SDL_EVENT_QUIT: running_ = false; break;
    case SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED:
        cam_.width = e.window.data1;
        cam_.height = e.window.data2;
        break;
    case SDL_EVENT_MOUSE_MOTION:
        mouse_ = {e.motion.x, e.motion.y};
        grab_.target = cam_.toWorld(mouse_);
        break;
    case SDL_EVENT_MOUSE_BUTTON_DOWN:
        mouse_ = {e.button.x, e.button.y};
        if (e.button.button == SDL_BUTTON_LEFT) leftPressed_ = leftHeld_ = true;
        if (e.button.button == SDL_BUTTON_RIGHT) rightPressed_ = true;
        if (e.button.button == SDL_BUTTON_MIDDLE) middlePressed_ = true;
        break;
    case SDL_EVENT_MOUSE_BUTTON_UP:
        if (e.button.button == SDL_BUTTON_LEFT) {
            leftHeld_ = false;
            grab_.end(world_);
        }
        break;
    case SDL_EVENT_MOUSE_WHEEL:
        if (screen_ == Screen::Simulation) cam_.scale *= e.wheel.y > 0 ? 1.1f : 1.0f / 1.1f;
        break;
    case SDL_EVENT_KEY_DOWN: {
        const SDL_Keycode key = e.key.key;
        if (key == SDLK_ESCAPE) {
            if (screen_ == Screen::Simulation) {
                grab_.end(world_);
                screen_ = Screen::Menu2D;
            } else if (screen_ == Screen::Menu2D) {
                screen_ = Screen::Home;
            } else {
                running_ = false;
            }
            break;
        }
        if (screen_ != Screen::Simulation) break;
        switch (key) {
        case SDLK_SPACE: paused_ = !paused_; break;
        case SDLK_S: stepOnce_ = true; break;
        case SDLK_R: loadScene(sceneIndex_); break;
        case SDLK_C: showContacts_ = !showContacts_; break;
        case SDLK_LEFT: cam_.center.x -= 1.0f; break;
        case SDLK_RIGHT: cam_.center.x += 1.0f; break;
        case SDLK_UP: cam_.center.y += 1.0f; break;
        case SDLK_DOWN: cam_.center.y -= 1.0f; break;
        default: {
            auto list = scenesIn(category_);
            int n = static_cast<int>(key) - static_cast<int>(SDLK_1);
            if (n >= 0 && n < static_cast<int>(list.size())) loadScene(list[n]);
        }
        }
        break;
    }
    }
}

void App::simulate(World& world, float dt, bool interactive) {
    auto doStep = [&] {
        Uint64 t0 = SDL_GetPerformanceCounter();
        if (interactive) grab_.update(world);
        world.step(kFixedDt);
        if (interactive) stepMs_ = 1000.0f * (SDL_GetPerformanceCounter() - t0) / SDL_GetPerformanceFrequency();
    };
    if (interactive && paused_) {
        if (stepOnce_) doStep();
        stepOnce_ = false;
        return;
    }
    accumulator_ = std::min(accumulator_ + dt, 0.25f);
    while (accumulator_ >= kFixedDt) {
        doStep();
        accumulator_ -= kFixedDt;
    }
}

void App::frame(float dt) {
    ui_.begin(mouse_.x, mouse_.y, leftPressed_);
    gfx_.clear(theme::background);

    switch (screen_) {
    case Screen::Home: drawBackdrop(dt); drawHome(); break;
    case Screen::Menu2D: drawBackdrop(dt); drawMenu2D(); break;
    case Screen::Simulation:
        simulate(world_, dt, true);
        drawWorld(gfx_, world_, showContacts_);
        drawSimulation();
        if (screen_ == Screen::Simulation && !ui_.overUi()) handleWorldClicks();
        break;
    }
    leftPressed_ = rightPressed_ = middlePressed_ = false;
}

void App::handleWorldClicks() {
    Vec2 p = cam_.toWorld(mouse_);
    BodyDef d;
    d.position = p;
    if (leftPressed_) {
        grab_.begin(world_, p);
        if (!grab_.active()) world_.createBody(Shape::box(0.5f, 0.5f), d);
    }
    if (rightPressed_) world_.createBody(Shape::circle(0.5f), d);
    if (middlePressed_) world_.createBody(randomShape(0.6f), d);
}

void App::drawBackdrop(float dt) {
    backdropTime_ += dt;
    if (backdropTime_ > 25.0f) resetBackdrop();
    backdrop_.step(std::min(dt, 1.0f / 30.0f));

    Camera saveCam = cam_;
    cam_.center = {0.0f, 9.0f};
    cam_.scale = std::min(cam_.width / 34.0f, cam_.height / 22.0f);
    drawWorld(gfx_, backdrop_, false);
    cam_ = saveCam;
    gfx_.fillRect({0, 0, float(cam_.width), float(cam_.height)}, {18, 20, 26, 215});
}

void App::drawHome() {
    const float W = float(cam_.width), H = float(cam_.height);
    ui_.labelCentered(W * 0.5f, H * 0.14f, "PHYSICS ENGINE", theme::text, 5.0f);
    ui_.labelCentered(W * 0.5f, H * 0.14f + 58.0f, "Classical mechanics, simulated from scratch", theme::textDim, 1.75f);
    ui_.labelCentered(W * 0.5f, H * 0.36f, "Choose a simulation space", theme::text, 2.0f);

    const float gap = 32.0f;
    const float cw = std::min(420.0f, (W - 3.0f * gap) * 0.5f), ch = 220.0f;
    const float x0 = W * 0.5f - cw - gap * 0.5f, y = H * 0.36f + 44.0f;

    char badge[32];
    std::snprintf(badge, sizeof badge, "%zu scenes", scenes().size());
    if (ui_.card({x0, y, cw, ch}, "2D", "Rigid bodies, joints, springs, soft bodies and cloth",
                 "Gravity, electromagnetism and fluids on the way", badge, true, theme::accent, 6.0f))
        screen_ = Screen::Menu2D;
    ui_.card({x0 + cw + gap, y, cw, ch}, "3D", "Rigid bodies and constraints in three dimensions",
             "Planned once the 2D engine is complete", "Coming soon", false, theme::accent, 6.0f);

    ui_.labelCentered(W * 0.5f, H - 40.0f, "ESC to quit", theme::textFaint, 1.25f);
}

void App::drawMenu2D() {
    const float W = float(cam_.width), H = float(cam_.height);
    ui_.panel({0, 0, W, kTopBar});
    if (ui_.button({12, 10, 100, kTopBar - 20}, "< Back")) {
        screen_ = Screen::Home;
        return;
    }
    ui_.label(132, (kTopBar - Renderer::textHeight(2.0f)) * 0.5f, "2D Simulations", theme::text, 2.0f);

    const auto& cats = categories();
    const int cols = W > 1100 ? 3 : (W > 720 ? 2 : 1);
    const float gap = 24.0f, margin = 40.0f;
    const float cw = std::min(400.0f, (W - 2.0f * margin - (cols - 1) * gap) / cols), ch = 170.0f;
    const float gridW = cols * cw + (cols - 1) * gap;
    const float x0 = (W - gridW) * 0.5f, y0 = kTopBar + 70.0f;

    ui_.labelCentered(W * 0.5f, kTopBar + 28.0f, "Pick a category", theme::textDim, 1.5f);

    for (int i = 0; i < static_cast<int>(cats.size()); ++i) {
        const auto& c = cats[i];
        SDL_FRect r{x0 + (i % cols) * (cw + gap), y0 + (i / cols) * (ch + gap), cw, ch};
        char badge[32];
        std::string details;
        const bool playable = c.status == nullptr;
        if (playable) {
            auto list = scenesIn(i);
            std::snprintf(badge, sizeof badge, "%zu scenes", list.size());
            for (size_t k = 0; k < list.size(); ++k) details += (k ? " / " : "") + std::string(scenes()[list[k]].name);
        } else {
            details = "Coming in " + std::string(c.status);
        }
        if (ui_.card(r, c.name, c.description, details.c_str(), playable ? badge : "Locked", playable,
                     kCategoryColors[i % std::size(kCategoryColors)])) {
            openCategory(i);
            return;
        }
    }
    ui_.labelCentered(W * 0.5f, H - 40.0f, "ESC to go back", theme::textFaint, 1.25f);
}

void App::drawSimulation() {
    const float W = float(cam_.width), H = float(cam_.height);
    ui_.panel({0, 0, W, kTopBar});
    if (ui_.button({12, 10, 100, kTopBar - 20}, "< Menu")) {
        grab_.end(world_);
        screen_ = Screen::Menu2D;
        return;
    }

    Color stripe = kCategoryColors[category_ % std::size(kCategoryColors)];
    gfx_.fillRect({124, 12, 4, kTopBar - 24}, stripe);
    const char* catName = categories()[category_].name;
    ui_.label(138, (kTopBar - Renderer::textHeight(1.5f)) * 0.5f, catName, theme::text, 1.5f);

    float x = 138 + Renderer::textWidth(catName, 1.5f) + 24.0f;
    for (int idx : scenesIn(category_)) {
        const char* name = scenes()[idx].name;
        float w = Renderer::textWidth(name, 1.25f) + 28.0f;
        if (ui_.button({x, 10, w, kTopBar - 20}, name, idx == sceneIndex_, 1.25f)) loadScene(idx);
        x += w + 8.0f;
    }

    float rx = W - 12.0f;
    auto rightButton = [&](const char* label, bool active) {
        float w = Renderer::textWidth(label, 1.25f) + 28.0f;
        rx -= w;
        bool clicked = ui_.button({rx, 10, w, kTopBar - 20}, label, active, 1.25f);
        rx -= 8.0f;
        return clicked;
    };
    if (rightButton("Contacts", showContacts_)) showContacts_ = !showContacts_;
    if (rightButton("Reset", false)) loadScene(sceneIndex_);
    if (rightButton(paused_ ? "Play" : "Pause", paused_)) paused_ = !paused_;

    ui_.panel({0, H - 46, W, 46});
    char stats[160];
    std::snprintf(stats, sizeof stats, "bodies %zu   joints %zu   particles %zu   step %.2f ms", world_.bodies().size(),
                  world_.joints().size(), world_.particles.particles.size(), stepMs_);
    ui_.label(12, H - 38, stats, theme::textDim, 1.25f);
    ui_.label(12, H - 18,
              "LMB drag / spawn box   RMB ball   MMB shape   wheel zoom   arrows pan   SPACE pause   S step   R reset   ESC menu",
              theme::textFaint, 1.0f);
}
