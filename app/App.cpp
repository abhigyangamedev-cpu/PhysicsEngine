#include "App.h"
#include "Problems3.h"
#include "Scenes3.h"
#include "WorldDraw.h"
#include <algorithm>
#include <cstdio>
#include <string>
#include "field/Fields.h"

using namespace pe;

namespace {

constexpr float kFixedDt = 1.0f / 60.0f;
constexpr float kTopBar = 52.0f;
constexpr float kTabBar = 40.0f;

constexpr Color kCategoryColors[] = {
    {231, 111, 81}, {244, 162, 97}, {42, 157, 143}, {94, 129, 172}, {180, 142, 173}, {72, 170, 220}, {214, 160, 72},
    {120, 190, 110}, {225, 95, 130}, {150, 120, 220}, {90, 190, 190}, {230, 130, 60}, {200, 200, 120},
};

} // namespace

void Grab::begin(World& w, Vec2 p) {
    target = p;
    particle = w.particles.nearest(p, 0.4f);
    if (particle >= 0) {
        particleInvMass = w.particles.particles[particle].invMass;
        w.particles.particles[particle].invMass = 0.0f;
    } else if (Body* b = w.bodyAt(p)) {
        if (b->isStatic) {
            staticBody = b;
            staticOffset = b->position - p;
        } else {
            joint = w.createJoint<MouseJoint>(b, p);
        }
    }
}

void Grab::update(World& w) {
    if (joint) joint->target = target;
    if (particle >= 0) w.particles.particles[particle].position = target;
    if (staticBody) {
        staticBody->position = target + staticOffset;
        staticBody->updateTransform();
    }
}

void Grab::end(World& w) {
    if (joint) w.destroyJoint(joint);
    if (particle >= 0) w.particles.particles[particle].invMass = particleInvMass;
    joint = nullptr;
    staticBody = nullptr;
    particle = -1;
}

App::App(SDL_Renderer* sdl, int width, int height) : gfx_(sdl, cam_), ui_(gfx_), editor_(gfx_, ui_, cam_) {
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
    const Scene& scene = scenes()[index];
    grab_ = {};
    sceneIndex_ = index;
    category_ = scene.category;
    world_.clear();
    world_.settings = {};
    world_.particles.damping = 0.5f;
    scene.setup(world_);
    accumulator_ = 0.0f;
    cam_.center = scene.cameraCenter;
    cam_.scale = scene.cameraScale;
    cam_.insetTop = kTopBar + kTabBar;
    cam_.insetBottom = 46.0f;
    showTrails_ = scene.trails;
    showField_ = scene.fieldLines;
    trails_.clear();
}

void App::openCategory(int category) {
    auto list = scenesIn(category);
    if (list.empty()) return;
    paused_ = false;
    loadScene(list.front());
    screen_ = Screen::Simulation;
}

void App::handleEvent(const SDL_Event& e) {
    if (e.type == SDL_EVENT_TEXT_INPUT) ui_.textInput(e.text.text);
    if (e.type == SDL_EVENT_KEY_DOWN && ui_.editing()) {
        ui_.keyDown(e.key.key);
        return;
    }
    if (screen_ == Screen::Editor) {
        if (e.type == SDL_EVENT_MOUSE_MOTION) mouse_ = {e.motion.x, e.motion.y};
        if (e.type == SDL_EVENT_MOUSE_BUTTON_DOWN) {
            mouse_ = {e.button.x, e.button.y};
            if (e.button.button == SDL_BUTTON_LEFT) leftPressed_ = true;
        }
        if (e.type == SDL_EVENT_QUIT) running_ = false;
        if (e.type == SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED) {
            cam_.width = e.window.data1;
            cam_.height = e.window.data2;
        }
        editor_.handleEvent(e);
        return;
    }
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
        if (screen_ == Screen::Menu2D || screen_ == Screen::Problems) menuScroll_ -= e.wheel.y * 60.0f;
        break;
    case SDL_EVENT_KEY_DOWN: {
        const SDL_Keycode key = e.key.key;
        if (key == SDLK_ESCAPE) {
            if (screen_ == Screen::Simulation) {
                grab_.end(world_);
                screen_ = Screen::Menu2D;
            } else if (screen_ == Screen::Menu2D || screen_ == Screen::EditorPick || screen_ == Screen::Problems) {
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
        if (interactive) {
            if (auto update = scenes()[sceneIndex_].update) update(world, kFixedDt);
            recordTrails();
        }
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
    case Screen::EditorPick: drawBackdrop(dt); drawEditorPick(); break;
    case Screen::Problems: drawBackdrop(dt); drawProblems(); break;
    case Screen::Menu2D: drawBackdrop(dt); drawMenu2D(); break;
    case Screen::Simulation:
        if (stirring_ && leftHeld_ && dt > 0.0f) {
            Vec2 m = cam_.toWorld(mouse_);
            world_.fluid.stir(m, 1.5f, (m - lastMouseWorld_) / std::max(dt, 1e-3f));
        } else {
            stirring_ = false;
        }
        lastMouseWorld_ = cam_.toWorld(mouse_);
        simulate(world_, dt, true);
        drawFieldOverlays();
        if (showField_) drawFieldLines();
        if (showTrails_) drawTrails();
        drawWorld(gfx_, world_, showContacts_);
        drawReadout();
        drawSimulation();
        if (screen_ == Screen::Simulation && !ui_.overUi()) handleWorldClicks();
        break;
    case Screen::Editor:
        editor_.frame(dt);
        if (editor_.exitRequested()) {
            screen_ = editor_.inProblem() ? Screen::Problems : Screen::EditorPick;
            problemTab_ = 0;
        }
        break;
    }
    leftPressed_ = rightPressed_ = middlePressed_ = false;
}

void App::handleWorldClicks() {
    Vec2 p = cam_.toWorld(mouse_);
    BodyDef d;
    d.position = p;
    const Scene& scene = scenes()[sceneIndex_];
    auto click = scene.click;
    if (leftPressed_) {
        grab_.begin(world_, p);
        if (!grab_.active() && !(click && click(world_, p, 1))) {
            if (scene.stir) stirring_ = true;
            else world_.createBody(Shape::box(0.5f, 0.5f), d);
        }
    }
    if (rightPressed_ && !(click && click(world_, p, 3))) world_.createBody(Shape::circle(0.5f), d);
    if (middlePressed_) world_.createBody(randomShape(0.6f), d);
}

void App::drawBackdrop(float dt) {
    backdropTime_ += dt;
    if (backdropTime_ > 25.0f) resetBackdrop();
    backdrop_.step(std::min(dt, 1.0f / 30.0f));

    Camera saveCam = cam_;
    cam_.insetTop = cam_.insetBottom = 0.0f;
    cam_.center = {0.0f, 9.0f};
    cam_.scale = std::min(cam_.width / 34.0f, cam_.height / 22.0f);
    drawWorld(gfx_, backdrop_, false);
    cam_ = saveCam;
    gfx_.fillRect({0, 0, float(cam_.width), float(cam_.height)}, {18, 20, 26, 215});
}

void App::resumeAfter3D(const view3d::Launch& from) {
    switch (from.mode) {
    case view3d::Launch::Mode::Scenes: screen_ = Screen::Home; break;
    case view3d::Launch::Mode::Editor: screen_ = Screen::EditorPick; break;
    case view3d::Launch::Mode::Problem:
        screen_ = Screen::Problems;
        problemTab_ = 1;
        break;
    }
}

namespace {

// Cards in one row when they fit, otherwise two per row; returns each card's rect in order.
std::vector<SDL_FRect> cardGrid(float W, float top, int count, float maxW, float h, float gap) {
    const int cols = W >= count * 250.0f + (count + 1) * gap ? count : 2;
    const float cw = std::min(maxW, (W - (cols + 1) * gap) / cols);
    const float x0 = W * 0.5f - (cols * cw + (cols - 1) * gap) * 0.5f;
    std::vector<SDL_FRect> out;
    for (int i = 0; i < count; ++i) out.push_back({x0 + (i % cols) * (cw + gap), top + (i / cols) * (h + gap), cw, h});
    return out;
}

// Cuts text at a word boundary so a card's subtitle stays inside the card.
std::string clip(const char* text, size_t maxChars) {
    std::string s(text);
    if (s.size() <= maxChars) return s;
    const size_t cut = s.rfind(' ', maxChars - 3);
    return s.substr(0, cut == std::string::npos ? maxChars - 3 : cut) + "...";
}

} // namespace

void App::drawHome() {
    const float W = float(cam_.width), H = float(cam_.height);
    ui_.labelCentered(W * 0.5f, H * 0.11f, "PHYSICS ENGINE", theme::text, 5.0f);
    ui_.labelCentered(W * 0.5f, H * 0.11f + 58.0f, "Classical mechanics, simulated from scratch", theme::textDim, 1.75f);
    ui_.labelCentered(W * 0.5f, H * 0.29f, "Choose a simulation space", theme::text, 2.0f);

    const auto cards = cardGrid(W, H * 0.29f + 44.0f, 4, 320.0f, H < 700.0f ? 190.0f : 220.0f, 24.0f);
    char badge[32];
    std::snprintf(badge, sizeof badge, "%zu scenes", scenes().size());
    if (ui_.card(cards[0], "2D", "Classical mechanics, waves, heat, fields, fluids and light",
                 "Rigid and soft bodies, machines, orbits, gases and optics", badge, true, theme::accent, 3.5f))
        screen_ = Screen::Menu2D;
    if (ui_.card(cards[1], "Editor", "Build any problem in 2D or 3D: set masses, velocities, forces and links",
                 "Blocks, balls, wedges, springs, rods, ropes, hinges and pulleys", "2D + 3D", true, {244, 162, 97}, 3.5f))
        screen_ = Screen::EditorPick;
    std::snprintf(badge, sizeof badge, "%zu problems", editor::examples().size() + editor3::problems().size());
    if (ui_.card(cards[2], "Problems", "Real-life questions solved by simulation, with worked answers",
                 "Cars, kicks, ramps, tetherballs, billiards", badge, true, {225, 95, 130}, 3.5f)) {
        screen_ = Screen::Problems;
        menuScroll_ = 0.0f;
    }
    std::snprintf(badge, sizeof badge, "%zu scenes", scenes3::all().size());
    if (ui_.card(cards[3], "3D", "Rigid bodies and joints in three dimensions",
                 "Pendulums, Newton's cradle, bridges, ragdolls and collapsing towers", badge, true, theme::accent, 3.5f))
        open3D_ = view3d::Launch{};

    ui_.labelCentered(W * 0.5f, H - 40.0f, "ESC to quit", theme::textFaint, 1.25f);
}

void App::drawEditorPick() {
    const float W = float(cam_.width), H = float(cam_.height);
    ui_.labelCentered(W * 0.5f, kTopBar + H * 0.08f, "Which space do you want to build in?", theme::text, 2.0f);
    const auto cards = cardGrid(W, kTopBar + H * 0.08f + 50.0f, 2, 420.0f, 230.0f, 32.0f);
    if (ui_.card(cards[0], "2D Editor", "Blocks, balls, wedges and ground; springs, rods, ropes, hinges and pulleys",
                 "Forces, constant-velocity drives, E and B fields, graphs of any quantity", "Sandbox", true, {244, 162, 97},
                 4.0f)) {
        editor_.enter();
        screen_ = Screen::Editor;
        return;
    }
    if (ui_.card(cards[1], "3D Editor", "Boxes and balls on a floor; rods, ropes, springs, ball joints and hinges",
                 "Set position, tilt, velocity and spin in x, y, z; live readouts and graphs", "Sandbox", true, theme::accent,
                 4.0f)) {
        open3D_ = view3d::Launch{view3d::Launch::Mode::Editor};
        return;
    }

    ui_.panel({0, 0, W, kTopBar});
    if (ui_.button({12, 10, 100, kTopBar - 20}, "< Back")) screen_ = Screen::Home;
    ui_.label(132, (kTopBar - Renderer::textHeight(2.0f)) * 0.5f, "Editor", theme::text, 2.0f);
    ui_.label(W - 12.0f - Renderer::textWidth("ESC back", 1.25f), (kTopBar - Renderer::textHeight(1.25f)) * 0.5f, "ESC back",
              theme::textFaint, 1.25f);
}

void App::drawProblems() {
    const float W = float(cam_.width), H = float(cam_.height);
    struct Item {
        const char *name, *topic, *question;
    };
    std::vector<Item> items;
    if (problemTab_ == 0)
        for (const auto& ex : editor::examples()) items.push_back({ex.name, ex.topic, ex.question});
    else
        for (const auto& p : editor3::problems()) items.push_back({p.name, p.topic, p.question});

    const int cols = W > 1100 ? 3 : (W > 720 ? 2 : 1);
    const float gap = 24.0f, margin = 40.0f, ch = 200.0f;
    const float cw = std::min(400.0f, (W - 2.0f * margin - (cols - 1) * gap) / cols);
    const float gridW = cols * cw + (cols - 1) * gap;
    const int rows = (static_cast<int>(items.size()) + cols - 1) / cols;
    const float contentH = 70.0f + rows * (ch + gap) + 40.0f;
    const float maxScroll = std::max(0.0f, contentH - (H - kTopBar - kTabBar));
    menuScroll_ = std::clamp(menuScroll_, 0.0f, maxScroll);
    const float x0 = (W - gridW) * 0.5f, y0 = kTopBar + kTabBar + 56.0f - menuScroll_;

    ui_.labelCentered(W * 0.5f, kTopBar + kTabBar + 20.0f - menuScroll_,
                      problemTab_ == 0 ? "Pick a question: it opens in the 2D simulation, ready to run"
                                       : "Pick a question: it opens in the 3D simulation, ready to run",
                      theme::textDim, 1.25f);
    const bool belowBars = mouse_.y > kTopBar + kTabBar;
    const size_t perLine = static_cast<size_t>((cw - 44.0f) / Renderer::textWidth("M", 1.25f));
    for (int i = 0; i < static_cast<int>(items.size()); ++i) {
        const SDL_FRect r{x0 + (i % cols) * (cw + gap), y0 + (i / cols) * (ch + gap), cw, ch};
        if (r.y > H || r.y + r.h < kTopBar + kTabBar) continue;
        const std::string question = clip(items[i].question, perLine * 4);
        const Color stripe = kCategoryColors[std::hash<std::string>{}(items[i].topic) % std::size(kCategoryColors)];
        if (ui_.card(r, items[i].name, question.c_str(), nullptr, items[i].topic, true, stripe, 1.75f) && belowBars) {
            if (problemTab_ == 0) {
                editor_.openProblem(i);
                screen_ = Screen::Editor;
            } else {
                open3D_ = view3d::Launch{view3d::Launch::Mode::Problem, i};
            }
            return;
        }
    }

    if (maxScroll > 0.0f) {
        const float top = kTopBar + kTabBar, trackH = H - top - 16.0f, thumbH = std::max(40.0f, trackH * (H - top) / contentH);
        const float thumbY = top + 8.0f + (trackH - thumbH) * menuScroll_ / maxScroll;
        gfx_.fillRect({W - 10.0f, top + 8.0f, 4.0f, trackH}, theme::panel);
        gfx_.fillRect({W - 10.0f, thumbY, 4.0f, thumbH}, theme::border);
    }

    ui_.panel({0, 0, W, kTopBar});
    if (ui_.button({12, 10, 100, kTopBar - 20}, "< Back")) {
        screen_ = Screen::Home;
        return;
    }
    ui_.label(132, (kTopBar - Renderer::textHeight(2.0f)) * 0.5f, "Real-life Problems", theme::text, 2.0f);
    ui_.label(W - 12.0f - Renderer::textWidth("wheel to scroll   ESC back", 1.25f), (kTopBar - Renderer::textHeight(1.25f)) * 0.5f,
              maxScroll > 0.0f ? "wheel to scroll   ESC back" : "ESC back", theme::textFaint, 1.25f);

    ui_.panel({0, kTopBar, W, kTabBar}, {24, 27, 35});
    float x = 12.0f;
    for (int t = 0; t < 2; ++t) {
        char name[48];
        std::snprintf(name, sizeof name, "%s problems  (%zu)", t == 0 ? "2D" : "3D",
                      t == 0 ? editor::examples().size() : editor3::problems().size());
        const float w = Renderer::textWidth(name, 1.25f) + 28.0f;
        if (ui_.button({x, kTopBar + 6, w, kTabBar - 12}, name, problemTab_ == t, 1.25f) && problemTab_ != t) {
            problemTab_ = t;
            menuScroll_ = 0.0f;
        }
        x += w + 8.0f;
    }
}

void App::drawMenu2D() {
    const float W = float(cam_.width), H = float(cam_.height);
    const auto& cats = categories();
    const int cols = W > 1100 ? 3 : (W > 720 ? 2 : 1);
    const float gap = 24.0f, margin = 40.0f;
    const float cw = std::min(400.0f, (W - 2.0f * margin - (cols - 1) * gap) / cols), ch = 160.0f;
    const float gridW = cols * cw + (cols - 1) * gap;
    const int rows = (static_cast<int>(cats.size()) + cols - 1) / cols;
    const float contentH = 70.0f + rows * (ch + gap) + 40.0f;
    const float maxScroll = std::max(0.0f, contentH - (H - kTopBar));
    menuScroll_ = std::clamp(menuScroll_, 0.0f, maxScroll);
    const float x0 = (W - gridW) * 0.5f, y0 = kTopBar + 70.0f - menuScroll_;

    ui_.labelCentered(W * 0.5f, kTopBar + 28.0f - menuScroll_, "Pick a category", theme::textDim, 1.5f);

    // the cards scroll under the top bar, which is drawn afterwards
    const bool belowBar = mouse_.y > kTopBar;
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
        if (r.y > H || r.y + r.h < kTopBar) continue;
        if (ui_.card(r, c.name, c.description, details.c_str(), playable ? badge : "Locked", playable,
                     kCategoryColors[i % std::size(kCategoryColors)]) &&
            belowBar) {
            openCategory(i);
            return;
        }
    }

    if (maxScroll > 0.0f) {
        const float trackH = H - kTopBar - 16.0f, thumbH = std::max(40.0f, trackH * (H - kTopBar) / contentH);
        const float thumbY = kTopBar + 8.0f + (trackH - thumbH) * menuScroll_ / maxScroll;
        gfx_.fillRect({W - 10.0f, kTopBar + 8.0f, 4.0f, trackH}, theme::panel);
        gfx_.fillRect({W - 10.0f, thumbY, 4.0f, thumbH}, theme::border);
    }

    ui_.panel({0, 0, W, kTopBar});
    if (ui_.button({12, 10, 100, kTopBar - 20}, "< Back")) {
        screen_ = Screen::Home;
        return;
    }
    ui_.label(132, (kTopBar - Renderer::textHeight(2.0f)) * 0.5f, "2D Simulations", theme::text, 2.0f);
    ui_.label(W - 12.0f - Renderer::textWidth("wheel to scroll   ESC back", 1.25f), (kTopBar - Renderer::textHeight(1.25f)) * 0.5f,
              maxScroll > 0.0f ? "wheel to scroll   ESC back" : "ESC back", theme::textFaint, 1.25f);
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

    ui_.panel({0, kTopBar, W, kTabBar}, {24, 27, 35});
    float x = 12.0f;
    int n = 1;
    for (int idx : scenesIn(category_)) {
        char name[64];
        std::snprintf(name, sizeof name, "%d  %s", n++, scenes()[idx].name);
        float w = Renderer::textWidth(name, 1.25f) + 28.0f;
        if (ui_.button({x, kTopBar + 6, w, kTabBar - 12}, name, idx == sceneIndex_, 1.25f)) loadScene(idx);
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
    if (rightButton("Trails", showTrails_)) {
        showTrails_ = !showTrails_;
        trails_.clear();
    }
    if (category_ == 4 && rightButton("Field", showField_)) showField_ = !showField_;
    if (rightButton("Reset", false)) loadScene(sceneIndex_);
    if (rightButton(paused_ ? "Play" : "Pause", paused_)) paused_ = !paused_;

    ui_.panel({0, H - 46, W, 46});
    char stats[160];
    std::snprintf(stats, sizeof stats, "bodies %zu   joints %zu   particles %zu   fluid %zu   step %.2f ms",
                  world_.bodies().size(), world_.joints().size(), world_.particles.particles.size(), world_.fluid.size(),
                  stepMs_);
    ui_.label(12, H - 38, stats, theme::textDim, 1.25f);
    ui_.label(12, H - 18,
              scenes()[sceneIndex_].stir
                  ? "LMB drag body / stir fluid   RMB drop object   MMB shape   wheel zoom   arrows pan   SPACE pause   R reset   ESC menu"
                  : "LMB drag / spawn box   RMB ball   MMB shape   wheel zoom   arrows pan   SPACE pause   S step   R reset   ESC menu",
              theme::textFaint, 1.0f);
}

void App::drawReadout() {
    auto fn = scenes()[sceneIndex_].readout;
    if (!fn) return;
    Readout r;
    fn(world_, r);

    for (const auto& l : r.lines) gfx_.line(l.a, l.b, rgba(l.color));

    const float s = 1.25f, lineH = Renderer::textHeight(s) + 6.0f, pad = 12.0f;
    const float histW = 300.0f, histH = 110.0f;
    const bool hasHist = !r.histogram.empty();
    float w = hasHist ? histW : 0.0f;
    for (const auto& t : r.text) w = std::max(w, Renderer::textWidth(t.c_str(), s));
    if (w == 0.0f) return;
    float h = r.text.size() * lineH + (hasHist ? histH + 34.0f : 0.0f);
    const float x = 12.0f, y = kTopBar + kTabBar + 12.0f;
    ui_.panel({x, y, w + 2 * pad, h + 2 * pad - 6.0f}, {24, 27, 35, 225});

    float ty = y + pad;
    for (size_t i = 0; i < r.text.size(); ++i, ty += lineH)
        ui_.label(x + pad, ty, r.text[i].c_str(), i == 0 ? theme::text : theme::textDim, s);
    if (!hasHist) return;

    ty += 4.0f;
    ui_.label(x + pad, ty, r.histogramLabel.c_str(), theme::textFaint, 1.0f);
    ty += 16.0f;
    float top = 1e-6f;
    for (float v : r.histogram) top = std::max(top, v);
    for (float v : r.histogramCurve) top = std::max(top, v);
    const size_t n = r.histogram.size();
    const float bw = histW / n, base = ty + histH;
    for (size_t i = 0; i < n; ++i) {
        float bh = histH * r.histogram[i] / top;
        gfx_.fillRect({x + pad + i * bw + 1.0f, base - bh, bw - 2.0f, bh}, {72, 149, 239, 200});
    }
    for (size_t i = 1; i < r.histogramCurve.size(); ++i)
        gfx_.screenLine(x + pad + (i - 0.5f) * bw, base - histH * r.histogramCurve[i - 1] / top, x + pad + (i + 0.5f) * bw,
                        base - histH * r.histogramCurve[i] / top, {255, 200, 87});
    gfx_.screenLine(x + pad, base, x + pad + histW, base, theme::border);
}

void App::recordTrails() {
    if (!showTrails_ || ++trailTick_ % 2) return;
    constexpr size_t kMaxPoints = 360;
    for (const auto& b : world_.bodies()) {
        if (b->isStatic) continue;
        auto& t = trails_[b->id];
        if (!t.empty() && (t.back() - b->position).lengthSq() > 9.0f) t.clear(); // teleported
        t.push_back(b->position);
        if (t.size() > kMaxPoints) t.pop_front();
    }
    if (trails_.size() > world_.bodies().size() * 2) {
        std::erase_if(trails_, [&](const auto& kv) {
            return std::none_of(world_.bodies().begin(), world_.bodies().end(),
                                [&](const auto& b) { return b->id == kv.first; });
        });
    }
}

void App::drawTrails() {
    for (const auto& b : world_.bodies()) {
        auto it = trails_.find(b->id);
        if (it == trails_.end() || it->second.size() < 2) continue;
        const auto& t = it->second;
        Color c = bodyColor(*b);
        for (size_t i = 1; i < t.size(); ++i) {
            c.a = Uint8(200.0f * i / t.size());
            gfx_.line(t[i - 1], t[i], c);
        }
    }
}

void App::drawFieldOverlays() {
    const Vec2 lo = cam_.toWorld({0.0f, float(cam_.height)}), hi = cam_.toWorld({float(cam_.width), 0.0f});
    for (const auto& f : world_.fields()) {
        if (auto* m = dynamic_cast<const UniformMagneticField*>(f.get())) {
            Aabb box = m->region.everywhere() ? Aabb{lo, hi} : m->region.box;
            Vec2 a = cam_.toScreen({box.min.x, box.max.y}), b = cam_.toScreen({box.max.x, box.min.y});
            gfx_.fillRect({a.x, a.y, b.x - a.x, b.y - a.y}, {120, 90, 200, 28});
            // dots = field out of the screen, crosses = into the screen
            const float step = 1.5f;
            for (float x = std::ceil(std::max(box.min.x, lo.x) / step) * step; x <= std::min(box.max.x, hi.x); x += step) {
                for (float y = std::ceil(std::max(box.min.y, lo.y) / step) * step; y <= std::min(box.max.y, hi.y);
                     y += step) {
                    Color c{150, 120, 230, 120};
                    if (m->B > 0.0f) {
                        gfx_.point({x, y}, 3.0f, c);
                    } else {
                        gfx_.line({x - 0.12f, y - 0.12f}, {x + 0.12f, y + 0.12f}, c);
                        gfx_.line({x - 0.12f, y + 0.12f}, {x + 0.12f, y - 0.12f}, c);
                    }
                }
            }
        } else if (auto* e = dynamic_cast<const UniformElectricField*>(f.get())) {
            Vec2 dir = e->E.normalized() * 0.6f;
            const float step = 3.0f;
            for (float x = std::ceil(lo.x / step) * step; x <= hi.x; x += step)
                for (float y = std::ceil(lo.y / step) * step; y <= hi.y; y += step) {
                    if (!e->region.contains({x, y})) continue;
                    Color c{240, 200, 90, 90};
                    const Vec2 o{x, y};
                    gfx_.line(o - dir, o + dir, c);
                    gfx_.line(o + dir, o + dir * 0.4f + dir.perp() * 0.5f, c);
                    gfx_.line(o + dir, o + dir * 0.4f - dir.perp() * 0.5f, c);
                }
        }
    }
}

void App::drawFieldLines() {
    std::vector<const Body*> sources;
    for (const auto& b : world_.bodies())
        if (b->isStatic && b->charge != 0.0f) sources.push_back(b.get());
    if (sources.empty()) return;

    const Vec2 lo = cam_.toWorld({0.0f, float(cam_.height)}), hi = cam_.toWorld({float(cam_.width), 0.0f});
    auto inView = [&](Vec2 p) { return p.x > lo.x - 2 && p.x < hi.x + 2 && p.y > lo.y - 2 && p.y < hi.y + 2; };

    for (const Body* src : sources) {
        const int lines = std::clamp(static_cast<int>(6.0f * std::abs(src->charge)), 8, 28);
        const float sign = src->charge > 0.0f ? 1.0f : -1.0f;
        for (int i = 0; i < lines; ++i) {
            float a = 2.0f * 3.14159265f * (i + 0.5f) / lines;
            Vec2 p = src->position + Vec2{std::cos(a), std::sin(a)} * (src->shape.radius + 0.05f);
            for (int s = 0; s < 400 && inView(p); ++s) {
                Vec2 e1 = world_.electricFieldAt(p).normalized() * (0.12f * sign);
                Vec2 mid = p + e1 * 0.5f;
                Vec2 e2 = world_.electricFieldAt(mid).normalized() * (0.12f * sign);
                Vec2 next = p + e2;
                gfx_.line(p, next, {240, 200, 90, 110});
                p = next;
                bool absorbed = false;
                for (const Body* other : sources)
                    if (other != src && other->charge * src->charge < 0.0f &&
                        (p - other->position).lengthSq() < other->shape.radius * other->shape.radius)
                        absorbed = true;
                if (absorbed) break;
            }
        }
    }
}
