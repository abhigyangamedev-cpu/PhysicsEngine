#include "Viewer3.h"
#include "Editor3.h"
#include "Overlay3.h"
#include "Renderer3.h"
#include "Scenes3.h"
#include <SDL3/SDL.h>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

using namespace pe3;

namespace view3d {

namespace {

constexpr float kTopBar = 52.0f, kTabBar = 40.0f, kBottomBar = 46.0f;
constexpr Color kCategoryColors[] = {{42, 157, 143}, {244, 162, 97}};

enum class Screen { Picker, Scene, Editor };

struct Grab {
    MouseJoint* joint = nullptr;
    float depth = 0.0f;
};

class Viewer {
public:
    Viewer(SDL_Window* window, Overlay& overlay, Launch launch) : window_(window), overlay_(overlay), editor_(overlay) {
        if (launch.mode == Launch::Mode::Scenes) {
            load(8); // Newton's cradle swings behind the scene picker
            return;
        }
        screen_ = Screen::Editor;
        if (launch.mode == Launch::Mode::Problem) editor_.openProblem(launch.problem);
        else if (launch.mode == Launch::Mode::Custom) editor_.openCustom(launch.custom);
        else editor_.openSandbox();
    }

    bool running = true;
    Exit exit = Exit::Quit;

    void handle(const SDL_Event& e) {
        if (screen_ == Screen::Editor) {
            if (e.type == SDL_EVENT_QUIT) running = false, exit = Exit::Quit;
            else editor_.handle(e);
            return;
        }
        switch (e.type) {
        case SDL_EVENT_QUIT: running = false, exit = Exit::Quit; break;
        case SDL_EVENT_KEY_DOWN: key(e.key.key, e.key.mod); break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN:
            mouse_ = {e.button.x, e.button.y};
            if (overUi()) {
                if (e.button.button == SDL_BUTTON_LEFT) uiClick_ = true;
            } else if (e.button.button == SDL_BUTTON_LEFT) {
                const RayHit hit = world_.raycast(cam_.eye(), mouseRay());
                if (hit.body && !hit.body->isStatic) grab_ = {world_.createJoint<MouseJoint>(hit.body, hit.point), hit.t};
                else orbiting_ = true;
            } else if (e.button.button == SDL_BUTTON_RIGHT || e.button.button == SDL_BUTTON_MIDDLE) {
                panning_ = true;
            }
            break;
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (e.button.button == SDL_BUTTON_LEFT) {
                releaseGrab();
                orbiting_ = false;
            } else {
                panning_ = false;
            }
            break;
        case SDL_EVENT_MOUSE_MOTION:
            mouse_ = {e.motion.x, e.motion.y};
            if (orbiting_) {
                cam_.yaw -= e.motion.xrel * 0.006f;
                cam_.pitch = std::clamp(cam_.pitch + e.motion.yrel * 0.006f, -0.2f, 1.5f);
            } else if (panning_) {
                const Vec3 f = (cam_.target - cam_.eye()).normalized();
                const Vec3 s = cross(f, {0, 1, 0}).normalized();
                const Vec3 u = cross(s, f);
                const float k = cam_.distance * 0.0015f;
                cam_.target += (s * -e.motion.xrel + u * e.motion.yrel) * k;
            }
            break;
        case SDL_EVENT_MOUSE_WHEEL:
            if (screen_ == Screen::Picker) pickerScroll_ -= e.wheel.y * 60.0f;
            else if (!overUi()) cam_.distance = std::clamp(cam_.distance * std::pow(0.9f, e.wheel.y), 2.0f, 150.0f);
            break;
        default: break;
        }
    }

    void frame(float dt, Renderer3& renderer, int pixelW, int pixelH) {
        if (screen_ == Screen::Editor) {
            int w, h;
            SDL_GetWindowSize(window_, &w, &h);
            editor_.frame(dt, renderer, w, h, pixelW, pixelH);
            if (editor_.exitRequested()) running = false, exit = Exit::Back;
            return;
        }
        accumulator_ += std::min(dt, 0.1f);
        int steps = 0;
        while (accumulator_ >= kStep && steps < 4) {
            if (!paused_ || stepOnce_ || screen_ == Screen::Picker) {
                applyGrab();
                const Uint64 t0 = SDL_GetPerformanceCounter();
                world_.step(kStep);
                stepMs_ = stepMs_ * 0.9f + 0.1f * 1000.0f * (SDL_GetPerformanceCounter() - t0) / SDL_GetPerformanceFrequency();
                stepOnce_ = false;
            }
            accumulator_ -= kStep;
            ++steps;
        }
        if (steps == 4) accumulator_ = 0.0f;

        renderer.draw(world_, cam_, pixelW, pixelH, grab_.joint ? grab_.joint->b : nullptr, tintSleeping_);

        int w, h;
        SDL_GetWindowSize(window_, &w, &h);
        overlay_.begin(w, h, mouse_.x, mouse_.y, uiClick_);
        uiClick_ = false;
        if (screen_ == Screen::Picker) drawPicker();
        else drawSceneBars();
        overlay_.end(pixelW, pixelH);
    }

private:
    static constexpr float kStep = 1.0f / 60.0f;
    SDL_Window* window_;
    Overlay& overlay_;
    Editor3 editor_;
    World world_;
    Camera cam_;
    Grab grab_;
    Screen screen_ = Screen::Picker;
    int scene_ = 0;
    bool paused_ = false, tintSleeping_ = false, stepOnce_ = false, orbiting_ = false, panning_ = false;
    bool uiClick_ = false;
    float accumulator_ = 0.0f, stepMs_ = 0.0f, pickerScroll_ = 0.0f;
    struct { float x = 0, y = 0; } mouse_;
    std::mt19937 rng_{42};

    float windowW() const { return static_cast<float>(overlay_.width()); }
    float windowH() const { return static_cast<float>(overlay_.height()); }

    bool overUi() const {
        if (screen_ == Screen::Picker) return true;
        return mouse_.y < kTopBar + kTabBar || mouse_.y > windowH() - kBottomBar;
    }

    std::vector<int> scenesIn(int category) const {
        std::vector<int> out;
        const auto& all = scenes3::all();
        for (int i = 0; i < static_cast<int>(all.size()); ++i)
            if (all[i].category == category) out.push_back(i);
        return out;
    }

    void releaseGrab() {
        if (grab_.joint) world_.destroyJoint(grab_.joint);
        grab_ = {};
    }

    void load(int index) {
        scene_ = index;
        grab_ = {};
        world_.clear();
        const bool sleep = world_.settings.enableSleep;
        world_.settings = {};
        world_.settings.enableSleep = sleep;
        const auto& scene = scenes3::all()[index];
        scene.build(world_);
        cam_ = {};
        cam_.target = scene.focus;
        cam_.distance = scene.distance;
        accumulator_ = 0.0f;
    }

    void open(int index) {
        load(index);
        paused_ = false;
        screen_ = Screen::Scene;
    }

    // ---------------------------------------------------------------- scene picker

    void drawPicker() {
        Ui& ui = overlay_.ui();
        Renderer& gfx = overlay_.gfx();
        const float W = windowW(), H = windowH();
        gfx.fillRect({0, 0, W, H}, {18, 20, 26, 200});

        const auto& all = scenes3::all();
        const auto& cats = scenes3::categories();
        const int cols = W > 1100 ? 3 : (W > 720 ? 2 : 1);
        const float gap = 20.0f, margin = 40.0f, ch = 132.0f, header = 46.0f;
        const float cw = std::min(400.0f, (W - 2.0f * margin - (cols - 1) * gap) / cols);
        const float gridW = cols * cw + (cols - 1) * gap, x0 = (W - gridW) * 0.5f;

        float contentH = 60.0f;
        for (size_t c = 0; c < cats.size(); ++c) {
            const int rows = (static_cast<int>(scenesIn(static_cast<int>(c)).size()) + cols - 1) / cols;
            contentH += header + rows * (ch + gap);
        }
        const float maxScroll = std::max(0.0f, contentH + 20.0f - (H - kTopBar));
        pickerScroll_ = std::clamp(pickerScroll_, 0.0f, maxScroll);

        const bool belowBar = mouse_.y > kTopBar;
        float y = kTopBar + 24.0f - pickerScroll_;
        ui.labelCentered(W * 0.5f, y, "Pick a scene", theme::textDim, 1.5f);
        y += 36.0f;
        for (size_t c = 0; c < cats.size(); ++c) {
            const Color stripe = kCategoryColors[c % std::size(kCategoryColors)];
            gfx.fillRect({x0, y + 10.0f, 4.0f, 18.0f}, stripe);
            ui.label(x0 + 14.0f, y + 12.0f, cats[c], theme::text, 1.5f);
            y += header;
            const auto list = scenesIn(static_cast<int>(c));
            for (size_t k = 0; k < list.size(); ++k) {
                const int idx = list[k];
                const SDL_FRect r{x0 + (k % cols) * (cw + gap), y + (k / cols) * (ch + gap), cw, ch};
                if (r.y > H || r.y + r.h < kTopBar) continue;
                char badge[24];
                std::snprintf(badge, sizeof badge, "%zu", k + 1);
                const char* details = all[idx].settles ? nullptr : "keeps moving";
                if (ui.card(r, all[idx].name.c_str(), all[idx].description, details, badge, true, stripe, 1.75f) && belowBar) {
                    open(idx);
                    return;
                }
            }
            y += ((list.size() + cols - 1) / cols) * (ch + gap);
        }

        if (maxScroll > 0.0f) {
            const float trackH = H - kTopBar - 16.0f, thumbH = std::max(40.0f, trackH * (H - kTopBar) / (contentH + 20.0f));
            const float thumbY = kTopBar + 8.0f + (trackH - thumbH) * pickerScroll_ / maxScroll;
            gfx.fillRect({W - 10.0f, kTopBar + 8.0f, 4.0f, trackH}, theme::panel);
            gfx.fillRect({W - 10.0f, thumbY, 4.0f, thumbH}, theme::border);
        }

        ui.panel({0, 0, W, kTopBar});
        if (ui.button({12, 10, 100, kTopBar - 20}, "< Back")) {
            running = false;
            exit = Exit::Back;
        }
        ui.label(132, (kTopBar - Renderer::textHeight(2.0f)) * 0.5f, "3D Simulations", theme::text, 2.0f);
        const char* hint = maxScroll > 0.0f ? "click a scene   wheel to scroll   ESC back" : "click a scene   ESC back";
        ui.label(W - 12.0f - Renderer::textWidth(hint, 1.25f), (kTopBar - Renderer::textHeight(1.25f)) * 0.5f, hint,
                 theme::textFaint, 1.25f);
    }

    // ---------------------------------------------------------------- scene bars

    void drawSceneBars() {
        Ui& ui = overlay_.ui();
        Renderer& gfx = overlay_.gfx();
        const float W = windowW(), H = windowH();
        const auto& all = scenes3::all();
        const int category = all[scene_].category;

        ui.panel({0, 0, W, kTopBar});
        if (ui.button({12, 10, 110, kTopBar - 20}, "< Scenes")) {
            releaseGrab();
            screen_ = Screen::Picker;
            return;
        }
        gfx.fillRect({134, 12, 4, kTopBar - 24}, kCategoryColors[category % std::size(kCategoryColors)]);
        ui.label(148, (kTopBar - Renderer::textHeight(1.5f)) * 0.5f, scenes3::categories()[category], theme::text, 1.5f);

        float rx = W - 12.0f;
        auto rightButton = [&](const char* label, bool active) {
            const float w = Renderer::textWidth(label, 1.25f) + 28.0f;
            rx -= w;
            const bool clicked = ui.button({rx, 10, w, kTopBar - 20}, label, active, 1.25f);
            rx -= 8.0f;
            return clicked;
        };
        if (rightButton("Show sleep", tintSleeping_)) tintSleeping_ = !tintSleeping_;
        if (rightButton("Sleep", world_.settings.enableSleep)) world_.settings.enableSleep = !world_.settings.enableSleep;
        if (rightButton("Reset", false)) load(scene_);
        if (paused_ && rightButton("Step", false)) stepOnce_ = true;
        if (rightButton(paused_ ? "Play" : "Pause", paused_)) paused_ = !paused_;

        // scene tabs for this category; shrink the text if they don't fit on one row
        ui.panel({0, kTopBar, W, kTabBar}, {24, 27, 35});
        const auto list = scenesIn(category);
        std::vector<std::string> names;
        for (size_t k = 0; k < list.size(); ++k) names.push_back(std::to_string(k + 1) + "  " + all[list[k]].name);
        float scale = 1.25f;
        for (; scale > 0.75f; scale -= 0.25f) {
            float total = 12.0f;
            for (const auto& n : names) total += Renderer::textWidth(n.c_str(), scale) + 28.0f + 8.0f;
            if (total <= W) break;
        }
        float x = 12.0f;
        for (size_t k = 0; k < list.size(); ++k) {
            const float w = Renderer::textWidth(names[k].c_str(), scale) + 28.0f;
            if (ui.button({x, kTopBar + 6, w, kTabBar - 12}, names[k].c_str(), list[k] == scene_, scale)) {
                open(list[k]);
                return;
            }
            x += w + 8.0f;
        }

        ui.panel({0, H - kBottomBar, W, kBottomBar});
        char stats[200];
        std::snprintf(stats, sizeof stats, "bodies %zu   awake %d   joints %zu   step %.2f ms%s", world_.bodies().size(),
                      world_.awakeCount(), world_.joints().size(), stepMs_, paused_ ? "   PAUSED" : "");
        ui.label(12, H - 38, stats, theme::textDim, 1.25f);
        ui.label(12, H - 18,
                 "LMB drag object / orbit   RMB pan   wheel zoom   F shoot ball   B drop box   SPACE pause   . step   R reset   "
                 "ESC scenes",
                 theme::textFaint, 1.0f);
    }

    // ---------------------------------------------------------------- input

    // in a scene: 1-9 pick from the tab row, Tab / Shift+Tab cycle through every scene
    void key(SDL_Keycode k, SDL_Keymod mod) {
        if (k == SDLK_ESCAPE) {
            if (screen_ == Screen::Scene) {
                releaseGrab();
                screen_ = Screen::Picker;
            } else {
                running = false;
                exit = Exit::Back;
            }
            return;
        }
        if (screen_ != Screen::Scene) return;
        const int count = static_cast<int>(scenes3::all().size());
        const auto tabs = scenesIn(scenes3::all()[scene_].category);
        const int digit = k >= SDLK_1 && k <= SDLK_9 ? static_cast<int>(k - SDLK_1) : -1;
        if (digit >= 0 && digit < static_cast<int>(tabs.size())) open(tabs[digit]);
        else if (k == SDLK_TAB) open((scene_ + ((mod & SDL_KMOD_SHIFT) ? count - 1 : 1)) % count);
        else if (k == SDLK_R) load(scene_);
        else if (k == SDLK_SPACE) paused_ = !paused_;
        else if (k == SDLK_PERIOD) stepOnce_ = true;
        else if (k == SDLK_F) shoot();
        else if (k == SDLK_B) dropBox();
        else if (k == SDLK_Z) tintSleeping_ = !tintSleeping_;
        else if (k == SDLK_S) world_.settings.enableSleep = !world_.settings.enableSleep;
    }

    Vec3 mouseRay() const {
        const float sw = windowW(), sh = windowH();
        return cam_.ray(2.0f * mouse_.x / sw - 1.0f, 1.0f - 2.0f * mouse_.y / sh, sw / sh);
    }

    void shoot() {
        BodyDef d;
        d.position = cam_.eye();
        d.velocity = mouseRay() * 30.0f;
        d.density = 4.0f;
        d.rollingResistance = 0.05f;
        d.color = 0x30343bff;
        world_.createBody(Shape::sphere(0.4f), d);
    }

    void dropBox() {
        std::uniform_real_distribution<float> u(-0.5f, 0.5f), s(0.3f, 0.8f);
        BodyDef d;
        d.position = cam_.target + Vec3{u(rng_), 8.0f, u(rng_)};
        d.orientation = Quat::axisAngle({u(rng_), u(rng_), u(rng_) + 0.01f}, u(rng_) * 3.0f);
        d.color = 0xff8c42ff;
        world_.createBody(Shape::box({s(rng_), s(rng_), s(rng_)}), d);
    }

    void applyGrab() {
        if (grab_.joint) grab_.joint->target = cam_.eye() + mouseRay() * grab_.depth;
    }
};

} // namespace

Exit run(int width, int height, Launch launch, FrameHook hook) {
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);

    SDL_Window* window = SDL_CreateWindow("Physics Engine 3D", width, height,
                                          SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIGH_PIXEL_DENSITY);
    if (!window) {
        SDL_Log("Window creation failed: %s", SDL_GetError());
        return Exit::Back;
    }
    SDL_SetWindowMinimumSize(window, 960, 600); // the overlay's panels need at least this much room
    SDL_GLContext ctx = SDL_GL_CreateContext(window);
    if (!ctx) {
        SDL_Log("GL context failed: %s", SDL_GetError());
        SDL_DestroyWindow(window);
        return Exit::Back;
    }
    SDL_GL_SetSwapInterval(1);
    SDL_StartTextInput(window); // the editor's number fields take typed values

    Exit result = Exit::Back;
    {
        Renderer3 renderer;
        Overlay overlay;
        if (renderer.init() && overlay.init()) {
            Viewer viewer(window, overlay, launch);
            Uint64 last = SDL_GetPerformanceCounter();
            for (int frame = 0; viewer.running; ++frame) {
                SDL_Event e;
                while (SDL_PollEvent(&e)) viewer.handle(e);
                const Uint64 now = SDL_GetPerformanceCounter();
                const float dt = static_cast<float>(now - last) / SDL_GetPerformanceFrequency();
                last = now;
                int w, h;
                SDL_GetWindowSizeInPixels(window, &w, &h);
                viewer.frame(dt, renderer, w, h);
                if (hook) hook(frame, w, h);
                SDL_GL_SwapWindow(window);
            }
            result = viewer.exit;
        }
    }
    SDL_GL_DestroyContext(ctx);
    SDL_DestroyWindow(window);
    SDL_GL_ResetAttributes(); // the 2D SDL_Renderer must not inherit the core-profile request
    return result;
}

} // namespace view3d
