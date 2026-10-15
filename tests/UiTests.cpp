// Drives the real app: the 2D screens in a hidden window, and the 3D window in every mode it opens in.
// Clicks are placed from the layouts at the fixed 1280 x 760 test window size.
#include "App.h"
#include "Check.h"
#include "Problems3.h"
#include "Scenes3.h"
#include "Viewer3.h"
#include <cmath>
#include <memory>
#include <vector>

namespace {

constexpr int kW = 1280, kH = 760;

// One hidden 2D window per test, with the app on it; events go straight to handleEvent.
struct Harness {
    SDL_Window* window = nullptr;
    SDL_Renderer* sdl = nullptr;
    std::unique_ptr<App> app;

    Harness() {
        SDL_Init(SDL_INIT_VIDEO);
        SDL_CreateWindowAndRenderer("ui test", kW, kH, SDL_WINDOW_HIDDEN, &window, &sdl);
        SDL_SetRenderDrawBlendMode(sdl, SDL_BLENDMODE_BLEND);
        app = std::make_unique<App>(sdl, kW, kH);
        frames(2);
    }
    ~Harness() {
        app.reset();
        SDL_DestroyRenderer(sdl);
        SDL_DestroyWindow(window);
    }
    void frames(int n) {
        for (int i = 0; i < n; ++i) {
            app->frame(1.0f / 60.0f);
            SDL_RenderPresent(sdl);
        }
    }
    void send(SDL_Event e) { app->handleEvent(e); }
    void click(float x, float y) {
        SDL_Event e{};
        e.type = SDL_EVENT_MOUSE_MOTION;
        e.motion.x = x, e.motion.y = y;
        send(e);
        frames(1);
        e = {};
        e.type = SDL_EVENT_MOUSE_BUTTON_DOWN;
        e.button.button = SDL_BUTTON_LEFT;
        e.button.x = x, e.button.y = y, e.button.down = true;
        send(e);
        frames(1);
        e.type = SDL_EVENT_MOUSE_BUTTON_UP;
        e.button.down = false;
        send(e);
        frames(1);
    }
    void key(SDL_Keycode k) {
        SDL_Event e{};
        e.type = SDL_EVENT_KEY_DOWN;
        e.key.key = k, e.key.down = true;
        send(e);
        frames(1);
    }
    Screen screen() const { return app->screen(); }
};

// Home: four 290-wide cards from x = 24, 314 apart, centred at y = 374.
void clickHomeCard(Harness& h, int i) { h.click(169.0f + i * 314.0f, 374.0f); }
// Editor picker: two 420-wide cards from x = 204, centred at y = 278.
void clickPickCard(Harness& h, int i) { h.click(414.0f + i * 452.0f, 278.0f); }
// Problems: three 384-wide columns from x = 40, rows 224 apart from y = 148.
void clickProblemCard(Harness& h, int i) { h.click(232.0f + (i % 3) * 408.0f, 248.0f + (i / 3) * 224.0f); }
void clickProblemTab(Harness& h, int tab) { h.click(tab == 0 ? 100.0f : 300.0f, 72.0f); }

} // namespace

TEST(ui_home_cards_open_their_screens) {
    {
        Harness h;
        CHECK(h.screen() == Screen::Home);
        clickHomeCard(h, 0);
        CHECK(h.screen() == Screen::Menu2D);
    }
    {
        Harness h;
        clickHomeCard(h, 1);
        CHECK(h.screen() == Screen::EditorPick);
    }
    {
        Harness h;
        clickHomeCard(h, 2);
        CHECK(h.screen() == Screen::Problems);
        CHECK(h.app->problemTab() == 0);
    }
    {
        Harness h;
        clickHomeCard(h, 3);
        auto launch = h.app->take3DRequest();
        CHECK(launch && launch->mode == view3d::Launch::Mode::Scenes);
        CHECK(!h.app->take3DRequest()); // taken once
    }
}

TEST(ui_editor_picker_opens_2d_and_requests_3d) {
    Harness h;
    clickHomeCard(h, 1);
    clickPickCard(h, 1);
    auto launch = h.app->take3DRequest();
    CHECK(launch && launch->mode == view3d::Launch::Mode::Editor);
    CHECK(h.screen() == Screen::EditorPick);
    clickPickCard(h, 0);
    CHECK(h.screen() == Screen::Editor);
    CHECK(!h.app->editor().inProblem());
}

TEST(ui_escape_walks_back_to_home_then_quits) {
    Harness h;
    clickHomeCard(h, 2);
    h.key(SDLK_ESCAPE);
    CHECK(h.screen() == Screen::Home);
    clickHomeCard(h, 1);
    h.key(SDLK_ESCAPE);
    CHECK(h.screen() == Screen::Home);
    CHECK(h.app->running());
    h.key(SDLK_ESCAPE);
    CHECK(!h.app->running());
}

TEST(ui_every_2d_problem_opens_locked_and_solves) {
    const auto& list = editor::examples();
    for (int i = 0; i < static_cast<int>(list.size()); ++i) {
        Harness h;
        clickHomeCard(h, 2);
        clickProblemCard(h, i);
        CHECK(h.screen() == Screen::Editor);
        const auto& ed = h.app->editor();
        CHECK(ed.inProblem());
        CHECK(ed.problem() == i);

        // locked: a tool key and a canvas click add nothing
        const size_t objects = ed.scene().objects.size();
        h.key(SDLK_2);
        h.click(400.0f, 300.0f);
        CHECK(ed.scene().objects.size() == objects);

        h.key(SDLK_SPACE);
        CHECK(ed.playing());
        // the app steps by wall time; 1/60 s frames reach 30 simulated seconds well within this
        for (int f = 0; f < 2400 && !ed.problemRun().done; ++f) h.frames(1);
        CHECK(ed.problemRun().done);
        CHECK_REL(ed.problemRun().result, list[i].expected, 0.005);
    }
}

TEST(ui_problem_back_button_returns_to_problems) {
    Harness h;
    clickHomeCard(h, 2);
    clickProblemCard(h, 0);
    CHECK(h.screen() == Screen::Editor);
    h.click(67.0f, 26.0f); // "< Problems"
    CHECK(h.screen() == Screen::Problems);
    // and the sandbox opened afterwards starts empty-handed, not on the problem
    h.key(SDLK_ESCAPE);
    clickHomeCard(h, 1);
    clickPickCard(h, 0);
    CHECK(!h.app->editor().inProblem());
    CHECK(h.app->editor().problem() < 0);
}

TEST(ui_sandbox_tools_place_objects) {
    Harness h;
    clickHomeCard(h, 1);
    clickPickCard(h, 0);
    const size_t before = h.app->editor().scene().objects.size();
    h.key(SDLK_2); // block tool
    h.click(400.0f, 300.0f);
    CHECK(h.app->editor().scene().objects.size() == before + 1);
    h.key(SDLK_3); // ball tool
    h.click(250.0f, 250.0f);
    CHECK(h.app->editor().scene().objects.size() == before + 2);
}

TEST(ui_3d_tab_lists_and_launches_3d_problems) {
    const auto& list = editor3::problems();
    for (int i = 0; i < static_cast<int>(list.size()); ++i) {
        Harness h;
        clickHomeCard(h, 2);
        clickProblemTab(h, 1);
        CHECK(h.app->problemTab() == 1);
        clickProblemCard(h, i);
        auto launch = h.app->take3DRequest();
        CHECK(launch && launch->mode == view3d::Launch::Mode::Problem && launch->problem == i);
    }
}

TEST(ui_resume_after_3d_lands_on_the_screen_that_opened_it) {
    Harness h;
    h.app->resumeAfter3D({view3d::Launch::Mode::Problem, 2});
    CHECK(h.screen() == Screen::Problems);
    CHECK(h.app->problemTab() == 1);
    h.app->resumeAfter3D({view3d::Launch::Mode::Editor});
    CHECK(h.screen() == Screen::EditorPick);
    h.app->resumeAfter3D({});
    CHECK(h.screen() == Screen::Home);
}

TEST(ui_every_2d_demo_scene_opens_and_runs) {
    // menu cards: three 384-wide columns from x = 40, rows 184 apart from y = 122; the first three rows
    // fit the window, which is every category up to index 8
    Harness h;
    clickHomeCard(h, 0);
    int opened = 0;
    for (int c = 0; c < static_cast<int>(categories().size()) && c < 9; ++c) {
        if (h.screen() == Screen::Simulation) h.key(SDLK_ESCAPE);
        CHECK(h.screen() == Screen::Menu2D);
        h.click(232.0f + (c % 3) * 408.0f, 202.0f + (c / 3) * 184.0f);
        if (categories()[c].status) { // locked categories don't open
            CHECK(h.screen() == Screen::Menu2D);
            continue;
        }
        CHECK(h.screen() == Screen::Simulation);
        ++opened;
        for (size_t k = 0; k < scenesIn(c).size() && k < 9; ++k) {
            h.key(static_cast<SDL_Keycode>(SDLK_1 + k));
            h.frames(20);
        }
        CHECK(h.app->running());
    }
    CHECK(opened > 0);
}

// ---------------------------------------------------------------- the 3D window

namespace {

struct Script3 {
    int frame;
    SDL_EventType type;
    float x = 0, y = 0;
    SDL_Keycode key = 0;
};
std::vector<Script3> g_script;
int g_lastFrame = 0;

void push(const Script3& s) {
    SDL_Event e{};
    e.type = s.type;
    switch (s.type) {
    case SDL_EVENT_MOUSE_MOTION: e.motion.x = s.x, e.motion.y = s.y; break;
    case SDL_EVENT_MOUSE_BUTTON_DOWN:
    case SDL_EVENT_MOUSE_BUTTON_UP:
        e.button.button = SDL_BUTTON_LEFT;
        e.button.x = s.x, e.button.y = s.y, e.button.down = s.type == SDL_EVENT_MOUSE_BUTTON_DOWN;
        break;
    case SDL_EVENT_KEY_DOWN: e.key.key = s.key, e.key.down = true; break;
    case SDL_EVENT_MOUSE_WHEEL: e.wheel.y = 1.0f; break;
    case SDL_EVENT_TEXT_INPUT: e.text.text = "7"; break;
    default: break;
    }
    SDL_PushEvent(&e);
}

void hook(int frame, int, int) {
    g_lastFrame = frame;
    for (const auto& s : g_script)
        if (s.frame == frame) push(s);
    if (frame == 900) push({frame, SDL_EVENT_QUIT}); // a script that never gets back out fails as Quit
}

void click3(int frame, float x, float y) {
    g_script.push_back({frame, SDL_EVENT_MOUSE_MOTION, x, y});
    g_script.push_back({frame + 1, SDL_EVENT_MOUSE_BUTTON_DOWN, x, y});
    g_script.push_back({frame + 2, SDL_EVENT_MOUSE_BUTTON_UP, x, y});
}
void key3(int frame, SDL_Keycode k) { g_script.push_back({frame, SDL_EVENT_KEY_DOWN, 0, 0, k}); }

// The OS sends events the moment a window shows, before anything is drawn: the window must take them.
void pushEarlyEvents() {
    push({0, SDL_EVENT_MOUSE_MOTION, 640, 400});
    push({0, SDL_EVENT_MOUSE_WHEEL});
    push({0, SDL_EVENT_TEXT_INPUT});
    push({0, SDL_EVENT_MOUSE_MOTION, 1100, 200});
    push({0, SDL_EVENT_MOUSE_WHEEL});
}

view3d::Exit run3(view3d::Launch launch) {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_FlushEvents(SDL_EVENT_FIRST, SDL_EVENT_LAST);
    pushEarlyEvents();
    g_lastFrame = 0;
    return view3d::run(kW, kH, launch, hook);
}

} // namespace

TEST(ui_3d_scene_picker_opens_and_backs_out) {
    g_script = {};
    click3(10, 200, 220); // the first scene card
    key3(40, SDLK_TAB);
    key3(60, SDLK_ESCAPE); // scene -> picker
    key3(70, SDLK_ESCAPE); // picker -> back
    CHECK(run3({}) == view3d::Exit::Back);
    CHECK(g_lastFrame < 900);
}

TEST(ui_3d_editor_opens_builds_runs_and_backs_out) {
    g_script = {};
    click3(10, 245, 26);  // Box tool
    click3(15, 400, 420); // on the floor
    click3(20, 300, 26);  // Ball tool
    click3(25, 300, 380);
    click3(30, 425, 26);  // Rope tool
    click3(35, 300, 360); // the ball
    click3(40, 300, 120); // empty sky
    key3(45, SDLK_SPACE);
    key3(140, SDLK_R);
    key3(150, SDLK_ESCAPE); // deselect
    key3(155, SDLK_ESCAPE); // back
    CHECK(run3({view3d::Launch::Mode::Editor}) == view3d::Exit::Back);
    CHECK(g_lastFrame < 900);
}

TEST(ui_every_3d_problem_opens_runs_and_backs_out) {
    for (int i = 0; i < static_cast<int>(editor3::problems().size()); ++i) {
        g_script = {};
        key3(10, SDLK_SPACE);
        key3(200, SDLK_ESCAPE); // deselect the focus object
        key3(205, SDLK_ESCAPE); // back
        CHECK(run3({view3d::Launch::Mode::Problem, i}) == view3d::Exit::Back);
        CHECK(g_lastFrame < 900);
    }
}

TEST(ui_3d_problem_index_out_of_range_falls_back_to_sandbox) {
    g_script = {};
    key3(20, SDLK_ESCAPE);
    CHECK(run3({view3d::Launch::Mode::Problem, 999}) == view3d::Exit::Back);
}
