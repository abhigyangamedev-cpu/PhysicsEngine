#pragma once
#include "Renderer.h"
#include "Scenes.h"
#include "Ui.h"
#include <deque>
#include <unordered_map>

enum class Screen { Home, Menu2D, Simulation };

struct Grab {
    pe::MouseJoint* joint = nullptr;
    pe::Body* staticBody = nullptr;
    pe::Vec2 staticOffset;
    int particle = -1;
    float particleInvMass = 0.0f;
    pe::Vec2 target;

    void begin(pe::World& w, pe::Vec2 p);
    void update(pe::World& w);
    void end(pe::World& w);
    bool active() const { return joint || staticBody || particle >= 0; }
};

class App {
public:
    App(SDL_Renderer* sdl, int width, int height);
    void handleEvent(const SDL_Event& e);
    void frame(float dt);
    bool running() const { return running_; }

private:
    Camera cam_;
    Renderer gfx_;
    Ui ui_;
    Screen screen_ = Screen::Home;
    bool running_ = true;

    pe::World world_;
    pe::World backdrop_;
    float backdropTime_ = 0.0f;
    Grab grab_;
    int sceneIndex_ = 0;
    int category_ = 0;

    bool paused_ = false, stepOnce_ = false, showContacts_ = false;
    bool showTrails_ = false, showField_ = false;
    std::unordered_map<uint32_t, std::deque<pe::Vec2>> trails_;
    int trailTick_ = 0;
    float accumulator_ = 0.0f, stepMs_ = 0.0f;

    pe::Vec2 mouse_;
    bool leftPressed_ = false, rightPressed_ = false, middlePressed_ = false;
    bool leftHeld_ = false;
    bool stirring_ = false;
    pe::Vec2 lastMouseWorld_;

    void loadScene(int index);
    void openCategory(int category);
    void resetBackdrop();
    void simulate(pe::World& world, float dt, bool interactive);

    void drawHome();
    void drawMenu2D();
    void drawSimulation();
    void drawBackdrop(float dt);
    void recordTrails();
    void drawTrails();
    void drawFieldOverlays();
    void drawFieldLines();
    void handleWorldClicks();
};
