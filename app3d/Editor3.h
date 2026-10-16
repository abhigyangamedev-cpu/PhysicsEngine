#pragma once
#include "EditorScene3.h"
#include "Problems3.h"
#include <memory>
#include "Renderer3.h"
#include <SDL3/SDL.h>
#include <deque>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace view3d {

class Overlay;

// Sandbox for building a 3D problem from boxes, balls and links, then running it with live readouts.
// It also presents the 3D real-life problems, with editing locked until opened in the editor.
class Editor3 {
public:
    explicit Editor3(Overlay& overlay);
    void openSandbox();
    void openProblem(int index);
    void openCustom(std::shared_ptr<const editor3::CustomProblem> problem); // a problem made at run time
    void handle(const SDL_Event& e);
    // window* in window coordinates (the mouse's), pixel* in framebuffer pixels
    void frame(float dt, Renderer3& renderer, int windowW, int windowH, int pixelW, int pixelH);
    bool exitRequested() { return std::exchange(exit_, false); }

private:
    enum class Tool { Select, Box, Ball, Rod, Rope, Spring, BallJoint, Hinge };
    enum class Plot { Y, Speed, Vx, Vy, Vz, Accel, KE, Energy };
    enum class Drag { None, Press, Orbit, Pan, Move, Grab };
    struct Sample {
        float t;
        pe3::Vec3 p, v, a;
        float ke;
    };
    struct Hit {
        int obj = 0; // object id, 0 = none
        bool ground = false;
        pe3::Vec3 point, normal;
    };

    Overlay& overlay_;
    Camera cam_;
    editor3::Scene scene_;
    editor3::Simulation sim_; // built from scene_ on every edit, run from it on Play
    bool dirty_ = true;

    Tool tool_ = Tool::Select;
    int selObj_ = 0, selLink_ = 0;
    std::vector<editor3::Endpoint> pending_; // first click of a link tool
    bool pendingShift_ = false;

    bool started_ = false, paused_ = true, stepOnce_ = false;
    float accumulator_ = 0.0f;
    std::unordered_map<int, std::deque<Sample>> samples_;
    std::deque<std::pair<float, float>> energy_;
    bool showLabels_ = true, showVel_ = true, showTrails_ = false, showGraph_ = true;
    Plot plot_ = Plot::Speed;

    int problem_ = -1; // showing this built-in problem; editing is locked
    std::shared_ptr<const editor3::CustomProblem> custom_; // or this one, made at run time
    bool showAnswer_ = false;
    editor3::ProblemRun run_;

    std::string status_;
    float statusTime_ = 0.0f;

    Drag drag_ = Drag::None;
    float mouseX_ = 0, mouseY_ = 0, pressX_ = 0, pressY_ = 0;
    int pressButton_ = 0, pressObj_ = 0;
    bool lPress_ = false, overUi_ = false, exit_ = false;
    pe3::Vec3 moveOffset_;
    float moveStartY_ = 0.0f;
    pe3::MouseJoint* grab_ = nullptr;
    float grabDepth_ = 0.0f;
    int winW_ = 1280, winH_ = 760;
    std::vector<Line3> extras_;

    const editor3::Problem* problemDef() const; // the problem being shown, if any
    void loadProblem(const editor3::Problem& p);
    bool locked() const { return started_ || problemDef(); }
    void setStatus(std::string s);
    void select(int obj, int link);

    void rebuild();
    void start();
    void reset();
    void togglePlay();
    void stepOnce();
    void simulate(float dt);
    void stepWorld();
    void releaseGrab();
    void newScene();
    void deleteSelection();
    void duplicateSelection();
    void focusSelection();

    pe3::Vec3 mouseRay(float x, float y) const;
    Hit pick(float x, float y) const;
    int linkAt(float x, float y) const;
    void click(float x, float y);
    void place(const Hit& hit);
    void clickLinkTool(const Hit& hit);
    void moveSelected();

    void buildExtras();
    void drawLabels();
    void drawToolbar();
    void drawInspector();
    void drawObjectInspector(editor3::ObjectSpec& o, float x, float y, float w);
    void drawLinkInspector(editor3::LinkSpec& l, float x, float y, float w);
    void drawWorldInspector(float x, float y, float w);
    void drawProblemPanel(float x, float y, float w);
    void drawGraph();
    void drawHints();
};

} // namespace view3d
