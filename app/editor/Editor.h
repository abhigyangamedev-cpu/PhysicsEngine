#pragma once
#include "Renderer.h"
#include "Ui.h"
#include "editor/EditorScene.h"
#include "editor/Examples.h"
#include "editor/Simulation.h"
#include <deque>
#include <string>
#include <unordered_map>
#include <utility>

namespace editor {

enum class Tool { Select, Block, Ball, Wedge, Ground, Spring, Rod, Rope, Hinge, Pulley };
enum class Plot { X, Y, Vx, Vy, Speed, Ax, Ay, Accel, KE, Energy };

struct Sample {
    float t;
    Vec2 p, v, a;
    float ke;
};

// Sandbox for building a problem from objects and links, then running it with live readouts.
// It also presents the real-life problems, with editing locked until one is opened in the editor.
class Editor {
public:
    Editor(Renderer& gfx, Ui& ui, Camera& cam);
    void enter(); // the sandbox
    void openProblem(int index);
    bool inProblem() const { return problemMode_; }

    // read-only views for the UI tests
    const Scene& scene() const { return scene_; }
    int problem() const { return example_; }
    const ExampleRun& problemRun() const { return exampleRun_; }
    bool playing() const { return started_ && !paused_; }
    void handleEvent(const SDL_Event& e);
    void frame(float dt);
    bool exitRequested() { return std::exchange(exit_, false); }

private:
    Renderer& gfx_;
    Ui& ui_;
    Camera& cam_;

    Scene scene_;     // what is being edited
    Simulation sim_; // built from scene_ on every edit, run from it on Play
    bool dirty_ = true;

    Tool tool_ = Tool::Select;
    int selObj_ = 0, selLink_ = 0; // ids, 0 = none
    std::vector<Endpoint> pending_; // clicks so far of a multi-click tool

    bool started_ = false, paused_ = true, stepOnce_ = false;
    float accumulator_ = 0.0f;
    std::unordered_map<int, std::deque<Sample>> samples_;
    std::deque<std::pair<float, float>> energy_;

    bool showGrid_ = true, showLabels_ = true, showVel_ = true, showAcc_ = false, showForce_ = true;
    bool showTrails_ = false, showGraph_ = true;
    Plot plot_ = Plot::Speed;
    int example_ = -1;          // the problem being shown or run, -1 = none
    bool problemMode_ = false;  // showing example_ as a locked real-life problem
    bool showAnswer_ = false;
    ExampleRun exampleRun_;

    std::string status_;
    float statusTime_ = 0.0f;

    enum class Drag { None, Object, Velocity, Pan, EndA, EndB, Wheel };
    Drag drag_ = Drag::None;
    Vec2 dragOffset_, panAnchor_, pressAt_;
    bool dragMoved_ = false;
    Vec2 mouse_;
    bool lPress_ = false, lRelease_ = false, rPress_ = false, mPress_ = false;
    bool lHeld_ = false, rHeld_ = false, mHeld_ = false;
    bool exit_ = false;

    void rebuild();
    void start();
    void reset();
    void togglePlay();
    void stepOnce();
    void saveScene();
    void loadScene();
    void newScene();
    void simulate(float dt);
    void stepWorld();

    int objectAt(Vec2 p) const;
    int linkAt(Vec2 p) const;
    std::vector<std::pair<Vec2, Vec2>> linkSegments(int linkIndex) const;
    void select(int obj, int link);
    void place(Vec2 p);
    void clickLinkTool(Vec2 p);
    void deleteSelection();
    void duplicateSelection();
    void setStatus(std::string s);

    void handleCanvas();
    void updateDrag();

    void drawGrid();
    void drawFieldOverlay();
    void drawTrails();
    void drawOverlays();
    void drawArrow(Vec2 a, Vec2 b, Color c);

    // EditorPanels.cpp
    void drawToolbar();
    void drawInspector();
    void drawObjectInspector(ObjectSpec& o, float x, float y, float w);
    void drawLinkInspector(LinkSpec& l, float x, float y, float w);
    void drawWorldInspector(float x, float y, float w);
    void drawProblemPanel(float x, float y, float w);
    void loadExample(int index);
    bool locked() const { return started_ || problemMode_; }
    void drawGraph();
    void drawHints();
};

} // namespace editor
