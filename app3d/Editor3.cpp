#include "Editor3.h"
#include "InspectorRows.h"
#include "Overlay3.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>

using namespace pe3;
using namespace editor3;

namespace view3d {

namespace {

constexpr float kTop = 52.0f, kBar = 40.0f, kHint = 26.0f, kGraphH = 190.0f, kInspW = 360.0f;
constexpr float kDt = Simulation::kDt;
constexpr float kDeg = std::numbers::pi_v<float> / 180.0f;
constexpr float kVelScale = 0.3f; // arrow metres per m/s
constexpr size_t kMaxSamples = 12000;
constexpr const char* kToolNames[] = {"Select", "Box", "Ball", "Rod", "Rope", "Spring", "Ball joint", "Hinge"};
constexpr const char* kPlotNames[] = {"y", "|v|", "vx", "vy", "vz", "|a|", "KE", "E total"};
constexpr const char* kLinkNames[] = {"Rod", "Rope", "Spring", "Ball joint", "Hinge"};
constexpr uint32_t kVelColor = 0x6edc78ff, kAccent = 0x3abaaaff, kPending = 0xc8c878ff;

float snap(float v, float step) { return std::round(v / step) * step; }
Vec3 snap3(Vec3 v, float step) { return {snap(v.x, step), snap(v.y, step), snap(v.z, step)}; }
bool ctrlHeld() { return SDL_GetModState() & SDL_KMOD_CTRL; }
bool shiftHeld() { return SDL_GetModState() & SDL_KMOD_SHIFT; }

float segmentDistance(float px, float py, float ax, float ay, float bx, float by) {
    const float dx = bx - ax, dy = by - ay, len2 = dx * dx + dy * dy;
    const float t = len2 > 0.0f ? std::clamp(((px - ax) * dx + (py - ay) * dy) / len2, 0.0f, 1.0f) : 0.0f;
    return std::hypot(px - ax - dx * t, py - ay - dy * t);
}


} // namespace

Editor3::Editor3(Overlay& overlay) : overlay_(overlay), scene_(defaultScene()) {}

void Editor3::openSandbox() {
    problem_ = -1;
    custom_.reset();
    scene_ = defaultScene();
    reset();
    select(0, 0);
    tool_ = Tool::Select;
    pending_.clear();
    cam_ = {};
    cam_.target = {0.0f, 1.0f, 0.0f};
    cam_.distance = 12.0f;
}

void Editor3::openProblem(int index) {
    const auto& list = problems();
    if (index < 0 || index >= static_cast<int>(list.size())) {
        openSandbox();
        return;
    }
    custom_.reset();
    problem_ = index;
    loadProblem(list[index]);
}

void Editor3::openCustom(std::shared_ptr<const CustomProblem> problem) {
    if (!problem) {
        openSandbox();
        return;
    }
    problem_ = -1;
    custom_ = std::move(problem);
    loadProblem(custom_->problem);
}

const Problem* Editor3::problemDef() const {
    if (custom_) return &custom_->problem;
    return problem_ >= 0 ? &problems()[problem_] : nullptr;
}

void Editor3::loadProblem(const Problem& p) {
    scene_ = p.setup();
    showAnswer_ = false;
    reset();
    tool_ = Tool::Select;
    pending_.clear();
    cam_ = {};
    cam_.target = p.cameraTarget;
    cam_.distance = p.cameraDistance;
    cam_.yaw = p.cameraYaw;
    cam_.pitch = p.cameraPitch;
    showTrails_ = showGraph_ = true;
    select(0, 0);
    for (const auto& o : scene_.objects)
        if (o.name == p.focus) select(o.id, 0);
}

void Editor3::setStatus(std::string s) {
    status_ = std::move(s);
    statusTime_ = 4.0f;
}

void Editor3::select(int obj, int link) {
    selObj_ = obj;
    selLink_ = link;
}

// ---------------------------------------------------------------- simulation

void Editor3::rebuild() {
    grab_ = nullptr; // it lived in the old world
    sim_.load(scene_);
    dirty_ = false;
}

void Editor3::start() {
    rebuild();
    started_ = true;
    accumulator_ = 0.0f;
    samples_.clear();
    energy_.clear();
    run_ = {};
    pending_.clear();
    tool_ = Tool::Select;
}

void Editor3::reset() {
    releaseGrab();
    started_ = false;
    paused_ = true;
    accumulator_ = 0.0f;
    samples_.clear();
    energy_.clear();
    run_ = {};
    dirty_ = true;
}

void Editor3::togglePlay() {
    if (!started_) start();
    paused_ = !paused_;
}

void Editor3::stepOnce() {
    if (!started_) start();
    paused_ = true;
    stepOnce_ = true;
}

void Editor3::releaseGrab() {
    if (grab_) sim_.world().destroyJoint(grab_);
    grab_ = nullptr;
}

void Editor3::simulate(float dt) {
    if (!started_) return;
    if (paused_) {
        if (stepOnce_) stepWorld();
        stepOnce_ = false;
        return;
    }
    accumulator_ = std::min(accumulator_ + dt * scene_.world.timeScale, 0.2f);
    while (accumulator_ >= kDt && !paused_) {
        stepWorld();
        accumulator_ -= kDt;
    }
}

void Editor3::stepWorld() {
    if (grab_) grab_->target = cam_.eye() + mouseRay(mouseX_, mouseY_) * grabDepth_;
    if (!sim_.blewUp()) sim_.step();
    if (sim_.blewUp()) {
        paused_ = true;
        accumulator_ = 0.0f;
        setStatus("Simulation blew up: an object got too fast or too far. Check stiff springs or big speeds.");
        return;
    }
    const float t = sim_.time();
    for (size_t i = 0; i < sim_.size(); ++i) {
        const Body* b = sim_.body(static_cast<int>(i));
        if (b->isStatic) continue;
        auto& s = samples_[sim_.scene().objects[i].id];
        s.push_back({t, b->position, b->velocity, sim_.acceleration(static_cast<int>(i)), kineticEnergy(*b)});
        if (s.size() > kMaxSamples) s.pop_front();
    }
    energy_.push_back({t, sim_.totalEnergy()});
    if (energy_.size() > kMaxSamples) energy_.pop_front();

    if (const Problem* p = problemDef(); p && p->monitor(sim_, run_)) {
        paused_ = true;
        accumulator_ = 0.0f;
        setStatus("The asked-for moment was reached");
    }
    const float stopAt = scene_.world.stopAt;
    if (stopAt > 0.0f && t >= stopAt - kDt * 0.5f && t - kDt < stopAt - kDt * 0.5f) {
        paused_ = true;
        accumulator_ = 0.0f;
        char buf[64];
        std::snprintf(buf, sizeof buf, "Stopped at t = %.3f s", t);
        setStatus(buf);
    }
}

// ---------------------------------------------------------------- editing

void Editor3::newScene() {
    scene_ = defaultScene();
    reset();
    select(0, 0);
}

void Editor3::deleteSelection() {
    if (locked()) return;
    if (selObj_) scene_.removeObject(selObj_);
    if (selLink_) scene_.removeLink(selLink_);
    select(0, 0);
    dirty_ = true;
}

void Editor3::duplicateSelection() {
    const ObjectSpec* src = selObj_ ? scene_.object(selObj_) : nullptr;
    if (locked() || !src) return;
    ObjectSpec o = *src;
    o.id = scene_.nextId++;
    o.name = scene_.nextName();
    o.color = 0;
    o.pos.x += (o.kind == ObjKind::Ball ? o.radius * 2.0f : o.size.x) + 0.2f;
    scene_.objects.push_back(o);
    select(o.id, 0);
    dirty_ = true;
}

void Editor3::focusSelection() {
    const int idx = sim_.indexOf(selObj_);
    if (const Body* b = sim_.body(idx)) cam_.target = b->position;
}

Vec3 Editor3::mouseRay(float x, float y) const {
    const float w = static_cast<float>(winW_), h = static_cast<float>(winH_);
    return cam_.ray(2.0f * x / w - 1.0f, 1.0f - 2.0f * y / h, w / h);
}

Editor3::Hit Editor3::pick(float x, float y) const {
    Hit hit;
    const Vec3 eye = cam_.eye(), dir = mouseRay(x, y);
    const RayHit r = sim_.world().raycast(eye, dir);
    if (r.body) {
        hit.point = r.point;
        hit.normal = r.normal;
        hit.ground = r.body == sim_.built().ground;
        for (size_t i = 0; i < sim_.size(); ++i)
            if (sim_.built().bodies[i] == r.body) hit.obj = sim_.scene().objects[i].id;
        return hit;
    }
    if (dir.y < -1e-4f) { // no ground body: still give the y = 0 plane
        hit.point = eye + dir * (-eye.y / dir.y);
        hit.normal = {0, 1, 0};
        hit.ground = true;
    }
    return hit;
}

int Editor3::linkAt(float x, float y) const {
    const auto& links = sim_.scene().links;
    for (size_t i = 0; i < links.size(); ++i) {
        const Joint* j = i < sim_.built().joints.size() ? sim_.built().joints[i] : nullptr;
        if (!j) continue;
        float ax, ay, bx, by;
        const Vec3 pa = j->anchorA(), pb = j->anchorB();
        if (!cam_.project(pa, float(winW_), float(winH_), ax, ay) || !cam_.project(pb, float(winW_), float(winH_), bx, by))
            continue;
        if (segmentDistance(x, y, ax, ay, bx, by) < 7.0f) return links[i].id;
    }
    return 0;
}

void Editor3::click(float x, float y) {
    const Hit hit = pick(x, y);
    if (locked() || tool_ == Tool::Select) {
        const int link = linkAt(x, y);
        if (link && (!hit.obj || hit.ground)) select(0, link);
        else select(hit.obj, hit.obj ? 0 : link);
        return;
    }
    if (tool_ == Tool::Box || tool_ == Tool::Ball) place(hit);
    else clickLinkTool(hit);
}

void Editor3::place(const Hit& hit) {
    ObjectSpec o;
    o.id = scene_.nextId++;
    o.name = scene_.nextName();
    o.kind = tool_ == Tool::Ball ? ObjKind::Ball : ObjKind::Box;
    const float half = o.halfHeight();
    Vec3 p;
    if (hit.obj || hit.ground) p = hit.point + hit.normal * half;
    else p = cam_.target + Vec3{0, half, 0};
    if (!ctrlHeld()) p = {snap(p.x, 0.05f), hit.ground ? half : snap(p.y, 0.05f), snap(p.z, 0.05f)};
    o.pos = p;
    scene_.objects.push_back(o);
    select(o.id, 0);
    tool_ = Tool::Select;
    dirty_ = true;
}

void Editor3::clickLinkTool(const Hit& hit) {
    const bool shift = shiftHeld();
    if (pending_.empty()) {
        if (!hit.obj) {
            setStatus("Click an object to start the link");
            return;
        }
        pending_.push_back(shift ? scene_.endpointAt(hit.obj, hit.point) : Endpoint{hit.obj, {}});
        pendingShift_ = shift;
        return;
    }
    const Endpoint first = pending_[0];
    if (hit.obj == first.obj) return;
    const bool joint = tool_ == Tool::BallJoint || tool_ == Tool::Hinge;
    const Vec3 firstWorld = scene_.endpointWorld(first);

    LinkSpec l;
    l.kind = tool_ == Tool::Rod      ? LinkKind::Rod
             : tool_ == Tool::Rope   ? LinkKind::Rope
             : tool_ == Tool::Spring ? LinkKind::Spring
             : tool_ == Tool::Hinge  ? LinkKind::Hinge
                                     : LinkKind::BallJoint;
    l.b = first;
    if (hit.obj) {
        Vec3 at = hit.point;
        if (!shift) {
            const ObjectSpec* o = scene_.object(hit.obj);
            at = joint ? (o->pos + firstWorld) * 0.5f : o->pos; // joints pivot between the two
        }
        l.a = scene_.endpointAt(hit.obj, at);
    } else if (hit.ground && (hit.point - firstWorld).length() < 15.0f) { // near the horizon counts as sky
        l.a = {-1, ctrlHeld() ? hit.point : snap3(hit.point, 0.05f)};
    } else {
        // empty sky: a fixed point above (a joint pivots where the first click landed, if shift-clicked)
        const Vec3 above = joint && pendingShift_ ? firstWorld : firstWorld + Vec3{0, joint ? 1.0f : 2.0f, 0};
        l.a = {-1, above};
    }
    if (joint) l.b = {first.obj, {}};
    l.id = scene_.nextId++;
    scene_.links.push_back(l);
    pending_.clear();
    select(0, l.id);
    tool_ = Tool::Select;
    dirty_ = true;
}

void Editor3::moveSelected() {
    ObjectSpec* o = scene_.object(selObj_);
    if (!o) return;
    Vec3 p = o->pos;
    if (shiftHeld()) {
        p.y = moveStartY_ + (pressY_ - mouseY_) * cam_.distance * 0.0015f;
    } else {
        const Vec3 eye = cam_.eye(), dir = mouseRay(mouseX_, mouseY_);
        if (std::abs(dir.y) < 1e-4f) return;
        const float t = (moveStartY_ - eye.y) / dir.y;
        if (t <= 0.0f) return;
        const Vec3 hit = eye + dir * t + moveOffset_;
        p = {hit.x, moveStartY_, hit.z};
    }
    if (!ctrlHeld()) p = snap3(p, 0.05f);
    if (p.x != o->pos.x || p.y != o->pos.y || p.z != o->pos.z) {
        o->pos = p;
        dirty_ = true;
    }
}

// ---------------------------------------------------------------- input

void Editor3::handle(const SDL_Event& e) {
    Ui& ui = overlay_.ui();
    switch (e.type) {
    case SDL_EVENT_TEXT_INPUT: ui.textInput(e.text.text); break;
    case SDL_EVENT_MOUSE_MOTION:
        mouseX_ = e.motion.x, mouseY_ = e.motion.y;
        if (drag_ == Drag::Press && std::hypot(mouseX_ - pressX_, mouseY_ - pressY_) > 4.0f) {
            if (pressButton_ != SDL_BUTTON_LEFT) {
                drag_ = Drag::Pan;
            } else if (pressObj_ && pressObj_ == selObj_ && !locked() && tool_ == Tool::Select) {
                drag_ = Drag::Move;
                const ObjectSpec* o = scene_.object(selObj_);
                moveStartY_ = o->pos.y;
                const Vec3 eye = cam_.eye(), dir = mouseRay(pressX_, pressY_);
                const float t = std::abs(dir.y) > 1e-4f ? (moveStartY_ - eye.y) / dir.y : 0.0f;
                moveOffset_ = t > 0.0f ? o->pos - (eye + dir * t) : Vec3{};
            } else {
                drag_ = Drag::Orbit;
            }
        }
        if (drag_ == Drag::Orbit) {
            cam_.yaw -= e.motion.xrel * 0.006f;
            cam_.pitch = std::clamp(cam_.pitch + e.motion.yrel * 0.006f, -0.2f, 1.5f);
        } else if (drag_ == Drag::Pan) {
            const Vec3 f = (cam_.target - cam_.eye()).normalized();
            const Vec3 s = cross(f, {0, 1, 0}).normalized();
            const Vec3 u = cross(s, f);
            cam_.target += (s * -e.motion.xrel + u * e.motion.yrel) * (cam_.distance * 0.0015f);
        } else if (drag_ == Drag::Move) {
            moveSelected();
        }
        break;
    case SDL_EVENT_MOUSE_BUTTON_DOWN:
        mouseX_ = e.button.x, mouseY_ = e.button.y;
        if (e.button.button == SDL_BUTTON_LEFT) lPress_ = true;
        if (overUi_ || drag_ != Drag::None) break;
        pressX_ = mouseX_, pressY_ = mouseY_;
        pressButton_ = e.button.button;
        pressObj_ = 0;
        drag_ = Drag::Press;
        if (e.button.button == SDL_BUTTON_LEFT) {
            const Hit hit = pick(mouseX_, mouseY_);
            pressObj_ = hit.obj;
            const int idx = sim_.indexOf(hit.obj);
            Body* b = sim_.body(idx);
            if (started_ && b && !b->isStatic) { // running: drag bodies about with a soft spring
                releaseGrab();
                grab_ = sim_.world().createJoint<MouseJoint>(b, hit.point);
                grabDepth_ = (hit.point - cam_.eye()).length();
                drag_ = Drag::Grab;
                select(hit.obj, 0);
            }
        }
        break;
    case SDL_EVENT_MOUSE_BUTTON_UP:
        if (e.button.button != pressButton_ && drag_ != Drag::None) break;
        if (drag_ == Drag::Press && e.button.button == SDL_BUTTON_LEFT) click(mouseX_, mouseY_);
        if (drag_ == Drag::Grab) releaseGrab();
        drag_ = Drag::None;
        pressButton_ = 0;
        break;
    case SDL_EVENT_MOUSE_WHEEL:
        if (overUi_) ui.scroll(e.wheel.y);
        else cam_.distance = std::clamp(cam_.distance * std::pow(0.9f, e.wheel.y), 0.3f, 150.0f);
        break;
    case SDL_EVENT_KEY_DOWN: {
        if (ui.editing()) {
            ui.keyDown(e.key.key);
            break;
        }
        const SDL_Keycode key = e.key.key;
        const bool ctrl = e.key.mod & SDL_KMOD_CTRL;
        if (key == SDLK_ESCAPE) {
            if (!pending_.empty()) pending_.clear();
            else if (tool_ != Tool::Select) tool_ = Tool::Select;
            else if (selObj_ || selLink_) select(0, 0);
            else exit_ = true;
        } else if (key == SDLK_SPACE) {
            togglePlay();
        } else if (key == SDLK_R) {
            reset();
        } else if (key == SDLK_PERIOD) {
            stepOnce();
        } else if (key == SDLK_DELETE || key == SDLK_BACKSPACE) {
            deleteSelection();
        } else if (key == SDLK_D && ctrl) {
            duplicateSelection();
        } else if (key == SDLK_F) {
            focusSelection();
        } else if (key == SDLK_G) {
            showGraph_ = !showGraph_;
        } else if (key >= SDLK_1 && key <= SDLK_8 && !locked()) {
            tool_ = static_cast<Tool>(key - SDLK_1);
            pending_.clear();
        }
        break;
    }
    default: break;
    }
}

// ---------------------------------------------------------------- frame

void Editor3::frame(float dt, Renderer3& renderer, int windowW, int windowH, int pixelW, int pixelH) {
    winW_ = std::max(windowW, 1), winH_ = std::max(windowH, 1);
    statusTime_ -= dt;
    if (dirty_ && !started_) rebuild();
    simulate(dt);

    buildExtras();
    const int idx = sim_.indexOf(selObj_);
    renderer.draw(sim_.world(), cam_, pixelW, pixelH, sim_.body(idx), false, extras_);

    overlay_.begin(winW_, winH_, mouseX_, mouseY_, lPress_);
    if (showLabels_) drawLabels();
    drawToolbar();
    drawInspector();
    if (showGraph_) drawGraph();
    drawHints();
    overUi_ = overlay_.ui().overUi() || mouseY_ < kTop + kBar || mouseX_ > winW_ - kInspW;
    overlay_.end(pixelW, pixelH);
    lPress_ = false;
}

void Editor3::buildExtras() {
    extras_.clear();
    for (size_t i = 0; i < sim_.size(); ++i) {
        const Body* b = sim_.body(static_cast<int>(i));
        if (b->isStatic) continue;
        if (showVel_ && b->velocity.lengthSq() > 1e-6f) {
            const Vec3 tip = b->position + b->velocity * kVelScale;
            extras_.push_back({b->position, tip, 0.012f, kVelColor});
            const Vec3 back = (b->position - tip).normalized() * std::min(0.12f, (tip - b->position).length() * 0.4f);
            Vec3 side, up;
            basis(back.normalized(), side, up);
            extras_.push_back({tip, tip + back + side * back.length() * 0.45f, 0.01f, kVelColor});
            extras_.push_back({tip, tip + back - side * back.length() * 0.45f, 0.01f, kVelColor});
        }
        if (showTrails_) {
            auto it = samples_.find(sim_.scene().objects[i].id);
            if (it == samples_.end() || it->second.size() < 2) continue;
            const auto& s = it->second;
            const size_t stride = std::max<size_t>(1, s.size() / 300);
            for (size_t k = stride; k < s.size(); k += stride)
                extras_.push_back({s[k - stride].p, s[k].p, 0.008f, b->color ? b->color : 0xb0b8c8ff});
        }
    }
    // the selected link, and a link being placed
    const auto& links = sim_.scene().links;
    for (size_t i = 0; i < links.size() && i < sim_.built().joints.size(); ++i)
        if (links[i].id == selLink_ && sim_.built().joints[i])
            extras_.push_back({sim_.built().joints[i]->anchorA(), sim_.built().joints[i]->anchorB(), 0.035f, kAccent});
    if (!pending_.empty()) {
        const Hit hit = pick(mouseX_, mouseY_);
        const Vec3 from = scene_.endpointWorld(pending_[0]);
        const Vec3 to = hit.obj || hit.ground ? hit.point : from + Vec3{0, 2, 0};
        extras_.push_back({from, to, 0.015f, kPending});
    }
}

void Editor3::drawLabels() {
    Renderer& gfx = overlay_.gfx();
    for (size_t i = 0; i < sim_.size(); ++i) {
        const ObjectSpec& o = sim_.scene().objects[i];
        const Body* b = sim_.body(static_cast<int>(i));
        float x, y;
        if (!cam_.project(b->position + Vec3{0, b->shape.type == ShapeType::Sphere ? b->shape.radius : b->aabb.max.y - b->position.y, 0},
                          float(winW_), float(winH_), x, y))
            continue;
        y -= 16.0f;
        if (y < kTop + kBar || x > winW_ - kInspW || y > winH_ - kHint - (showGraph_ ? kGraphH : 0.0f)) continue;
        gfx.text(x - Renderer::textWidth(o.name.c_str(), 1.25f) * 0.5f, y, o.name.c_str(), o.fixed ? theme::textDim : theme::text,
                 1.25f);
    }
}

void Editor3::drawToolbar() {
    Ui& ui = overlay_.ui();
    const float W = float(winW_);
    ui.panel({0, 0, W, kTop});
    if (ui.button({12, 10, 110, kTop - 20}, custom_ ? "< Back" : problem_ >= 0 ? "< Problems" : "< Home", false, 1.25f)) {
        exit_ = true;
        return;
    }
    float x = 134.0f;
    if (problemDef()) {
        std::string title = std::string("3D problem: ") + problemDef()->name;
        ui.label(x, (kTop - Renderer::textHeight(1.5f)) * 0.5f, title.c_str(), theme::text, 1.5f);
    } else {
        for (int i = 0; i < static_cast<int>(std::size(kToolNames)); ++i) {
            const float w = Renderer::textWidth(kToolNames[i], 1.25f) + 20.0f;
            if (ui.button({x, 10, w, kTop - 20}, kToolNames[i], tool_ == static_cast<Tool>(i), 1.25f)) {
                if (started_ && i != 0) {
                    setStatus("Reset (R) to edit the scene");
                } else {
                    tool_ = static_cast<Tool>(i);
                    pending_.clear();
                }
            }
            x += w + 6.0f;
        }
    }

    float rx = W - 12.0f;
    auto rightButton = [&](const char* label, bool active) {
        const float w = Renderer::textWidth(label, 1.25f) + 24.0f;
        rx -= w;
        const bool clicked = ui.button({rx, 10, w, kTop - 20}, label, active, 1.25f);
        rx -= 6.0f;
        return clicked;
    };
    if (rightButton("Reset", false)) reset();
    if (rightButton("Step", false)) stepOnce();
    if (rightButton(started_ && !paused_ ? "Pause" : "Play", started_ && !paused_)) togglePlay();

    ui.panel({0, kTop, W, kBar}, {24, 27, 35});
    x = 12.0f;
    auto toggle = [&](const char* label, bool& v) {
        const float w = Renderer::textWidth(label, 1.0f) + 18.0f;
        if (ui.button({x, kTop + 7, w, kBar - 14}, label, v, 1.0f)) v = !v;
        x += w + 5.0f;
    };
    toggle("Labels", showLabels_);
    toggle("Velocity", showVel_);
    toggle("Trails", showTrails_);
    toggle("Graph", showGraph_);
    x += 14.0f;
    auto action = [&](const char* label) {
        const float w = Renderer::textWidth(label, 1.0f) + 18.0f;
        const bool clicked = ui.button({x, kTop + 7, w, kBar - 14}, label, false, 1.0f);
        x += w + 5.0f;
        return clicked;
    };
    if (action("Focus")) focusSelection();
    if (!problemDef() && action("New")) newScene();

    char buf[64];
    std::snprintf(buf, sizeof buf, "t = %.3f s", sim_.time());
    const float tw = Renderer::textWidth(buf, 1.5f);
    ui.label(W - 12.0f - tw, kTop + (kBar - Renderer::textHeight(1.5f)) * 0.5f, buf,
             started_ ? (paused_ ? theme::warning : theme::accentHover) : theme::textDim, 1.5f);
    if (statusTime_ > 0.0f)
        ui.label(W - 30.0f - tw - Renderer::textWidth(status_.c_str(), 1.0f), kTop + (kBar - Renderer::textHeight(1.0f)) * 0.5f,
                 status_.c_str(), theme::warning, 1.0f);
}

void Editor3::drawInspector() {
    Ui& ui = overlay_.ui();
    Renderer& gfx = overlay_.gfx();
    const float W = float(winW_), H = float(winH_);
    const float x = W - kInspW, y = kTop + kBar;
    ui.panel({x, y, kInspW, H - y - kHint}, {26, 29, 37, 240});
    gfx.fillRect({x, y, 1.0f, H - y - kHint}, theme::border);
    const float inner = kInspW - 28.0f;
    if (problemDef()) {
        drawProblemPanel(x + 14.0f, y + 12.0f, inner);
        return;
    }
    if (ObjectSpec* o = selObj_ ? scene_.object(selObj_) : nullptr) drawObjectInspector(*o, x + 14.0f, y + 12.0f, inner);
    else if (LinkSpec* l = selLink_ ? scene_.findLink(selLink_) : nullptr) drawLinkInspector(*l, x + 14.0f, y + 12.0f, inner);
    else drawWorldInspector(x + 14.0f, y + 12.0f, inner);
}

void Editor3::drawObjectInspector(ObjectSpec& o, float x, float y, float w) {
    char title[64];
    std::snprintf(title, sizeof title, "%s %s", o.kind == ObjKind::Ball ? "Ball" : "Box", o.name.c_str());
    Rows r{overlay_.ui(), x, y, w, !locked()};
    r.header(title, started_ ? "initial values (Reset to edit)" : "initial values, type or scroll to change");
    const int id = 1000 + o.id * 64;
    r.toggles({{"Fixed", &o.fixed}});
    r.one("Mass kg", id, o.mass, 0.5f);
    if (o.kind == ObjKind::Box) r.three("Size x, y, z m", id + 1, o.size.x, o.size.y, o.size.z, 0.1f);
    else r.one("Radius m", id + 1, o.radius, 0.05f);
    r.three("Position m", id + 4, o.pos.x, o.pos.y, o.pos.z, 0.1f);
    r.three("Tilt x,y,z deg", id + 7, o.tiltDeg.x, o.tiltDeg.y, o.tiltDeg.z, 5.0f);
    if (!o.fixed) {
        r.three("Velocity m/s", id + 10, o.velocity.x, o.velocity.y, o.velocity.z, 0.5f);
        r.three("Spin rad/s", id + 13, o.spin.x, o.spin.y, o.spin.z, 0.5f);
    }
    r.two("Friction u, e", id + 16, o.friction, o.restitution, 0.05f);
    if (o.kind == ObjKind::Ball) r.one("Rolling resist", id + 18, o.rolling, 0.01f);
    if (r.changed) {
        o.sanitize();
        dirty_ = true;
    }
    Ui& ui = overlay_.ui();
    if (!locked()) {
        r.y += 4.0f;
        if (ui.button({x, r.y, 110, 26}, "Duplicate", false, 1.0f)) duplicateSelection();
        if (ui.button({x + 118, r.y, 90, 26}, "Delete", false, 1.0f)) {
            deleteSelection();
            return;
        }
        r.y += 34.0f;
        r.note("Drag to move it across the floor, Shift+drag to lift it. Ctrl turns off snapping.");
    }

    const int idx = sim_.indexOf(o.id);
    const Body* b = sim_.body(idx);
    if (!started_ || !b || o.fixed) return;
    const Vec3 a = sim_.acceleration(idx);
    r.section("Live");
    r.text("position  %8.3f %8.3f %8.3f m", b->position.x, b->position.y, b->position.z);
    r.text("velocity  %8.3f %8.3f %8.3f", b->velocity.x, b->velocity.y, b->velocity.z);
    r.text("speed %8.3f m/s   |a| %8.3f m/s^2", b->velocity.length(), a.length());
    r.text("spin  %8.3f rad/s", b->angularVelocity.length());
    r.text("KE %9.3f J   PE (mgy) %9.3f J", kineticEnergy(*b), b->mass * scene_.world.g * b->position.y);
    r.text("momentum %8.3f %8.3f %8.3f", b->mass * b->velocity.x, b->mass * b->velocity.y, b->mass * b->velocity.z);
}

void Editor3::drawLinkInspector(LinkSpec& l, float x, float y, float w) {
    auto nameOf = [&](const Endpoint& e) -> std::string {
        const ObjectSpec* o = e.obj >= 0 ? scene_.object(e.obj) : nullptr;
        return o ? o->name : "fixed point";
    };
    char sub[96];
    std::snprintf(sub, sizeof sub, "%s  to  %s", nameOf(l.b).c_str(), nameOf(l.a).c_str());
    Rows r{overlay_.ui(), x, y, w, !locked()};
    r.header(kLinkNames[int(l.kind)], sub);
    const int id = 500000 + l.id * 32;
    Ui& ui = overlay_.ui();

    float shown = l.length >= 0.0f ? l.length : scene_.initialLength(l);
    auto lengthRow = [&](const char* name) {
        if (r.one(name, id, shown, 0.1f)) l.length = std::max(shown, 0.0f);
        if (!locked() && ui.button({x, r.y, 160, 24}, l.length < 0.0f ? "auto: as placed" : "use placed length", l.length < 0.0f,
                                   1.0f)) {
            l.length = -1.0f;
            dirty_ = true;
        }
        r.y += 32.0f;
    };
    switch (l.kind) {
    case LinkKind::Rod: lengthRow("Length m"); break;
    case LinkKind::Rope: lengthRow("Max length m"); break;
    case LinkKind::Spring:
        r.two("k N/m, damping", id + 1, l.k, l.dampingRatio, 5.0f);
        lengthRow("Natural length");
        break;
    case LinkKind::BallJoint:
        r.one("Swing limit deg", id + 3, l.swingLimitDeg, 5.0f);
        r.note("0 swings freely. A limit keeps the object within that angle of how it was placed, like a neck.");
        break;
    case LinkKind::Hinge: r.three("Axis x, y, z", id + 4, l.axis.x, l.axis.y, l.axis.z, 0.1f); break;
    }
    if (l.a.obj < 0) r.three(l.kind == LinkKind::BallJoint || l.kind == LinkKind::Hinge ? "Pivot m" : "Fixed point m", id + 8,
                             l.a.local.x, l.a.local.y, l.a.local.z, 0.1f);
    if (r.changed) {
        l.sanitize();
        dirty_ = true;
    }
    if (!locked()) {
        if (ui.button({x, r.y, 90, 26}, "Delete", false, 1.0f)) {
            deleteSelection();
            return;
        }
        r.y += 34.0f;
    }

    int idx = -1;
    for (size_t i = 0; i < sim_.scene().links.size(); ++i)
        if (sim_.scene().links[i].id == l.id) idx = static_cast<int>(i);
    if (!started_ || idx < 0 || idx >= static_cast<int>(sim_.built().joints.size())) return;
    const Joint* j = sim_.built().joints[idx];
    r.section("Live");
    const float len = (j->anchorB() - j->anchorA()).length();
    r.text("anchor distance %8.3f m", len);
    if (auto* d = dynamic_cast<const DistanceJoint*>(j)) {
        r.text("stretch %8.4f m", len - d->length);
        if (l.kind == LinkKind::Spring) r.text("spring force kx %9.3f N", l.k * (len - d->length));
    } else if (auto* h = dynamic_cast<const HingeJoint*>(j)) {
        r.text("hinge angle %8.2f deg", h->angle() / kDeg);
    } else if (auto* b = dynamic_cast<const BallJoint*>(j); b && b->enableCone) {
        r.text("swing %8.2f deg of %.1f", b->swingAngle() / kDeg, b->coneAngle / kDeg);
    }
}

void Editor3::drawWorldInspector(float x, float y, float w) {
    WorldSpec& ws = scene_.world;
    Rows r{overlay_.ui(), x, y, w, !locked()};
    r.header("World", "nothing selected: scene settings");
    const int id = 900000;
    r.one("g m/s^2", id, ws.g, 0.1f);
    r.toggles({{"Ground", &ws.ground}});
    if (ws.ground) r.one("Ground friction", id + 1, ws.groundFriction, 0.05f);
    float substeps = static_cast<float>(ws.substeps);
    if (r.one("Substeps", id + 2, substeps, 1.0f)) ws.substeps = static_cast<int>(std::lround(substeps));
    r.enabled = true; // these can change mid-run
    r.one("Time scale", id + 3, ws.timeScale, 0.1f);
    r.one("Stop at t s", id + 4, ws.stopAt, 0.5f);
    if (r.changed) {
        ws.sanitize();
        if (!started_) dirty_ = true;
    }
    if (started_) {
        r.section("System");
        float ke = 0.0f;
        Vec3 p;
        for (const Body* b : sim_.built().bodies) {
            if (b->isStatic) continue;
            ke += kineticEnergy(*b);
            p += b->velocity * b->mass;
        }
        r.text("kinetic energy %10.3f J", ke);
        r.text("total energy   %10.3f J", sim_.totalEnergy());
        r.text("momentum %8.3f %8.3f %8.3f", p.x, p.y, p.z);
    }
    r.y += 8.0f;
    r.note("Tools: click the floor or an object to place a Box or Ball. Link tools: click an object, then another "
           "object, the floor, or empty sky for a fixed point above. Shift attaches at the exact spot clicked.");
    r.note("Values accept expressions: 10*cos(37), sqrt(2*9.81*5). Angles are in degrees.");
}

void Editor3::drawProblemPanel(float x, float y, float w) {
    Ui& ui = overlay_.ui();
    const Problem& p = *problemDef();
    ui.label(x, y, p.topic, theme::accentHover, 1.0f);
    y += 18.0f;
    ui.label(x, y, "Question", theme::text, 1.25f);
    y += 18.0f;
    y += ui.paragraph(x, y, w, p.question, theme::textDim, 1.0f, 5.0f) + 6.0f;

    if (ui.button({x, y, 120.0f, 26.0f}, started_ && !paused_ ? "Pause" : "Run", started_ && !paused_, 1.0f)) togglePlay();
    if (ui.button({x + 128.0f, y, 90.0f, 26.0f}, "Restart", false, 1.0f)) reset();
    y += 36.0f;

    ui.label(x, y, run_.done ? "Answer from the simulation" : "Measuring", theme::text, 1.25f);
    y += 18.0f;
    if (run_.lines.empty()) {
        ui.label(x, y, "Press Run (or SPACE)", theme::textFaint, 1.0f);
        y += 16.0f;
    }
    for (const auto& line : run_.lines) {
        ui.label(x, y, line.c_str(), run_.done ? theme::accentHover : theme::text, 1.0f);
        y += 16.0f;
    }
    if (run_.done) {
        char buf[96];
        if (p.expected != 0.0f)
            std::snprintf(buf, sizeof buf, "textbook %.4g %s  (sim %+.2f%%)", p.expected, p.unit,
                          100.0 * (run_.result - p.expected) / p.expected);
        else
            std::snprintf(buf, sizeof buf, "textbook %.4g %s", p.expected, p.unit);
        ui.label(x, y + 4.0f, buf, theme::textDim, 1.0f);
        y += 20.0f;
    }
    y += 10.0f;
    if (ui.button({x, y, 150.0f, 26.0f}, showAnswer_ ? "Hide solution" : "Show solution", showAnswer_, 1.0f)) showAnswer_ = !showAnswer_;
    if (ui.button({x + 158.0f, y, 150.0f, 26.0f}, "Open in 3D editor", false, 1.0f)) {
        problem_ = -1; // keep the scene, unlock it
        custom_.reset();
        reset();
        setStatus("Problem opened in the 3D editor: change anything and press Play");
        return;
    }
    y += 34.0f;
    if (showAnswer_) ui.paragraph(x, y, w, p.answer, theme::warning, 1.0f, 5.0f);
}

void Editor3::drawGraph() {
    Ui& ui = overlay_.ui();
    Renderer& gfx = overlay_.gfx();
    const float W = float(winW_) - kInspW, H = float(winH_);
    const float y0 = H - kHint - kGraphH;
    ui.panel({0, y0, W, kGraphH}, {22, 25, 32, 240});
    gfx.fillRect({0, y0, W, 1.0f}, theme::border);

    float x = 12.0f;
    for (int i = 0; i < static_cast<int>(std::size(kPlotNames)); ++i) {
        const float w = Renderer::textWidth(kPlotNames[i], 1.0f) + 16.0f;
        if (ui.button({x, y0 + 8, w, 22}, kPlotNames[i], plot_ == static_cast<Plot>(i), 1.0f)) plot_ = static_cast<Plot>(i);
        x += w + 4.0f;
    }

    const SDL_FRect area{58.0f, y0 + 40.0f, W - 80.0f, kGraphH - 62.0f};
    std::vector<std::pair<float, float>> pts;
    std::string title;
    if (plot_ == Plot::Energy) {
        pts.assign(energy_.begin(), energy_.end());
        title = "total energy (KE + mgy) J";
    } else if (const ObjectSpec* o = selObj_ ? scene_.object(selObj_) : nullptr) {
        if (auto it = samples_.find(o->id); it != samples_.end()) {
            pts.reserve(it->second.size());
            for (const Sample& s : it->second) {
                float v = 0.0f;
                switch (plot_) {
                case Plot::Y: v = s.p.y; break;
                case Plot::Speed: v = s.v.length(); break;
                case Plot::Vx: v = s.v.x; break;
                case Plot::Vy: v = s.v.y; break;
                case Plot::Vz: v = s.v.z; break;
                case Plot::Accel: v = s.a.length(); break;
                default: v = s.ke; break;
                }
                pts.push_back({s.t, v});
            }
        }
        title = std::string(kPlotNames[int(plot_)]) + " of " + o->name;
    }
    ui.label(x + 16.0f, y0 + 13.0f, title.c_str(), theme::textDim, 1.0f);

    if (pts.size() < 2) {
        ui.labelCentered(area.x + area.w * 0.5f, area.y + area.h * 0.5f - 6.0f,
                         plot_ == Plot::Energy ? "Press Play to record" : "Select an object and press Play to graph it",
                         theme::textFaint, 1.25f);
        return;
    }
    float lo = pts[0].second, hi = lo;
    for (auto [t, v] : pts) lo = std::min(lo, v), hi = std::max(hi, v);
    if (hi - lo < 1e-3f) lo -= 0.5f, hi += 0.5f;
    const float pad = (hi - lo) * 0.08f;
    lo -= pad, hi += pad;
    const float t0 = pts.front().first, t1 = std::max(pts.back().first, t0 + 1e-3f);
    auto sx = [&](float t) { return area.x + (t - t0) / (t1 - t0) * area.w; };
    auto sy = [&](float v) { return area.y + area.h - (v - lo) / (hi - lo) * area.h; };

    gfx.strokeRect(area, theme::border);
    if (lo < 0.0f && hi > 0.0f) gfx.screenLine(area.x, sy(0.0f), area.x + area.w, sy(0.0f), {70, 76, 92});
    const size_t stride = std::max<size_t>(1, pts.size() / static_cast<size_t>(area.w));
    for (size_t i = stride; i < pts.size(); i += stride)
        gfx.screenLine(sx(pts[i - stride].first), sy(pts[i - stride].second), sx(pts[i].first), sy(pts[i].second), {255, 200, 87});

    char buf[48];
    std::snprintf(buf, sizeof buf, "%.3g", hi);
    gfx.text(6.0f, area.y, buf, theme::textFaint, 1.0f);
    std::snprintf(buf, sizeof buf, "%.3g", lo);
    gfx.text(6.0f, area.y + area.h - 8.0f, buf, theme::textFaint, 1.0f);
    std::snprintf(buf, sizeof buf, "%.2f s", t0);
    gfx.text(area.x, area.y + area.h + 6.0f, buf, theme::textFaint, 1.0f);
    std::snprintf(buf, sizeof buf, "%.2f s", t1);
    gfx.text(area.x + area.w - Renderer::textWidth(buf, 1.0f), area.y + area.h + 6.0f, buf, theme::textFaint, 1.0f);
    std::snprintf(buf, sizeof buf, "now %.4g", pts.back().second);
    gfx.text(area.x + area.w * 0.5f - Renderer::textWidth(buf, 1.0f) * 0.5f, area.y + area.h + 6.0f, buf, {255, 200, 87}, 1.0f);

    if (ui.hovered(area)) {
        const float t = t0 + (mouseX_ - area.x) / area.w * (t1 - t0);
        auto it = std::lower_bound(pts.begin(), pts.end(), t, [](const auto& a, float v) { return a.first < v; });
        if (it != pts.end()) {
            gfx.screenLine(sx(it->first), area.y, sx(it->first), area.y + area.h, theme::border);
            std::snprintf(buf, sizeof buf, "t %.3f  %.4g", it->first, it->second);
            gfx.text(sx(it->first) + 6.0f, area.y + 4.0f, buf, theme::text, 1.0f);
        }
    }
}

void Editor3::drawHints() {
    Ui& ui = overlay_.ui();
    const float W = float(winW_), H = float(winH_);
    ui.panel({0, H - kHint, W, kHint});
    const char* hint = "";
    if (started_) {
        hint = "SPACE play/pause   . step   R reset   LMB drag a body to pull it   click to inspect   RMB pan   wheel zoom";
    } else if (problemDef()) {
        hint = "SPACE run   R restart   click an object to graph it   LMB drag orbit   RMB pan   wheel zoom   ESC back";
    } else {
        switch (tool_) {
        case Tool::Select:
            hint = "LMB select / drag selected to move (Shift lifts)   LMB drag empty: orbit   RMB pan   Del delete   Ctrl+D "
                   "duplicate   F focus   SPACE play";
            break;
        case Tool::Box:
        case Tool::Ball: hint = "Click the floor or an object to place it   ESC cancel"; break;
        default:
            hint = pending_.empty() ? "Click the object to link (Shift: attach at the exact spot)"
                                    : "Click another object, the floor, or empty sky for a fixed point   ESC cancel";
            break;
        }
    }
    ui.label(12.0f, H - kHint + (kHint - Renderer::textHeight(1.0f)) * 0.5f, hint, theme::textFaint, 1.0f);
}

} // namespace view3d
