#include "editor/Layout.h"
#include "WorldDraw.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace editor {

using namespace pe;
using namespace layout;

namespace {

constexpr float kDt = Simulation::kDt;
constexpr float kVelScale = 0.3f;   // arrow metres per m/s
constexpr float kAccScale = 0.15f;  // per m/s^2
constexpr float kForceScale = 0.1f; // per N
constexpr size_t kMaxSamples = 12000;
constexpr Color kVelColor{110, 220, 120}, kAccColor{245, 160, 70}, kForceColor{235, 90, 90};

float snap(float v, float step) { return std::round(v / step) * step; }
bool ctrlHeld() { return SDL_GetModState() & SDL_KMOD_CTRL; }
bool shiftHeld() { return SDL_GetModState() & SDL_KMOD_SHIFT; }

float segmentDistance(Vec2 p, Vec2 a, Vec2 b) {
    Vec2 ab = b - a;
    float t = ab.lengthSq() > 0.0f ? std::clamp(dot(p - a, ab) / ab.lengthSq(), 0.0f, 1.0f) : 0.0f;
    return (p - (a + ab * t)).length();
}

} // namespace

Editor::Editor(Renderer& gfx, Ui& ui, Camera& cam) : gfx_(gfx), ui_(ui), cam_(cam), scene_(defaultScene()) {}

void Editor::enter() {
    cam_.center = {0.0f, 4.0f};
    cam_.scale = 50.0f;
    tool_ = Tool::Select;
    pending_.clear();
    drag_ = Drag::None;
    lHeld_ = rHeld_ = mHeld_ = false;
}

void Editor::setStatus(std::string s) {
    status_ = std::move(s);
    statusTime_ = 4.0f;
}

// ---- simulation ----

void Editor::rebuild() {
    sim_.load(scene_);
    dirty_ = false;
}

void Editor::start() {
    rebuild();
    started_ = true;
    accumulator_ = 0.0f;
    samples_.clear();
    energy_.clear();
    exampleRun_ = {};
    pending_.clear();
    tool_ = Tool::Select;
}

void Editor::reset() {
    started_ = false;
    paused_ = true;
    accumulator_ = 0.0f;
    samples_.clear();
    energy_.clear();
    exampleRun_ = {};
    dirty_ = true;
}

void Editor::stepOnce() {
    if (!started_) start();
    paused_ = true;
    stepOnce_ = true;
}

void Editor::saveScene() { setStatus(scene_.save(kSavePath) ? "Saved to editor_scene.txt" : "Save failed"); }

void Editor::loadScene() {
    if (!scene_.load(kSavePath)) {
        setStatus("No editor_scene.txt to load");
        return;
    }
    example_ = -1;
    reset();
    select(0, 0);
    setStatus("Loaded editor_scene.txt");
}

void Editor::newScene() {
    scene_ = defaultScene();
    example_ = -1;
    reset();
    select(0, 0);
}

void Editor::loadExample(int index) {
    const Example& ex = examples()[index];
    scene_ = ex.setup();
    example_ = index;
    showAnswer_ = false;
    reset();
    tool_ = Tool::Select;
    pending_.clear();
    cam_.center = ex.cameraCenter;
    cam_.scale = ex.cameraScale;
    showTrails_ = showGraph_ = true;
    plot_ = static_cast<Plot>(ex.plot);
    select(0, 0);
    for (const auto& o : scene_.objects)
        if (o.name == ex.focus) select(o.id, 0);
}

void Editor::togglePlay() {
    if (!started_) start();
    paused_ = !paused_;
}

void Editor::simulate(float dt) {
    if (!started_) return;
    if (paused_) {
        if (stepOnce_) stepWorld();
        stepOnce_ = false;
        return;
    }
    accumulator_ = std::min(accumulator_ + dt * std::max(scene_.world.timeScale, 0.0f), 0.25f);
    while (accumulator_ >= kDt && !paused_) {
        stepWorld();
        accumulator_ -= kDt;
    }
}

void Editor::stepWorld() {
    sim_.step();
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

    if (example_ >= 0 && examples()[example_].monitor(sim_, exampleRun_)) {
        paused_ = true;
        accumulator_ = 0.0f;
        setStatus("Example: the asked-for moment was reached");
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

// ---- picking and editing ----

int Editor::objectAt(Vec2 p) const {
    for (int i = static_cast<int>(sim_.size()) - 1; i >= 0; --i)
        if (sim_.body(i)->contains(p)) return sim_.scene().objects[i].id;
    return 0;
}

std::vector<std::pair<Vec2, Vec2>> Editor::linkSegments(int i) const {
    std::vector<std::pair<Vec2, Vec2>> segs;
    const Built& built = sim_.built();
    if (i < 0 || i >= static_cast<int>(built.joints.size())) return segs;
    if (int s = built.springIndex[i]; s >= 0) {
        const auto& sp = built.springs->springs[s];
        segs.push_back({built.springs->endA(sp), built.springs->endB(sp)});
    } else if (auto* p = dynamic_cast<const PulleyJoint*>(built.joints[i])) {
        segs.push_back({p->groundA, p->anchorA()});
        segs.push_back({p->groundB, p->anchorB()});
    } else if (const Joint* j = built.joints[i]) {
        segs.push_back({j->anchorA(), j->anchorB()});
    }
    return segs;
}

int Editor::linkAt(Vec2 p) const {
    const float tol = 8.0f / cam_.scale;
    for (size_t i = 0; i < sim_.scene().links.size(); ++i) {
        for (auto [a, b] : linkSegments(static_cast<int>(i)))
            if (segmentDistance(p, a, b) < tol || ((a - b).length() < tol && (p - a).length() < tol * 1.5f))
                return sim_.scene().links[i].id;
        const auto& l = sim_.scene().links[i];
        if (l.kind == LinkKind::Pulley && std::abs((p - l.wheel).length() - l.wheelRadius) < tol) return l.id;
    }
    return 0;
}

void Editor::select(int obj, int link) {
    selObj_ = obj;
    selLink_ = link;
}

void Editor::place(Vec2 p) {
    ObjectSpec o;
    o.id = scene_.nextId++;
    o.name = scene_.nextName();
    o.friction = 0.3f;
    p = ctrlHeld() ? p : Vec2{snap(p.x, 0.05f), snap(p.y, 0.05f)};
    switch (tool_) {
    case Tool::Ball: o.kind = ObjKind::Ball; break;
    case Tool::Wedge: {
        o.kind = ObjKind::Wedge;
        o.fixed = true;
        o.friction = 0.0f;
        o.mass = 10.0f;
        p.y += o.base * std::tan(o.slopeDeg * kDeg) / 3.0f; // click sets the bottom edge
        break;
    }
    case Tool::Ground:
        o.name = "Ground";
        o.width = 20.0f;
        o.fixed = true;
        o.friction = 1.0f;
        p.y -= 0.5f; // click sets the top surface
        break;
    default: break;
    }
    o.pos = p;
    scene_.objects.push_back(o);
    select(o.id, 0);
    tool_ = Tool::Select;
    dirty_ = true;
}

void Editor::clickLinkTool(Vec2 p) {
    const int obj = objectAt(p);
    auto anchor = [&](int id) {
        if (id == 0) return Endpoint{-1, ctrlHeld() ? p : Vec2{snap(p.x, 0.05f), snap(p.y, 0.05f)}};
        return shiftHeld() ? scene_.endpointAt(id, p) : Endpoint{id, {}}; // centre unless shift
    };

    LinkSpec l;
    if (tool_ == Tool::Hinge) {
        if (!obj) {
            setStatus("Click on an object to hinge it");
            return;
        }
        int other = 0;
        for (int i = static_cast<int>(sim_.size()) - 1; i >= 0; --i)
            if (sim_.scene().objects[i].id != obj && sim_.body(i)->contains(p)) {
                other = sim_.scene().objects[i].id;
                break;
            }
        l.kind = LinkKind::Hinge;
        l.a = scene_.endpointAt(other ? other : -1, p);
        l.b = scene_.endpointAt(obj, p);
    } else if (tool_ == Tool::Pulley) {
        if (pending_.size() < 2) {
            if (!obj) {
                setStatus(pending_.empty() ? "Pulley: click the first hanging object" : "Pulley: click the second object");
                return;
            }
            pending_.push_back({obj, {}});
            return;
        }
        l.kind = LinkKind::Pulley;
        l.a = pending_[0];
        l.b = pending_[1];
        l.wheel = ctrlHeld() ? p : Vec2{snap(p.x, 0.05f), snap(p.y, 0.05f)};
    } else {
        if (pending_.empty()) {
            pending_.push_back(anchor(obj));
            return;
        }
        Endpoint b = anchor(obj);
        if (b.obj == pending_[0].obj && (b.obj >= 0 || (b.local - pending_[0].local).lengthSq() < 1e-6f)) return;
        l.kind = tool_ == Tool::Spring ? LinkKind::Spring : tool_ == Tool::Rod ? LinkKind::Rod : LinkKind::Rope;
        l.a = pending_[0];
        l.b = b;
    }
    l.id = scene_.nextId++;
    scene_.links.push_back(l);
    pending_.clear();
    select(0, l.id);
    tool_ = Tool::Select;
    dirty_ = true;
}

void Editor::deleteSelection() {
    if (started_) return;
    if (selObj_) scene_.removeObject(selObj_);
    if (selLink_) scene_.removeLink(selLink_);
    select(0, 0);
    dirty_ = true;
}

void Editor::duplicateSelection() {
    const ObjectSpec* src = selObj_ ? scene_.object(selObj_) : nullptr;
    if (started_ || !src) return;
    ObjectSpec o = *src;
    o.id = scene_.nextId++;
    if (!o.fixed) o.name = scene_.nextName();
    o.pos.x += o.kind == ObjKind::Ball ? o.radius * 2.0f + 0.2f : o.kind == ObjKind::Block ? o.width + 0.2f : o.base + 0.2f;
    scene_.objects.push_back(o);
    select(o.id, 0);
    dirty_ = true;
}

// ---- input ----

void Editor::handleEvent(const SDL_Event& e) {
    switch (e.type) {
    case SDL_EVENT_MOUSE_MOTION: mouse_ = {e.motion.x, e.motion.y}; break;
    case SDL_EVENT_MOUSE_BUTTON_DOWN:
        mouse_ = {e.button.x, e.button.y};
        if (e.button.button == SDL_BUTTON_LEFT) lPress_ = lHeld_ = true;
        if (e.button.button == SDL_BUTTON_RIGHT) rPress_ = rHeld_ = true;
        if (e.button.button == SDL_BUTTON_MIDDLE) mPress_ = mHeld_ = true;
        break;
    case SDL_EVENT_MOUSE_BUTTON_UP:
        if (e.button.button == SDL_BUTTON_LEFT) {
            lHeld_ = false;
            lRelease_ = true;
        }
        if (e.button.button == SDL_BUTTON_RIGHT) rHeld_ = false;
        if (e.button.button == SDL_BUTTON_MIDDLE) mHeld_ = false;
        break;
    case SDL_EVENT_MOUSE_WHEEL:
        if (ui_.overUi()) {
            ui_.scroll(e.wheel.y);
        } else {
            Vec2 before = cam_.toWorld(mouse_);
            cam_.scale = std::clamp(cam_.scale * (e.wheel.y > 0 ? 1.1f : 1.0f / 1.1f), 2.0f, 2000.0f);
            cam_.center += before - cam_.toWorld(mouse_);
        }
        break;
    case SDL_EVENT_KEY_DOWN: {
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
        } else if (key == SDLK_S && ctrl) {
            saveScene();
        } else if (key == SDLK_O && ctrl) {
            loadScene();
        } else if (key == SDLK_D && ctrl) {
            duplicateSelection();
        } else if (key == SDLK_PERIOD) {
            stepOnce();
        } else if (key == SDLK_DELETE || key == SDLK_BACKSPACE) {
            deleteSelection();
        } else if ((key == SDLK_Q || key == SDLK_E) && selObj_ && !started_) {
            if (ObjectSpec* o = scene_.object(selObj_)) {
                o->angleDeg += key == SDLK_Q ? 5.0f : -5.0f;
                dirty_ = true;
            }
        } else if (key == SDLK_G) {
            showGraph_ = !showGraph_;
        } else if (key >= SDLK_1 && key <= SDLK_9 && !started_) {
            tool_ = static_cast<Tool>(key - SDLK_1);
            pending_.clear();
        } else if (key == SDLK_0 && !started_) {
            tool_ = Tool::Pulley;
            pending_.clear();
        } else if (key == SDLK_LEFT) cam_.center.x -= 1.0f;
        else if (key == SDLK_RIGHT) cam_.center.x += 1.0f;
        else if (key == SDLK_UP) cam_.center.y += 1.0f;
        else if (key == SDLK_DOWN) cam_.center.y -= 1.0f;
        break;
    }
    }
}

void Editor::handleCanvas() {
    const Vec2 p = cam_.toWorld(mouse_);
    if (mPress_) {
        drag_ = Drag::Pan;
        panAnchor_ = p;
        return;
    }
    if (rPress_) {
        const int obj = started_ ? 0 : objectAt(p);
        const ObjectSpec* o = obj ? scene_.object(obj) : nullptr;
        if (o && !o->fixed) {
            select(obj, 0);
            drag_ = Drag::Velocity;
        } else {
            drag_ = Drag::Pan;
            panAnchor_ = p;
        }
        return;
    }
    if (!lPress_) return;

    if (started_ || tool_ == Tool::Select) {
        if (!started_ && selLink_) {
            const LinkSpec* l = scene_.link(selLink_);
            const float tol = 10.0f / cam_.scale;
            if (l && l->kind == LinkKind::Pulley && (p - l->wheel).length() < l->wheelRadius + tol) {
                drag_ = Drag::Wheel;
                return;
            }
            if (l && l->a.obj < 0 && (p - l->a.local).length() < tol) {
                drag_ = Drag::EndA;
                return;
            }
            if (l && l->b.obj < 0 && (p - l->b.local).length() < tol) {
                drag_ = Drag::EndB;
                return;
            }
        }
        if (int obj = objectAt(p)) {
            select(obj, 0);
            if (!started_) {
                drag_ = Drag::Object;
                pressAt_ = mouse_;
                dragMoved_ = false;
                const Body* b = sim_.body(sim_.indexOf(obj));
                dragOffset_ = b->position - b->aabb.min; // snapping keeps the bounding box on the grid
                panAnchor_ = p - b->aabb.min;
            }
            return;
        }
        select(0, linkAt(p));
        return;
    }
    if (tool_ <= Tool::Ground) place(p);
    else clickLinkTool(p);
}

void Editor::updateDrag() {
    if (drag_ == Drag::None) return;
    const Vec2 p = cam_.toWorld(mouse_);
    const bool free = ctrlHeld();
    auto snapped = [&](Vec2 v) { return free ? v : Vec2{snap(v.x, 0.05f), snap(v.y, 0.05f)}; };

    switch (drag_) {
    case Drag::Pan:
        cam_.center += panAnchor_ - p;
        if (!mHeld_ && !rHeld_) drag_ = Drag::None;
        return;
    case Drag::Velocity:
        if (ObjectSpec* o = scene_.object(selObj_)) {
            Vec2 v = (p - o->pos) / kVelScale;
            o->velocity = free ? v : Vec2{snap(v.x, 0.1f), snap(v.y, 0.1f)};
            dirty_ = true;
        }
        if (!rHeld_) drag_ = Drag::None;
        return;
    case Drag::Object:
        dragMoved_ = dragMoved_ || (mouse_ - pressAt_).lengthSq() > 16.0f; // a plain click must not snap it
        if (ObjectSpec* o = dragMoved_ ? scene_.object(selObj_) : nullptr) {
            Vec2 corner = snapped(p - panAnchor_);
            if (corner.x != o->pos.x - dragOffset_.x || corner.y != o->pos.y - dragOffset_.y) {
                o->pos = corner + dragOffset_;
                dirty_ = true;
            }
        }
        break;
    case Drag::EndA:
    case Drag::EndB:
    case Drag::Wheel:
        if (LinkSpec* l = scene_.link(selLink_)) {
            Vec2& target = drag_ == Drag::Wheel ? l->wheel : drag_ == Drag::EndA ? l->a.local : l->b.local;
            target = snapped(p);
            dirty_ = true;
        }
        break;
    default: break;
    }
    if (!lHeld_) drag_ = Drag::None;
}

// ---- frame ----

void Editor::frame(float dt) {
    cam_.insetTop = kTop + kBar;
    cam_.insetBottom = kHint + (showGraph_ ? kGraphH : 0.0f);
    statusTime_ -= dt;

    updateDrag();
    if (dirty_ && !started_) rebuild();
    simulate(dt);

    drawGrid();
    drawFieldOverlay();
    if (showTrails_) drawTrails();
    drawWorld(gfx_, sim_.world(), false);
    drawOverlays();

    drawToolbar();
    drawInspector();
    if (showGraph_) drawGraph();
    drawHints();

    if (!ui_.overUi() && drag_ == Drag::None) handleCanvas();
    lPress_ = lRelease_ = rPress_ = mPress_ = false;
}

// ---- drawing ----

void Editor::drawGrid() {
    if (!showGrid_) return;
    const Vec2 lo = cam_.toWorld({0.0f, float(cam_.height)}), hi = cam_.toWorld({float(cam_.width), 0.0f});
    const float step = cam_.scale >= 25.0f ? 1.0f : cam_.scale >= 6.0f ? 5.0f : 25.0f;
    for (float x = std::floor(lo.x / step) * step; x <= hi.x; x += step) {
        const bool major = std::fmod(std::abs(x) + 0.01f, step * 5.0f) < 0.02f;
        Color c = std::abs(x) < 1e-3f ? Color{80, 88, 108} : major ? Color{44, 49, 62} : Color{30, 34, 43};
        gfx_.line({x, lo.y}, {x, hi.y}, c);
    }
    for (float y = std::floor(lo.y / step) * step; y <= hi.y; y += step) {
        const bool major = std::fmod(std::abs(y) + 0.01f, step * 5.0f) < 0.02f;
        Color c = std::abs(y) < 1e-3f ? Color{80, 88, 108} : major ? Color{44, 49, 62} : Color{30, 34, 43};
        gfx_.line({lo.x, y}, {hi.x, y}, c);
    }
    // metre marks along the axes
    const float labelStep = step * (cam_.scale * step >= 60.0f ? 1.0f : 5.0f);
    char buf[16];
    Vec2 origin = cam_.toScreen({0.0f, 0.0f});
    float ly = std::clamp(origin.y + 4.0f, kTop + kBar + 4.0f, float(cam_.height) - cam_.insetBottom - 14.0f);
    float lx = std::clamp(origin.x + 4.0f, 4.0f, float(cam_.width) - kInspW - 40.0f);
    for (float x = std::floor(lo.x / labelStep) * labelStep; x <= hi.x; x += labelStep) {
        if (std::abs(x) < 1e-3f) continue;
        std::snprintf(buf, sizeof buf, "%g", x);
        gfx_.text(cam_.toScreen({x, 0.0f}).x + 3.0f, ly, buf, theme::textFaint, 1.0f);
    }
    for (float y = std::floor(lo.y / labelStep) * labelStep; y <= hi.y; y += labelStep) {
        if (std::abs(y) < 1e-3f) continue;
        std::snprintf(buf, sizeof buf, "%g", y);
        gfx_.text(lx, cam_.toScreen({0.0f, y}).y + 3.0f, buf, theme::textFaint, 1.0f);
    }
}

void Editor::drawFieldOverlay() {
    const WorldSpec& ws = scene_.world;
    const Vec2 lo = cam_.toWorld({0.0f, float(cam_.height)}), hi = cam_.toWorld({float(cam_.width), 0.0f});
    if (ws.B != 0.0f) {
        const float step = std::max(1.0f, std::round(80.0f / cam_.scale));
        const float s = 4.0f / cam_.scale;
        for (float x = std::ceil(lo.x / step) * step; x <= hi.x; x += step)
            for (float y = std::ceil(lo.y / step) * step; y <= hi.y; y += step) {
                Color c{150, 120, 230, 110};
                if (ws.B > 0.0f) {
                    gfx_.point({x, y}, 3.0f, c);
                } else {
                    gfx_.line({x - s, y - s}, {x + s, y + s}, c);
                    gfx_.line({x - s, y + s}, {x + s, y - s}, c);
                }
            }
    }
    if (ws.E.lengthSq() > 0.0f) {
        const float step = std::max(1.0f, std::round(120.0f / cam_.scale));
        Vec2 dir = ws.E.normalized() * (18.0f / cam_.scale);
        for (float x = std::ceil(lo.x / step) * step; x <= hi.x; x += step)
            for (float y = std::ceil(lo.y / step) * step; y <= hi.y; y += step) {
                const Vec2 o{x, y};
                Color c{240, 200, 90, 80};
                gfx_.line(o - dir, o + dir, c);
                gfx_.line(o + dir, o + dir * 0.4f + dir.perp() * 0.4f, c);
                gfx_.line(o + dir, o + dir * 0.4f - dir.perp() * 0.4f, c);
            }
    }
}

void Editor::drawTrails() {
    for (size_t i = 0; i < sim_.size(); ++i) {
        auto it = samples_.find(sim_.scene().objects[i].id);
        if (it == samples_.end() || it->second.size() < 2) continue;
        const auto& s = it->second;
        Color c = bodyColor(*sim_.body(static_cast<int>(i)));
        const size_t stride = 2, n = s.size();
        for (size_t k = stride; k < n; k += stride) {
            c.a = Uint8(40 + 180.0f * k / n);
            gfx_.line(s[k - stride].p, s[k].p, c);
        }
    }
}

void Editor::drawArrow(Vec2 a, Vec2 b, Color c) {
    Vec2 d = b - a;
    float len = d.length();
    if (len * cam_.scale < 3.0f) return;
    Vec2 u = d / len;
    float head = std::min(10.0f / cam_.scale, len * 0.5f);
    Vec2 n = u.perp() * (1.0f / cam_.scale);
    gfx_.line(a, b, c);
    gfx_.line(a + n, b + n, c);
    gfx_.line(b, b - u * head + u.perp() * head * 0.5f, c);
    gfx_.line(b, b - u * head - u.perp() * head * 0.5f, c);
}

void Editor::drawOverlays() {
    const Color accent = theme::accentHover;
    for (int i = 0; i < static_cast<int>(sim_.size()); ++i) {
        const Body* b = sim_.body(i);
        const ObjectSpec& o = sim_.scene().objects[i];
        if (o.id == selObj_) {
            if (b->shape.type == ShapeType::Circle)
                gfx_.circle(b->position, b->shape.radius + 2.0f / cam_.scale, b->angle, {0, 0, 0, 0}, accent);
            else
                gfx_.polygon(b->worldVertices, {255, 255, 255, 18}, accent);
        }
        if (!b->isStatic) {
            if (showVel_) drawArrow(b->position, b->position + b->velocity * kVelScale, kVelColor);
            if (showAcc_ && started_) drawArrow(b->position, b->position + sim_.acceleration(i) * kAccScale, kAccColor);
            if (showForce_) {
                Vec2 f = o.force + (o.holdVelocity ? o.velocity.normalized() * sim_.driveForce(i) : Vec2{});
                drawArrow(b->position, b->position + f * kForceScale, kForceColor);
            }
        }
        if (showLabels_) {
            Vec2 s = cam_.toScreen(b->position);
            const float top = b->aabb.max.y;
            float ty = std::min(s.y - 6.0f, cam_.toScreen({0.0f, top}).y - 16.0f);
            if (o.fixed) ty = s.y - Renderer::textHeight(1.25f) * 0.5f;
            Color c = o.fixed ? theme::textFaint : theme::text;
            gfx_.text(s.x - Renderer::textWidth(o.name.c_str(), 1.25f) * 0.5f, ty, o.name.c_str(), c, 1.25f);
        }
    }

    const auto& links = sim_.scene().links;
    for (size_t i = 0; i < links.size(); ++i) {
        const LinkSpec& l = links[i];
        if (l.id != selLink_) continue;
        for (auto [a, b] : linkSegments(static_cast<int>(i))) {
            Vec2 n = (b - a).normalized().perp() * (1.5f / cam_.scale);
            gfx_.line(a + n, b + n, accent);
            gfx_.line(a - n, b - n, accent);
        }
        auto handle = [&](Vec2 p) {
            Vec2 s = cam_.toScreen(p);
            gfx_.strokeRect({s.x - 5.0f, s.y - 5.0f, 10.0f, 10.0f}, accent);
        };
        if (!started_) {
            if (l.a.obj < 0) handle(l.a.local);
            if (l.b.obj < 0 && l.kind != LinkKind::Hinge) handle(l.b.local);
            if (l.kind == LinkKind::Pulley) handle(l.wheel);
        }
    }

    // preview of a link being placed
    if (!pending_.empty()) {
        const Vec2 m = cam_.toWorld(mouse_);
        for (const auto& e : pending_) gfx_.line(scene_.endpointWorld(e), m, {200, 200, 120, 160});
    }
}

} // namespace editor
