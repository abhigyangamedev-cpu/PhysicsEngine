#include "editor/Layout.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace editor {

using namespace pe;
using namespace layout;

namespace {

float plotValue(const Sample& s, Plot p) {
    switch (p) {
    case Plot::X: return s.p.x;
    case Plot::Y: return s.p.y;
    case Plot::Vx: return s.v.x;
    case Plot::Vy: return s.v.y;
    case Plot::Speed: return s.v.length();
    case Plot::Ax: return s.a.x;
    case Plot::Ay: return s.a.y;
    case Plot::Accel: return s.a.length();
    default: return s.ke;
    }
}

} // namespace

void Editor::drawToolbar() {
    const float W = float(cam_.width);
    ui_.panel({0, 0, W, kTop});
    if (ui_.button({12, 10, 84, kTop - 20}, "< Home", false, 1.25f)) {
        exit_ = true;
        return;
    }
    float x = 108.0f;
    for (int i = 0; i < static_cast<int>(std::size(kToolNames)); ++i) {
        float w = Renderer::textWidth(kToolNames[i], 1.25f) + 20.0f;
        if (ui_.button({x, 10, w, kTop - 20}, kToolNames[i], tool_ == static_cast<Tool>(i), 1.25f)) {
            if (started_ && i != 0) {
                setStatus("Reset (R) to edit the scene");
            } else {
                tool_ = static_cast<Tool>(i);
                pending_.clear();
            }
        }
        x += w + 6.0f;
    }

    float rx = W - 12.0f;
    auto rightButton = [&](const char* label, bool active) {
        float w = Renderer::textWidth(label, 1.25f) + 24.0f;
        rx -= w;
        bool clicked = ui_.button({rx, 10, w, kTop - 20}, label, active, 1.25f);
        rx -= 6.0f;
        return clicked;
    };
    if (rightButton("Reset", false)) reset();
    if (rightButton("Step", false)) stepOnce();
    if (rightButton(started_ && !paused_ ? "Pause" : "Play", started_ && !paused_)) togglePlay();

    ui_.panel({0, kTop, W, kBar}, {24, 27, 35});
    x = 12.0f;
    auto toggle = [&](const char* label, bool& v) {
        float w = Renderer::textWidth(label, 1.0f) + 18.0f;
        if (ui_.button({x, kTop + 7, w, kBar - 14}, label, v, 1.0f)) v = !v;
        x += w + 5.0f;
    };
    toggle("Grid", showGrid_);
    toggle("Labels", showLabels_);
    toggle("Velocity", showVel_);
    toggle("Accel", showAcc_);
    toggle("Force", showForce_);
    toggle("Trails", showTrails_);
    toggle("Graph", showGraph_);
    x += 14.0f;
    auto action = [&](const char* label) {
        float w = Renderer::textWidth(label, 1.0f) + 18.0f;
        bool clicked = ui_.button({x, kTop + 7, w, kBar - 14}, label, false, 1.0f);
        x += w + 5.0f;
        return clicked;
    };
    if (action("Save")) saveScene();
    if (action("Load")) loadScene();
    if (action("New")) newScene();

    char buf[64];
    std::snprintf(buf, sizeof buf, "t = %.3f s", sim_.time());
    float tw = Renderer::textWidth(buf, 1.5f);
    ui_.label(W - 12.0f - tw, kTop + (kBar - Renderer::textHeight(1.5f)) * 0.5f, buf,
              started_ ? (paused_ ? theme::warning : theme::accentHover) : theme::textDim, 1.5f);
    if (statusTime_ > 0.0f)
        ui_.label(W - 30.0f - tw - Renderer::textWidth(status_.c_str(), 1.0f), kTop + (kBar - Renderer::textHeight(1.0f)) * 0.5f,
                  status_.c_str(), theme::warning, 1.0f);
}

void Editor::drawInspector() {
    const float W = float(cam_.width), H = float(cam_.height);
    const float x = W - kInspW, y = kTop + kBar;
    ui_.panel({x, y, kInspW, H - y - kHint}, {26, 29, 37, 240});
    gfx_.fillRect({x, y, 1.0f, H - y - kHint}, theme::border);

    static constexpr const char* kTabs[] = {"Inspector", "Standard Examples"};
    float tx = x + 10.0f;
    for (int i = 0; i < 2; ++i) {
        float w = Renderer::textWidth(kTabs[i], 1.0f) + 22.0f;
        if (ui_.button({tx, y + 8.0f, w, 24.0f}, kTabs[i], panelTab_ == i, 1.0f)) panelTab_ = i;
        tx += w + 6.0f;
    }
    gfx_.fillRect({x, y + 38.0f, kInspW, 1.0f}, theme::border);
    const float top = y + 48.0f;
    if (panelTab_ == 1) {
        drawExamples(x + 14.0f, top, kInspW - 28.0f);
        return;
    }

    ObjectSpec* o = selObj_ ? scene_.object(selObj_) : nullptr;
    LinkSpec* l = selLink_ ? scene_.link(selLink_) : nullptr;
    if (o) drawObjectInspector(*o, x + 14.0f, top, kInspW - 28.0f);
    else if (l) drawLinkInspector(*l, x + 14.0f, top, kInspW - 28.0f);
    else drawWorldInspector(x + 14.0f, top, kInspW - 28.0f);
}

namespace {

// Lays out inspector rows: a label, then one or two number fields.
struct Rows {
    Ui& ui;
    float x, y, w;
    bool enabled;
    bool changed = false;
    static constexpr float kRowH = 28.0f, kLabelW = 118.0f;

    void header(const char* title, const char* sub = nullptr) {
        ui.label(x, y, title, theme::text, 1.75f);
        y += Renderer::textHeight(1.75f) + 6.0f;
        if (sub) {
            ui.label(x, y, sub, theme::textFaint, 1.0f);
            y += Renderer::textHeight(1.0f) + 6.0f;
        }
        y += 6.0f;
    }
    void section(const char* title) {
        y += 6.0f;
        ui.label(x, y, title, theme::textDim, 1.25f);
        y += Renderer::textHeight(1.25f) + 8.0f;
    }
    void label(const char* s) { ui.label(x, y + (kRowH - 4.0f - Renderer::textHeight(1.0f)) * 0.5f, s, theme::textDim, 1.0f); }
    bool one(const char* name, int id, float& v, float step) {
        label(name);
        bool c = ui.numberField({x + kLabelW, y, w - kLabelW, kRowH - 4.0f}, id, v, step, enabled);
        y += kRowH;
        changed |= c;
        return c;
    }
    bool two(const char* name, int id, float& a, float& b, float step, bool* first = nullptr) {
        label(name);
        const float fw = (w - kLabelW - 6.0f) * 0.5f;
        bool ca = ui.numberField({x + kLabelW, y, fw, kRowH - 4.0f}, id, a, step, enabled);
        bool cb = ui.numberField({x + kLabelW + fw + 6.0f, y, fw, kRowH - 4.0f}, id + 1, b, step, enabled);
        y += kRowH;
        if (first) *first = ca;
        changed |= ca || cb;
        return ca || cb;
    }
    bool toggles(std::initializer_list<std::pair<const char*, bool*>> items) {
        float tx = x;
        bool any = false;
        for (auto [name, v] : items) {
            float bw = Renderer::textWidth(name, 1.0f) + 18.0f;
            if (ui.button({tx, y, bw, kRowH - 6.0f}, name, *v, 1.0f) && enabled) {
                *v = !*v;
                any = true;
            }
            tx += bw + 6.0f;
        }
        y += kRowH;
        changed |= any;
        return any;
    }
    void text(const char* fmt, double a = 0, double b = 0, double c = 0) {
        char buf[96];
        std::snprintf(buf, sizeof buf, fmt, a, b, c);
        ui.label(x, y, buf, theme::text, 1.0f);
        y += Renderer::textHeight(1.0f) + 7.0f;
    }
    float note(const char* s) {
        float h = ui.paragraph(x, y, w, s, theme::textFaint, 1.0f, 5.0f);
        y += h + 4.0f;
        return h;
    }
};

} // namespace

void Editor::drawObjectInspector(ObjectSpec& o, float x, float y, float w) {
    static constexpr const char* kKinds[] = {"Block", "Ball", "Wedge"};
    char title[64];
    std::snprintf(title, sizeof title, "%s %s", kKinds[int(o.kind)], o.name.c_str());
    Rows r{ui_, x, y, w, !started_};
    r.header(title, started_ ? "initial values (Reset to edit)" : "initial values, type or scroll to change");
    const int id = 1000 + o.id * 64;

    if (o.kind == ObjKind::Wedge) r.toggles({{"Fixed", &o.fixed}, {"Flip", &o.flip}});
    else if (o.fixed) r.toggles({{"Fixed", &o.fixed}, {"No spin", &o.noSpin}});
    else r.toggles({{"Fixed", &o.fixed}, {"No spin", &o.noSpin}, {"Const v", &o.holdVelocity}});

    r.two("Mass kg, q C", id, o.mass, o.charge, 0.5f); // fixed objects still attract and carry charge
    switch (o.kind) {
    case ObjKind::Block: r.two("Size w, h m", id + 2, o.width, o.height, 0.1f); break;
    case ObjKind::Ball: r.one("Radius m", id + 2, o.radius, 0.05f); break;
    case ObjKind::Wedge: r.two("Base m, slope", id + 2, o.base, o.slopeDeg, 1.0f); break;
    }
    r.two("Position x, y", id + 4, o.pos.x, o.pos.y, 0.1f);
    if (!o.fixed) {
        r.two("Tilt deg, w", id + 6, o.angleDeg, o.omega, 5.0f);
        r.two("Velocity vx,vy", id + 8, o.velocity.x, o.velocity.y, 0.5f);
        float speed = o.velocity.length();
        float dir = speed > 0.0f ? std::atan2(o.velocity.y, o.velocity.x) / kDeg : 0.0f;
        bool speedChanged = false;
        if (r.two("Speed, dir deg", id + 10, speed, dir, 1.0f, &speedChanged))
            o.velocity = Vec2{std::cos(dir * kDeg), std::sin(dir * kDeg)} * speed;
        r.two("Force Fx, Fy N", id + 12, o.force.x, o.force.y, 1.0f);
    } else {
        r.one("Tilt deg", id + 6, o.angleDeg, 5.0f);
    }
    r.two("Friction u, e", id + 14, o.friction, o.restitution, 0.05f);
    if (r.changed) {
        o.mass = std::max(o.mass, 1e-3f);
        o.friction = std::max(o.friction, 0.0f);
        o.restitution = std::clamp(o.restitution, 0.0f, 1.0f);
        dirty_ = true;
    }

    if (!started_) {
        r.y += 4.0f;
        if (ui_.button({x, r.y, 110, 26}, "Duplicate", false, 1.0f)) duplicateSelection();
        if (ui_.button({x + 118, r.y, 90, 26}, "Delete", false, 1.0f)) {
            deleteSelection();
            return;
        }
        r.y += 34.0f;
    }

    const int idx = sim_.indexOf(o.id);
    if (!started_ || idx < 0 || o.fixed) return;
    const Body* b = sim_.body(idx);
    const Vec2 acc = sim_.acceleration(idx);
    r.section("Live");
    r.text("position   x %8.3f   y %8.3f  m", b->position.x, b->position.y);
    r.text("velocity  vx %8.3f  vy %8.3f  m/s", b->velocity.x, b->velocity.y);
    r.text("speed  %8.3f m/s   dir %7.2f deg", b->velocity.length(), std::atan2(b->velocity.y, b->velocity.x) / kDeg);
    r.text("accel     ax %8.3f  ay %8.3f", acc.x, acc.y);
    r.text("|a|    %8.3f m/s^2  w %8.3f r/s", acc.length(), b->angularVelocity);
    r.text("KE  %9.3f J    PE (mgy) %9.3f J", kineticEnergy(*b), b->mass * scene_.world.g * b->position.y);
    r.text("momentum   px %8.3f  py %8.3f", b->mass * b->velocity.x, b->mass * b->velocity.y);
    if (o.holdVelocity) r.text("drive force to keep v  %9.3f N", sim_.driveForce(idx));
}

void Editor::drawLinkInspector(LinkSpec& l, float x, float y, float w) {
    auto nameOf = [&](const Endpoint& e) -> std::string {
        const ObjectSpec* o = e.obj >= 0 ? scene_.object(e.obj) : nullptr;
        return o ? o->name : "fixed point";
    };
    char title[96], sub[96];
    std::snprintf(title, sizeof title, "%s", kLinkNames[int(l.kind)]);
    std::snprintf(sub, sizeof sub, "%s  to  %s", nameOf(l.a).c_str(), nameOf(l.b).c_str());
    Rows r{ui_, x, y, w, !started_};
    r.header(title, sub);
    const int id = 500000 + l.id * 32;

    float shown = l.length >= 0.0f ? l.length : scene_.initialLength(l);
    auto lengthRow = [&](const char* name) {
        if (r.one(name, id + 4, shown, 0.1f)) l.length = std::max(shown, 0.0f);
        if (!started_ && ui_.button({x, r.y, 160, 24}, l.length < 0.0f ? "auto: as placed" : "use placed length",
                                    l.length < 0.0f, 1.0f)) {
            l.length = -1.0f;
            dirty_ = true;
        }
        r.y += 32.0f;
    };
    switch (l.kind) {
    case LinkKind::Spring:
        r.two("k N/m, damping", id, l.k, l.damping, 5.0f);
        lengthRow("Natural length");
        break;
    case LinkKind::Rod: lengthRow("Length m"); break;
    case LinkKind::Rope: lengthRow("Max length m"); break;
    case LinkKind::Pulley:
        r.two("Ratio, wheel r", id, l.ratio, l.wheelRadius, 0.1f);
        r.note("Ratio n hangs the second object from n strands: it moves 1/n as far with n times the tension.");
        break;
    case LinkKind::Hinge: r.note("Pins the object so it can only rotate about this point."); break;
    }
    if (r.changed) {
        l.k = std::max(l.k, 0.0f);
        l.ratio = std::max(l.ratio, 0.1f);
        l.wheelRadius = std::max(l.wheelRadius, 0.02f);
        dirty_ = true;
    }
    if (!started_) {
        if (ui_.button({x, r.y, 90, 26}, "Delete", false, 1.0f)) {
            deleteSelection();
            return;
        }
        r.y += 34.0f;
        r.note("Drag the square handles to move fixed points.");
    }

    int idx = -1;
    const Built& built = sim_.built();
    for (size_t i = 0; i < sim_.scene().links.size(); ++i)
        if (sim_.scene().links[i].id == l.id) idx = static_cast<int>(i);
    if (!started_ || idx < 0) return;
    r.section("Live");
    if (int s = built.springIndex[idx]; s >= 0) {
        const auto& sp = built.springs->springs[s];
        float len = (built.springs->endB(sp) - built.springs->endA(sp)).length();
        r.text("length %8.3f m   natural %8.3f m", len, sp.rest);
        r.text("extension x %8.3f m", len - sp.rest);
        r.text("force kx %9.3f N   PE %9.3f J", sp.k * (len - sp.rest), 0.5f * sp.k * (len - sp.rest) * (len - sp.rest));
    } else if (auto* p = dynamic_cast<const PulleyJoint*>(built.joints[idx])) {
        r.text("tension %9.3f N", p->tension());
        r.text("rope lengths %7.3f  %7.3f m", p->lengthA(), p->lengthB());
    } else if (auto* d = dynamic_cast<const DistanceJoint*>(built.joints[idx])) {
        r.text("length %8.3f m", (d->anchorB() - d->anchorA()).length());
    }
}

void Editor::drawWorldInspector(float x, float y, float w) {
    WorldSpec& ws = scene_.world;
    Rows r{ui_, x, y, w, !started_};
    r.header("World", "nothing selected: scene settings");
    const int id = 900000;
    r.one("g m/s^2", id, ws.g, 0.1f);
    r.enabled = true; // these can change mid-run
    r.one("Time scale", id + 1, ws.timeScale, 0.1f);
    r.one("Stop at t s", id + 2, ws.stopAt, 0.5f);
    r.enabled = !started_;
    r.one("Air drag 1/s", id + 3, ws.drag, 0.05f);
    r.two("E field V/m", id + 4, ws.E.x, ws.E.y, 1.0f);
    r.one("Bz T (+out)", id + 6, ws.B, 0.1f);
    r.toggles({{"Coulomb", &ws.coulomb}, {"Mutual gravity", &ws.mutualGravity}});
    if (ws.coulomb) r.one("Coulomb k", id + 7, ws.coulombK, 0.0f);
    if (ws.mutualGravity) r.one("Gravitation G", id + 8, ws.G, 0.1f);
    if (r.changed) {
        ws.timeScale = std::clamp(ws.timeScale, 0.01f, 10.0f);
        ws.stopAt = std::max(ws.stopAt, 0.0f);
        ws.drag = std::max(ws.drag, 0.0f);
        if (!started_) dirty_ = true;
    }

    if (started_) {
        r.section("System");
        float ke = 0.0f;
        Vec2 p;
        for (const Body* b : sim_.built().bodies) {
            if (b->isStatic) continue;
            ke += kineticEnergy(*b);
            p += b->velocity * b->mass;
        }
        r.text("kinetic energy %10.3f J", ke);
        r.text("total energy   %10.3f J", sim_.totalEnergy());
        r.text("momentum  px %8.3f  py %8.3f", p.x, p.y);
    }
    r.y += 8.0f;
    r.note("Values accept expressions: 10*cos(37), sqrt(2*g*5), 2e-6. Angles are in degrees.");
    r.note("Contact friction uses the smaller u of the two surfaces, so ground is u = 1 and the object's u decides.");
}

void Editor::drawExamples(float x, float y, float w) {
    const auto& list = examples();
    ui_.label(x, y, "Pick a problem to load it", theme::textDim, 1.0f);
    y += 18.0f;
    for (int i = 0; i < static_cast<int>(list.size()); ++i) {
        char name[64];
        std::snprintf(name, sizeof name, "%d. %s", i + 1, list[i].name);
        if (ui_.button({x, y, w, 26.0f}, name, i == example_, 1.0f)) loadExample(i);
        y += 32.0f;
    }
    if (example_ < 0) return;

    const Example& ex = list[example_];
    y += 8.0f;
    ui_.label(x, y, "Question", theme::text, 1.25f);
    y += 18.0f;
    y += ui_.paragraph(x, y, w, ex.question, theme::textDim, 1.0f, 5.0f) + 6.0f;

    if (ui_.button({x, y, 120.0f, 26.0f}, started_ && !paused_ ? "Pause" : "Run", started_ && !paused_, 1.0f)) togglePlay();
    if (ui_.button({x + 128.0f, y, 90.0f, 26.0f}, "Restart", false, 1.0f)) reset();
    y += 36.0f;

    ui_.label(x, y, exampleRun_.done ? "Answer from the simulation" : "Measuring", theme::text, 1.25f);
    y += 18.0f;
    if (exampleRun_.lines.empty()) {
        ui_.label(x, y, "Press Run (or SPACE)", theme::textFaint, 1.0f);
        y += 16.0f;
    }
    for (const auto& line : exampleRun_.lines) {
        ui_.label(x, y, line.c_str(), exampleRun_.done ? theme::accentHover : theme::text, 1.0f);
        y += 16.0f;
    }
    y += 10.0f;
    if (ui_.button({x, y, 150.0f, 26.0f}, showAnswer_ ? "Hide solution" : "Show solution", showAnswer_, 1.0f))
        showAnswer_ = !showAnswer_;
    y += 34.0f;
    if (showAnswer_) ui_.paragraph(x, y, w, ex.answer, theme::warning, 1.0f, 5.0f);
}

void Editor::drawGraph() {
    const float W = float(cam_.width) - kInspW, H = float(cam_.height);
    const float y0 = H - kHint - kGraphH;
    ui_.panel({0, y0, W, kGraphH}, {22, 25, 32, 240});
    gfx_.fillRect({0, y0, W, 1.0f}, theme::border);

    float x = 12.0f;
    for (int i = 0; i < static_cast<int>(std::size(kPlotNames)); ++i) {
        float w = Renderer::textWidth(kPlotNames[i], 1.0f) + 16.0f;
        if (ui_.button({x, y0 + 8, w, 22}, kPlotNames[i], plot_ == static_cast<Plot>(i), 1.0f)) plot_ = static_cast<Plot>(i);
        x += w + 4.0f;
    }

    const SDL_FRect area{58.0f, y0 + 40.0f, W - 80.0f, kGraphH - 62.0f};
    std::vector<std::pair<float, float>> pts;
    std::string title;
    if (plot_ == Plot::Energy) {
        pts.assign(energy_.begin(), energy_.end());
        title = "total energy (KE + mgy + spring) J";
    } else if (const ObjectSpec* o = selObj_ ? scene_.object(selObj_) : nullptr) {
        if (auto it = samples_.find(o->id); it != samples_.end())
            for (const auto& s : it->second) pts.push_back({s.t, plotValue(s, plot_)});
        title = std::string(kPlotNames[int(plot_)]) + " of " + o->name;
    }
    ui_.label(x + 16.0f, y0 + 13.0f, title.c_str(), theme::textDim, 1.0f);

    if (pts.size() < 2) {
        ui_.labelCentered(area.x + area.w * 0.5f, area.y + area.h * 0.5f - 6.0f,
                          plot_ == Plot::Energy ? "Press Play to record" : "Select an object and press Play to graph it",
                          theme::textFaint, 1.25f);
        return;
    }
    float lo = pts[0].second, hi = lo;
    for (auto [t, v] : pts) {
        lo = std::min(lo, v);
        hi = std::max(hi, v);
    }
    if (hi - lo < 1e-3f) {
        lo -= 0.5f;
        hi += 0.5f;
    }
    const float pad = (hi - lo) * 0.08f;
    lo -= pad;
    hi += pad;
    const float t0 = pts.front().first, t1 = std::max(pts.back().first, t0 + 1e-3f);
    auto sx = [&](float t) { return area.x + (t - t0) / (t1 - t0) * area.w; };
    auto sy = [&](float v) { return area.y + area.h - (v - lo) / (hi - lo) * area.h; };

    gfx_.strokeRect(area, theme::border);
    if (lo < 0.0f && hi > 0.0f) gfx_.screenLine(area.x, sy(0.0f), area.x + area.w, sy(0.0f), {70, 76, 92});
    const size_t stride = std::max<size_t>(1, pts.size() / static_cast<size_t>(area.w));
    for (size_t i = stride; i < pts.size(); i += stride)
        gfx_.screenLine(sx(pts[i - stride].first), sy(pts[i - stride].second), sx(pts[i].first), sy(pts[i].second),
                        {255, 200, 87});

    char buf[48];
    std::snprintf(buf, sizeof buf, "%.3g", hi);
    gfx_.text(6.0f, area.y, buf, theme::textFaint, 1.0f);
    std::snprintf(buf, sizeof buf, "%.3g", lo);
    gfx_.text(6.0f, area.y + area.h - 8.0f, buf, theme::textFaint, 1.0f);
    std::snprintf(buf, sizeof buf, "%.2f s", t0);
    gfx_.text(area.x, area.y + area.h + 6.0f, buf, theme::textFaint, 1.0f);
    std::snprintf(buf, sizeof buf, "%.2f s", t1);
    gfx_.text(area.x + area.w - Renderer::textWidth(buf, 1.0f), area.y + area.h + 6.0f, buf, theme::textFaint, 1.0f);
    std::snprintf(buf, sizeof buf, "now %.4g", pts.back().second);
    gfx_.text(area.x + area.w * 0.5f - Renderer::textWidth(buf, 1.0f) * 0.5f, area.y + area.h + 6.0f, buf, {255, 200, 87},
              1.0f);

    // value under the cursor
    if (ui_.hovered(area)) {
        float t = t0 + (mouse_.x - area.x) / area.w * (t1 - t0);
        auto it = std::lower_bound(pts.begin(), pts.end(), t, [](const auto& a, float v) { return a.first < v; });
        if (it != pts.end()) {
            gfx_.screenLine(sx(it->first), area.y, sx(it->first), area.y + area.h, theme::border);
            std::snprintf(buf, sizeof buf, "t %.3f  %.4g", it->first, it->second);
            gfx_.text(sx(it->first) + 6.0f, area.y + 4.0f, buf, theme::text, 1.0f);
        }
    }
}

void Editor::drawHints() {
    const float W = float(cam_.width), H = float(cam_.height);
    ui_.panel({0, H - kHint, W, kHint});
    const char* hint = "";
    if (started_) {
        hint = "SPACE play/pause   . step   R reset to initial values   click to inspect   wheel zoom   MMB/RMB drag pan";
    } else {
        switch (tool_) {
        case Tool::Select:
            hint = "LMB select/drag (Ctrl: no snap)   RMB drag object: set velocity   Q/E tilt   Del delete   Ctrl+D duplicate   "
                   "SPACE play";
            break;
        case Tool::Block:
        case Tool::Ball:
        case Tool::Wedge: hint = "Click to place   ESC cancel"; break;
        case Tool::Ground: hint = "Click to place a fixed surface (its top at the click)"; break;
        case Tool::Spring:
        case Tool::Rod:
        case Tool::Rope:
            hint = pending_.empty() ? "Click an object (or empty space for a fixed point)   Shift: attach at the exact spot"
                                    : "Click the other end   ESC cancel";
            break;
        case Tool::Hinge: hint = "Click on an object: pins it there (to a second object if they overlap, else to the world)"; break;
        case Tool::Pulley:
            hint = pending_.empty()      ? "Click the first object on the rope"
                   : pending_.size() == 1 ? "Click the second object"
                                          : "Click where the pulley wheel goes";
            break;
        }
    }
    ui_.label(12.0f, H - kHint + (kHint - Renderer::textHeight(1.0f)) * 0.5f, hint, theme::textFaint, 1.0f);
}

} // namespace editor
