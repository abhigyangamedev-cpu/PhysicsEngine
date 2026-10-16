#include "QuestionSpec.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <set>

namespace ai {

using pe3::Vec3;

namespace {

constexpr float kDeg = std::numbers::pi_v<float> / 180.0f;

// ---- reading ----

// The UI font is ASCII only; map the symbols physics text uses most and drop the rest.
std::string ascii(const std::string& s) {
    static const std::pair<const char*, const char*> kMap[] = {
        {"\xc2\xb0", " deg"}, {"\xc2\xb2", "^2"}, {"\xc2\xb3", "^3"}, {"\xc2\xb7", "*"}, {"\xc3\x97", "x"},
        {"\xc2\xb5", "u"}, {"\xce\xb8", "theta"}, {"\xce\xbc", "mu"}, {"\xcf\x80", "pi"}, {"\xce\x94", "delta"},
        {"\xcf\x89", "omega"}, {"\xce\xb1", "alpha"}, {"\xce\xb2", "beta"}, {"\xcf\x81", "rho"}, {"\xce\xbb", "lambda"},
        {"\xe2\x88\x9a", "sqrt"}, {"\xe2\x88\x92", "-"}, {"\xe2\x80\x93", "-"}, {"\xe2\x80\x94", "-"},
        {"\xe2\x80\x98", "'"}, {"\xe2\x80\x99", "'"}, {"\xe2\x80\x9c", "\""}, {"\xe2\x80\x9d", "\""},
        {"\xe2\x89\x88", "~"}, {"\xe2\x86\x92", "->"}, {"\xe2\x80\xa6", "..."}};
    std::string out;
    for (size_t i = 0; i < s.size();) {
        const unsigned char c = s[i];
        if (c < 0x80) {
            out += c == '\t' ? ' ' : char(c);
            ++i;
            continue;
        }
        bool mapped = false;
        for (const auto& [from, to] : kMap) {
            const size_t n = std::char_traits<char>::length(from);
            if (s.compare(i, n, from) == 0) {
                out += to;
                i += n;
                mapped = true;
                break;
            }
        }
        if (mapped) continue;
        i += c >= 0xf0 ? 4 : c >= 0xe0 ? 3 : c >= 0xc0 ? 2 : 1; // skip one UTF-8 sequence
    }
    return out;
}

float num(const json::Value& o, const char* key, float fallback) {
    const double d = o[key].asNumber(fallback);
    return std::isfinite(d) ? float(d) : fallback;
}

pe::Vec2 vec2(const json::Value& o, const char* key, pe::Vec2 fallback = {}) {
    const json::Value& a = o[key];
    if (!a.isArray()) return fallback;
    const float x = float(a[0].asNumber(fallback.x)), y = float(a[1].asNumber(fallback.y));
    return std::isfinite(x) && std::isfinite(y) ? pe::Vec2{x, y} : fallback;
}

Vec3 vec3(const json::Value& o, const char* key, Vec3 fallback = {}) {
    const json::Value& a = o[key];
    if (!a.isArray()) return fallback;
    const float x = float(a[0].asNumber(fallback.x)), y = float(a[1].asNumber(fallback.y)), z = float(a[2].asNumber(fallback.z));
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) ? Vec3{x, y, z} : fallback;
}

template <typename E, size_t N>
bool pick(const std::string& word, const char* const (&names)[N], E& out) {
    for (size_t i = 0; i < N; ++i)
        if (word == names[i]) {
            out = static_cast<E>(i);
            return true;
        }
    return false;
}

template <size_t N>
std::string choices(const char* const (&names)[N]) {
    std::string s;
    for (size_t i = 0; i < N; ++i) s += (i ? ", " : "") + std::string(names[i]);
    return s;
}

constexpr const char* kEvents[] = {"time", "lands", "crosses_x", "crosses_y", "crosses_z", "apex", "stops", "distance", "reversals"};
constexpr const char* kQuantities[] = {"time", "x", "y", "z", "speed", "vx", "vy", "vz", "distance", "range",
                                       "max_height", "max_speed", "kinetic_energy", "period"};
constexpr const char* kKinds2[] = {"block", "ball", "wedge"};
constexpr const char* kLinks2[] = {"spring", "rod", "rope", "hinge", "pulley"};
constexpr const char* kKinds3[] = {"box", "ball"};
constexpr const char* kLinks3[] = {"rod", "rope", "spring", "ball_joint", "hinge"};

struct Reader {
    std::string error;
    std::set<std::string> names;

    void fail(const std::string& why) {
        if (error.empty()) error = why;
    }
    bool ok() const { return error.empty(); }

    std::string objectName(const json::Value& o, size_t index) {
        std::string name = ascii(o.str("name"));
        if (name.empty()) fail("object " + std::to_string(index) + " has no name");
        else if (!names.insert(name).second) fail("two objects are both named \"" + name + "\"; names must be unique");
        return name;
    }
    void positive(float v, const std::string& what) {
        if (!(v > 0.0f)) fail(what + " must be greater than zero");
    }
};

template <typename Scene>
int endpointObject(const Scene& s, const std::string& name, Reader& r, const char* linkKind) {
    if (name.empty() || name == "world") return -1;
    for (const auto& o : s.objects)
        if (o.name == name) return o.id;
    r.fail(std::string("a ") + linkKind + " link names object \"" + name + "\", which doesn't exist");
    return -1;
}

void read2D(const json::Value& j, editor::Scene& s, Reader& r) {
    using namespace editor;
    const json::Value& w = j["world"];
    s = {};
    if (w.flag("ground", true)) { // a fixed slab whose top is y = 0
        s = defaultScene();
        s.objects.front().friction = num(w, "ground_friction", 1.0f);
        r.names.insert("Ground");
    }
    s.world.g = num(w, "g", 9.81f);
    s.world.drag = std::max(num(w, "drag", 0.0f), 0.0f);
    s.world.E = vec2(w, "electric_field");
    s.world.B = num(w, "magnetic_field", 0.0f);
    s.world.coulomb = w.flag("coulomb");
    s.world.coulombK = num(w, "coulomb_k", 8.99e9f);
    s.world.mutualGravity = w.flag("mutual_gravity");
    s.world.G = num(w, "gravitational_constant", 1.0f);

    const auto& objects = j["objects"].items();
    if (objects.empty()) r.fail("the 2D scene has no objects");
    for (size_t i = 0; i < objects.size(); ++i) {
        const json::Value& o = objects[i];
        ObjectSpec spec;
        spec.id = s.nextId++;
        spec.name = r.objectName(o, i);
        const std::string kind = o.str("kind");
        if (!pick(kind, kKinds2, spec.kind)) r.fail("object \"" + spec.name + "\" has kind \"" + kind + "\"; use one of " + choices(kKinds2));
        spec.angleDeg = num(o, "angle_deg", 0.0f);
        const pe::Vec2 size = vec2(o, "size", {1.0f, 1.0f});
        spec.width = size.x;
        spec.height = size.y;
        spec.radius = num(o, "radius", 0.5f);
        spec.base = num(o, "wedge_base", 4.0f);
        spec.slopeDeg = num(o, "wedge_slope_deg", 30.0f);
        spec.flip = o.flag("wedge_flip");
        spec.mass = num(o, "mass", 1.0f);
        spec.velocity = vec2(o, "velocity");
        spec.omega = num(o, "omega", 0.0f);
        spec.force = vec2(o, "force");
        spec.friction = num(o, "friction", 0.0f);
        spec.restitution = num(o, "restitution", 0.0f);
        spec.charge = num(o, "charge", 0.0f);
        spec.fixed = o.flag("fixed");
        spec.noSpin = o.flag("no_spin");
        spec.holdVelocity = o.flag("hold_velocity");
        spec.pos = vec2(o, "position");
        if (spec.kind == ObjKind::Ball) r.positive(spec.radius, "the radius of \"" + spec.name + "\"");
        if (spec.kind == ObjKind::Block) r.positive(std::min(spec.width, spec.height), "the size of \"" + spec.name + "\"");
        if (spec.kind == ObjKind::Wedge) {
            r.positive(spec.base, "the wedge_base of \"" + spec.name + "\"");
            if (spec.slopeDeg < 1.0f || spec.slopeDeg > 85.0f) r.fail("the wedge_slope_deg of \"" + spec.name + "\" must be 1 to 85");
            // position names the bottom-left corner of the wedge's box; the engine places shapes by centroid
            const float h = spec.base * std::tan(std::clamp(spec.slopeDeg, 1.0f, 85.0f) * kDeg);
            const pe::Vec2 centroid{spec.flip ? spec.base / 3.0f : spec.base * 2.0f / 3.0f, h / 3.0f};
            spec.pos += pe::Rot(spec.angleDeg * kDeg).apply(centroid);
        }
        if (!spec.fixed) r.positive(spec.mass, "the mass of \"" + spec.name + "\"");
        if (spec.restitution < 0.0f || spec.restitution > 1.0f) r.fail("restitution of \"" + spec.name + "\" must be 0 to 1");
        s.objects.push_back(spec);
    }

    for (const json::Value& l : j["links"].items()) {
        LinkSpec spec;
        spec.id = s.nextId++;
        const std::string kind = l.str("kind");
        if (!pick(kind, kLinks2, spec.kind)) r.fail("link kind \"" + kind + "\" is unknown; use one of " + choices(kLinks2));
        spec.a = {endpointObject(s, ascii(l.str("a")), r, kind.c_str()), vec2(l, "a_offset")};
        spec.b = {endpointObject(s, ascii(l.str("b")), r, kind.c_str()), vec2(l, "b_offset")};
        spec.k = num(l, "k", 50.0f);
        spec.damping = num(l, "damping", 0.0f);
        spec.length = num(l, "length", -1.0f);
        spec.wheel = vec2(l, "pulley_wheel");
        spec.wheelRadius = num(l, "pulley_radius", 0.3f);
        spec.ratio = num(l, "pulley_ratio", 1.0f);
        if (spec.a.obj < 0 && spec.b.obj < 0) r.fail("a " + kind + " link joins two fixed points; one end must be an object");
        if (spec.kind == LinkKind::Pulley && (spec.a.obj < 0 || spec.b.obj < 0)) r.fail("a pulley needs an object on both ends");
        if (spec.kind == LinkKind::Hinge && spec.b.obj < 0) r.fail("a hinge's b end must be an object (a gives the pivot)");
        if (spec.kind == LinkKind::Spring) r.positive(spec.k, "a spring's k");
        s.links.push_back(spec);
    }
    s.sanitize();
}

void read3D(const json::Value& j, editor3::Scene& s, Reader& r) {
    using namespace editor3;
    const json::Value& w = j["world"];
    s = {};
    s.world.g = num(w, "g", 9.81f);
    s.world.ground = w.flag("ground", true);
    s.world.groundFriction = num(w, "ground_friction", 0.6f);
    s.world.substeps = std::clamp(int(num(w, "substeps", 8.0f)), 1, 32);

    const auto& objects = j["objects"].items();
    if (objects.empty()) r.fail("the 3D scene has no objects");
    for (size_t i = 0; i < objects.size(); ++i) {
        const json::Value& o = objects[i];
        const std::string name = r.objectName(o, i);
        ObjKind kind = ObjKind::Box;
        const std::string word = o.str("kind");
        if (!pick(word, kKinds3, kind)) r.fail("object \"" + name + "\" has kind \"" + word + "\"; use one of " + choices(kKinds3));
        ObjectSpec& spec = s.add(name.c_str(), kind, vec3(o, "position"));
        spec.size = vec3(o, "size", {1.0f, 1.0f, 1.0f});
        spec.radius = num(o, "radius", 0.5f);
        spec.tiltDeg = vec3(o, "tilt_deg");
        spec.mass = num(o, "mass", 1.0f);
        spec.velocity = vec3(o, "velocity");
        spec.spin = vec3(o, "spin");
        spec.friction = num(o, "friction", 0.5f);
        spec.restitution = num(o, "restitution", 0.2f);
        spec.rolling = num(o, "rolling_resistance", 0.0f);
        spec.fixed = o.flag("fixed");
        if (kind == ObjKind::Ball) r.positive(spec.radius, "the radius of \"" + name + "\"");
        else r.positive(std::min({spec.size.x, spec.size.y, spec.size.z}), "the size of \"" + name + "\"");
        if (!spec.fixed) r.positive(spec.mass, "the mass of \"" + name + "\"");
    }

    for (const json::Value& l : j["links"].items()) {
        LinkKind kind = LinkKind::Rod;
        const std::string word = l.str("kind");
        if (!pick(word, kLinks3, kind)) r.fail("link kind \"" + word + "\" is unknown; use one of " + choices(kLinks3));
        const Endpoint a{endpointObject(s, ascii(l.str("a")), r, word.c_str()), vec3(l, "a_offset")};
        const Endpoint b{endpointObject(s, ascii(l.str("b")), r, word.c_str()), vec3(l, "b_offset")};
        if (a.obj < 0 && b.obj < 0) r.fail("a " + word + " link joins two fixed points; one end must be an object");
        LinkSpec& spec = s.link(kind, a, b);
        spec.length = num(l, "length", -1.0f);
        spec.k = num(l, "k", 50.0f);
        spec.dampingRatio = num(l, "damping_ratio", 0.0f);
        spec.axis = vec3(l, "hinge_axis", {0.0f, 0.0f, 1.0f});
        spec.swingLimitDeg = num(l, "swing_limit_deg", 0.0f);
        if (kind == LinkKind::Spring) r.positive(spec.k, "a spring's k");
    }
    s.sanitize();
}

// ---- measuring ----

struct State {
    Vec3 p, v;
    float bottom = 0.0f, ke = 0.0f;
};

State lerp(const State& a, const State& b, float f) {
    return {a.p + (b.p - a.p) * f, a.v + (b.v - a.v) * f, a.bottom + (b.bottom - a.bottom) * f, a.ke + (b.ke - a.ke) * f};
}

float component(Vec3 v, int axis) { return axis == 0 ? v.x : axis == 1 ? v.y : v.z; }

State initial2D(const editor::Scene& s, const std::string& name) {
    for (const auto& o : s.objects) {
        if (o.name != name) continue;
        const float c = std::abs(std::cos(o.angleDeg * kDeg)), sn = std::abs(std::sin(o.angleDeg * kDeg));
        const float half = o.kind == editor::ObjKind::Ball ? o.radius
                           : o.kind == editor::ObjKind::Block
                               ? 0.5f * (o.width * sn + o.height * c)
                               : o.base * std::tan(std::clamp(o.slopeDeg, 1.0f, 85.0f) * kDeg) / 3.0f;
        const float speed2 = o.velocity.lengthSq();
        return {{o.pos.x, o.pos.y, 0.0f}, {o.velocity.x, o.velocity.y, 0.0f}, o.pos.y - half, 0.5f * o.mass * speed2};
    }
    return {};
}

State initial3D(const editor3::Scene& s, const std::string& name) {
    for (const auto& o : s.objects)
        if (o.name == name) return {o.pos, o.velocity, o.pos.y - o.halfHeight(), 0.5f * o.mass * o.velocity.lengthSq()};
    return {};
}

const char* eventText(const Measure& m) {
    switch (m.event) {
    case Event::Time: return "the set time";
    case Event::Lands: return "landing";
    case Event::CrossesX: return "crossing x";
    case Event::CrossesY: return "crossing y";
    case Event::CrossesZ: return "crossing z";
    case Event::Apex: return "the top of its path";
    case Event::Stops: return "coming to rest";
    case Event::Distance: return "the set distance";
    case Event::Reversals: return "the set number of turnarounds";
    }
    return "";
}

// Watches one run: finds the step the event happens in, then reads the quantity off at the exact
// moment inside that step by interpolating between the states either side of it.
class Tracker {
public:
    Tracker(Measure m, bool threeD, State ev0, State q0, std::string unit)
        : m_(std::move(m)), threeD_(threeD), ev0_(ev0), q0_(q0), unit_(std::move(unit)) {
        reset();
    }

    void reset() {
        evPrev_ = ev0_;
        qPrev_ = q0_;
        tPrev_ = 0.0f;
        qAccel_ = {};
        maxY_ = q0_.p.y;
        maxSpeed_ = q0_.v.length();
        peak_ = ev0_.v.length();
        reversals_ = 0;
        firstRev_ = lastRev_ = 0.0f;
        lastSign_ = sign(component(ev0_.v, m_.axis));
        done_ = false;
    }

    // Returns true on the step the measurement completes.
    bool step(float t, const State& ev, const State& q, float& result, std::vector<std::string>& lines) {
        if (done_) return false;
        float f = -1.0f; // where in this step the event happened, 0..1
        switch (m_.event) {
        case Event::Time:
            if (t >= m_.value) f = (m_.value - tPrev_) / std::max(t - tPrev_, 1e-9f);
            break;
        case Event::Lands: {
            const float above = evPrev_.bottom - m_.value;
            const bool touched = ev.bottom <= m_.value + 0.01f;
            const bool kicked = ev.bottom < m_.value + 0.05f && ev.v.y > evPrev_.v.y + 0.5f;
            if (evPrev_.v.y < -0.05f && (touched || kicked)) // falling, not resting on it
                f = above <= 0.0f ? 0.0f : above / (-evPrev_.v.y * (t - tPrev_)); // from the velocity going in
            break;
        }
        case Event::CrossesX:
        case Event::CrossesY:
        case Event::CrossesZ: {
            const int axis = int(m_.event) - int(Event::CrossesX);
            const float a = component(evPrev_.p, axis) - m_.value, b = component(ev.p, axis) - m_.value;
            if (a != 0.0f && (a < 0.0f) != (b < 0.0f)) f = a / (a - b);
            break;
        }
        case Event::Apex:
            if (evPrev_.v.y > 0.0f && ev.v.y <= 0.0f) f = evPrev_.v.y / (evPrev_.v.y - ev.v.y);
            break;
        case Event::Stops: {
            const float speed = ev.v.length();
            peak_ = std::max(peak_, speed);
            if (peak_ > 0.05f && speed < std::max(0.01f, 0.02f * peak_)) f = 1.0f;
            break;
        }
        case Event::Distance: {
            const float a = (evPrev_.p - ev0_.p).length(), b = (ev.p - ev0_.p).length();
            if (b >= m_.value && a < m_.value) f = (m_.value - a) / std::max(b - a, 1e-9f);
            break;
        }
        case Event::Reversals: {
            const float a = component(evPrev_.v, m_.axis), b = component(ev.v, m_.axis);
            const int s = sign(b);
            if (s != 0 && lastSign_ != 0 && s != lastSign_) {
                const float frac = a != b ? std::clamp(a / (a - b), 0.0f, 1.0f) : 1.0f;
                const float tr = tPrev_ + frac * (t - tPrev_);
                if (++reversals_ == 1) firstRev_ = tr;
                lastRev_ = tr;
                if (reversals_ >= std::max(1, int(std::lround(m_.value)))) f = frac;
            }
            if (s != 0) lastSign_ = s;
            break;
        }
        }

        if (f < 0.0f) {
            maxY_ = std::max(maxY_, q.p.y);
            maxSpeed_ = std::max(maxSpeed_, q.v.length());
            progress(t, ev, lines);
            if (t > tPrev_) qAccel_ = (q.v - qPrev_.v) * (1.0f / (t - tPrev_));
            evPrev_ = ev;
            qPrev_ = q;
            tPrev_ = t;
            return false;
        }
        f = std::clamp(f, 0.0f, 1.0f);
        const float tHit = tPrev_ + f * (t - tPrev_);
        // a landing step ends after the bounce, so carry the state before it forward instead
        const State at = m_.event == Event::Lands ? extrapolate(qPrev_, tHit - tPrev_) : lerp(qPrev_, q, f);
        maxY_ = std::max(maxY_, at.p.y);
        maxSpeed_ = std::max(maxSpeed_, at.v.length());
        result = read(at, tHit);
        lines = {m_.label + " = " + formatValue(result) + (unit_.empty() ? "" : " " + unit_)};
        if (m_.quantity != Quantity::Time) {
            char buf[64];
            std::snprintf(buf, sizeof buf, "at t = %.3f s", tHit);
            lines.push_back(buf);
        }
        done_ = true;
        return true;
    }

private:
    Measure m_;
    bool threeD_;
    State ev0_, q0_, evPrev_, qPrev_;
    std::string unit_;
    Vec3 qAccel_; // of the quantity object over the last step
    float tPrev_ = 0.0f, maxY_ = 0.0f, maxSpeed_ = 0.0f, peak_ = 0.0f, firstRev_ = 0.0f, lastRev_ = 0.0f;
    int reversals_ = 0, lastSign_ = 0;
    bool done_ = false;

    static int sign(float v) { return v > 1e-4f ? 1 : v < -1e-4f ? -1 : 0; }

    State extrapolate(const State& s, float dt) const {
        State out = s;
        out.p = s.p + s.v * dt + qAccel_ * (0.5f * dt * dt);
        out.v = s.v + qAccel_ * dt;
        const float before = s.v.lengthSq();
        if (before > 1e-12f) out.ke = s.ke * out.v.lengthSq() / before; // mostly translational at impact
        return out;
    }

    float read(const State& s, float t) const {
        const Vec3 d = s.p - q0_.p;
        switch (m_.quantity) {
        case Quantity::Time: return t;
        case Quantity::X: return s.p.x;
        case Quantity::Y: return s.p.y;
        case Quantity::Z: return s.p.z;
        case Quantity::Speed: return s.v.length();
        case Quantity::Vx: return s.v.x;
        case Quantity::Vy: return s.v.y;
        case Quantity::Vz: return s.v.z;
        case Quantity::Distance: return d.length();
        case Quantity::Range: return threeD_ ? std::sqrt(d.x * d.x + d.z * d.z) : std::abs(d.x);
        case Quantity::MaxHeight: return maxY_;
        case Quantity::MaxSpeed: return maxSpeed_;
        case Quantity::KineticEnergy: return s.ke;
        case Quantity::Period: return reversals_ >= 2 ? 2.0f * (lastRev_ - firstRev_) / float(reversals_ - 1) : 0.0f;
        }
        return 0.0f;
    }

    void progress(float t, const State& ev, std::vector<std::string>& lines) const {
        char buf[96];
        lines.resize(2);
        std::snprintf(buf, sizeof buf, "t = %.2f s, waiting for %s", t, eventText(m_));
        lines[0] = buf;
        if (threeD_) std::snprintf(buf, sizeof buf, "%s at (%.2f, %.2f, %.2f) m", m_.eventObject.c_str(), ev.p.x, ev.p.y, ev.p.z);
        else std::snprintf(buf, sizeof buf, "%s at (%.2f, %.2f) m", m_.eventObject.c_str(), ev.p.x, ev.p.y);
        lines[1] = buf;
    }
};

State state2D(const editor::Simulation& sim, const std::string& name) {
    const pe::Body* b = sim.body(name.c_str());
    if (!b) return {};
    return {{b->position.x, b->position.y, 0.0f}, {b->velocity.x, b->velocity.y, 0.0f}, b->aabb.min.y, editor::kineticEnergy(*b)};
}

State state3D(const editor3::Simulation& sim, const std::string& name) {
    const pe3::Body* b = sim.body(name.c_str());
    if (!b) return {};
    return {b->position, b->velocity, b->aabb.min.y, editor3::kineticEnergy(*b)};
}

// A monitor for the editors. It keeps its tracker between steps and starts it over whenever the
// editor hands it a fresh run (primed is false).
auto monitor2D(const Spec& spec) {
    auto tracker = std::make_shared<Tracker>(spec.measure, false, initial2D(spec.scene2, spec.measure.eventObject),
                                             initial2D(spec.scene2, spec.measure.object), spec.unit);
    return [tracker, ev = spec.measure.eventObject, q = spec.measure.object](const editor::Simulation& sim, editor::ExampleRun& run) {
        if (!run.primed) tracker->reset(), run.primed = true;
        if (run.done) return false;
        const bool hit = tracker->step(sim.time(), state2D(sim, ev), state2D(sim, q), run.result, run.lines);
        if (hit) run.done = true;
        return hit;
    };
}

auto monitor3D(const Spec& spec) {
    auto tracker = std::make_shared<Tracker>(spec.measure, true, initial3D(spec.scene3, spec.measure.eventObject),
                                             initial3D(spec.scene3, spec.measure.object), spec.unit);
    return [tracker, ev = spec.measure.eventObject, q = spec.measure.object](const editor3::Simulation& sim, editor3::ProblemRun& run) {
        if (!run.primed) tracker->reset(), run.primed = true;
        if (run.done) return false;
        const bool hit = tracker->step(sim.time(), state3D(sim, ev), state3D(sim, q), run.result, run.lines);
        if (hit) run.done = true;
        return hit;
    };
}

int plotFor(Quantity q) { // editor::Plot: X, Y, Vx, Vy, Speed, Ax, Ay, Accel, KE, Energy
    switch (q) {
    case Quantity::X:
    case Quantity::Range: return 0;
    case Quantity::Y:
    case Quantity::MaxHeight:
    case Quantity::Period: return 1;
    case Quantity::Vx: return 2;
    case Quantity::Vy: return 3;
    case Quantity::KineticEnergy: return 8;
    default: return 4;
    }
}

} // namespace

std::string formatValue(float v) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.4g", v);
    return buf;
}

std::shared_ptr<Spec> readSpec(const json::Value& v, std::string& error) {
    error.clear();
    if (!v.isObject()) {
        error = "the answer is not a JSON object";
        return nullptr;
    }
    if (const std::string why = ascii(v.str("cannot_simulate")); !why.empty()) {
        error = "cannot simulate: " + why;
        return nullptr;
    }
    auto spec = std::make_shared<Spec>();
    Reader r;
    spec->title = ascii(v.str("title", "Generated problem"));
    spec->topic = ascii(v.str("topic", "Mechanics"));
    spec->question = ascii(v.str("question"));
    spec->answer = ascii(v.str("solution"));
    spec->unit = ascii(v.str("unit"));
    spec->expected = num(v, "expected_answer", 0.0f);
    spec->source = v;
    if (spec->question.empty()) r.fail("question is empty");

    const std::string space = v.str("space");
    if (space == "2d") {
        spec->space = Space::TwoD;
        if (!v["scene_2d"].isObject()) r.fail("space is \"2d\" but scene_2d is missing");
        else read2D(v["scene_2d"], spec->scene2, r);
    } else if (space == "3d") {
        spec->space = Space::ThreeD;
        if (!v["scene_3d"].isObject()) r.fail("space is \"3d\" but scene_3d is missing");
        else read3D(v["scene_3d"], spec->scene3, r);
    } else {
        r.fail("space must be \"2d\" or \"3d\"");
    }

    const json::Value& m = v["measure"];
    Measure& out = spec->measure;
    const std::string event = m.str("event"), quantity = m.str("quantity");
    if (!pick(event, kEvents, out.event)) r.fail("measure.event \"" + event + "\" is unknown; use one of " + choices(kEvents));
    if (!pick(quantity, kQuantities, out.quantity))
        r.fail("measure.quantity \"" + quantity + "\" is unknown; use one of " + choices(kQuantities));
    out.eventObject = ascii(m.str("event_object"));
    out.object = ascii(m.str("quantity_object"));
    if (out.object.empty()) out.object = out.eventObject;
    out.value = num(m, "event_value", 0.0f);
    const std::string axis = m.str("axis", "y");
    out.axis = axis == "x" ? 0 : axis == "z" ? 2 : 1;
    out.label = ascii(m.str("label"));
    if (out.label.empty()) out.label = quantity;
    out.timeout = std::clamp(num(m, "timeout", 60.0f), 1.0f, 600.0f);
    if (out.event == Event::Time) {
        if (!(out.value > 0.0f)) r.fail("a \"time\" event needs event_value > 0 (the time in seconds)");
        out.timeout = std::max(out.timeout, out.value + 1.0f);
    }
    if (out.event == Event::Distance && !(out.value > 0.0f)) r.fail("a \"distance\" event needs event_value > 0");
    if (out.event == Event::Reversals && out.value < 1.0f) r.fail("a \"reversals\" event needs event_value >= 1");
    if (out.quantity == Quantity::Period && (out.event != Event::Reversals || out.value < 2.0f))
        r.fail("quantity \"period\" needs event \"reversals\" with event_value >= 2 (3 or more is better)");
    if (out.event == Event::CrossesZ && spec->space == Space::TwoD) r.fail("crosses_z needs a 3D scene");

    if (r.ok()) {
        auto has = [&](const std::string& name) {
            if (spec->space == Space::TwoD)
                return std::any_of(spec->scene2.objects.begin(), spec->scene2.objects.end(), [&](auto& o) { return o.name == name; });
            return std::any_of(spec->scene3.objects.begin(), spec->scene3.objects.end(), [&](auto& o) { return o.name == name; });
        };
        if (!has(out.eventObject)) r.fail("measure.event_object \"" + out.eventObject + "\" is not an object in the scene");
        if (!has(out.object)) r.fail("measure.quantity_object \"" + out.object + "\" is not an object in the scene");
    }

    const json::Value& view = v["view"];
    spec->viewCenter = vec3(view, "center", {0.0f, 2.0f, 0.0f});
    spec->viewSize = std::clamp(num(view, "size", 12.0f), 0.5f, 5000.0f);
    spec->focus = ascii(v.str("focus", out.object));
    if (spec->focus.empty()) spec->focus = out.object;

    if (!r.ok()) {
        error = r.error;
        return nullptr;
    }
    return spec;
}

Check verify(const Spec& spec) {
    Check c;
    const float timeout = spec.measure.timeout;
    if (spec.space == Space::TwoD) {
        editor::Simulation sim;
        sim.load(spec.scene2);
        editor::ExampleRun run;
        auto monitor = monitor2D(spec);
        while (sim.time() < timeout && !sim.blewUp())
            if (sim.step(), monitor(sim, run)) break;
        c.reached = run.done;
        c.blewUp = sim.blewUp();
        c.result = run.result;
        c.time = sim.time();
        c.lines = run.lines;
    } else {
        editor3::Simulation sim;
        sim.load(spec.scene3);
        editor3::ProblemRun run;
        auto monitor = monitor3D(spec);
        while (sim.time() < timeout && !sim.blewUp())
            if (sim.step(), monitor(sim, run)) break;
        c.reached = run.done;
        c.blewUp = sim.blewUp();
        c.result = run.result;
        c.time = sim.time();
        c.lines = run.lines;
    }

    char buf[320];
    if (c.blewUp) {
        std::snprintf(buf, sizeof buf,
                      "the simulation blew up at t = %.2f s (a body went non-finite or absurdly fast/far): check for "
                      "overlapping objects at the start, very stiff springs, or huge forces",
                      c.time);
        c.problem = buf;
    } else if (!c.reached) {
        std::snprintf(buf, sizeof buf,
                      "the measured event (%s of \"%s\") never happened in %.0f simulated seconds; last seen: %s",
                      eventText(spec.measure), spec.measure.eventObject.c_str(), timeout,
                      c.lines.size() > 1 ? c.lines[1].c_str() : "no data");
        c.problem = buf;
    } else if (!std::isfinite(c.result)) {
        c.problem = "the measured value is not a finite number";
    }
    return c;
}

std::shared_ptr<const editor::CustomExample> toExample(std::shared_ptr<const Spec> spec) {
    auto c = std::make_shared<editor::CustomExample>();
    c->name = spec->title;
    c->topic = spec->topic;
    c->question = spec->question;
    c->answer = spec->answer;
    c->focus = spec->focus;
    c->unit = spec->unit;
    editor::Example& e = c->example;
    e.name = c->name.c_str();
    e.topic = c->topic.c_str();
    e.question = c->question.c_str();
    e.answer = c->answer.c_str();
    e.expected = spec->expected;
    e.setup = [spec] { return spec->scene2; };
    e.cameraCenter = {spec->viewCenter.x, spec->viewCenter.y};
    e.cameraScale = std::clamp(900.0f / spec->viewSize, 0.5f, 400.0f);
    e.focus = c->focus.c_str();
    e.plot = plotFor(spec->measure.quantity);
    e.monitor = monitor2D(*spec);
    e.unit = c->unit.c_str();
    return c;
}

std::shared_ptr<const editor3::CustomProblem> toProblem(std::shared_ptr<const Spec> spec) {
    auto c = std::make_shared<editor3::CustomProblem>();
    c->name = spec->title;
    c->topic = spec->topic;
    c->question = spec->question;
    c->answer = spec->answer;
    c->unit = spec->unit;
    c->focus = spec->focus;
    editor3::Problem& p = c->problem;
    p.name = c->name.c_str();
    p.topic = c->topic.c_str();
    p.question = c->question.c_str();
    p.answer = c->answer.c_str();
    p.expected = spec->expected;
    p.unit = c->unit.c_str();
    p.setup = [spec] { return spec->scene3; };
    p.cameraTarget = spec->viewCenter;
    p.cameraDistance = std::clamp(spec->viewSize * 1.3f, 2.0f, 4000.0f);
    p.cameraYaw = 0.6f;
    p.cameraPitch = 0.4f;
    p.focus = c->focus.c_str();
    p.monitor = monitor3D(*spec);
    return c;
}

} // namespace ai
