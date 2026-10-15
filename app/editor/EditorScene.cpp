#include "editor/EditorScene.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <numbers>
#include <sstream>
#include <unordered_map>

namespace editor {

using namespace pe;

namespace {

constexpr float kDeg = std::numbers::pi_v<float> / 180.0f;

Body* addAnchor(World& w, Vec2 p) {
    BodyDef d;
    d.position = p;
    d.isStatic = true;
    d.collides = false;
    return w.createBody(Shape::circle(0.06f), d);
}

} // namespace

Shape ObjectSpec::shape() const {
    switch (kind) {
    case ObjKind::Ball: return Shape::circle(std::max(radius, 0.01f));
    case ObjKind::Wedge: {
        float b = std::max(base, 0.05f), h = b * std::tan(std::clamp(slopeDeg, 1.0f, 85.0f) * kDeg);
        return flip ? Shape::polygon({{0, 0}, {b, 0}, {0, h}}) : Shape::polygon({{0, 0}, {b, 0}, {b, h}});
    }
    default: return Shape::box(std::max(width, 0.02f) * 0.5f, std::max(height, 0.02f) * 0.5f);
    }
}

void ObjectSpec::sanitize() {
    if (int(kind) < 0 || int(kind) > int(ObjKind::Wedge)) kind = ObjKind::Block;
    mass = std::max(mass, 1e-3f);
    friction = std::max(friction, 0.0f);
    restitution = std::clamp(restitution, 0.0f, 1.0f);
}

void LinkSpec::sanitize() {
    if (int(kind) < 0 || int(kind) > int(LinkKind::Pulley)) kind = LinkKind::Rod;
    k = std::max(k, 0.0f);
    damping = std::max(damping, 0.0f);
    ratio = std::max(ratio, 0.1f);
    wheelRadius = std::max(wheelRadius, 0.02f);
}

void WorldSpec::sanitize() {
    timeScale = std::clamp(timeScale, 0.01f, 10.0f);
    stopAt = std::max(stopAt, 0.0f);
    drag = std::max(drag, 0.0f);
}

void Scene::sanitize() {
    world.sanitize();
    for (auto& o : objects) o.sanitize();
    for (auto& l : links) l.sanitize();
}

ObjectSpec* Scene::object(int id) {
    for (auto& o : objects)
        if (o.id == id) return &o;
    return nullptr;
}

const ObjectSpec* Scene::object(int id) const { return const_cast<Scene*>(this)->object(id); }

LinkSpec* Scene::link(int id) {
    for (auto& l : links)
        if (l.id == id) return &l;
    return nullptr;
}

void Scene::removeObject(int id) {
    std::erase_if(links, [id](const LinkSpec& l) { return l.a.obj == id || l.b.obj == id; });
    std::erase_if(objects, [id](const ObjectSpec& o) { return o.id == id; });
}

void Scene::removeLink(int id) {
    std::erase_if(links, [id](const LinkSpec& l) { return l.id == id; });
}

std::string Scene::nextName() const {
    for (int i = 0;; ++i) {
        std::string name = i < 26 ? std::string(1, char('A' + i)) : "A" + std::to_string(i - 25);
        if (std::none_of(objects.begin(), objects.end(), [&](const ObjectSpec& o) { return o.name == name; })) return name;
    }
}

Vec2 Scene::endpointWorld(const Endpoint& e) const {
    const ObjectSpec* o = e.obj >= 0 ? object(e.obj) : nullptr;
    if (!o) return e.local;
    return o->pos + Rot(o->angleDeg * kDeg).apply(e.local);
}

Endpoint Scene::endpointAt(int obj, Vec2 world) const {
    const ObjectSpec* o = obj >= 0 ? object(obj) : nullptr;
    if (!o) return {-1, world};
    return {obj, Rot(o->angleDeg * kDeg).applyT(world - o->pos)};
}

float Scene::initialLength(const LinkSpec& l) const {
    if (l.kind == LinkKind::Pulley) {
        Vec2 pa = endpointWorld(l.a), pb = endpointWorld(l.b);
        auto [ga, gb] = wheelContacts(l.wheel, l.wheelRadius, pa, pb);
        return (ga - pa).length() + l.ratio * (gb - pb).length();
    }
    return (endpointWorld(l.b) - endpointWorld(l.a)).length();
}

namespace {

int tangents(Vec2 c, float r, Vec2 p, Vec2 out[2]) {
    Vec2 d = p - c;
    float len = d.length();
    if (len <= r) return 0;
    float base = std::atan2(d.y, d.x), off = std::acos(r / len);
    out[0] = c + Vec2{std::cos(base + off), std::sin(base + off)} * r;
    out[1] = c + Vec2{std::cos(base - off), std::sin(base - off)} * r;
    return 2;
}

bool segmentsCross(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    float d1 = cross(b - a, c - a), d2 = cross(b - a, d - a), d3 = cross(d - c, a - c), d4 = cross(d - c, b - c);
    return d1 * d2 < 0.0f && d3 * d4 < 0.0f;
}

} // namespace

std::pair<Vec2, Vec2> wheelContacts(Vec2 c, float r, Vec2 pa, Vec2 pb) {
    Vec2 ta[2], tb[2];
    const Vec2 top = c + Vec2{0.0f, r};
    if (!tangents(c, r, pa, ta) || !tangents(c, r, pb, tb)) return {top, top};
    std::pair<Vec2, Vec2> best{top, top};
    float bestY = -1e30f;
    for (Vec2 a : ta) {
        for (Vec2 b : tb) {
            // rope runs pa -> a, around the wheel, b -> pb: the wrap sense must agree
            if ((cross(a - c, a - pa) > 0.0f) != (cross(b - c, pb - b) > 0.0f)) continue;
            if (segmentsCross(pa, a, pb, b)) continue;
            if (a.y + b.y > bestY) {
                bestY = a.y + b.y;
                best = {a, b};
            }
        }
    }
    return best;
}

Scene defaultScene() {
    Scene s;
    ObjectSpec ground;
    ground.id = s.nextId++;
    ground.name = "Ground";
    ground.pos = {0.0f, -0.5f};
    ground.width = 60.0f;
    ground.friction = 1.0f;
    ground.fixed = true;
    s.objects.push_back(ground);
    return s;
}

void build(const Scene& scene, World& w, Built& out) {
    w.clear();
    w.settings = {};
    const WorldSpec& ws = scene.world;
    w.settings.gravity = {0.0f, -ws.g};
    w.settings.substeps = 8;
    w.settings.relaxIterations = 20; // stacks need this for friction to reach mu N; editor scenes are small
    w.settings.angularDamping = 0.0f;
    w.settings.linearDamping = ws.drag;
    w.settings.minFriction = true;
    w.settings.restitutionThreshold = 0.2f;

    out = {};
    std::unordered_map<int, Body*> byId;
    for (const auto& o : scene.objects) {
        Shape shape = o.shape();
        BodyDef d;
        d.position = o.pos;
        d.angle = o.angleDeg * kDeg;
        d.velocity = o.velocity;
        d.angularVelocity = o.omega;
        float area = shape.computeMass(1.0f).mass;
        d.density = area > 0.0f ? std::max(o.mass, 1e-4f) / area : 1.0f;
        d.friction = o.friction;
        d.restitution = o.restitution;
        d.charge = o.charge;
        d.isStatic = o.fixed;
        Body* b = w.createBody(shape, d);
        if (o.noSpin) b->invInertia = 0.0f;
        if (o.fixed) b->mass = o.mass; // immovable, but still a source of gravity
        out.bodies.push_back(b);
        byId[o.id] = b;
    }

    if (ws.E.lengthSq() > 0.0f) w.addField<UniformElectricField>()->E = ws.E;
    if (ws.B != 0.0f) w.addField<UniformMagneticField>()->B = ws.B;
    if (ws.coulomb) {
        auto* c = w.addField<CoulombField>();
        c->k = ws.coulombK;
        c->softening = 0.02f;
    }
    if (ws.mutualGravity) {
        auto* n = w.addField<NBodyGravity>();
        n->G = ws.G;
        n->includeParticles = false;
        n->theta = 0.3f;
    }
    out.springs = w.addField<SpringField>();

    auto bodyOf = [&](const Endpoint& e) -> Body* {
        auto it = byId.find(e.obj);
        return it == byId.end() ? nullptr : it->second;
    };
    for (const auto& l : scene.links) {
        Body* a = bodyOf(l.a);
        Body* b = bodyOf(l.b);
        Vec2 pa = scene.endpointWorld(l.a), pb = scene.endpointWorld(l.b);
        out.springIndex.push_back(-1);
        out.joints.push_back(nullptr);

        if (l.kind == LinkKind::Spring) {
            if (!a && !b) continue;
            if (!a) {
                std::swap(a, b);
                std::swap(pa, pb);
            }
            auto& s = out.springs->add(a, pa, b, pb, l.k, l.damping);
            if (l.length >= 0.0f) s.rest = l.length;
            out.springIndex.back() = static_cast<int>(out.springs->springs.size()) - 1;
            continue;
        }
        if (l.kind == LinkKind::Pulley) {
            if (!a || !b) continue;
            BodyDef d;
            d.position = l.wheel;
            d.isStatic = true;
            d.collides = false;
            d.color = 0x8a93a6ff;
            w.createBody(Shape::circle(l.wheelRadius), d);
            auto [ga, gb] = wheelContacts(l.wheel, l.wheelRadius, pa, pb);
            auto* p = w.createJoint<PulleyJoint>(a, b, ga, gb, pa, pb, l.ratio);
            if (l.length >= 0.0f) p->total = l.length;
            p->collideConnected = true; // a rope doesn't stop the blocks touching
            w.rebuildJointPairs();
            out.joints.back() = p;
            continue;
        }
        if (!a) a = addAnchor(w, pa);
        if (!b) b = addAnchor(w, pb);
        if (l.kind == LinkKind::Hinge) {
            out.joints.back() = w.createJoint<RevoluteJoint>(a, b, pa);
        } else {
            auto* d = w.createJoint<DistanceJoint>(a, b, pa, pb);
            d->rope = l.kind == LinkKind::Rope;
            d->collideConnected = d->rope;
            w.rebuildJointPairs();
            if (l.length >= 0.0f) d->length = l.length;
            out.joints.back() = d;
        }
    }
}

bool Scene::save(const char* path) const {
    std::ofstream f(path);
    if (!f) return false;
    const WorldSpec& w = world;
    f << "world " << w.g << ' ' << w.timeScale << ' ' << w.stopAt << ' ' << w.drag << ' ' << w.E.x << ' ' << w.E.y << ' '
      << w.B << ' ' << w.coulomb << ' ' << w.coulombK << ' ' << w.mutualGravity << ' ' << w.G << '\n';
    for (const auto& o : objects)
        f << "obj " << o.id << ' ' << std::quoted(o.name) << ' ' << int(o.kind) << ' ' << o.pos.x << ' ' << o.pos.y << ' ' << o.angleDeg
          << ' ' << o.width << ' ' << o.height << ' ' << o.radius << ' ' << o.base << ' ' << o.slopeDeg << ' ' << o.flip << ' '
          << o.mass << ' ' << o.velocity.x << ' ' << o.velocity.y << ' ' << o.omega << ' ' << o.force.x << ' ' << o.force.y
          << ' ' << o.friction << ' ' << o.restitution << ' ' << o.charge << ' ' << o.fixed << ' ' << o.noSpin << ' ' << o.holdVelocity << '\n';
    for (const auto& l : links)
        f << "link " << l.id << ' ' << int(l.kind) << ' ' << l.a.obj << ' ' << l.a.local.x << ' ' << l.a.local.y << ' '
          << l.b.obj << ' ' << l.b.local.x << ' ' << l.b.local.y << ' ' << l.k << ' ' << l.damping << ' ' << l.length << ' '
          << l.wheel.x << ' ' << l.wheel.y << ' ' << l.wheelRadius << ' ' << l.ratio << '\n';
    return static_cast<bool>(f);
}

bool Scene::load(const char* path) {
    std::ifstream f(path);
    if (!f) return false;
    Scene s;
    std::string line;
    while (std::getline(f, line)) {
        std::istringstream in(line);
        std::string tag;
        in >> tag;
        int kind = 0;
        if (tag == "world") {
            WorldSpec& w = s.world;
            in >> w.g >> w.timeScale >> w.stopAt >> w.drag >> w.E.x >> w.E.y >> w.B >> w.coulomb >> w.coulombK >>
                w.mutualGravity >> w.G;
        } else if (tag == "obj") {
            ObjectSpec o;
            in >> o.id >> std::quoted(o.name) >> kind >> o.pos.x >> o.pos.y >> o.angleDeg >> o.width >> o.height >> o.radius >> o.base >>
                o.slopeDeg >> o.flip >> o.mass >> o.velocity.x >> o.velocity.y >> o.omega >> o.force.x >> o.force.y >>
                o.friction >> o.restitution >> o.charge >> o.fixed >> o.noSpin;
            o.kind = static_cast<ObjKind>(kind);
            if (!in) continue;
            in >> o.holdVelocity; // absent in older files
            s.objects.push_back(o);
        } else if (tag == "link") {
            LinkSpec l;
            in >> l.id >> kind >> l.a.obj >> l.a.local.x >> l.a.local.y >> l.b.obj >> l.b.local.x >> l.b.local.y >> l.k >>
                l.damping >> l.length >> l.wheel.x >> l.wheel.y >> l.wheelRadius >> l.ratio;
            l.kind = static_cast<LinkKind>(kind);
            if (in) s.links.push_back(l);
        }
    }
    s.sanitize();
    for (const auto& o : s.objects) s.nextId = std::max(s.nextId, o.id + 1);
    for (const auto& l : s.links) s.nextId = std::max(s.nextId, l.id + 1);
    *this = std::move(s);
    return true;
}

} // namespace editor
