#include "EditorScene3.h"
#include <algorithm>
#include <cmath>
#include <numbers>

namespace editor3 {

using namespace pe3;

namespace {

constexpr float kDeg = std::numbers::pi_v<float> / 180.0f;
constexpr uint32_t kPalette[] = {0xe07a5fff, 0x3d85c6ff, 0x81b29aff, 0xf2cc8fff, 0x9b5de5ff, 0xf4a261ff, 0x2a9d8fff};

float finiteOr(float v, float fallback) { return std::isfinite(v) ? v : fallback; }
Vec3 finiteOr(Vec3 v, Vec3 fallback = {}) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z) ? v : fallback;
}

} // namespace

Shape ObjectSpec::shape() const {
    if (kind == ObjKind::Ball) return Shape::sphere(std::max(radius, 0.01f));
    return Shape::box(vmax(size * 0.5f, Vec3{0.005f, 0.005f, 0.005f}));
}

Quat ObjectSpec::orientation() const {
    return Quat::axisAngle({0, 0, 1}, tiltDeg.z * kDeg) * Quat::axisAngle({0, 1, 0}, tiltDeg.y * kDeg) *
           Quat::axisAngle({1, 0, 0}, tiltDeg.x * kDeg);
}

float ObjectSpec::halfHeight() const { return kind == ObjKind::Ball ? radius : size.y * 0.5f; }

void ObjectSpec::sanitize() {
    if (int(kind) < 0 || int(kind) > int(ObjKind::Ball)) kind = ObjKind::Box;
    pos = finiteOr(pos);
    size = vmin(vmax(finiteOr(size, {1, 1, 1}), Vec3{0.01f, 0.01f, 0.01f}), Vec3{200, 200, 200});
    radius = std::clamp(finiteOr(radius, 0.5f), 0.01f, 100.0f);
    tiltDeg = finiteOr(tiltDeg);
    mass = std::clamp(finiteOr(mass, 1.0f), 1e-3f, 1e7f);
    velocity = finiteOr(velocity);
    spin = finiteOr(spin);
    friction = std::clamp(finiteOr(friction, 0.5f), 0.0f, 10.0f);
    restitution = std::clamp(finiteOr(restitution, 0.0f), 0.0f, 1.0f);
    rolling = std::clamp(finiteOr(rolling, 0.0f), 0.0f, 1.0f);
}

void LinkSpec::sanitize() {
    if (int(kind) < 0 || int(kind) > int(LinkKind::Hinge)) kind = LinkKind::Rod;
    length = finiteOr(length, -1.0f);
    k = std::max(finiteOr(k, 50.0f), 0.0f);
    dampingRatio = std::clamp(finiteOr(dampingRatio, 0.0f), 0.0f, 10.0f);
    axis = finiteOr(axis, {0, 0, 1});
    if (axis.lengthSq() < 1e-8f) axis = {0, 0, 1};
    swingLimitDeg = std::clamp(finiteOr(swingLimitDeg, 0.0f), 0.0f, 180.0f);
}

void WorldSpec::sanitize() {
    g = finiteOr(g, 9.81f);
    groundFriction = std::clamp(finiteOr(groundFriction, 0.6f), 0.0f, 10.0f);
    timeScale = std::clamp(finiteOr(timeScale, 1.0f), 0.01f, 10.0f);
    stopAt = std::max(finiteOr(stopAt, 0.0f), 0.0f);
    substeps = std::clamp(substeps, 1, 32);
}

void Scene::sanitize() {
    world.sanitize();
    for (auto& o : objects) o.sanitize();
    for (auto& l : links) l.sanitize();
    std::erase_if(links, [this](const LinkSpec& l) {
        return (l.a.obj >= 0 && !object(l.a.obj)) || (l.b.obj >= 0 && !object(l.b.obj)) || (l.a.obj < 0 && l.b.obj < 0);
    });
}

ObjectSpec& Scene::add(const char* name, ObjKind kind, Vec3 pos) {
    ObjectSpec o;
    o.id = nextId++;
    o.name = name;
    o.kind = kind;
    o.pos = pos;
    objects.push_back(o);
    return objects.back();
}

LinkSpec& Scene::link(LinkKind kind, Endpoint a, Endpoint b) {
    LinkSpec l;
    l.id = nextId++;
    l.kind = kind;
    l.a = a;
    l.b = b;
    links.push_back(l);
    return links.back();
}

ObjectSpec* Scene::object(int id) {
    for (auto& o : objects)
        if (o.id == id) return &o;
    return nullptr;
}

const ObjectSpec* Scene::object(int id) const { return const_cast<Scene*>(this)->object(id); }

LinkSpec* Scene::findLink(int id) {
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

Vec3 Scene::endpointWorld(const Endpoint& e) const {
    const ObjectSpec* o = e.obj >= 0 ? object(e.obj) : nullptr;
    if (!o) return e.local;
    return o->pos + o->orientation().rotate(e.local);
}

Endpoint Scene::endpointAt(int obj, Vec3 world) const {
    const ObjectSpec* o = obj >= 0 ? object(obj) : nullptr;
    if (!o) return {-1, world};
    return {obj, o->orientation().unrotate(world - o->pos)};
}

float Scene::initialLength(const LinkSpec& l) const { return (endpointWorld(l.b) - endpointWorld(l.a)).length(); }

Scene defaultScene() {
    Scene s;
    auto& box = s.add("A", ObjKind::Box, {0.0f, 0.5f, 0.0f});
    box.mass = 2.0f;
    return s;
}

void build(const Scene& scene, World& world, Built& out) {
    world.clear();
    world.settings = {};
    world.settings.gravity = {0.0f, -scene.world.g, 0.0f};
    world.settings.substeps = scene.world.substeps;
    world.settings.angularDamping = 0.0f; // the editor shows textbook physics: nothing loses spin by itself
    out = {};

    if (scene.world.ground) {
        BodyDef d;
        d.isStatic = true;
        d.position = {0.0f, -1.0f, 0.0f};
        d.color = 0x3a3f4bff;
        d.friction = scene.world.groundFriction;
        d.restitution = 0.0f;
        out.ground = world.createBody(Shape::box({40.0f, 1.0f, 40.0f}), d);
    }

    int colorIndex = 0;
    for (const ObjectSpec& o : scene.objects) {
        const Shape shape = o.shape();
        BodyDef d;
        d.position = o.pos;
        d.orientation = o.orientation();
        d.isStatic = o.fixed;
        d.velocity = o.velocity;
        d.angularVelocity = o.spin;
        d.density = o.mass / std::max(shape.volume(), 1e-9f);
        d.friction = o.friction;
        d.restitution = o.restitution;
        d.rollingResistance = o.rolling;
        d.color = o.color ? o.color : o.fixed ? 0x8d99aeff : kPalette[colorIndex++ % std::size(kPalette)];
        out.bodies.push_back(world.createBody(shape, d));
    }

    for (const LinkSpec& l : scene.links) {
        auto bodyOf = [&](const Endpoint& e) -> Body* {
            for (size_t i = 0; i < scene.objects.size(); ++i)
                if (scene.objects[i].id == e.obj) return out.bodies[i];
            return nullptr; // the world
        };
        Body* a = bodyOf(l.a);
        Body* b = bodyOf(l.b);
        const Vec3 pa = scene.endpointWorld(l.a), pb = scene.endpointWorld(l.b);
        Joint* joint = nullptr;
        switch (l.kind) {
        case LinkKind::Rod:
        case LinkKind::Rope:
        case LinkKind::Spring: {
            auto* d = world.createJoint<DistanceJoint>(a, b, pa, pb);
            if (l.length >= 0.0f) d->length = l.length;
            d->rope = l.kind == LinkKind::Rope;
            if (l.kind == LinkKind::Spring) {
                // the joint is tuned by frequency; k fixes it through the masses the spring joins
                const float invMass = (a ? a->invMass : 0.0f) + (b ? b->invMass : 0.0f);
                d->hertz = std::max(std::sqrt(l.k * invMass) / (2.0f * std::numbers::pi_v<float>), 1e-3f);
                d->dampingRatio = l.dampingRatio;
            }
            joint = d;
            break;
        }
        case LinkKind::BallJoint: {
            auto* j = world.createJoint<BallJoint>(a, b, pa);
            if (l.swingLimitDeg > 0.0f && b) {
                const Vec3 rest = pa - b->position; // b hangs this way from the pivot as placed
                j->limitSwing(rest.lengthSq() > 1e-8f ? rest : Vec3{0, 1, 0}, l.swingLimitDeg * kDeg);
            }
            joint = j;
            break;
        }
        case LinkKind::Hinge: joint = world.createJoint<HingeJoint>(a, b, pa, l.axis.normalized()); break;
        }
        out.joints.push_back(joint);
    }
}

float kineticEnergy(const Body& b) {
    if (b.isStatic) return 0.0f;
    const Vec3 wl = b.rotation.mulT(b.angularVelocity), inv = b.invInertiaLocal;
    return 0.5f * b.mass * b.velocity.lengthSq() + 0.5f * (wl.x * wl.x / inv.x + wl.y * wl.y / inv.y + wl.z * wl.z / inv.z);
}

void Simulation::load(const Scene& scene) {
    scene_ = scene;
    scene_.sanitize();
    build(scene_, world_, built_);
    time_ = 0.0f;
    blewUp_ = false;
    accel_.assign(size(), {});
    before_.assign(size(), {});
}

void Simulation::step() {
    const size_t n = size();
    for (size_t i = 0; i < n; ++i) before_[i] = built_.bodies[i]->velocity;
    world_.step(kDt);
    time_ += kDt;
    constexpr float kLimit = 1e6f;
    for (size_t i = 0; i < n; ++i) {
        const Body* b = built_.bodies[i];
        if (b->isStatic) continue;
        accel_[i] = accel_[i] * 0.5f + (b->velocity - before_[i]) * (0.5f / kDt);
        const Vec3 p = b->position;
        if (!(std::abs(p.x) < kLimit && std::abs(p.y) < kLimit && std::abs(p.z) < kLimit &&
              b->velocity.lengthSq() < kLimit * kLimit && b->angularVelocity.lengthSq() < kLimit * kLimit))
            blewUp_ = true;
    }
}

int Simulation::indexOf(int objId) const {
    for (size_t i = 0; i < scene_.objects.size(); ++i)
        if (scene_.objects[i].id == objId) return static_cast<int>(i);
    return -1;
}

int Simulation::indexNamed(const char* name) const {
    for (size_t i = 0; i < scene_.objects.size(); ++i)
        if (scene_.objects[i].name == name) return static_cast<int>(i);
    return -1;
}

float Simulation::totalEnergy() const {
    float e = 0.0f;
    for (const Body* b : built_.bodies)
        if (!b->isStatic) e += kineticEnergy(*b) + b->mass * scene_.world.g * b->position.y;
    return e;
}

} // namespace editor3
