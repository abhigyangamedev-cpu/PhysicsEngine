#pragma once
#include "core/World.h"
#include "field/Fields.h"
#include <string>
#include <utility>
#include <vector>

namespace editor {

using pe::Vec2;

enum class ObjKind { Block, Ball, Wedge };

// Initial conditions of one object, in SI units. pos is the centroid.
struct ObjectSpec {
    int id = 0;
    std::string name;
    ObjKind kind = ObjKind::Block;
    Vec2 pos;
    float angleDeg = 0.0f;
    float width = 1.0f, height = 1.0f; // block
    float radius = 0.5f;               // ball
    float base = 4.0f, slopeDeg = 30.0f; // wedge: right triangle, slope rises to the right (flip mirrors it)
    bool flip = false;
    float mass = 1.0f;
    Vec2 velocity;
    float omega = 0.0f;
    Vec2 force; // constant applied force
    float friction = 0.0f, restitution = 0.0f, charge = 0.0f;
    bool fixed = false, noSpin = false;
    bool holdVelocity = false; // a drive keeps the speed along the initial velocity, measuring the force it takes

    pe::Shape shape() const;
    void sanitize(); // clamps values into the ranges the inspector allows
};

enum class LinkKind { Spring, Rod, Rope, Hinge, Pulley };

// obj < 0: local is a fixed world point; otherwise it's in the object's frame
struct Endpoint {
    int obj = -1;
    Vec2 local;
};

struct LinkSpec {
    int id = 0;
    LinkKind kind = LinkKind::Spring;
    Endpoint a, b;
    float k = 50.0f, damping = 0.0f;
    float length = -1.0f; // natural length of a spring / length of a rod or rope; < 0 uses the initial distance
    Vec2 wheel;
    float wheelRadius = 0.3f, ratio = 1.0f;

    void sanitize();
};

struct WorldSpec {
    float g = 9.81f;
    float timeScale = 1.0f;
    float stopAt = 0.0f; // pause when t reaches this, 0 = never
    float drag = 0.0f;
    Vec2 E;
    float B = 0.0f;
    bool coulomb = false;
    float coulombK = 8.99e9f;
    bool mutualGravity = false;
    float G = 1.0f;

    void sanitize();
};

struct Scene {
    WorldSpec world;
    std::vector<ObjectSpec> objects;
    std::vector<LinkSpec> links;
    int nextId = 1;

    ObjectSpec* object(int id);
    const ObjectSpec* object(int id) const;
    LinkSpec* link(int id);
    void removeObject(int id);
    void removeLink(int id);
    std::string nextName() const;
    void sanitize();

    Vec2 endpointWorld(const Endpoint& e) const;
    Endpoint endpointAt(int obj, Vec2 world) const;
    float initialLength(const LinkSpec& l) const;

    bool save(const char* path) const;
    bool load(const char* path);
    void renumber(); // makes every id unique and positive, keeping links on the objects they named
};

Scene defaultScene();

// Where the two strands of a rope from a to b leave the pulley wheel: both wrap the same way and don't cross.
std::pair<Vec2, Vec2> wheelContacts(Vec2 center, float radius, Vec2 a, Vec2 b);

// The running world built from a scene; parallel to scene.objects / scene.links.
struct Built {
    std::vector<pe::Body*> bodies;
    std::vector<int> springIndex;  // per link, -1 when not a spring
    std::vector<pe::Joint*> joints; // per link, null when a spring
    pe::SpringField* springs = nullptr;
};

void build(const Scene& scene, pe::World& world, Built& out);

} // namespace editor
