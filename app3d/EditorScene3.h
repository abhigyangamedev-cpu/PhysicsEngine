#pragma once
// The 3D editor's scene model: initial conditions of every object and link, built into a pe3::World
// on demand. Kept free of SDL and OpenGL so the tests can run it headless.
#include "core/World3.h"
#include <string>
#include <vector>

namespace editor3 {

using pe3::Vec3;

enum class ObjKind { Box, Ball };

// Initial conditions of one object, in SI units. pos is the centre.
struct ObjectSpec {
    int id = 0;
    std::string name;
    ObjKind kind = ObjKind::Box;
    Vec3 pos;
    Vec3 size{1.0f, 1.0f, 1.0f}; // box: full edge lengths
    float radius = 0.5f;         // ball
    Vec3 tiltDeg;                // rotation about x, then y, then z
    float mass = 1.0f;
    Vec3 velocity;
    Vec3 spin; // angular velocity, rad/s
    float friction = 0.5f, restitution = 0.2f, rolling = 0.0f;
    bool fixed = false;
    uint32_t color = 0; // 0 = pick from the palette

    pe3::Shape shape() const;
    pe3::Quat orientation() const;
    float halfHeight() const; // how far the object reaches below its centre when untilted
    void sanitize();
};

enum class LinkKind { Rod, Rope, Spring, BallJoint, Hinge };

// obj < 0: local is a fixed world point; otherwise it's in the object's frame
struct Endpoint {
    int obj = -1;
    Vec3 local;
};

struct LinkSpec {
    int id = 0;
    LinkKind kind = LinkKind::Rod;
    Endpoint a, b; // ball joints and hinges pivot at a
    float length = -1.0f; // rod, rope, spring natural length; < 0 uses the distance as placed
    float k = 50.0f, dampingRatio = 0.0f; // spring
    Vec3 axis{0.0f, 0.0f, 1.0f};          // hinge
    float swingLimitDeg = 0.0f;           // ball joint cone, 0 = free

    void sanitize();
};

struct WorldSpec {
    float g = 9.81f;
    bool ground = true;
    float groundFriction = 0.6f;
    float timeScale = 1.0f;
    float stopAt = 0.0f; // pause when t reaches this, 0 = never
    int substeps = 4;

    void sanitize();
};

struct Scene {
    WorldSpec world;
    std::vector<ObjectSpec> objects;
    std::vector<LinkSpec> links;
    int nextId = 1;

    ObjectSpec& add(const char* name, ObjKind kind, Vec3 pos); // invalidates earlier references
    LinkSpec& link(LinkKind kind, Endpoint a, Endpoint b);
    ObjectSpec* object(int id);
    const ObjectSpec* object(int id) const;
    LinkSpec* findLink(int id);
    void removeObject(int id); // and the links touching it
    void removeLink(int id);
    std::string nextName() const;
    void sanitize();

    Vec3 endpointWorld(const Endpoint& e) const;
    Endpoint endpointAt(int obj, Vec3 world) const;
    float initialLength(const LinkSpec& l) const;
};

Scene defaultScene();

// The running world built from a scene; parallel to scene.objects / scene.links.
struct Built {
    std::vector<pe3::Body*> bodies;
    std::vector<pe3::Joint*> joints;
    pe3::Body* ground = nullptr;
};

void build(const Scene& scene, pe3::World& world, Built& out);

float kineticEnergy(const pe3::Body& b); // translational + rotational

// Runs a scene in fixed steps. Keeps its own copy of the scene, so indices into the bodies always
// match scene().objects.
class Simulation {
public:
    static constexpr float kDt = 1.0f / 120.0f;

    void load(const Scene& scene);
    void step();

    float time() const { return time_; }
    const Scene& scene() const { return scene_; }
    const Built& built() const { return built_; }
    const pe3::World& world() const { return world_; }
    pe3::World& world() { return world_; }
    size_t size() const { return built_.bodies.size(); }

    int indexOf(int objId) const;
    int indexNamed(const char* name) const;
    pe3::Body* body(int index) const { return index >= 0 && index < int(size()) ? built_.bodies[index] : nullptr; }
    pe3::Body* body(const char* name) const { return body(indexNamed(name)); }
    Vec3 acceleration(int index) const { return accel_[index]; }
    float totalEnergy() const; // KE + mgy
    bool blewUp() const { return blewUp_; }

private:
    Scene scene_;
    pe3::World world_;
    Built built_;
    float time_ = 0.0f;
    bool blewUp_ = false;
    std::vector<Vec3> accel_, before_;
};

} // namespace editor3
