#pragma once
#include "core/World3.h"
#include <cstdint>
#include <vector>

namespace view3d {

struct Mat4 {
    float m[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}; // column-major

    Mat4 operator*(const Mat4& o) const;
    static Mat4 perspective(float fovY, float aspect, float zNear, float zFar);
    static Mat4 ortho(float halfW, float halfH, float zNear, float zFar);
    static Mat4 lookAt(pe3::Vec3 eye, pe3::Vec3 target, pe3::Vec3 up);
};

struct Camera {
    pe3::Vec3 target{0, 2, 0};
    float yaw = 0.6f, pitch = 0.35f, distance = 18.0f;
    float fovY = 0.9f;

    pe3::Vec3 eye() const;
    Mat4 view() const;
    Mat4 projection(float aspect) const;
    pe3::Vec3 ray(float ndcX, float ndcY, float aspect) const;
};

unsigned linkProgram(const char* vertexSource, const char* fragmentSource); // 0 on failure, errors logged

class Renderer3 {
public:
    bool init();
    void draw(const pe3::World& world, const Camera& cam, int width, int height, const pe3::Body* highlight,
              bool tintSleeping = false);

private:
    struct Mesh {
        unsigned vao = 0, vbo = 0, ibo = 0;
        int indexCount = 0;
    };
    Mesh cube_, sphere_;
    unsigned instanceVbo_ = 0;
    unsigned program_ = 0, shadowProgram_ = 0;
    unsigned shadowFbo_ = 0, shadowTex_ = 0;
    static constexpr int kShadowSize = 2048;
    std::vector<float> boxes_, spheres_;

    Mesh makeMesh(const std::vector<float>& verts, const std::vector<unsigned>& indices);
    void drawInstances(const Mesh& mesh, const std::vector<float>& data);
};

} // namespace view3d
