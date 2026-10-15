#include "Renderer3.h"
#include "Gl.h"
#include <SDL3/SDL.h>
#include <cmath>
#include <numbers>

using namespace gl;
using pe3::Vec3;

namespace view3d {

Mat4 Mat4::operator*(const Mat4& o) const {
    Mat4 r;
    for (int c = 0; c < 4; ++c)
        for (int row = 0; row < 4; ++row) {
            float s = 0.0f;
            for (int k = 0; k < 4; ++k) s += m[k * 4 + row] * o.m[c * 4 + k];
            r.m[c * 4 + row] = s;
        }
    return r;
}

Mat4 Mat4::perspective(float fovY, float aspect, float n, float f) {
    Mat4 r;
    const float t = 1.0f / std::tan(fovY * 0.5f);
    r.m[0] = t / aspect;
    r.m[5] = t;
    r.m[10] = (f + n) / (n - f);
    r.m[11] = -1.0f;
    r.m[14] = 2.0f * f * n / (n - f);
    r.m[15] = 0.0f;
    return r;
}

Mat4 Mat4::ortho(float hw, float hh, float n, float f) {
    Mat4 r;
    r.m[0] = 1.0f / hw;
    r.m[5] = 1.0f / hh;
    r.m[10] = -2.0f / (f - n);
    r.m[14] = -(f + n) / (f - n);
    return r;
}

Mat4 Mat4::lookAt(Vec3 eye, Vec3 target, Vec3 up) {
    const Vec3 f = (target - eye).normalized();
    const Vec3 s = cross(f, up).normalized();
    const Vec3 u = cross(s, f);
    Mat4 r;
    r.m[0] = s.x, r.m[4] = s.y, r.m[8] = s.z;
    r.m[1] = u.x, r.m[5] = u.y, r.m[9] = u.z;
    r.m[2] = -f.x, r.m[6] = -f.y, r.m[10] = -f.z;
    r.m[12] = -dot(s, eye), r.m[13] = -dot(u, eye), r.m[14] = dot(f, eye);
    return r;
}

Vec3 Camera::eye() const {
    return target + Vec3{std::cos(pitch) * std::sin(yaw), std::sin(pitch), std::cos(pitch) * std::cos(yaw)} * distance;
}

Mat4 Camera::view() const { return Mat4::lookAt(eye(), target, {0, 1, 0}); }

Mat4 Camera::projection(float aspect) const { return Mat4::perspective(fovY, aspect, 0.1f, 400.0f); }

Vec3 Camera::ray(float x, float y, float aspect) const {
    const Vec3 f = (target - eye()).normalized();
    const Vec3 s = cross(f, {0, 1, 0}).normalized();
    const Vec3 u = cross(s, f);
    const float t = std::tan(fovY * 0.5f);
    return (f + s * (x * t * aspect) + u * (y * t)).normalized();
}

bool Camera::project(Vec3 p, float width, float height, float& x, float& y) const {
    const Mat4 vp = projection(width / std::max(height, 1.0f)) * view();
    const float* m = vp.m;
    const float cx = m[0] * p.x + m[4] * p.y + m[8] * p.z + m[12];
    const float cy = m[1] * p.x + m[5] * p.y + m[9] * p.z + m[13];
    const float cw = m[3] * p.x + m[7] * p.y + m[11] * p.z + m[15];
    if (cw <= 1e-4f) return false;
    x = (cx / cw * 0.5f + 0.5f) * width;
    y = (0.5f - cy / cw * 0.5f) * height;
    return true;
}

namespace {

const char* kVertex = R"(#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec4 iModel0;
layout(location = 3) in vec4 iModel1;
layout(location = 4) in vec4 iModel2;
layout(location = 5) in vec4 iModel3;
layout(location = 6) in vec4 iColor;
uniform mat4 uViewProj;
uniform mat4 uLightViewProj;
out vec3 vWorld;
out vec3 vNormal;
out vec3 vLocal;
out vec4 vColor;
out vec4 vLight;
void main() {
    mat4 model = mat4(iModel0, iModel1, iModel2, iModel3);
    vec4 world = model * vec4(aPos, 1.0);
    vWorld = world.xyz;
    vNormal = normalize(transpose(inverse(mat3(model))) * aNormal);
    vLocal = aPos;
    vColor = iColor;
    vLight = uLightViewProj * (world + vec4(vNormal * 0.02, 0.0));
    gl_Position = uViewProj * world;
}
)";

// iColor.a encodes the surface pattern: 0 plain, 1 floor grid, 2 ball caps
const char* kFragment = R"(#version 330 core
in vec3 vWorld;
in vec3 vNormal;
in vec3 vLocal;
in vec4 vColor;
in vec4 vLight;
uniform vec3 uLightDir;
uniform vec3 uEye;
uniform vec3 uSky;
uniform sampler2D uShadow;
out vec4 fragColor;

float shadow(float cosL) {
    vec3 p = vLight.xyz / vLight.w * 0.5 + 0.5;
    float bias = mix(0.004, 0.0008, cosL);
    if (p.z > 1.0 || p.x < 0.0 || p.x > 1.0 || p.y < 0.0 || p.y > 1.0) return 1.0;
    vec2 texel = 1.0 / vec2(textureSize(uShadow, 0));
    float lit = 0.0;
    for (int x = -1; x <= 1; ++x)
        for (int y = -1; y <= 1; ++y)
            lit += p.z - bias > texture(uShadow, p.xy + vec2(x, y) * texel).r ? 0.0 : 1.0;
    return lit / 9.0;
}

void main() {
    vec3 n = normalize(vNormal);
    vec3 base = pow(vColor.rgb, vec3(2.2));
    if (vColor.a > 0.5 && vColor.a < 1.5) {
        vec2 g = vWorld.xz;
        float checker = mod(floor(g.x) + floor(g.y), 2.0);
        base *= mix(0.8, 1.0, checker);
        vec2 f = abs(fract(g - 0.5) - 0.5) / fwidth(g);
        base *= mix(0.6, 1.0, clamp(min(f.x, f.y), 0.0, 1.0));
    } else if (vColor.a > 1.5) {
        base = mix(base, vec3(0.9), step(0.75, abs(vLocal.y)));
    }
    float sh = shadow(clamp(dot(n, uLightDir), 0.0, 1.0));
    float diff = max(dot(n, uLightDir), 0.0) * sh;
    vec3 sky = pow(uSky, vec3(2.2));
    vec3 ambient = mix(vec3(0.05, 0.045, 0.04), sky * 0.6, n.y * 0.5 + 0.5);
    vec3 v = normalize(uEye - vWorld);
    float spec = pow(max(dot(n, normalize(uLightDir + v)), 0.0), 48.0) * 0.25 * sh;
    vec3 c = base * (ambient + diff * vec3(1.0, 0.96, 0.9) * 1.1) + spec;
    float fog = 1.0 - exp(-max(length(uEye - vWorld) - 30.0, 0.0) * 0.015);
    c = mix(c, sky, fog);
    fragColor = vec4(pow(c, vec3(1.0 / 2.2)), 1.0);
}
)";

const char* kShadowVertex = R"(#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 2) in vec4 iModel0;
layout(location = 3) in vec4 iModel1;
layout(location = 4) in vec4 iModel2;
layout(location = 5) in vec4 iModel3;
uniform mat4 uLightViewProj;
void main() { gl_Position = uLightViewProj * mat4(iModel0, iModel1, iModel2, iModel3) * vec4(aPos, 1.0); }
)";

const char* kShadowFragment = R"(#version 330 core
void main() {}
)";

unsigned compile(GLenum type, const char* src) {
    unsigned s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    int ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetShaderInfoLog(s, sizeof log, nullptr, log);
        SDL_Log("shader error: %s", log);
    }
    return s;
}

} // namespace

unsigned linkProgram(const char* vs, const char* fs) {
    unsigned p = glCreateProgram();
    unsigned v = compile(GL_VERTEX_SHADER, vs), f = compile(GL_FRAGMENT_SHADER, fs);
    glAttachShader(p, v);
    glAttachShader(p, f);
    glLinkProgram(p);
    glDeleteShader(v);
    glDeleteShader(f);
    int ok = 0;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetProgramInfoLog(p, sizeof log, nullptr, log);
        SDL_Log("link error: %s", log);
        return 0;
    }
    return p;
}

namespace {

void cubeMesh(std::vector<float>& v, std::vector<unsigned>& idx) {
    for (int axis = 0; axis < 3; ++axis) {
        for (float s : {1.0f, -1.0f}) {
            Vec3 n, a, b;
            n[axis] = s;
            a[(axis + 1) % 3] = 1.0f;
            b[(axis + 2) % 3] = 1.0f;
            if (s < 0) std::swap(a, b);
            const unsigned base = static_cast<unsigned>(v.size() / 6);
            for (auto [x, y] : {std::pair{-1.0f, -1.0f}, {1.0f, -1.0f}, {1.0f, 1.0f}, {-1.0f, 1.0f}}) {
                Vec3 p = n + a * x + b * y;
                v.insert(v.end(), {p.x, p.y, p.z, n.x, n.y, n.z});
            }
            idx.insert(idx.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
        }
    }
}

void sphereMesh(std::vector<float>& v, std::vector<unsigned>& idx) {
    const int rings = 18, segs = 28;
    const float pi = std::numbers::pi_v<float>;
    for (int r = 0; r <= rings; ++r) {
        const float phi = pi * r / rings;
        for (int s = 0; s <= segs; ++s) {
            const float th = 2 * pi * s / segs;
            Vec3 p{std::sin(phi) * std::cos(th), std::cos(phi), std::sin(phi) * std::sin(th)};
            v.insert(v.end(), {p.x, p.y, p.z, p.x, p.y, p.z});
        }
    }
    for (int r = 0; r < rings; ++r)
        for (int s = 0; s < segs; ++s) {
            unsigned a = r * (segs + 1) + s, b = a + segs + 1;
            idx.insert(idx.end(), {a, a + 1, b, a + 1, b + 1, b});
        }
}

void pushRaw(std::vector<float>& out, const pe3::Mat3& axes, Vec3 pos, uint32_t col, float pattern, bool highlight = false,
             bool dim = false) {
    for (const Vec3& c : axes.c) out.insert(out.end(), {c.x, c.y, c.z, 0.0f});
    out.insert(out.end(), {pos.x, pos.y, pos.z, 1.0f});
    float rgb[3] = {((col >> 24) & 255) / 255.0f, ((col >> 16) & 255) / 255.0f, ((col >> 8) & 255) / 255.0f};
    if (highlight)
        for (float& c : rgb) c = c * 0.6f + 0.4f;
    if (dim)
        for (float& c : rgb) c = c * 0.35f + 0.1f;
    out.insert(out.end(), {rgb[0], rgb[1], rgb[2], pattern});
}

void pushInstance(std::vector<float>& out, const pe3::Body& b, Vec3 scale, float pattern, bool highlight, bool dim) {
    pe3::Mat3 axes;
    for (int i = 0; i < 3; ++i) axes.c[i] = b.rotation.c[i] * scale[i];
    pushRaw(out, axes, b.position, b.color ? b.color : 0xb0b8c8ff, pattern, highlight, dim);
}

// a square bar of half-thickness r from p to q
void pushSegment(std::vector<float>& out, Vec3 p, Vec3 q, float r, uint32_t color) {
    const Vec3 d = q - p;
    const float len = d.length();
    if (len < 1e-5f) return;
    pe3::Mat3 axes;
    Vec3 y, z;
    pe3::basis(d / len, y, z);
    axes.c[0] = d * 0.5f;
    axes.c[1] = y * r;
    axes.c[2] = z * r;
    pushRaw(out, axes, (p + q) * 0.5f, color, 0.0f);
}

void pushDot(std::vector<float>& out, Vec3 p, float r, uint32_t color) {
    pushRaw(out, pe3::Mat3::diagonal({r, r, r}), p, color, 0.0f);
}

void pushCoil(std::vector<float>& out, Vec3 p, Vec3 q, uint32_t color) {
    const Vec3 d = q - p;
    const float len = d.length();
    if (len < 1e-5f) return;
    Vec3 u, v;
    pe3::basis(d / len, u, v);
    const int turns = 10, perTurn = 8;
    const float radius = 0.08f;
    Vec3 prev = p;
    for (int i = 1; i <= turns * perTurn; ++i) {
        const float t = static_cast<float>(i) / (turns * perTurn);
        const float a = t * turns * 2.0f * std::numbers::pi_v<float>;
        const float fade = std::min({1.0f, t * 8.0f, (1.0f - t) * 8.0f}); // straight ends attach at the anchors
        const Vec3 cur = p + d * t + (u * std::cos(a) + v * std::sin(a)) * (radius * fade);
        pushSegment(out, prev, cur, 0.012f, color);
        prev = cur;
    }
}

} // namespace

Renderer3::Mesh Renderer3::makeMesh(const std::vector<float>& verts, const std::vector<unsigned>& indices) {
    Mesh m;
    m.indexCount = static_cast<int>(indices.size());
    glGenVertexArrays(1, &m.vao);
    glBindVertexArray(m.vao);
    glGenBuffers(1, &m.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, m.vbo);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glGenBuffers(1, &m.ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m.ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned), indices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, instanceVbo_);
    for (int i = 0; i < 5; ++i) {
        glVertexAttribPointer(2 + i, 4, GL_FLOAT, GL_FALSE, 20 * sizeof(float), reinterpret_cast<void*>(i * 4 * sizeof(float)));
        glEnableVertexAttribArray(2 + i);
        glVertexAttribDivisor(2 + i, 1);
    }
    glBindVertexArray(0);
    return m;
}

bool Renderer3::init() {
    if (!gl::load()) return false;
    program_ = linkProgram(kVertex, kFragment);
    shadowProgram_ = linkProgram(kShadowVertex, kShadowFragment);
    if (!program_ || !shadowProgram_) return false;
    uViewProj_ = glGetUniformLocation(program_, "uViewProj");
    uLightViewProj_ = glGetUniformLocation(program_, "uLightViewProj");
    uLightDir_ = glGetUniformLocation(program_, "uLightDir");
    uEye_ = glGetUniformLocation(program_, "uEye");
    uSky_ = glGetUniformLocation(program_, "uSky");
    uShadow_ = glGetUniformLocation(program_, "uShadow");
    uShadowLightVP_ = glGetUniformLocation(shadowProgram_, "uLightViewProj");

    glGenBuffers(1, &instanceVbo_);
    std::vector<float> v;
    std::vector<unsigned> idx;
    cubeMesh(v, idx);
    cube_ = makeMesh(v, idx);
    v.clear();
    idx.clear();
    sphereMesh(v, idx);
    sphere_ = makeMesh(v, idx);

    glGenTextures(1, &shadowTex_);
    glBindTexture(GL_TEXTURE_2D, shadowTex_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, kShadowSize, kShadowSize, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    const float border[] = {1, 1, 1, 1};
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border);
    glGenFramebuffers(1, &shadowFbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, shadowFbo_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, shadowTex_, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    return true;
}

void Renderer3::drawInstances(const Mesh& mesh, const std::vector<float>& data) {
    if (data.empty()) return;
    glBindBuffer(GL_ARRAY_BUFFER, instanceVbo_);
    glBufferData(GL_ARRAY_BUFFER, data.size() * sizeof(float), data.data(), GL_STREAM_DRAW);
    glBindVertexArray(mesh.vao);
    glDrawElementsInstanced(GL_TRIANGLES, mesh.indexCount, GL_UNSIGNED_INT, nullptr, static_cast<int>(data.size() / 20));
}

void Renderer3::draw(const pe3::World& world, const Camera& cam, int width, int height, const pe3::Body* highlight,
                     bool tintSleeping, std::span<const Line3> extras) {
    boxes_.clear();
    spheres_.clear();
    for (const auto& b : world.bodies()) {
        const bool dim = tintSleeping && !b->isStatic && !b->awake;
        if (b->shape.type == pe3::ShapeType::Box)
            pushInstance(boxes_, *b, b->shape.halfExtents, b->isStatic && b->shape.halfExtents.x > 10 ? 1.0f : 0.0f,
                         b.get() == highlight, dim);
        else
            pushInstance(spheres_, *b, Vec3{1, 1, 1} * b->shape.radius, 2.0f, b.get() == highlight, dim);
    }
    for (const auto& j : world.joints()) {
        const Vec3 pa = j->anchorA(), pb = j->anchorB();
        if (auto* d = dynamic_cast<const pe3::DistanceJoint*>(j.get())) {
            if (d->hertz > 0.0f) pushCoil(boxes_, pa, pb, 0xc9ccd1ff);
            else pushSegment(boxes_, pa, pb, d->rope ? 0.012f : 0.025f, d->rope ? 0x9c7a54ff : 0x7d828aff);
        } else if (auto* m = dynamic_cast<const pe3::MouseJoint*>(j.get())) {
            pushSegment(boxes_, pb, m->target, 0.01f, 0xffd23fff);
            pushDot(spheres_, m->target, 0.05f, 0xffd23fff);
        } else {
            pushDot(spheres_, pb, 0.06f, 0x2b2f36ff);
            if (auto* h = dynamic_cast<const pe3::HingeJoint*>(j.get()))
                pushSegment(boxes_, pb - h->axis() * 0.15f, pb + h->axis() * 0.15f, 0.02f, 0x2b2f36ff);
        }
    }

    for (const Line3& l : extras) pushSegment(boxes_, l.a, l.b, l.radius, l.color);

    const Vec3 lightDir = Vec3{-0.55f, 1.0f, 0.25f}.normalized();
    const Vec3 focus{cam.target.x, 0.0f, cam.target.z};
    const float extent = std::clamp(cam.distance * 1.2f, 12.0f, 60.0f);
    const Mat4 lightVP = Mat4::ortho(extent, extent, 1.0f, 200.0f) * Mat4::lookAt(focus + lightDir * 80.0f, focus, {0, 1, 0});

    glBindFramebuffer(GL_FRAMEBUFFER, shadowFbo_);
    glViewport(0, 0, kShadowSize, kShadowSize);
    glClear(GL_DEPTH_BUFFER_BIT);
    glUseProgram(shadowProgram_);
    glUniformMatrix4fv(uShadowLightVP_, 1, GL_FALSE, lightVP.m);
    glCullFace(GL_FRONT);
    drawInstances(cube_, boxes_);
    drawInstances(sphere_, spheres_);
    glCullFace(GL_BACK);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    const Vec3 sky{0.62f, 0.72f, 0.82f};
    glViewport(0, 0, width, height);
    glClearColor(sky.x, sky.y, sky.z, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glUseProgram(program_);
    const float aspect = static_cast<float>(width) / std::max(height, 1);
    const Mat4 viewProj = cam.projection(aspect) * cam.view();
    const Vec3 eye = cam.eye();
    glUniformMatrix4fv(uViewProj_, 1, GL_FALSE, viewProj.m);
    glUniformMatrix4fv(uLightViewProj_, 1, GL_FALSE, lightVP.m);
    glUniform3f(uLightDir_, lightDir.x, lightDir.y, lightDir.z);
    glUniform3f(uEye_, eye.x, eye.y, eye.z);
    glUniform3f(uSky_, sky.x, sky.y, sky.z);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, shadowTex_);
    glUniform1i(uShadow_, 0);
    drawInstances(cube_, boxes_);
    drawInstances(sphere_, spheres_);
}

} // namespace view3d
