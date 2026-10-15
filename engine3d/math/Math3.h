#pragma once
#include <algorithm>
#include <cmath>

namespace pe3 {

struct Vec3 {
    float x = 0.0f, y = 0.0f, z = 0.0f;

    constexpr Vec3() = default;
    constexpr Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    constexpr Vec3 operator+(Vec3 o) const { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(Vec3 o) const { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }
    constexpr Vec3 operator-() const { return {-x, -y, -z}; }
    constexpr Vec3& operator+=(Vec3 o) { x += o.x; y += o.y; z += o.z; return *this; }
    constexpr Vec3& operator-=(Vec3 o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    constexpr Vec3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }
    constexpr float operator[](int i) const { return i == 0 ? x : i == 1 ? y : z; }
    constexpr float& operator[](int i) { return i == 0 ? x : i == 1 ? y : z; }

    constexpr float lengthSq() const { return x * x + y * y + z * z; }
    float length() const { return std::sqrt(lengthSq()); }
    Vec3 normalized() const {
        float l = length();
        return l > 1e-12f ? *this / l : Vec3{};
    }
};

constexpr Vec3 operator*(float s, Vec3 v) { return v * s; }
constexpr float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
constexpr Vec3 cross(Vec3 a, Vec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
constexpr Vec3 mul(Vec3 a, Vec3 b) { return {a.x * b.x, a.y * b.y, a.z * b.z}; }
inline Vec3 vmin(Vec3 a, Vec3 b) { return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)}; }
inline Vec3 vmax(Vec3 a, Vec3 b) { return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)}; }

// Two unit vectors completing an orthonormal basis with unit n.
inline void basis(Vec3 n, Vec3& t1, Vec3& t2) {
    t1 = std::abs(n.x) >= 0.57735f ? Vec3{n.y, -n.x, 0.0f} : Vec3{0.0f, n.z, -n.y};
    t1 = t1.normalized();
    t2 = cross(n, t1);
}

// Column-major: c[i] is column i.
struct Mat3 {
    Vec3 c[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

    static constexpr Mat3 diagonal(Vec3 d) {
        Mat3 m;
        m.c[0] = {d.x, 0, 0};
        m.c[1] = {0, d.y, 0};
        m.c[2] = {0, 0, d.z};
        return m;
    }
    static constexpr Mat3 zero() { return diagonal({}); }

    constexpr Vec3 operator*(Vec3 v) const { return c[0] * v.x + c[1] * v.y + c[2] * v.z; }
    constexpr Mat3 operator*(const Mat3& o) const {
        Mat3 m;
        for (int i = 0; i < 3; ++i) m.c[i] = *this * o.c[i];
        return m;
    }
    constexpr Mat3 transposed() const {
        Mat3 m;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) m.c[i][j] = c[j][i];
        return m;
    }
    constexpr Vec3 mulT(Vec3 v) const { return {dot(c[0], v), dot(c[1], v), dot(c[2], v)}; }
    constexpr Mat3 operator+(const Mat3& o) const {
        Mat3 m;
        for (int i = 0; i < 3; ++i) m.c[i] = c[i] + o.c[i];
        return m;
    }
    constexpr Mat3 operator*(float s) const {
        Mat3 m;
        for (int i = 0; i < 3; ++i) m.c[i] = c[i] * s;
        return m;
    }
    // [v]x such that skew(v) * u = cross(v, u)
    static constexpr Mat3 skew(Vec3 v) {
        Mat3 m;
        m.c[0] = {0, v.z, -v.y};
        m.c[1] = {-v.z, 0, v.x};
        m.c[2] = {v.y, -v.x, 0};
        return m;
    }
    Vec3 solve(Vec3 b) const {
        float det = dot(c[0], cross(c[1], c[2]));
        if (std::abs(det) < 1e-20f) return {};
        det = 1.0f / det;
        return {det * dot(b, cross(c[1], c[2])), det * dot(c[0], cross(b, c[2])), det * dot(c[0], cross(c[1], b))};
    }
};

struct Quat {
    float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;

    static Quat axisAngle(Vec3 axis, float angle) {
        Vec3 a = axis.normalized() * std::sin(angle * 0.5f);
        return {std::cos(angle * 0.5f), a.x, a.y, a.z};
    }

    constexpr Quat operator*(Quat o) const {
        return {w * o.w - x * o.x - y * o.y - z * o.z, w * o.x + x * o.w + y * o.z - z * o.y,
                w * o.y - x * o.z + y * o.w + z * o.x, w * o.z + x * o.y - y * o.x + z * o.w};
    }
    Quat normalized() const {
        float l = std::sqrt(w * w + x * x + y * y + z * z);
        return l > 1e-12f ? Quat{w / l, x / l, y / l, z / l} : Quat{};
    }
    constexpr Quat conjugate() const { return {w, -x, -y, -z}; }

    constexpr Vec3 rotate(Vec3 v) const {
        Vec3 q{x, y, z};
        Vec3 t = cross(q, v) * 2.0f;
        return v + t * w + cross(q, t);
    }
    constexpr Vec3 unrotate(Vec3 v) const { return conjugate().rotate(v); }

    constexpr Mat3 toMat3() const {
        Mat3 m;
        m.c[0] = {1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)};
        m.c[1] = {2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)};
        m.c[2] = {2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)};
        return m;
    }

    // q += 0.5 * (0, w) * q * h
    Quat integrated(Vec3 omega, float h) const {
        Quat dq = Quat{0.0f, omega.x, omega.y, omega.z} * *this;
        float s = 0.5f * h;
        return Quat{w + dq.w * s, x + dq.x * s, y + dq.y * s, z + dq.z * s}.normalized();
    }
};

struct Aabb {
    Vec3 min, max;

    constexpr bool overlaps(const Aabb& o) const {
        return min.x <= o.max.x && max.x >= o.min.x && min.y <= o.max.y && max.y >= o.min.y &&
               min.z <= o.max.z && max.z >= o.min.z;
    }
};

} // namespace pe3
