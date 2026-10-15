#include "rigid/Shape.h"
#include <algorithm>
#include <numbers>

namespace pe {

namespace {

std::vector<Vec2> convexHull(std::vector<Vec2> pts) {
    std::sort(pts.begin(), pts.end(), [](Vec2 a, Vec2 b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
    if (pts.size() < 3) return pts;

    std::vector<Vec2> hull(pts.size() * 2);
    size_t k = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        while (k >= 2 && cross(hull[k - 1] - hull[k - 2], pts[i] - hull[k - 2]) <= 0) --k;
        hull[k++] = pts[i];
    }
    for (size_t i = pts.size() - 1, t = k + 1; i-- > 0;) {
        while (k >= t && cross(hull[k - 1] - hull[k - 2], pts[i] - hull[k - 2]) <= 0) --k;
        hull[k++] = pts[i];
    }
    hull.resize(k - 1);
    return hull;
}

} // namespace

Shape Shape::circle(float radius) {
    Shape s;
    s.type = ShapeType::Circle;
    s.radius = radius;
    return s;
}

Shape Shape::box(float hw, float hh) {
    return polygon({{-hw, -hh}, {hw, -hh}, {hw, hh}, {-hw, hh}});
}

Shape Shape::polygon(std::vector<Vec2> points) {
    constexpr float kMinHalf = 0.005f;
    Aabb box{{1e30f, 1e30f}, {-1e30f, -1e30f}};
    for (Vec2 p : points) {
        box.min = {std::min(box.min.x, p.x), std::min(box.min.y, p.y)};
        box.max = {std::max(box.max.x, p.x), std::max(box.max.y, p.y)};
    }

    Shape s;
    s.type = ShapeType::Polygon;
    s.vertices = convexHull(std::move(points));

    // shift so the centroid sits at the origin
    Vec2 c;
    float area = 0.0f;
    const size_t n = s.vertices.size();
    for (size_t i = 0; i < n; ++i) {
        Vec2 a = s.vertices[i], b = s.vertices[(i + 1) % n];
        float tri = 0.5f * cross(a, b);
        area += tri;
        c += (a + b) * (tri / 3.0f);
    }
    // too few or collinear points: fall back to a thin box over their extent
    if (n < 3 || !(area > 1e-8f)) {
        Vec2 half = box.min.x > box.max.x ? Vec2{} : (box.max - box.min) * 0.5f;
        half = {std::max(half.x, kMinHalf), std::max(half.y, kMinHalf)};
        return polygon({{-half.x, -half.y}, {half.x, -half.y}, {half.x, half.y}, {-half.x, half.y}});
    }
    c = c / area;
    for (auto& v : s.vertices) v -= c;

    s.normals.resize(n);
    for (size_t i = 0; i < n; ++i) {
        Vec2 e = s.vertices[(i + 1) % n] - s.vertices[i];
        s.normals[i] = Vec2{e.y, -e.x}.normalized();
    }
    return s;
}

MassData Shape::computeMass(float density) const {
    MassData md;
    if (type == ShapeType::Circle) {
        md.mass = density * std::numbers::pi_v<float> * radius * radius;
        md.inertia = 0.5f * md.mass * radius * radius;
        return md;
    }
    float area = 0.0f, inertia = 0.0f;
    const size_t n = vertices.size();
    for (size_t i = 0; i < n; ++i) {
        Vec2 a = vertices[i], b = vertices[(i + 1) % n];
        float cr = cross(a, b);
        area += 0.5f * cr;
        inertia += cr * (dot(a, a) + dot(a, b) + dot(b, b)) / 12.0f;
    }
    md.mass = density * area;
    md.inertia = density * inertia;
    return md;
}

} // namespace pe
