#include "field/QuadTree.h"
#include <algorithm>

namespace pe {

namespace {
constexpr int kMaxDepth = 24;
}

void QuadTree::build(std::span<const Vec2> positions, std::span<const float> masses) {
    pos_ = positions;
    mass_ = masses;
    nodes_.clear();
    if (positions.empty()) return;

    Vec2 lo{1e30f, 1e30f}, hi{-1e30f, -1e30f};
    for (Vec2 p : positions) {
        lo = {std::min(lo.x, p.x), std::min(lo.y, p.y)};
        hi = {std::max(hi.x, p.x), std::max(hi.y, p.y)};
    }
    Node root;
    root.center = (lo + hi) * 0.5f;
    root.half = std::max(hi.x - lo.x, hi.y - lo.y) * 0.5f + 1e-3f;
    nodes_.reserve(positions.size() * 3);
    nodes_.push_back(root);

    for (int i = 0; i < static_cast<int>(positions.size()); ++i)
        if (masses[i] > 0.0f) insert(0, i, 0);

    for (auto& n : nodes_)
        if (n.mass > 0.0f) n.weighted = n.weighted / n.mass;
}

int QuadTree::quadrant(const Node& n, Vec2 p) const {
    return (p.x >= n.center.x ? 1 : 0) | (p.y >= n.center.y ? 2 : 0);
}

void QuadTree::subdivide(int node) {
    const float h = nodes_[node].half * 0.5f;
    const Vec2 c = nodes_[node].center;
    const int first = static_cast<int>(nodes_.size());
    for (int q = 0; q < 4; ++q) {
        Node child;
        child.center = c + Vec2{(q & 1) ? h : -h, (q & 2) ? h : -h};
        child.half = h;
        nodes_.push_back(child);
    }
    nodes_[node].firstChild = first;
}

void QuadTree::insert(int node, int point, int depth) {
    const Vec2 p = pos_[point];
    const float m = mass_[point];
    for (;;) {
        nodes_[node].mass += m;
        nodes_[node].weighted += p * m;

        if (nodes_[node].firstChild < 0) {
            if (nodes_[node].point < 0 && nodes_[node].mass == m) {
                nodes_[node].point = point;
                return;
            }
            if (depth >= kMaxDepth) return; // coincident points: keep aggregated in this leaf

            int existing = nodes_[node].point;
            nodes_[node].point = -1;
            subdivide(node);
            if (existing >= 0) {
                int child = nodes_[node].firstChild + quadrant(nodes_[node], pos_[existing]);
                nodes_[child].mass += mass_[existing];
                nodes_[child].weighted += pos_[existing] * mass_[existing];
                nodes_[child].point = existing;
            }
        }
        node = nodes_[node].firstChild + quadrant(nodes_[node], p);
        ++depth;
    }
}

Vec2 QuadTree::acceleration(Vec2 p, float G, float softeningSq, float theta, int self) const {
    if (nodes_.empty()) return {};
    Vec2 a;
    const float thetaSq = theta * theta;
    int stack[256];
    int top = 0;
    stack[top++] = 0;
    while (top > 0) {
        const Node& n = nodes_[stack[--top]];
        if (n.mass == 0.0f) continue;
        if (n.point >= 0) {
            if (n.point == self) continue;
            if (n.mass == mass_[n.point]) {
                Vec2 d = pos_[n.point] - p;
                float r2 = d.lengthSq() + softeningSq;
                a += d * (G * n.mass / (r2 * std::sqrt(r2)));
                continue;
            }
        }
        Vec2 d = n.weighted - p;
        float distSq = d.lengthSq();
        float size = 2.0f * n.half;
        if (n.firstChild < 0 || size * size < thetaSq * distSq) {
            float r2 = distSq + softeningSq;
            a += d * (G * n.mass / (r2 * std::sqrt(r2)));
        } else {
            for (int q = 0; q < 4 && top < 252; ++q) stack[top++] = n.firstChild + q;
        }
    }
    return a;
}

} // namespace pe
