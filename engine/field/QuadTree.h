#pragma once
#include "math/Vec2.h"
#include <span>
#include <vector>

namespace pe {

// Barnes-Hut tree over point masses for O(n log n) gravity.
class QuadTree {
public:
    void build(std::span<const Vec2> positions, std::span<const float> masses);
    // Acceleration at p; theta trades accuracy for speed (0 = exact). self is skipped to avoid self-force.
    Vec2 acceleration(Vec2 p, float G, float softeningSq, float theta, int self = -1) const;

private:
    struct Node {
        Vec2 center;
        float half = 0.0f;
        Vec2 weighted; // sum of m * position; becomes the center of mass after finalize
        float mass = 0.0f;
        int firstChild = -1;
        int point = -1;
    };

    std::vector<Node> nodes_;
    std::span<const Vec2> pos_;
    std::span<const float> mass_;

    void insert(int node, int point, int depth);
    int quadrant(const Node& n, Vec2 p) const;
    void subdivide(int node);
};

} // namespace pe
