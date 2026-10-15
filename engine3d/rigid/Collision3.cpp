#include "rigid/Collision3.h"
#include <utility>

namespace pe3 {

namespace {

struct Obb {
    Vec3 c, u[3], e;
};

Obb toObb(const Body* b) { return {b->position, {b->rotation.c[0], b->rotation.c[1], b->rotation.c[2]}, b->shape.halfExtents}; }

float signOf(float v) { return v < 0.0f ? -1.0f : 1.0f; }

void addContact(Manifold& m, Vec3 point, float depth) {
    if (m.count >= kMaxContacts) return;
    m.contacts[m.count].point = point;
    m.contacts[m.count].depth = depth;
    ++m.count;
}

bool sphereSphere(const Body* a, const Body* b, float margin, Manifold& m) {
    Vec3 d = b->position - a->position;
    float dist = d.length();
    float sep = dist - a->shape.radius - b->shape.radius;
    if (sep > margin) return false;
    m.normal = dist > 1e-6f ? d / dist : Vec3{0, 1, 0};
    Vec3 pa = a->position + m.normal * a->shape.radius;
    Vec3 pb = b->position - m.normal * b->shape.radius;
    addContact(m, (pa + pb) * 0.5f, -sep);
    return true;
}

// normal from sphere to box
bool sphereBox(const Body* s, const Body* box, float margin, Manifold& m) {
    const Vec3 e = box->shape.halfExtents;
    const Vec3 c = box->toLocal(s->position);
    Vec3 q = vmax(-e, vmin(c, e));
    Vec3 nOut;
    float sep;
    if (q.x == c.x && q.y == c.y && q.z == c.z) {
        int axis = 0;
        float best = e.x - std::abs(c.x);
        for (int i = 1; i < 3; ++i) {
            float d = e[i] - std::abs(c[i]);
            if (d < best) best = d, axis = i;
        }
        nOut[axis] = signOf(c[axis]);
        q[axis] = signOf(c[axis]) * e[axis];
        sep = -best - s->shape.radius;
    } else {
        Vec3 delta = c - q;
        float dist = delta.length();
        sep = dist - s->shape.radius;
        if (sep > margin) return false;
        nOut = delta / dist;
    }
    if (sep > margin) return false;
    m.normal = -(box->rotation * nOut);
    Vec3 pBox = box->toWorld(q);
    Vec3 pSphere = s->position + m.normal * s->shape.radius;
    addContact(m, (pBox + pSphere) * 0.5f, -sep);
    return true;
}

void reduce(Vec3* pts, float* depths, int& n, Vec3 normal) {
    if (n <= kMaxContacts) return;
    int i0 = 0;
    for (int i = 1; i < n; ++i)
        if (depths[i] > depths[i0]) i0 = i;
    int i1 = -1;
    float far = -1.0f;
    for (int i = 0; i < n; ++i) {
        float d = (pts[i] - pts[i0]).lengthSq();
        if (i != i0 && d > far) far = d, i1 = i;
    }
    int i2 = -1, i3 = -1;
    float maxA = 0.0f, minA = 0.0f;
    const Vec3 edge = pts[i1] - pts[i0];
    for (int i = 0; i < n; ++i) {
        if (i == i0 || i == i1) continue;
        float area = dot(cross(edge, pts[i] - pts[i0]), normal);
        if (area > maxA) maxA = area, i2 = i;
        if (area < minA) minA = area, i3 = i;
    }
    Vec3 p[4];
    float d[4];
    int k = 0;
    for (int idx : {i0, i1, i2, i3})
        if (idx >= 0) p[k] = pts[idx], d[k] = depths[idx], ++k;
    for (int i = 0; i < k; ++i) pts[i] = p[i], depths[i] = d[i];
    n = k;
}

// nRef points from the reference box toward the incident box along reference axis i.
void clipFaces(const Obb& ref, const Obb& inc, int i, Vec3 nRef, float margin, Manifold& m) {
    const Vec3 refCenter = ref.c + nRef * ref.e[i];

    int j = 0;
    float best = -1.0f;
    for (int k = 0; k < 3; ++k) {
        float d = std::abs(dot(inc.u[k], nRef));
        if (d > best) best = d, j = k;
    }
    const float s = dot(inc.u[j], nRef) > 0.0f ? -1.0f : 1.0f;
    const Vec3 ic = inc.c + inc.u[j] * (s * inc.e[j]);
    const Vec3 a1 = inc.u[(j + 1) % 3] * inc.e[(j + 1) % 3];
    const Vec3 a2 = inc.u[(j + 2) % 3] * inc.e[(j + 2) % 3];

    // a quad clipped by 4 planes has at most 8 vertices; the extra room only absorbs float noise
    constexpr int kCap = 16;
    Vec3 poly[kCap] = {ic + a1 + a2, ic - a1 + a2, ic - a1 - a2, ic + a1 - a2};
    int count = 4;
    Vec3 tmp[kCap];

    for (int k : {(i + 1) % 3, (i + 2) % 3}) {
        for (float sign : {1.0f, -1.0f}) {
            const Vec3 pn = ref.u[k] * sign;
            const float off = dot(pn, ref.c) + ref.e[k];
            int out = 0;
            for (int v = 0; v < count && out < kCap - 1; ++v) {
                Vec3 p = poly[v], q = poly[(v + 1) % count];
                float dp = dot(pn, p) - off, dq = dot(pn, q) - off;
                // one classification for both tests, so a vertex on the plane is never also an intersection
                const bool inP = dp <= 0.0f, inQ = dq <= 0.0f;
                if (inP) tmp[out++] = p;
                if (inP != inQ) tmp[out++] = p + (q - p) * (dp / (dp - dq));
            }
            count = out;
            for (int v = 0; v < count; ++v) poly[v] = tmp[v];
            if (count == 0) return;
        }
    }

    Vec3 pts[kCap];
    float depths[kCap];
    int n = 0;
    for (int v = 0; v < count; ++v) {
        float sep = dot(nRef, poly[v] - refCenter);
        if (sep > margin) continue;
        pts[n] = poly[v] - nRef * (sep * 0.5f);
        depths[n] = -sep;
        ++n;
    }
    reduce(pts, depths, n, nRef);
    for (int v = 0; v < n; ++v) addContact(m, pts[v], depths[v]);
}

bool boxBox(const Body* a, const Body* b, float margin, Manifold& m) {
    const Obb A = toObb(a), B = toObb(b);
    const Vec3 d = B.c - A.c;

    auto separation = [&](Vec3 L) {
        float ra = 0.0f, rb = 0.0f;
        for (int k = 0; k < 3; ++k) {
            ra += A.e[k] * std::abs(dot(A.u[k], L));
            rb += B.e[k] * std::abs(dot(B.u[k], L));
        }
        return std::abs(dot(d, L)) - ra - rb;
    };

    float faceA = -1e30f, faceB = -1e30f, edge = -1e30f;
    int axA = 0, axB = 0, edgeI = 0, edgeJ = 0;
    Vec3 edgeAxis;
    for (int i = 0; i < 3; ++i) {
        float s = separation(A.u[i]);
        if (s > margin) return false;
        if (s > faceA) faceA = s, axA = i;
    }
    for (int i = 0; i < 3; ++i) {
        float s = separation(B.u[i]);
        if (s > margin) return false;
        if (s > faceB) faceB = s, axB = i;
    }
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Vec3 L = cross(A.u[i], B.u[j]);
            float len = L.length();
            if (len < 1e-4f) continue;
            L = L / len;
            float s = separation(L);
            if (s > margin) return false;
            if (s > edge) edge = s, edgeI = i, edgeJ = j, edgeAxis = L;
        }
    }

    const float faceBest = std::max(faceA, faceB);
    if (edge > faceBest + 0.01f) {
        Vec3 n = dot(d, edgeAxis) < 0.0f ? -edgeAxis : edgeAxis;
        Vec3 pA = A.c, pB = B.c;
        for (int k = 0; k < 3; ++k) {
            if (k != edgeI) pA += A.u[k] * (A.e[k] * signOf(dot(A.u[k], n)));
            if (k != edgeJ) pB += B.u[k] * (B.e[k] * -signOf(dot(B.u[k], n)));
        }
        const Vec3 dA = A.u[edgeI], dB = B.u[edgeJ];
        const Vec3 r = pA - pB;
        const float bb = dot(dA, dB), c = dot(dA, r), f = dot(dB, r);
        const float denom = 1.0f - bb * bb;
        float sA = denom > 1e-6f ? (bb * f - c) / denom : 0.0f;
        sA = std::clamp(sA, -A.e[edgeI], A.e[edgeI]);
        float sB = std::clamp(bb * sA + f, -B.e[edgeJ], B.e[edgeJ]);
        m.normal = n;
        addContact(m, (pA + dA * sA + pB + dB * sB) * 0.5f, -edge);
        return true;
    }

    if (faceB > faceA + 0.001f) {
        Vec3 nRef = dot(d, B.u[axB]) < 0.0f ? B.u[axB] : -B.u[axB];
        m.normal = -nRef;
        clipFaces(B, A, axB, nRef, margin, m);
    } else {
        Vec3 nRef = dot(d, A.u[axA]) < 0.0f ? -A.u[axA] : A.u[axA];
        m.normal = nRef;
        clipFaces(A, B, axA, nRef, margin, m);
    }
    return m.count > 0;
}

} // namespace

bool collide(Body* a, Body* b, float margin, Manifold& m) {
    m.a = a;
    m.b = b;
    m.count = 0;
    const ShapeType ta = a->shape.type, tb = b->shape.type;
    bool hit;
    if (ta == ShapeType::Sphere && tb == ShapeType::Sphere) {
        hit = sphereSphere(a, b, margin, m);
    } else if (ta == ShapeType::Sphere) {
        hit = sphereBox(a, b, margin, m);
    } else if (tb == ShapeType::Sphere) {
        hit = sphereBox(b, a, margin, m);
        m.normal = -m.normal;
    } else {
        hit = boxBox(a, b, margin, m);
    }
    if (!hit) return false;
    basis(m.normal, m.tangent[0], m.tangent[1]);
    for (int i = 0; i < m.count; ++i) m.contacts[i].localA = a->toLocal(m.contacts[i].point);
    return true;
}

} // namespace pe3
