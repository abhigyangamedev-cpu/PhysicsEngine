// Optics: a 2D ray tracer over glass and mirror bodies. Rays refract by Snell's law, reflect
// totally past the critical angle, and the prism's index depends on wavelength (Cauchy's law).
// Every optical element and light source is a static body you can drag.
#include "SceneKit.h"
#include <algorithm>
#include <cmath>

using namespace pe;
using namespace kit;

namespace {

struct Optic {
    Body* body;
    bool mirror;
    float cauchyA = 1.5f, cauchyB = 0.0f; // n(lambda) = A + B / lambda^2, lambda in micrometres

    float index(float lambda) const { return cauchyA + cauchyB / (lambda * lambda); }
};
std::vector<Optic> g_optics;

constexpr uint32_t kGlass = 0x8fc8ff40, kMirror = 0xd0d6e0ff, kLight = 0xffe9a0ff;

Body* addElement(World& w, std::vector<Vec2> worldPoints, uint32_t color, bool draggable = true) {
    Body* b = addStaticPolygon(w, std::move(worldPoints));
    b->collides = false;
    b->draggable = draggable;
    b->color = color;
    return b;
}

void addGlass(World& w, std::vector<Vec2> pts, float a = 1.5f, float b = 0.0f) {
    g_optics.push_back({addElement(w, std::move(pts), kGlass), false, a, b});
}

void addMirror(World& w, Vec2 a, Vec2 b, bool draggable = true) {
    Vec2 n = (b - a).normalized().perp() * 0.06f;
    g_optics.push_back({addElement(w, {a - n, b - n, b + n, a + n}, kMirror, draggable), true});
}

Body* addSource(World& w, Vec2 pos) {
    BodyDef d;
    d.position = pos;
    d.isStatic = true;
    d.collides = false;
    d.draggable = true;
    d.color = kLight;
    return w.createBody(Shape::circle(0.3f), d);
}

struct Hit {
    float t = 1e30f;
    Vec2 normal; // outward from the element
    const Optic* optic = nullptr;
};

Hit nearestHit(Vec2 o, Vec2 d) {
    Hit best;
    for (const Optic& op : g_optics) {
        const auto& v = op.body->worldVertices;
        const auto& n = op.body->worldNormals;
        for (size_t i = 0; i < v.size(); ++i) {
            Vec2 a = v[i], e = v[(i + 1) % v.size()] - a;
            float den = cross(d, e);
            if (std::abs(den) < 1e-9f) continue;
            float t = cross(a - o, e) / den, s = cross(a - o, d) / den;
            if (t > 1e-4f && s >= 0.0f && s <= 1.0f && t < best.t) best = {t, n[i], &op};
        }
    }
    return best;
}

// Traces one ray, drawing each leg, and returns its final direction. `legs` collects each leg's start and direction.
Vec2 trace(Readout& r, Vec2 o, Vec2 d, float lambda, uint32_t color, uint32_t firstLegColor = 0,
           std::vector<std::pair<Vec2, Vec2>>* legs = nullptr) {
    for (int bounce = 0; bounce < 40; ++bounce) {
        Hit h = nearestHit(o, d);
        Vec2 end = o + d * std::min(h.t, 60.0f);
        r.line(o, end, bounce == 0 && firstLegColor ? firstLegColor : color);
        if (legs) legs->push_back({o, d});
        if (!h.optic) break;
        o = end;
        if (h.optic->mirror) {
            d = d - h.normal * (2.0f * dot(d, h.normal));
            continue;
        }
        const bool entering = dot(d, h.normal) < 0.0f;
        const float n = h.optic->index(lambda), eta = entering ? 1.0f / n : n;
        const Vec2 N = entering ? h.normal : -h.normal; // faces the incoming ray
        const float cosi = -dot(d, N), k = 1.0f - eta * eta * (1.0f - cosi * cosi);
        d = k < 0.0f ? d + N * (2.0f * cosi) // total internal reflection
                     : d * eta + N * (eta * cosi - std::sqrt(k));
        d = d.normalized();
    }
    return d;
}

float degrees(float rad) { return rad * 180.0f / kPi; }

// ---- Refraction and total internal reflection ----

Body* g_source = nullptr;
Body* g_block = nullptr;

void refraction(World& w) {
    g_optics.clear();
    w.settings.gravity = {};
    // half disc, flat face on the right: rays aimed at its centre cross the curved side head-on
    std::vector<Vec2> pts;
    const Vec2 c{3.0f, 7.0f};
    for (int i = 0; i <= 24; ++i) {
        float a = kPi * 0.5f + kPi * i / 24.0f;
        pts.push_back(c + Vec2{std::cos(a), std::sin(a)} * 4.5f);
    }
    addGlass(w, pts);
    g_block = g_optics.back().body;
    g_source = addSource(w, {-8.0f, 3.0f});
}

void refractionReadout(const World&, Readout& r) {
    const float n = 1.5f, critical = std::asin(1.0f / n);
    // the half disc's centre is the midpoint of its flat face
    const Vec2 center = g_block->position + g_block->rot.apply({4.5f * 4.0f / (3.0f * kPi), 0.0f});
    const Vec2 aim = (center - g_source->position).normalized();
    for (int i = -4; i <= 4; ++i) {
        float a = i * 0.012f;
        Vec2 d{aim.x * std::cos(a) - aim.y * std::sin(a), aim.x * std::sin(a) + aim.y * std::cos(a)};
        trace(r, g_source->position, d, 0.55f, i == 0 ? 0xffe14fff : 0xffe14f70);
    }
    const float incidence = std::acos(std::clamp(std::abs(aim.x), 0.0f, 1.0f)); // the flat face is vertical
    r.add("Refraction: n1 sin(t1) = n2 sin(t2).  Glass n = 1.5, critical angle %.1f deg", degrees(critical));
    if (incidence < critical)
        r.add("angle at the flat face %.1f deg -> leaves at %.1f deg", degrees(incidence),
              degrees(std::asin(std::min(1.0f, n * std::sin(incidence)))));
    else
        r.add("angle at the flat face %.1f deg -> beyond critical: TOTAL INTERNAL REFLECTION", degrees(incidence));
    r.add("Drag the light (yellow) up or down, or drag the glass");
}

// ---- Prism dispersion ----

struct Wave {
    float lambda;
    uint32_t color;
};
constexpr Wave kSpectrum[] = {{0.41f, 0x9a5cffd0}, {0.45f, 0x4f6fffd0}, {0.49f, 0x2fd0e0d0}, {0.53f, 0x4fe04fd0},
                              {0.58f, 0xffe14fd0}, {0.62f, 0xffa03fd0}, {0.68f, 0xff4f4fd0}};
constexpr float kPrismA = 1.5f, kPrismB = 0.012f;
Body* g_prism = nullptr;

void prism(World& w) {
    g_optics.clear();
    w.settings.gravity = {};
    const Vec2 c{1.0f, 6.5f};
    const float side = 7.0f, h = side * std::sqrt(3.0f) * 0.5f;
    addGlass(w, {c + Vec2{-side * 0.5f, -h / 3.0f}, c + Vec2{side * 0.5f, -h / 3.0f}, c + Vec2{0.0f, h * 2.0f / 3.0f}},
             kPrismA, kPrismB);
    g_prism = g_optics.back().body;
    g_source = addSource(w, {-12.0f, 4.0f});
}

void prismReadout(const World&, Readout& r) {
    const Vec2 target = g_prism->position + Vec2{-0.9f, 0.4f};
    const Vec2 aim = (target - g_source->position).normalized();
    float devMin = 1e9f, devMax = -1e9f;
    for (const Wave& wv : kSpectrum) {
        Vec2 out = trace(r, g_source->position, aim, wv.lambda, wv.color, 0xffffffe0);
        float dev = degrees(std::acos(std::clamp(dot(out, aim), -1.0f, 1.0f)));
        devMin = std::min(devMin, dev);
        devMax = std::max(devMax, dev);
    }
    r.add("Dispersion: glass bends short wavelengths more, n = A + B / lambda^2");
    r.add("n(violet 410 nm) = %.3f   n(red 680 nm) = %.3f", kPrismA + kPrismB / (0.41f * 0.41f), kPrismA + kPrismB / (0.68f * 0.68f));
    r.add("deviation: red %.1f deg, violet %.1f deg   spread %.1f deg", devMin, devMax, devMax - devMin);
    r.add("Drag the light or the prism");
}

// ---- Lenses ----

struct Lens {
    Body* body;
    float radius, focal, thickness;
};
std::vector<Lens> g_lenses;
constexpr float kLensN = 1.5f, kAxisY = 7.0f;

// Biconvex lens: the overlap of two circles of radius R, cut to a half-aperture.
void addLens(World& w, float x, float R, float aperture) {
    const float sag = R - std::sqrt(R * R - aperture * aperture);
    std::vector<Vec2> pts;
    for (int side : {-1, 1})
        for (int i = 0; i <= 64; ++i) { // fine facets: a coarse polygon focuses noticeably short
            float y = -aperture + 2.0f * aperture * i / 64.0f;
            float bulge = std::sqrt(R * R - y * y) - (R - sag); // thickness of this side at height y
            pts.push_back({x + side * (bulge + 0.05f), kAxisY + y});
        }
    addGlass(w, pts, kLensN);
    // thin lens, lensmaker's equation: 1/f = (n - 1)(2 / R)
    g_lenses.push_back({g_optics.back().body, R, R / (2.0f * (kLensN - 1.0f)), 2.0f * sag + 0.1f});
}

void lenses(World& w) {
    g_optics.clear();
    g_lenses.clear();
    w.settings.gravity = {};
    addLens(w, -6.0f, 8.0f, 2.6f);
    addLens(w, 6.0f, 4.0f, 2.6f); // f1 + f2 apart: a Keplerian telescope
}

void lensesReadout(const World&, Readout& r) {
    float focusSum = 0.0f;
    int focusCount = 0;
    for (int i = -4; i <= 4; ++i) {
        if (i == 0) continue;
        const float y = kAxisY + i * 0.55f;
        std::vector<std::pair<Vec2, Vec2>> legs;
        trace(r, {-16.0f, y}, {1.0f, 0.0f}, 0.55f, 0xffe14fd0, 0, &legs);
        // where the inner rays cross the axis after the first lens: its paraxial focus
        if (std::abs(i) == 1 && legs.size() > 2) {
            auto [o, d] = legs[2];
            if (std::abs(d.y) > 1e-5f) {
                focusSum += o.x + d.x * (kAxisY - o.y) / d.y;
                ++focusCount;
            }
        }
    }
    const Lens &a = g_lenses[0], &b = g_lenses[1];
    // a real lens has thickness t: 1/f = (n-1)(2/R - (n-1) t / (n R^2)), focus f (1 - (n-1) t / (n R)) past its back
    const float n = kLensN, t = a.thickness, R = a.radius;
    const float fThick = 1.0f / ((n - 1.0f) * (2.0f / R - (n - 1.0f) * t / (n * R * R)));
    const float backFocus = fThick * (1.0f - (n - 1.0f) * t / (n * R)) + 0.5f * t;
    r.add("Lenses: lensmaker's equation 1/f = (n - 1)(1/R1 - 1/R2), n = %.1f", kLensN);
    r.add("lens 1: R = %.0f, thin-lens f = %.1f; this %.1f m thick lens focuses %.1f m past its centre, measured %.1f",
          a.radius, a.focal, t, backFocus, focusCount ? focusSum / focusCount - a.body->position.x : 0.0f);
    r.add("lens 2: R = %.0f, f = %.1f   %.1f m apart (f1 + f2 = %.1f: parallel in, parallel out, %.1fx narrower)",
          b.radius, b.focal, b.body->position.x - a.body->position.x, a.focal + b.focal, a.focal / b.focal);
    r.add("Outer rays focus short of the paraxial focus: spherical aberration. Drag the lenses");
    for (float x = -16.0f; x < 16.0f; x += 0.8f) r.line({x, kAxisY}, {x + 0.4f, kAxisY}, 0x5a5f6eff);
}

// ---- Mirrors ----

constexpr float kMirrorFocal = 4.0f;
const Vec2 kVertex{11.0f, 7.0f};
Body* g_planeMirror = nullptr;

void mirrors(World& w) {
    g_optics.clear();
    w.settings.gravity = {};
    // a parabola, x = vertex - y^2 / 4f, built from short flat mirrors
    auto curve = [](float y) { return Vec2{kVertex.x - (y - kVertex.y) * (y - kVertex.y) / (4.0f * kMirrorFocal), y}; };
    for (int i = 0; i < 80; ++i) {
        float y0 = kVertex.y - 5.0f + 10.0f * i / 80.0f, y1 = kVertex.y - 5.0f + 10.0f * (i + 1) / 80.0f;
        addMirror(w, curve(y0), curve(y1), false);
    }
    addMirror(w, {-9.0f, 0.0f}, {-6.5f, 2.5f});
    g_planeMirror = g_optics.back().body;
}

void mirrorsReadout(const World&, Readout& r) {
    float focusSum = 0.0f;
    int focusCount = 0;
    for (int i = -4; i <= 4; ++i) {
        const float y = kVertex.y + i * 0.9f;
        std::vector<std::pair<Vec2, Vec2>> legs;
        trace(r, {-16.0f, y}, {1.0f, 0.0f}, 0.55f, 0xffe14fd0, 0, &legs);
        if (i != 0 && legs.size() == 2 && legs[1].first.x > 5.0f) { // reflected once, by the parabola
            auto [o, d] = legs[1];
            if (std::abs(d.y) > 1e-5f) {
                focusSum += o.x + d.x * (kVertex.y - o.y) / d.y;
                ++focusCount;
            }
        }
    }
    r.add("Mirrors: angle of incidence = angle of reflection. A parabola sends every ray");
    r.add("parallel to its axis through one focus: f = %.1f m, measured %.2f m from the vertex", kMirrorFocal,
          focusCount ? kVertex.x - focusSum / focusCount : 0.0f);
    r.add("Drag the flat mirror (bottom left) into the beam");
    ring(r, kVertex - Vec2{kMirrorFocal, 0.0f}, 0.15f, 0xffffffff, 12);
}

} // namespace

std::vector<Scene> opticsScenes(int c) {
    return {
        {.name = "Refraction & TIR", .category = c, .setup = refraction, .readout = refractionReadout},
        {.name = "Prism", .category = c, .setup = prism, .readout = prismReadout},
        {.name = "Lenses", .category = c, .setup = lenses, .readout = lensesReadout},
        {.name = "Mirrors", .category = c, .setup = mirrors, .readout = mirrorsReadout},
    };
}
