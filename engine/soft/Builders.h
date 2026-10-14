#pragma once
#include "soft/ParticleSystem.h"

namespace pe {

struct ClothDef {
    Vec2 topLeft;
    int columns = 20, rows = 15;
    float spacing = 0.4f;
    float particleMass = 0.05f;
    float stretchCompliance = 0.0f;
    float shearCompliance = 1e-4f;
    float bendCompliance = 1e-2f;
    float tearRatio = 0.0f;
};

struct BlobDef {
    Vec2 center;
    float radius = 1.0f;
    int segments = 24;
    float mass = 1.0f;
    float pressure = 1.0f;
    float edgeCompliance = 1e-5f;
    float pressureCompliance = 1e-4f;
};

struct JellyDef {
    Vec2 center;
    float width = 2.0f, height = 2.0f;
    int columns = 6, rows = 6;
    float mass = 2.0f;
    float compliance = 1e-4f;
};

// Returned indices are row-major: index = row * columns + col.
std::vector<uint32_t> makeCloth(ParticleSystem& ps, const ClothDef& def);
std::vector<uint32_t> makeRope(ParticleSystem& ps, Vec2 from, Vec2 to, int segments, float mass,
                               float compliance = 0.0f);
std::vector<uint32_t> makeBlob(ParticleSystem& ps, const BlobDef& def);
std::vector<uint32_t> makeJelly(ParticleSystem& ps, const JellyDef& def);

} // namespace pe
