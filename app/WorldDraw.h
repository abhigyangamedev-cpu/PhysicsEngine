#pragma once
#include "Renderer.h"
#include "core/World.h"

Color rgba(uint32_t c);
Color bodyColor(const pe::Body& b);
Color darker(Color c);
void drawCoil(Renderer& gfx, pe::Vec2 a, pe::Vec2 b, Color c);
void drawWorld(Renderer& gfx, const pe::World& world, bool showContacts);
