#include "Renderer.h"
#include "Scenes.h"
#include <cstdio>

using namespace pe;

namespace {

Color bodyColor(const Body& b) {
    if (b.isStatic) return {70, 75, 85};
    static constexpr Color palette[] = {
        {231, 111, 81}, {244, 162, 97}, {233, 196, 106}, {42, 157, 143}, {94, 129, 172}, {180, 142, 173},
    };
    return palette[b.id % std::size(palette)];
}

Color darker(Color c) { return {Uint8(c.r * 0.6f), Uint8(c.g * 0.6f), Uint8(c.b * 0.6f), 255}; }

struct MouseGrab {
    Body* body = nullptr;
    Vec2 localAnchor;
    Vec2 target;

    Vec2 anchor() const { return body->position + body->rot.apply(localAnchor); }

    void apply() const {
        if (!body) return;
        constexpr float omega = 2.0f * 3.14159f * 4.0f, zeta = 0.7f;
        Vec2 p = anchor();
        Vec2 f = (target - p) * (body->mass * omega * omega) - body->velocityAt(p) * (2.0f * body->mass * zeta * omega);
        body->applyForceAt(f, p);
    }
};

} // namespace

int main() {
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return 1;
    }

    Camera cam;
    SDL_Window* window = nullptr;
    SDL_Renderer* sdl = nullptr;
    if (!SDL_CreateWindowAndRenderer("Physics Engine", cam.width, cam.height, SDL_WINDOW_RESIZABLE, &window, &sdl)) {
        SDL_Log("Window creation failed: %s", SDL_GetError());
        return 1;
    }
    SDL_SetRenderVSync(sdl, 1);
    SDL_SetRenderDrawBlendMode(sdl, SDL_BLENDMODE_BLEND);
    Renderer gfx(sdl, cam);

    World world;
    size_t sceneIndex = 0;
    auto loadScene = [&](size_t i) {
        sceneIndex = i;
        world.clear();
        scenes()[i].setup(world);
    };
    loadScene(0);

    MouseGrab grab;
    bool paused = false, stepOnce = false, showContacts = false;
    constexpr float dt = 1.0f / 60.0f;
    float accumulator = 0.0f;
    float stepMs = 0.0f;
    Uint64 last = SDL_GetPerformanceCounter();

    for (bool running = true; running;) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            switch (e.type) {
            case SDL_EVENT_QUIT: running = false; break;
            case SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED:
                cam.width = e.window.data1;
                cam.height = e.window.data2;
                break;
            case SDL_EVENT_KEY_DOWN:
                switch (e.key.key) {
                case SDLK_ESCAPE: running = false; break;
                case SDLK_SPACE: paused = !paused; break;
                case SDLK_S: stepOnce = true; break;
                case SDLK_R: grab.body = nullptr; loadScene(sceneIndex); break;
                case SDLK_C: showContacts = !showContacts; break;
                case SDLK_LEFT: cam.center.x -= 1.0f; break;
                case SDLK_RIGHT: cam.center.x += 1.0f; break;
                case SDLK_UP: cam.center.y += 1.0f; break;
                case SDLK_DOWN: cam.center.y -= 1.0f; break;
                default:
                    if (e.key.key >= SDLK_1 && e.key.key < SDLK_1 + scenes().size()) {
                        grab.body = nullptr;
                        loadScene(e.key.key - SDLK_1);
                    }
                }
                break;
            case SDL_EVENT_MOUSE_BUTTON_DOWN: {
                Vec2 p = cam.toWorld({e.button.x, e.button.y});
                BodyDef d;
                d.position = p;
                if (e.button.button == SDL_BUTTON_LEFT) {
                    if (Body* b = world.bodyAt(p)) {
                        grab = {b, b->rot.applyT(p - b->position), p};
                    } else {
                        world.createBody(Shape::box(0.5f, 0.5f), d);
                    }
                } else if (e.button.button == SDL_BUTTON_RIGHT) {
                    world.createBody(Shape::circle(0.5f), d);
                } else if (e.button.button == SDL_BUTTON_MIDDLE) {
                    world.createBody(randomShape(0.6f), d);
                }
                break;
            }
            case SDL_EVENT_MOUSE_BUTTON_UP:
                if (e.button.button == SDL_BUTTON_LEFT) grab.body = nullptr;
                break;
            case SDL_EVENT_MOUSE_MOTION:
                grab.target = cam.toWorld({e.motion.x, e.motion.y});
                break;
            case SDL_EVENT_MOUSE_WHEEL: cam.scale *= e.wheel.y > 0 ? 1.1f : 1.0f / 1.1f; break;
            }
        }

        Uint64 now = SDL_GetPerformanceCounter();
        float frame = static_cast<float>(now - last) / SDL_GetPerformanceFrequency();
        last = now;

        auto doStep = [&] {
            Uint64 t0 = SDL_GetPerformanceCounter();
            grab.apply();
            world.step(dt);
            stepMs = 1000.0f * (SDL_GetPerformanceCounter() - t0) / SDL_GetPerformanceFrequency();
        };
        if (!paused) {
            accumulator = std::min(accumulator + frame, 0.25f);
            while (accumulator >= dt) {
                doStep();
                accumulator -= dt;
            }
        } else if (stepOnce) {
            doStep();
        }
        stepOnce = false;

        gfx.clear({24, 26, 32});
        for (const auto& b : world.bodies()) {
            Color c = bodyColor(*b);
            if (b->shape.type == ShapeType::Circle)
                gfx.circle(b->position, b->shape.radius, b->angle, c, darker(c));
            else
                gfx.polygon(b->worldVertices, c, darker(c));
        }
        if (showContacts) {
            for (const auto& [key, m] : world.manifolds()) {
                for (int i = 0; i < m.count; ++i) {
                    gfx.point(m.contacts[i].point, 5.0f, {255, 60, 60});
                    gfx.line(m.contacts[i].point, m.contacts[i].point + m.normal * 0.3f, {255, 255, 255});
                }
            }
        }
        if (grab.body) gfx.line(grab.anchor(), grab.target, {255, 255, 255});

        char hud[256];
        std::snprintf(hud, sizeof hud, "[%zu] %s   bodies: %zu   contacts: %zu   step: %.2f ms%s", sceneIndex + 1,
                      scenes()[sceneIndex].name, world.bodies().size(), world.manifolds().size(), stepMs,
                      paused ? "   PAUSED" : "");
        gfx.text(10, 10, hud, {230, 230, 230});
        gfx.text(10, 24, "1-4 scenes | LMB drag/box | RMB circle | MMB polygon | wheel zoom | arrows pan", {150, 150, 160});
        gfx.text(10, 38, "SPACE pause | S step | R reset | C contacts", {150, 150, 160});
        SDL_RenderPresent(sdl);
    }

    SDL_DestroyRenderer(sdl);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
