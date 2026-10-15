#include "App.h"

int main() {
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return 1;
    }

    SDL_Window* window = nullptr;
    SDL_Renderer* sdl = nullptr;
    if (!SDL_CreateWindowAndRenderer("Physics Engine", 1280, 760, SDL_WINDOW_RESIZABLE, &window, &sdl)) {
        SDL_Log("Window creation failed: %s", SDL_GetError());
        return 1;
    }
    SDL_SetRenderVSync(sdl, 1);
    SDL_StartTextInput(window);
    SDL_SetRenderDrawBlendMode(sdl, SDL_BLENDMODE_BLEND);

    int w = 0, h = 0;
    SDL_GetRenderOutputSize(sdl, &w, &h);
    App app(sdl, w, h);

    Uint64 last = SDL_GetPerformanceCounter();
    while (app.running()) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) app.handleEvent(e);

        Uint64 now = SDL_GetPerformanceCounter();
        float dt = static_cast<float>(now - last) / SDL_GetPerformanceFrequency();
        last = now;

        app.frame(dt);
        SDL_RenderPresent(sdl);
    }

    SDL_DestroyRenderer(sdl);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
