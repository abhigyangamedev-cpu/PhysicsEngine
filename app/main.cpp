#include "App.h"
#include "Viewer3.h"
#include <cstdio>
#include <cstdlib>
#include <optional>

int main() {
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return 1;
    }

    int width = 1280, height = 760;
    std::optional<view3d::Launch> resume; // what the 3D window last opened, to come back to the screen that asked
    bool quit = false;
    while (!quit) {
        SDL_Window* window = nullptr;
        SDL_Renderer* sdl = nullptr;
        if (!SDL_CreateWindowAndRenderer("Physics Engine", width, height, SDL_WINDOW_RESIZABLE, &window, &sdl)) {
            SDL_Log("Window creation failed: %s", SDL_GetError());
            return 1;
        }
        SDL_SetWindowMinimumSize(window, 960, 600); // the layouts need at least this much room
        SDL_SetRenderVSync(sdl, 1);
        SDL_StartTextInput(window);
        SDL_SetRenderDrawBlendMode(sdl, SDL_BLENDMODE_BLEND);

        int w = 0, h = 0;
        SDL_GetRenderOutputSize(sdl, &w, &h);
        std::optional<view3d::Launch> launch;
        {
            App app(sdl, w, h);
            if (resume) app.resumeAfter3D(*resume);
            Uint64 last = SDL_GetPerformanceCounter();
            while (app.running() && !launch) {
                SDL_Event e;
                while (SDL_PollEvent(&e)) app.handleEvent(e);

                Uint64 now = SDL_GetPerformanceCounter();
                float dt = static_cast<float>(now - last) / SDL_GetPerformanceFrequency();
                last = now;

                app.frame(dt);
                SDL_RenderPresent(sdl);
                launch = app.take3DRequest();
            }
        }
        SDL_GetWindowSize(window, &width, &height);
        SDL_DestroyRenderer(sdl);
        SDL_DestroyWindow(window);

        // the 3D views need an OpenGL window, so they run in their own and hand back on ESC
        quit = !launch || view3d::run(width, height, *launch) == view3d::Exit::Quit;
        resume = launch;
    }

    SDL_Quit();
    // A question still being answered runs on a detached thread that may be waiting on the network for
    // minutes; ending without static destructors keeps it from reading the prompt and schema as they go.
    std::fflush(nullptr);
    std::quick_exit(0);
}
