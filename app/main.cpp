#include "App.h"
#include "Viewer3.h"

int main() {
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return 1;
    }

    int width = 1280, height = 760;
    bool quit = false;
    while (!quit) {
        SDL_Window* window = nullptr;
        SDL_Renderer* sdl = nullptr;
        if (!SDL_CreateWindowAndRenderer("Physics Engine", width, height, SDL_WINDOW_RESIZABLE, &window, &sdl)) {
            SDL_Log("Window creation failed: %s", SDL_GetError());
            return 1;
        }
        SDL_SetRenderVSync(sdl, 1);
        SDL_StartTextInput(window);
        SDL_SetRenderDrawBlendMode(sdl, SDL_BLENDMODE_BLEND);

        int w = 0, h = 0;
        SDL_GetRenderOutputSize(sdl, &w, &h);
        bool open3D = false;
        {
            App app(sdl, w, h);
            Uint64 last = SDL_GetPerformanceCounter();
            while (app.running() && !open3D) {
                SDL_Event e;
                while (SDL_PollEvent(&e)) app.handleEvent(e);

                Uint64 now = SDL_GetPerformanceCounter();
                float dt = static_cast<float>(now - last) / SDL_GetPerformanceFrequency();
                last = now;

                app.frame(dt);
                SDL_RenderPresent(sdl);
                open3D = app.take3DRequest();
            }
        }
        SDL_GetWindowSize(window, &width, &height);
        SDL_DestroyRenderer(sdl);
        SDL_DestroyWindow(window);

        // the 3D viewer needs an OpenGL window, so it runs in its own and hands back on ESC
        quit = !open3D || view3d::run(width, height) == view3d::Exit::Quit;
    }

    SDL_Quit();
    return 0;
}
