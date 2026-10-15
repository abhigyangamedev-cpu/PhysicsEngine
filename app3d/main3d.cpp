#include "Viewer3.h"
#include <SDL3/SDL.h>

int main() {
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return 1;
    }
    view3d::run(1280, 760);
    SDL_Quit();
    return 0;
}
