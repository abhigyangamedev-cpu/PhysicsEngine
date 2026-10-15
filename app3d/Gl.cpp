#include "Gl.h"
#include <SDL3/SDL.h>

namespace gl {
#define PE_GL_DEFINE(type, name) type name = nullptr;
PE_GL_FUNCS(PE_GL_DEFINE)
#undef PE_GL_DEFINE

bool load() {
    bool ok = true;
#define PE_GL_LOAD(type, name)                                                                                         \
    name = reinterpret_cast<type>(SDL_GL_GetProcAddress(#name));                                                       \
    if (!name) {                                                                                                       \
        SDL_Log("missing GL function %s", #name);                                                                      \
        ok = false;                                                                                                    \
    }
    PE_GL_FUNCS(PE_GL_LOAD)
#undef PE_GL_LOAD
    return ok;
}
} // namespace gl
