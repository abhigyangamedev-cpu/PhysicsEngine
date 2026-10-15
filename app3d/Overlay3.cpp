#include "Overlay3.h"
#include "Gl.h"
#include "Renderer3.h"

using namespace gl;

namespace view3d {

namespace {

// one oversized triangle covering the screen; the surface's first row is the top of the window
const char* kVertex = R"(#version 330 core
out vec2 vUv;
void main() {
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    vUv = vec2(p.x, 1.0 - p.y);
    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
)";

const char* kFragment = R"(#version 330 core
in vec2 vUv;
uniform sampler2D uUi;
out vec4 fragColor;
void main() { fragColor = texture(uUi, vUv); }
)";

} // namespace

Overlay::Overlay() = default;

Overlay::~Overlay() {
    ui_.reset();
    gfx_.reset();
    if (soft_) SDL_DestroyRenderer(soft_);
    if (surface_) SDL_DestroySurface(surface_);
}

bool Overlay::init() {
    program_ = linkProgram(kVertex, kFragment);
    if (!program_) return false;
    glGenVertexArrays(1, &vao_);
    glGenTextures(1, &texture_);
    glBindTexture(GL_TEXTURE_2D, texture_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    resize(1, 1); // ui() must be valid before the first begin(): the OS sends events as soon as the window shows
    return true;
}

void Overlay::resize(int width, int height) {
    ui_.reset();
    gfx_.reset();
    if (soft_) SDL_DestroyRenderer(soft_);
    if (surface_) SDL_DestroySurface(surface_);
    width_ = width;
    height_ = height;
    surface_ = SDL_CreateSurface(width, height, SDL_PIXELFORMAT_RGBA32);
    soft_ = SDL_CreateSoftwareRenderer(surface_);
    SDL_SetRenderDrawBlendMode(soft_, SDL_BLENDMODE_BLEND);
    cam2d_.width = width;
    cam2d_.height = height;
    gfx_ = std::make_unique<Renderer>(soft_, cam2d_);
    ui_ = std::make_unique<Ui>(*gfx_);
    textureSized_ = false;
}

void Overlay::begin(int width, int height, float mouseX, float mouseY, bool clicked) {
    width = std::max(width, 1), height = std::max(height, 1);
    if (width != width_ || height != height_ || !soft_) resize(width, height);
    gfx_->clear({0, 0, 0, 0});
    ui_->begin(mouseX, mouseY, clicked);
}

void Overlay::end(int pixelWidth, int pixelHeight) {
    SDL_FlushRenderer(soft_);
    glBindTexture(GL_TEXTURE_2D, texture_);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, surface_->pitch / 4);
    if (!textureSized_) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width_, height_, 0, GL_RGBA, GL_UNSIGNED_BYTE, surface_->pixels);
        textureSized_ = true;
    } else {
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width_, height_, GL_RGBA, GL_UNSIGNED_BYTE, surface_->pixels);
    }
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);

    glViewport(0, 0, pixelWidth, pixelHeight);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glUseProgram(program_);
    glActiveTexture(GL_TEXTURE0);
    glUniform1i(glGetUniformLocation(program_, "uUi"), 0);
    glBindVertexArray(vao_);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
}

} // namespace view3d
