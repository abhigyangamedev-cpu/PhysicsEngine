#pragma once
#include "Renderer.h"
#include "Ui.h"
#include <memory>

namespace view3d {

// The 2D app's Ui drawn by SDL's software renderer into a surface, then blended over the 3D view
// as a texture, so menus and buttons look the same in both spaces.
class Overlay {
public:
    Overlay();
    ~Overlay();
    bool init();

    // width/height in window coordinates (the mouse's space); clicked = left button pressed this frame
    void begin(int width, int height, float mouseX, float mouseY, bool clicked);
    void end(int pixelWidth, int pixelHeight); // uploads and draws over whatever GL drew

    Ui& ui() { return *ui_; }
    Renderer& gfx() { return *gfx_; }
    int width() const { return width_; }
    int height() const { return height_; }

private:
    SDL_Surface* surface_ = nullptr;
    SDL_Renderer* soft_ = nullptr;
    ::Camera cam2d_;
    std::unique_ptr<Renderer> gfx_;
    std::unique_ptr<Ui> ui_;
    int width_ = 0, height_ = 0;
    unsigned texture_ = 0, program_ = 0, vao_ = 0;
    bool textureSized_ = false;

    void resize(int width, int height);
};

} // namespace view3d
