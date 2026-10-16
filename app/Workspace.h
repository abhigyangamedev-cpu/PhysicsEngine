#pragma once
// The question workspace: type, paste or upload a physics question and Claude or an OpenAI model
// builds a simulation of it, checked by running it before it opens in the 2D editor or the 3D window.
#include "Generator.h"
#include "Renderer.h"
#include "Ui.h"
#include <memory>
#include <string>

class Workspace {
public:
    struct Action {
        enum class Kind { None, Back, Open } kind = Kind::None;
        std::shared_ptr<const ai::Spec> spec; // for Open
    };

    Workspace(Renderer& gfx, Ui& ui, Camera& cam, SDL_Window* window);
    Action frame(); // draws the screen and reports what the user asked for
    void dropFile(const char* path); // a file dragged onto the window
    void handleWheel(float wheel);

    // for the UI tests
    void setQuestion(std::string text);
    void showResult(std::shared_ptr<const ai::Spec> spec);
    bool busy() const;

private:
    Renderer& gfx_;
    Ui& ui_;
    Camera& cam_;
    SDL_Window* window_;

    void drawInput(float x, float y, float w, float h);
    void drawResult(float x, float y, float w, float h, Action& action);
    void drawSaved(float x, float y, float w, float h);
    void openDialog();
    void startGeneration();
    void poll();
    void attach(const std::string& path);
};

inline constexpr const char* kSavedProblemsDir = "ai_problems";
