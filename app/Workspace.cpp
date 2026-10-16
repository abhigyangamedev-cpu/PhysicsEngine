#include "Workspace.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <optional>
#include <vector>

namespace {

constexpr float kTopBar = 52.0f;
constexpr Color kPink{225, 95, 130};

// Whichever provider has a key in the environment; Claude when both or neither do.
ai::Provider startingProvider() {
    const bool claude = !ai::apiKeyFromEnvironment(ai::Provider::Claude).empty();
    const bool openai = !ai::apiKeyFromEnvironment(ai::Provider::OpenAI).empty();
    return openai && !claude ? ai::Provider::OpenAI : ai::Provider::Claude;
}

// Lives as long as the program: the app is rebuilt each time a 3D window opens, and a generation
// started before that should still be there on the way back.
struct State {
    std::string question, space; // space: "", "2d" or "3d"
    ai::Provider provider = startingProvider();
    std::string apiKeys[2];  // pasted, per provider
    std::string openaiModel = ai::defaultModel(ai::Provider::OpenAI);
    std::string jobAuthor;   // who the running generation is asking
    std::optional<ai::Image> image;
    std::string imageNote;

    std::shared_ptr<ai::Generation> job;
    std::shared_ptr<const ai::Spec> result;
    std::optional<ai::Check> check; // of the result, when it was just generated
    std::string warning, error, savedPath;
    bool showSolution = false;

    std::vector<ai::Saved> saved;
    bool savedLoaded = false;
    float savedScroll = 0.0f;

    std::mutex dialogMutex; // the file dialog answers on its own thread
    std::optional<std::string> dialogPick;
    bool dialogOpen = false;
};

State& state() {
    static State s;
    return s;
}

void reloadSaved() {
    State& s = state();
    s.saved = ai::loadSaved(kSavedProblemsDir);
    s.savedLoaded = true;
}

void onDialog(void*, const char* const* files, int) {
    State& s = state();
    std::lock_guard lock(s.dialogMutex);
    s.dialogOpen = false;
    if (files && files[0]) s.dialogPick = files[0];
}

std::string apiKey(ai::Provider p) {
    const std::string env = ai::apiKeyFromEnvironment(p);
    return env.empty() ? state().apiKeys[int(p)] : env;
}

std::string percent(float sim, float expected) {
    if (expected == 0.0f) return "";
    char buf[32];
    std::snprintf(buf, sizeof buf, "  (%+.2f%% from the worked answer)", 100.0 * (sim - expected) / expected);
    return buf;
}

} // namespace

Workspace::Workspace(Renderer& gfx, Ui& ui, Camera& cam, SDL_Window* window) : gfx_(gfx), ui_(ui), cam_(cam), window_(window) {}

bool Workspace::busy() const { return state().job != nullptr; }

void Workspace::setQuestion(std::string text) { state().question = std::move(text); }

void Workspace::showResult(std::shared_ptr<const ai::Spec> spec) {
    State& s = state();
    s.result = std::move(spec);
    s.check.reset();
    s.warning.clear();
    s.error.clear();
    s.showSolution = false;
}

void Workspace::dropFile(const char* path) {
    if (path) attach(path);
}

void Workspace::handleWheel(float wheel) { state().savedScroll -= wheel * 50.0f; }

void Workspace::attach(const std::string& path) {
    State& s = state();
    ai::Image image;
    std::string error;
    if (ai::loadImage(path, image, error)) {
        s.image = std::move(image);
        s.imageNote.clear();
    } else {
        s.imageNote = error;
    }
}

void Workspace::openDialog() {
    State& s = state();
    {
        std::lock_guard lock(s.dialogMutex);
        if (s.dialogOpen) return;
        s.dialogOpen = true;
    }
    static const SDL_DialogFileFilter kFilters[] = {{"Images", "png;jpg;jpeg;gif;webp"}, {"All files", "*"}};
    SDL_ShowOpenFileDialog(onDialog, nullptr, window_, kFilters, 2, nullptr, false);
}

void Workspace::startGeneration() {
    State& s = state();
    if (s.job) return;
    s.error.clear();
    const std::string key = apiKey(s.provider);
    if (key.empty()) {
        s.error = s.provider == ai::Provider::Claude
                      ? "No Anthropic API key: set ANTHROPIC_API_KEY before starting the app, or paste a key above."
                      : "No OpenAI API key: set OPENAI_API_KEY before starting the app, or paste a key above.";
        return;
    }
    if (const auto owner = ai::providerOfKey(key); owner && *owner != s.provider) {
        s.error = std::string("That is ") + (*owner == ai::Provider::Claude ? "an Anthropic" : "an OpenAI") +
                  " key, but " + ai::providerName(s.provider) + " is selected: click " + ai::providerName(*owner) + " above.";
        return;
    }
    ai::Question q;
    q.text = s.question;
    q.image = s.image;
    q.space = s.space;
    q.provider = s.provider;
    if (s.provider == ai::Provider::OpenAI) q.model = s.openaiModel;
    if (q.text.find_first_not_of(" \n") == std::string::npos && !q.image) {
        s.error = "Type a question or upload an image of one first.";
        return;
    }
    s.jobAuthor = s.provider == ai::Provider::Claude ? "Claude" : (q.model.empty() ? ai::defaultModel(q.provider) : q.model);
    s.job = ai::Generation::start(std::move(q), ai::transportFor(s.provider, key), kSavedProblemsDir);
}

void Workspace::poll() {
    State& s = state();
    std::optional<std::string> picked;
    {
        std::lock_guard lock(s.dialogMutex);
        picked = std::exchange(s.dialogPick, std::nullopt);
    }
    if (picked) attach(*picked);
    if (!s.job) return;
    const ai::Generation::Status st = s.job->status();
    if (!st.finished) return;
    s.job.reset();
    const ai::Outcome& out = st.outcome;
    if (out.spec) {
        s.result = out.spec;
        s.check = out.check;
        s.warning = out.warning;
        s.savedPath = out.savedPath;
        s.showSolution = false;
        reloadSaved();
    } else {
        s.error = out.error;
    }
}

Workspace::Action Workspace::frame() {
    State& s = state();
    if (!s.savedLoaded) reloadSaved();
    poll();
    Action action;
    const float W = float(cam_.width), H = float(cam_.height);
    const float margin = 24.0f, top = kTopBar + 18.0f;
    const float leftW = std::max(320.0f, W * 0.46f - margin), rightX = margin * 2 + leftW, rightW = W - rightX - margin;
    drawInput(margin, top, leftW, H - top - margin);
    const float resultH = std::max(220.0f, (H - top - margin) * 0.58f);
    drawResult(rightX, top, rightW, resultH, action);
    drawSaved(rightX, top + resultH + 16.0f, rightW, H - top - resultH - 16.0f - margin);

    ui_.panel({0, 0, W, kTopBar});
    if (ui_.button({12, 10, 100, kTopBar - 20}, "< Back")) action.kind = Action::Kind::Back;
    ui_.label(132, (kTopBar - Renderer::textHeight(2.0f)) * 0.5f, "Question Workspace", theme::text, 2.0f);
    const char* hint = "drop an image on the window to attach it   ESC back";
    ui_.label(W - 12.0f - Renderer::textWidth(hint, 1.25f), (kTopBar - Renderer::textHeight(1.25f)) * 0.5f, hint, theme::textFaint,
              1.25f);
    return action;
}

void Workspace::drawInput(float x, float y, float w, float h) {
    State& s = state();
    const float bottom = y + h;
    ui_.label(x, y, "Your question", theme::text, 1.5f);
    y += 24.0f;
    const float boxH = std::clamp(h * 0.42f, 120.0f, 360.0f);
    ui_.textArea({x, y, w, boxH}, 9001, s.question,
                 "Click and type or paste (Ctrl+V) a physics question, e.g. \"A ball is thrown at 20 m/s at 35 degrees "
                 "above the horizontal from a 10 m cliff. How far from the base of the cliff does it land?\" You can "
                 "also upload a photo or screenshot of a textbook question.");
    y += boxH + 12.0f;

    if (ui_.button({x, y, 170.0f, 30.0f}, "Upload image...", false, 1.25f)) openDialog();
    float lx = x + 182.0f;
    if (s.image) {
        if (ui_.button({lx, y, 90.0f, 30.0f}, "Remove", false, 1.25f)) s.image.reset();
        lx += 102.0f;
    }
    const std::string note = s.image ? "attached: " + s.image->name : s.imageNote.empty() ? "PNG, JPEG, GIF or WebP" : s.imageNote;
    ui_.label(lx, y + 9.0f, note.c_str(), s.image ? theme::accentHover : s.imageNote.empty() ? theme::textFaint : theme::warning, 1.0f);
    y += 44.0f;

    ui_.label(x, y + 8.0f, "Space", theme::textDim, 1.25f);
    const char* names[] = {"Auto", "2D", "3D"};
    const char* values[] = {"", "2d", "3d"};
    for (int i = 0; i < 3; ++i)
        if (ui_.button({x + 80.0f + i * 78.0f, y, 70.0f, 30.0f}, names[i], s.space == values[i], 1.25f)) s.space = values[i];
    ui_.label(x + 80.0f + 3 * 78.0f + 8.0f, y + 10.0f, s.space.empty() ? "the AI picks" : "", theme::textFaint, 1.0f);
    y += 44.0f;

    ui_.label(x, y + 8.0f, "AI", theme::textDim, 1.25f);
    if (ui_.button({x + 80.0f, y, 90.0f, 30.0f}, "Claude", s.provider == ai::Provider::Claude, 1.25f) && !s.job)
        s.provider = ai::Provider::Claude;
    if (ui_.button({x + 178.0f, y, 90.0f, 30.0f}, "OpenAI", s.provider == ai::Provider::OpenAI, 1.25f) && !s.job)
        s.provider = ai::Provider::OpenAI;
    if (s.provider == ai::Provider::OpenAI) {
        ui_.label(x + 284.0f, y + 9.0f, "model", theme::textDim, 1.0f);
        ui_.textArea({x + 334.0f, y, w - 334.0f, 30.0f}, 9003, s.openaiModel, ai::kOpenAIModel, false, false, 64);
    } else {
        ui_.label(x + 284.0f, y + 10.0f, ai::kModel, theme::textFaint, 1.0f);
    }
    y += 44.0f;

    if (ai::apiKeyFromEnvironment(s.provider).empty()) {
        ui_.label(x, y + 9.0f, "API key", theme::textDim, 1.25f);
        std::string& key = s.apiKeys[int(s.provider)];
        if (ui_.textArea({x + 80.0f, y, w - 80.0f, 30.0f}, s.provider == ai::Provider::Claude ? 9002 : 9004, key,
                         s.provider == ai::Provider::Claude ? "paste an sk-ant- key (kept in memory only)"
                                                            : "paste an OpenAI key (kept in memory only)",
                         false, true, 256)) {
            // a key pasted under the wrong provider goes where it belongs
            if (const auto owner = ai::providerOfKey(key); owner && *owner != s.provider && !s.job) {
                s.apiKeys[int(*owner)] = std::exchange(key, std::string());
                s.provider = *owner;
                s.error = std::string("That was ") + (*owner == ai::Provider::Claude ? "an Anthropic" : "an OpenAI") +
                          " key, so " + ai::providerName(*owner) + " is now selected.";
            }
        }
        y += 44.0f;
    }

    const bool running = s.job != nullptr;
    if (ui_.button({x, y, w, 44.0f}, running ? "Generating..." : "Generate simulation", !running, 1.75f) && !running)
        startGeneration();
    y += 58.0f;

    if (running) {
        const ai::Generation::Status st = s.job->status();
        char buf[200];
        const char* who = s.jobAuthor.c_str();
        switch (st.stage) {
        case ai::Stage::Asking:
            std::snprintf(buf, sizeof buf, "%s is reading the question and building the scene... %.0f s", who, st.seconds);
            break;
        case ai::Stage::Checking:
            std::snprintf(buf, sizeof buf, "Running the scene to check it (attempt %d of %d)... %.0f s", st.attempt, ai::kMaxAttempts,
                          st.seconds);
            break;
        default:
            std::snprintf(buf, sizeof buf, "The check found a problem; %s is fixing it (attempt %d of %d)... %.0f s", who, st.attempt,
                          ai::kMaxAttempts, st.seconds);
            break;
        }
        // a pulse so a long think doesn't look frozen
        const float pulse = 0.5f + 0.5f * std::sin(st.seconds * 4.0f);
        gfx_.fillRect({x, y + 3.0f, 8.0f, 8.0f}, {42, 157, 143, Uint8(80 + 175 * pulse)});
        ui_.paragraph(x + 16.0f, y, w - 16.0f, buf, theme::text, 1.25f);
        ui_.paragraph(x + 16.0f, y + 26.0f, w - 16.0f,
                      "Every scene is run headless before you see it; if it misbehaves or disagrees with the worked "
                      "answer, the AI gets the details and tries again.",
                      theme::textFaint, 1.0f, 5.0f);
    } else if (!s.error.empty()) {
        ui_.paragraph(x, y, w, s.error.c_str(), theme::warning, 1.25f);
    } else if (y + 40.0f < bottom) {
        ui_.paragraph(x, y, w,
                      "Works best for mechanics: projectiles, inclines, pulleys, springs, pendulums, collisions, circular "
                      "motion, charges in fields and orbits.",
                      theme::textFaint, 1.0f, 5.0f);
    }
}

void Workspace::drawResult(float x, float y, float w, float h, Action& action) {
    State& s = state();
    ui_.panel({x, y, w, h});
    gfx_.strokeRect({x, y, w, h}, theme::border);
    gfx_.fillRect({x, y, 6.0f, h}, s.result ? kPink : theme::border);
    const float pad = 20.0f, ix = x + pad, iw = w - 2 * pad, bottom = y + h - pad;
    float cy = y + pad;
    if (!s.result) {
        ui_.label(ix, cy, "No simulation yet", theme::textDim, 1.75f);
        ui_.paragraph(ix, cy + 34.0f, iw,
                      "Generate one from a question, or pick a saved one below. It opens in the 2D editor or the 3D "
                      "window, locked as a problem: press Run to watch the simulation measure the answer, then 'Open in "
                      "editor' to change anything.",
                      theme::textFaint, 1.25f);
        return;
    }
    const ai::Spec& spec = *s.result;
    const char* space = spec.space == ai::Space::TwoD ? "2D" : "3D";
    const float badgeW = Renderer::textWidth(space, 1.25f) + 16.0f;
    gfx_.strokeRect({x + w - pad - badgeW, cy - 4.0f, badgeW, Renderer::textHeight(1.25f) + 10.0f}, kPink);
    ui_.label(x + w - pad - badgeW + 8.0f, cy + 1.0f, space, kPink, 1.25f);
    ui_.label(ix, cy, spec.topic.c_str(), theme::accentHover, 1.0f);
    cy += 18.0f;
    ui_.label(ix, cy, spec.title.c_str(), theme::text, 2.0f);
    cy += 30.0f;

    // the facts first, so they stay visible however long the text is
    std::vector<std::pair<std::string, Color>> facts;
    facts.push_back({"Measures: " + spec.measure.label + "   worked answer " + ai::formatValue(spec.expected) + " " + spec.unit,
                     theme::text});
    if (s.check && s.check->reached)
        facts.push_back({"Check run: " + ai::formatValue(s.check->result) + " " + spec.unit + percent(s.check->result, spec.expected),
                         s.warning.empty() ? theme::accentHover : theme::warning});
    if (!s.warning.empty()) facts.push_back({s.warning, theme::warning});
    for (const auto& [line, color] : facts) cy += ui_.paragraph(ix, cy, iw, line.c_str(), color, 1.0f, 5.0f);
    cy += 6.0f;

    const float buttonsY = bottom - 30.0f;
    const std::string& body = s.showSolution ? spec.answer : spec.question;
    const size_t perLine = static_cast<size_t>(iw / Renderer::textWidth("M", 1.25f));
    const size_t lines = static_cast<size_t>(std::max(0.0f, (buttonsY - 8.0f - cy) / (Renderer::textHeight(1.25f) + 6.0f)));
    std::string shown = body;
    if (shown.size() > perLine * lines) { // trim to the space there is, at a word
        const size_t cut = shown.rfind(' ', perLine * lines > 4 ? perLine * lines - 4 : 0);
        shown = shown.substr(0, cut == std::string::npos ? 0 : cut) + " ...";
    }
    if (lines > 0) ui_.paragraph(ix, cy, iw, shown.c_str(), s.showSolution ? theme::warning : theme::textDim, 1.25f);

    if (ui_.button({ix, buttonsY, 190.0f, 30.0f}, "Open simulation", true, 1.25f)) {
        action.kind = Action::Kind::Open;
        action.spec = s.result;
    }
    if (ui_.button({ix + 200.0f, buttonsY, 160.0f, 30.0f}, s.showSolution ? "Show question" : "Show solution", s.showSolution, 1.25f))
        s.showSolution = !s.showSolution;
}

void Workspace::drawSaved(float x, float y, float w, float h) {
    State& s = state();
    ui_.label(x, y, "Saved simulations", theme::text, 1.5f);
    char count[64];
    std::snprintf(count, sizeof count, "%zu in %s/", s.saved.size(), kSavedProblemsDir);
    ui_.label(x + w - Renderer::textWidth(count, 1.0f), y + 4.0f, count, theme::textFaint, 1.0f);
    y += 26.0f;
    h -= 26.0f;
    if (s.saved.empty()) {
        ui_.label(x, y + 4.0f, "Each generated simulation is saved here.", theme::textFaint, 1.0f);
        return;
    }
    const float rowH = 46.0f;
    const float maxScroll = std::max(0.0f, s.saved.size() * rowH - h);
    s.savedScroll = std::clamp(s.savedScroll, 0.0f, maxScroll);
    const int first = static_cast<int>(s.savedScroll / rowH);
    const int visible = static_cast<int>(h / rowH);
    for (int i = first; i < static_cast<int>(s.saved.size()) && i < first + visible; ++i) {
        const ai::Saved& item = s.saved[i];
        const SDL_FRect r{x, y + (i - first) * rowH, w, rowH - 6.0f};
        const SDL_FRect del{r.x + r.w - 34.0f, r.y + 8.0f, 26.0f, r.h - 16.0f};
        const bool overDelete = ui_.hovered(del);
        const bool current = s.result == item.spec;
        if (ui_.button(r, "", current, 1.0f) && !overDelete) {
            showResult(item.spec);
            break;
        }
        ui_.label(r.x + 12.0f, r.y + 7.0f, item.spec->title.c_str(), theme::text, 1.25f);
        const std::string sub = item.spec->topic + (item.spec->space == ai::Space::TwoD ? "  -  2D" : "  -  3D");
        ui_.label(r.x + 12.0f, r.y + 24.0f, sub.c_str(), theme::textDim, 1.0f);
        if (ui_.button(del, "x", false, 1.25f)) {
            if (current) s.result.reset();
            ai::removeSaved(item.path);
            reloadSaved();
            break;
        }
    }
}
