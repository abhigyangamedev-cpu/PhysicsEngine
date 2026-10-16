#pragma once
// Turns a question (text and/or an image) into a checked simulation: asks a model (Claude or an OpenAI
// model) for a scene, runs it headless, and sends any problem back for a fix before the user sees it.
#include "Http.h"
#include "QuestionSpec.h"
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace ai {

enum class Provider { Claude, OpenAI };

inline constexpr const char* kModel = "claude-opus-5-5";
inline constexpr const char* kOpenAIModel = "gpt-5"; // OPENAI_MODEL or the workspace's model field overrides it
inline constexpr int kMaxAttempts = 3;                // the first answer plus two repairs

const char* providerName(Provider p); // "Claude" or "OpenAI"
std::string defaultModel(Provider p);

struct Image {
    std::string name, mediaType, base64;
};
// Reads a PNG, JPEG, GIF or WebP file for upload.
bool loadImage(const std::string& path, Image& out, std::string& error);
std::string base64(const std::string& bytes);

struct Question {
    std::string text;
    std::optional<Image> image;
    std::string space; // "", "2d" or "3d": a preference passed on to the model
    Provider provider = Provider::Claude;
    std::string model; // empty: defaultModel(provider)
};

enum class Stage { Asking, Checking, Repairing, Done, Failed };

struct Outcome {
    std::shared_ptr<const Spec> spec; // set when a simulation was made
    Check check;                      // its headless run
    std::string warning;              // usable, but something didn't line up
    std::string error;                // why nothing was made
    int attempts = 0;
    std::string savedPath;
};

// Sends one request body to the provider's API and returns the HTTP answer. The tests swap in a script.
using Transport = std::function<http::Response(const std::string& body)>;
Transport transportFor(Provider p, std::string apiKey);
std::string apiKeyFromEnvironment(Provider p); // ANTHROPIC_API_KEY / OPENAI_API_KEY, or empty
std::string trimKey(const std::string& key);   // without the spaces and line breaks a paste brings along
// Which provider a key belongs to, from its prefix: "sk-ant-" is Anthropic's, other "sk-" keys OpenAI's.
std::optional<Provider> providerOfKey(const std::string& key);

// The conversation's opening messages (OpenAI's carries the system prompt), and a request body for it.
json::Value initialMessages(const Question& q);
std::string requestBody(const Question& q, const json::Value& messages);

// The whole exchange, on the calling thread. onStage reports progress (stage, attempt number).
Outcome generate(const Question& q, const Transport& send, const std::function<void(Stage, int)>& onStage = {});

// One generation running on a worker thread; poll status() each frame.
class Generation {
public:
    struct Status {
        Stage stage = Stage::Asking;
        int attempt = 1;
        float seconds = 0.0f; // since it started
        bool finished = false;
        Outcome outcome;      // valid once finished
    };

    // Saves a successful result into saveDir (created if missing) when it is not empty.
    static std::shared_ptr<Generation> start(Question q, Transport send, std::string saveDir);
    Status status() const;

private:
    mutable std::mutex mutex_;
    Status status_;
    std::chrono::steady_clock::time_point began_ = std::chrono::steady_clock::now();
};

// ---- saved problems ----

struct Saved {
    std::string path;
    std::shared_ptr<const Spec> spec;
};
bool saveSpec(const Spec& spec, const std::string& dir, std::string& path);
std::vector<Saved> loadSaved(const std::string& dir); // newest first; unreadable files are skipped
bool removeSaved(const std::string& path);

} // namespace ai
