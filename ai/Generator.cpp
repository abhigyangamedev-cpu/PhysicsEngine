#include "Generator.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <thread>

namespace ai {

namespace fs = std::filesystem;

namespace {

// Paths travel as UTF-8 strings (what SDL's file dialog and drop events give).
fs::path toPath(const std::string& utf8) { return fs::path(std::u8string(utf8.begin(), utf8.end())); }
std::string toUtf8(const fs::path& p) {
    const std::u8string u = p.u8string();
    return std::string(u.begin(), u.end());
}

constexpr int kMaxImageBytes = 3'750'000; // 5 MB once base64-encoded, the API's per-image limit
constexpr float kMismatch = 0.08f;        // relative gap between simulation and paper answer worth a second look

json::Value text(const std::string& s) {
    json::Value block = json::Value::object();
    block.set("type", "text");
    block.set("text", s);
    return block;
}

json::Value message(const char* role, json::Value content) {
    json::Value m = json::Value::object();
    m.set("role", role);
    m.set("content", std::move(content));
    return m;
}

// The assistant turn to send back with a repair request. Blocks before a mid-answer model fallback
// are dropped except text, as the fallback rules require.
json::Value echo(const json::Value& content) {
    size_t boundary = 0;
    for (size_t i = 0; i < content.size(); ++i)
        if (content[i].str("type") == "fallback") boundary = i;
    json::Value out = json::Value::array();
    for (size_t i = 0; i < content.size(); ++i)
        if (i >= boundary || content[i].str("type") == "text") out.push(content[i]);
    return out;
}

std::string modelOf(const Question& q) { return q.model.empty() ? defaultModel(q.provider) : q.model; }

// Who wrote the answer, for the messages the user sees.
std::string author(const Question& q) { return q.provider == Provider::Claude ? "Claude" : modelOf(q); }

std::string apiError(const Question& q, const http::Response& r) {
    json::Value body;
    std::string detail, code;
    if (json::parse(r.body, body)) {
        detail = body["error"].str("message");
        code = body["error"].str("code");
    }
    const char* env = q.provider == Provider::Claude ? "ANTHROPIC_API_KEY" : "OPENAI_API_KEY";
    const std::string name = providerName(q.provider);
    if (code == "insufficient_quota") return "the " + name + " account has no credit left (429). Add billing credit and try again.";
    if (r.status == 404 || code == "model_not_found")
        return "the model \"" + modelOf(q) + "\" isn't available to this key (" + std::to_string(r.status) + "). " + detail;
    switch (r.status) {
    case 401: return name + " rejected the API key (401). Check " + env + " or the key you pasted.";
    case 403: return "this API key isn't allowed to use " + modelOf(q) + " (403). " + detail;
    case 413: return "the request is too large (413): use a smaller image.";
    case 429: return "rate limited by " + name + " (429). Wait a minute and try again.";
    default: break;
    }
    return name + " answered " + std::to_string(r.status) + (detail.empty() ? "" : ": " + detail);
}

// Reasoning models take an effort setting; older chat models reject it.
bool reasons(const std::string& model) {
    return model.rfind("gpt-5", 0) == 0 || model.rfind("gpt-6", 0) == 0 ||
           (model.size() > 1 && model[0] == 'o' && std::isdigit(static_cast<unsigned char>(model[1])));
}

// What one API reply said: the answer text and the assistant turn to keep in the conversation.
struct Reply {
    std::string answer, error;
    json::Value turn;
};

Reply readClaude(const Question& q, const json::Value& reply) {
    Reply out;
    const std::string stop = reply.str("stop_reason");
    if (stop == "refusal") {
        const std::string why = reply["stop_details"].str("explanation");
        out.error = "Claude declined this question" + (why.empty() ? std::string(".") : ": " + why);
    } else if (stop == "max_tokens") {
        out.error = author(q) + "'s answer ran out of room before it finished; try a shorter or simpler question";
    }
    for (const json::Value& block : reply["content"].items())
        if (block.str("type") == "text") out.answer += block.str("text");
    out.turn = message("assistant", echo(reply["content"]));
    return out;
}

Reply readOpenAI(const Question& q, const json::Value& reply) {
    Reply out;
    const json::Value& choice = reply["choices"][0];
    const json::Value& msg = choice["message"];
    if (const std::string refusal = msg.str("refusal"); !refusal.empty())
        out.error = author(q) + " declined this question: " + refusal;
    else if (choice.str("finish_reason") == "length")
        out.error = author(q) + "'s answer ran out of room before it finished; try a shorter or simpler question";
    else if (!msg.isObject())
        out.error = "OpenAI's reply had no message in it";
    out.answer = msg.str("content");
    out.turn = message("assistant", out.answer);
    return out;
}

json::Value feedbackTurn(const Question& q, const std::string& feedback) {
    if (q.provider == Provider::OpenAI) return message("user", feedback);
    json::Value content = json::Value::array();
    content.push(text(feedback));
    return message("user", content);
}

std::string questionPrompt(const Question& q) {
    std::string prompt;
    if (!q.text.empty()) prompt = "Question:\n" + q.text + "\n";
    if (q.image) prompt += q.text.empty() ? "The question is in the attached image.\n" : "The attached image is part of the question.\n";
    if (q.space == "2d") prompt += "Build it in the 2D space.\n";
    if (q.space == "3d") prompt += "Build it in the 3D space.\n";
    return prompt + "Return the simulation JSON.";
}

bool retryable(const http::Response& r) {
    if (!r.error.empty() || r.status >= 500) return true;
    if (r.status != 429) return false;
    json::Value body;
    return !(json::parse(r.body, body) && body["error"].str("code") == "insufficient_quota"); // waiting won't add credit
}

std::string slug(const std::string& title) {
    std::string s;
    for (char c : title) {
        if (std::isalnum(static_cast<unsigned char>(c))) s += char(std::tolower(static_cast<unsigned char>(c)));
        else if (!s.empty() && s.back() != '-') s += '-';
        if (s.size() >= 40) break;
    }
    while (!s.empty() && s.back() == '-') s.pop_back();
    return s.empty() ? "problem" : s;
}

} // namespace

std::string base64(const std::string& bytes) {
    static const char* kAlphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve((bytes.size() + 2) / 3 * 4);
    size_t i = 0;
    for (; i + 2 < bytes.size(); i += 3) {
        const unsigned n = (unsigned char)bytes[i] << 16 | (unsigned char)bytes[i + 1] << 8 | (unsigned char)bytes[i + 2];
        out += kAlphabet[n >> 18];
        out += kAlphabet[(n >> 12) & 63];
        out += kAlphabet[(n >> 6) & 63];
        out += kAlphabet[n & 63];
    }
    if (i + 1 == bytes.size()) {
        const unsigned n = (unsigned char)bytes[i] << 16;
        out += kAlphabet[n >> 18];
        out += kAlphabet[(n >> 12) & 63];
        out += "==";
    } else if (i + 2 == bytes.size()) {
        const unsigned n = (unsigned char)bytes[i] << 16 | (unsigned char)bytes[i + 1] << 8;
        out += kAlphabet[n >> 18];
        out += kAlphabet[(n >> 12) & 63];
        out += kAlphabet[(n >> 6) & 63];
        out += '=';
    }
    return out;
}

bool loadImage(const std::string& path, Image& out, std::string& error) {
    std::ifstream f(toPath(path), std::ios::binary);
    if (!f) {
        error = "couldn't open " + path;
        return false;
    }
    std::string bytes((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    if (bytes.size() > size_t(kMaxImageBytes)) {
        error = "the image is " + std::to_string(bytes.size() / 1'000'000) + " MB; crop or shrink it below 3.7 MB";
        return false;
    }
    auto starts = [&](const char* magic, size_t at = 0) { return bytes.compare(at, std::char_traits<char>::length(magic), magic) == 0; };
    std::string type;
    if (starts("\x89PNG")) type = "image/png";
    else if (starts("\xff\xd8\xff")) type = "image/jpeg";
    else if (starts("GIF8")) type = "image/gif";
    else if (starts("RIFF") && starts("WEBP", 8)) type = "image/webp";
    if (type.empty()) {
        error = "that file isn't a PNG, JPEG, GIF or WebP image";
        return false;
    }
    out.name = toUtf8(toPath(path).filename());
    out.mediaType = type;
    out.base64 = base64(bytes);
    return true;
}

const char* providerName(Provider p) { return p == Provider::Claude ? "Claude" : "OpenAI"; }

std::string defaultModel(Provider p) {
    if (p == Provider::Claude) return kModel;
    const char* env = std::getenv("OPENAI_MODEL");
    return env && *env ? env : kOpenAIModel;
}

std::string apiKeyFromEnvironment(Provider p) {
    const char* key = std::getenv(p == Provider::Claude ? "ANTHROPIC_API_KEY" : "OPENAI_API_KEY");
    return key ? key : "";
}

std::string trimKey(const std::string& key) {
    std::string out;
    for (char c : key)
        if (!std::isspace(static_cast<unsigned char>(c))) out += c;
    return out;
}

std::optional<Provider> providerOfKey(const std::string& key) {
    const std::string k = trimKey(key);
    if (k.rfind("sk-ant-", 0) == 0) return Provider::Claude;
    if (k.rfind("sk-", 0) == 0) return Provider::OpenAI;
    return std::nullopt;
}

Transport transportFor(Provider p, std::string apiKey) {
    apiKey = trimKey(apiKey);
    if (p == Provider::OpenAI)
        return [apiKey = std::move(apiKey)](const std::string& body) {
            return http::post("api.openai.com", "/v1/chat/completions",
                              {{"content-type", "application/json"}, {"authorization", "Bearer " + apiKey}}, body, 600);
        };
    return [apiKey = std::move(apiKey)](const std::string& body) {
        return http::post("api.anthropic.com", "/v1/messages",
                          {{"content-type", "application/json"},
                           {"x-api-key", apiKey},
                           {"anthropic-version", "2023-06-01"},
                           {"anthropic-beta", "server-side-fallback-2026-07-01"}},
                          body, 600);
    };
}

json::Value initialMessages(const Question& q) {
    json::Value messages = json::Value::array();
    json::Value content = json::Value::array();
    if (q.provider == Provider::OpenAI) {
        messages.push(message("system", systemPrompt()));
        content.push(text(questionPrompt(q)));
        if (q.image) {
            json::Value url = json::Value::object();
            url.set("url", "data:" + q.image->mediaType + ";base64," + q.image->base64);
            url.set("detail", "high");
            json::Value block = json::Value::object();
            block.set("type", "image_url");
            block.set("image_url", url);
            content.push(block);
        }
        messages.push(message("user", content));
        return messages;
    }
    if (q.image) {
        json::Value source = json::Value::object();
        source.set("type", "base64");
        source.set("media_type", q.image->mediaType);
        source.set("data", q.image->base64);
        json::Value block = json::Value::object();
        block.set("type", "image");
        block.set("source", source);
        content.push(block);
    }
    content.push(text(questionPrompt(q)));
    messages.push(message("user", content));
    return messages;
}

std::string requestBody(const Question& q, const json::Value& messages) {
    json::Value body = json::Value::object();
    body.set("model", modelOf(q));
    if (q.provider == Provider::OpenAI) {
        json::Value schema = json::Value::object();
        schema.set("name", "physics_simulation");
        schema.set("strict", true);
        schema.set("schema", outputSchema());
        json::Value format = json::Value::object();
        format.set("type", "json_schema");
        format.set("json_schema", schema);
        body.set("messages", messages);
        body.set("response_format", format);
        body.set("max_completion_tokens", 32000); // reasoning tokens count against it
        if (reasons(modelOf(q))) body.set("reasoning_effort", "high");
        return body.dump();
    }

    json::Value format = json::Value::object();
    format.set("type", "json_schema");
    format.set("schema", outputSchema());
    json::Value output = json::Value::object();
    output.set("effort", "high");
    output.set("format", format);
    json::Value thinking = json::Value::object();
    thinking.set("type", "adaptive");
    json::Value cache = json::Value::object();
    cache.set("type", "ephemeral"); // repairs resend the same prefix

    body.set("max_tokens", 16000);
    body.set("thinking", thinking);
    body.set("output_config", output);
    body.set("fallbacks", "default"); // a declined request is retried server-side on the recommended model
    body.set("cache_control", cache);
    body.set("system", systemPrompt());
    body.set("messages", messages);
    return body.dump();
}

Outcome generate(const Question& q, const Transport& send, const std::function<void(Stage, int)>& onStage) {
    Outcome out;
    auto stage = [&](Stage s, int attempt) {
        if (onStage) onStage(s, attempt);
    };
    if (q.text.empty() && !q.image) {
        out.error = "type a question or upload an image of one";
        return out;
    }
    json::Value messages = initialMessages(q);

    std::string feedback;
    for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
        out.attempts = attempt;
        stage(attempt == 1 ? Stage::Asking : Stage::Repairing, attempt);

        http::Response r;
        for (int tries = 0;; ++tries) {
            r = send(requestBody(q, messages));
            if (!retryable(r) || tries == 2) break;
            std::this_thread::sleep_for(std::chrono::seconds(tries == 0 ? 3 : 10));
        }
        if (!r.error.empty()) {
            out.error = r.error;
            return out;
        }
        if (r.status != 200) {
            out.error = apiError(q, r);
            return out;
        }
        json::Value reply;
        if (!json::parse(r.body, reply)) {
            out.error = "the API's answer wasn't readable JSON";
            return out;
        }
        Reply read = q.provider == Provider::Claude ? readClaude(q, reply) : readOpenAI(q, reply);
        if (!read.error.empty()) {
            out.error = read.error;
            return out;
        }
        const std::string answer = read.answer;
        messages.push(std::move(read.turn));

        json::Value parsed;
        std::string error;
        std::shared_ptr<Spec> spec;
        if (!json::parse(answer, parsed, &error)) {
            feedback = "Your answer was not valid JSON (" + error + ").";
        } else if (spec = readSpec(parsed, error); !spec) {
            if (error.rfind("cannot simulate", 0) == 0) {
                out.error = "This question can't be simulated here: " + error.substr(error.find(':') + 2);
                return out;
            }
            feedback = "The app couldn't build your scene: " + error + ".";
        } else {
            stage(Stage::Checking, attempt);
            const Check check = verify(*spec);
            const float gap = spec->expected != 0.0f ? std::abs(check.result - spec->expected) / std::abs(spec->expected)
                                                     : std::abs(check.result);
            if (!check.problem.empty()) {
                feedback = "The app ran your scene and " + check.problem + ".";
                if (attempt == kMaxAttempts) out.warning = "The check run found a problem: " + check.problem;
            } else if (check.reached && gap > kMismatch) {
                feedback = "The app ran your scene: it measured " + spec->measure.label + " = " + formatValue(check.result) + " " +
                           spec->unit + ", but your expected_answer is " + formatValue(spec->expected) + " " + spec->unit +
                           ". Either the scene doesn't match the question (positions, sizes, masses, friction on both "
                           "surfaces, links, the measure settings) or the worked answer is wrong. Fix whichever is wrong.";
                if (attempt == kMaxAttempts)
                    out.warning = "The simulation (" + formatValue(check.result) + ") and the worked answer (" +
                                  formatValue(spec->expected) + ") differ by " + formatValue(gap * 100.0f) + "%.";
            } else {
                feedback.clear();
            }
            if (feedback.empty() || attempt == kMaxAttempts) {
                out.spec = spec;
                out.check = check;
                stage(Stage::Done, attempt);
                return out;
            }
        }
        if (attempt == kMaxAttempts) break;
        messages.push(feedbackTurn(q, feedback + " Return the complete corrected JSON."));
    }
    out.error = author(q) + "'s scene still didn't work after " + std::to_string(kMaxAttempts) + " tries. Last problem: " + feedback;
    return out;
}

std::shared_ptr<Generation> Generation::start(Question q, Transport send, std::string saveDir) {
    auto g = std::make_shared<Generation>();
    std::thread([g, q = std::move(q), send = std::move(send), saveDir = std::move(saveDir)] {
        Outcome out = generate(q, send, [&](Stage s, int attempt) {
            std::lock_guard lock(g->mutex_);
            g->status_.stage = s;
            g->status_.attempt = attempt;
        });
        if (out.spec && !saveDir.empty()) saveSpec(*out.spec, saveDir, out.savedPath);
        std::lock_guard lock(g->mutex_);
        g->status_.stage = out.spec ? Stage::Done : Stage::Failed;
        g->status_.outcome = std::move(out);
        g->status_.finished = true;
    }).detach();
    return g;
}

Generation::Status Generation::status() const {
    std::lock_guard lock(mutex_);
    Status s = status_;
    if (!s.finished) s.seconds = std::chrono::duration<float>(std::chrono::steady_clock::now() - began_).count();
    return s;
}

bool saveSpec(const Spec& spec, const std::string& dir, std::string& path) {
    std::error_code ec;
    fs::create_directories(toPath(dir), ec);
    const std::time_t now = std::time(nullptr);
    std::tm local{};
#ifdef _WIN32
    localtime_s(&local, &now); // this runs on the generation's worker thread
#else
    localtime_r(&now, &local);
#endif
    char stamp[32];
    std::strftime(stamp, sizeof stamp, "%Y%m%d-%H%M%S", &local);
    // the same question asked twice in a second must not overwrite the first answer
    const std::string base = std::string(stamp) + "-" + slug(spec.title);
    fs::path file = toPath(dir) / (base + ".json");
    for (int n = 2; fs::exists(file, ec) && n < 1000; ++n) file = toPath(dir) / (base + "-" + std::to_string(n) + ".json");
    std::ofstream f(file, std::ios::binary);
    if (!f) return false;
    f << spec.source.dump(2) << '\n';
    path = toUtf8(file);
    return static_cast<bool>(f);
}

std::vector<Saved> loadSaved(const std::string& dir) {
    std::vector<Saved> out;
    std::error_code ec;
    // advanced with increment(ec): a range-for's ++ throws on an unreadable entry and would take the app down
    for (fs::directory_iterator it(toPath(dir), ec), end; !ec && it != end; it.increment(ec)) {
        const fs::directory_entry& entry = *it;
        if (entry.path().extension() != ".json" || !entry.is_regular_file(ec)) continue;
        std::ifstream f(entry.path(), std::ios::binary);
        std::stringstream text;
        text << f.rdbuf();
        json::Value v;
        std::string error;
        if (!json::parse(text.str(), v)) continue;
        if (auto spec = readSpec(v, error)) out.push_back({toUtf8(entry.path()), spec});
    }
    // file names start with a timestamp, so newest first is reverse name order
    std::sort(out.begin(), out.end(), [](const Saved& a, const Saved& b) { return a.path > b.path; });
    return out;
}

bool removeSaved(const std::string& path) {
    std::error_code ec;
    return fs::remove(toPath(path), ec);
}

} // namespace ai
