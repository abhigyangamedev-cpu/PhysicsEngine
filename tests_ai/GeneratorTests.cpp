// The question-to-simulation exchange against a scripted API: request shape, repairs, refusals, saving.
#include "Check.h"
#include "Generator.h"
#include "SpecHelpers.h"
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <vector>

using namespace spec_json;

namespace {

// An API reply carrying text as the model's answer, after a thinking block.
std::string reply(const std::string& text, const char* stop = "end_turn") {
    json::Value thinking = json::Value::object();
    thinking.set("type", "thinking");
    thinking.set("thinking", "");
    thinking.set("signature", "sig");
    json::Value answer = json::Value::object();
    answer.set("type", "text");
    answer.set("text", text);
    json::Value content = json::Value::array();
    content.push(thinking);
    content.push(answer);
    json::Value r = json::Value::object();
    r.set("type", "message");
    r.set("content", content);
    r.set("stop_reason", stop);
    return r.dump();
}

// Plays the given replies in order and keeps every request body it was sent.
struct Script {
    std::vector<ai::http::Response> replies;
    std::vector<json::Value> requests;

    ai::Transport transport() {
        return [this](const std::string& body) {
            json::Value v;
            json::parse(body, v);
            requests.push_back(v);
            if (requests.size() > replies.size()) return ai::http::Response{500, "{}", ""};
            return replies[requests.size() - 1];
        };
    }
};

ai::http::Response ok(const std::string& text) { return {200, reply(text), ""}; }

const double kDropTime = std::sqrt(2.0 * 10.0 / 9.81);

std::string goodAnswer(double expected = kDropTime) {
    return answer2(object2("Ball", "ball", R"("position":[0,10.1],"radius":0.1,"no_spin":true)"), "",
                   measure("lands", "Ball", 0, "time"), expected);
}

ai::Question question() {
    ai::Question q;
    q.text = "A ball is dropped from 10 m. How long does it take to land?";
    return q;
}

} // namespace

TEST(RequestAsksForStructuredJsonFromOpus) {
    ai::Question q = question();
    q.image = ai::Image{"q.png", "image/png", "AAAA"};
    q.space = "2d";
    json::Value body;
    CHECK(json::parse(ai::requestBody(q, ai::initialMessages(q)), body));
    CHECK(body.str("model") == "claude-opus-5-5");
    CHECK(body["thinking"].str("type") == "adaptive");
    CHECK(body["output_config"].str("effort") == "high");
    CHECK(body["output_config"]["format"].str("type") == "json_schema");
    CHECK(body["output_config"]["format"]["schema"].str("type") == "object");
    CHECK(body.str("fallbacks") == "default");
    CHECK(!body.str("system").empty());
    const json::Value& content = body["messages"][0]["content"];
    CHECK(content[0].str("type") == "image" && content[0]["source"].str("media_type") == "image/png");
    CHECK(content[1].str("text").find("10 m") != std::string::npos);
    CHECK(content[1].str("text").find("2D") != std::string::npos);
}

TEST(GenerateAcceptsAGoodFirstAnswer) {
    Script s;
    s.replies = {ok(goodAnswer())};
    std::vector<ai::Stage> stages;
    const ai::Outcome out = ai::generate(question(), s.transport(), [&](ai::Stage st, int) { stages.push_back(st); });
    CHECK(out.spec && out.error.empty() && out.warning.empty());
    CHECK(out.attempts == 1 && s.requests.size() == 1);
    CHECK(out.check.reached);
    CHECK_NEAR(out.check.result, kDropTime, 0.01);
    CHECK(stages.size() == 3 && stages.front() == ai::Stage::Asking && stages.back() == ai::Stage::Done);
}

TEST(GenerateSendsBuildErrorsBackForARepair) {
    Script s;
    const std::string broken = answer2(object2("Ball", "ball", R"("position":[0,10.1],"radius":0.1)"),
                                       link2("rope", R"("a":"","a_offset":[0,12],"b":"Balloon")"), measure("lands", "Ball", 0, "time"),
                                       kDropTime);
    s.replies = {ok(broken), ok(goodAnswer())};
    const ai::Outcome out = ai::generate(question(), s.transport());
    CHECK(out.spec && out.attempts == 2 && s.requests.size() == 2);
    const json::Value& messages = s.requests[1]["messages"];
    CHECK(messages.size() == 3);
    CHECK(messages[1].str("role") == "assistant");
    CHECK(messages[1]["content"][0].str("type") == "thinking" && messages[1]["content"][0].str("signature") == "sig");
    const std::string feedback = messages[2]["content"][0].str("text");
    CHECK(messages[2].str("role") == "user" && feedback.find("Balloon") != std::string::npos);
}

TEST(GenerateRepairsAnswersThatDisagreeWithTheSimulation) {
    Script s;
    s.replies = {ok(goodAnswer(2.0 * kDropTime)), ok(goodAnswer())};
    const ai::Outcome out = ai::generate(question(), s.transport());
    CHECK(out.spec && out.attempts == 2 && out.warning.empty());
    const std::string feedback = s.requests[1]["messages"][2]["content"][0].str("text");
    CHECK(feedback.find("measured") != std::string::npos && feedback.find("expected_answer") != std::string::npos);
}

TEST(GenerateKeepsAMismatchAfterTheLastTryWithAWarning) {
    Script s;
    s.replies = {ok(goodAnswer(3.0)), ok(goodAnswer(3.0)), ok(goodAnswer(3.0))};
    const ai::Outcome out = ai::generate(question(), s.transport());
    CHECK(out.spec && out.attempts == ai::kMaxAttempts);
    CHECK(!out.warning.empty());
}

TEST(GenerateRepairsInvalidJson) {
    Script s;
    s.replies = {ok("{not json"), ok(goodAnswer())};
    const ai::Outcome out = ai::generate(question(), s.transport());
    CHECK(out.spec && out.attempts == 2);
    CHECK(s.requests[1]["messages"][2]["content"][0].str("text").find("not valid JSON") != std::string::npos);
}

TEST(GenerateStopsOnQuestionsItCannotSimulate) {
    Script s;
    s.replies = {ok(R"({"cannot_simulate":"It is about an electric circuit.","space":"2d"})")};
    const ai::Outcome out = ai::generate(question(), s.transport());
    CHECK(!out.spec && s.requests.size() == 1);
    CHECK(out.error.find("electric circuit") != std::string::npos);
}

TEST(GenerateReportsRefusalsAndApiErrors) {
    {
        Script s;
        s.replies = {{200, reply("", "refusal"), ""}};
        const ai::Outcome out = ai::generate(question(), s.transport());
        CHECK(!out.spec && out.error.find("declined") != std::string::npos);
    }
    {
        Script s;
        s.replies = {{401, R"({"type":"error","error":{"type":"authentication_error","message":"invalid x-api-key"}})", ""}};
        const ai::Outcome out = ai::generate(question(), s.transport());
        CHECK(!out.spec && out.error.find("API key") != std::string::npos && s.requests.size() == 1);
    }
    {
        Script s;
        s.replies = {{400, R"({"type":"error","error":{"type":"invalid_request_error","message":"bad schema"}})", ""}};
        const ai::Outcome out = ai::generate(question(), s.transport());
        CHECK(!out.spec && out.error.find("bad schema") != std::string::npos);
    }
    {
        Script s;
        s.replies = {{200, reply("{\"title\":", "max_tokens"), ""}};
        const ai::Outcome out = ai::generate(question(), s.transport());
        CHECK(!out.spec && out.error.find("ran out of room") != std::string::npos);
    }
}

TEST(GenerateNeedsAQuestion) {
    Script s;
    const ai::Outcome out = ai::generate(ai::Question{}, s.transport());
    CHECK(!out.spec && s.requests.empty() && !out.error.empty());
}

TEST(SavedSpecsLoadBackNewestFirst) {
    namespace fs = std::filesystem;
    const fs::path dir = fs::temp_directory_path() / "pe_ai_tests_saved";
    std::error_code ec;
    fs::remove_all(dir, ec);
    std::string error;
    auto spec = ai::readSpec(parsed(goodAnswer()), error);
    CHECK(spec);
    if (!spec) return;
    std::string path;
    CHECK(ai::saveSpec(*spec, dir.string(), path));
    CHECK(fs::exists(fs::path(path)) && path.find("test") != std::string::npos);
    {
        std::ofstream junk(dir / "junk.json");
        junk << "{ nope";
    }
    const auto saved = ai::loadSaved(dir.string());
    CHECK(saved.size() == 1);
    if (!saved.empty()) {
        CHECK(saved[0].spec->title == "Test");
        CHECK(saved[0].spec->scene2.objects.size() == spec->scene2.objects.size());
        CHECK(ai::removeSaved(saved[0].path));
    }
    CHECK(ai::loadSaved(dir.string()).empty());
    fs::remove_all(dir, ec);
}

// ---- OpenAI ----

namespace {

std::string openaiReply(const std::string& content, const char* finish = "stop", const std::string& refusal = "") {
    json::Value msg = json::Value::object();
    msg.set("role", "assistant");
    msg.set("content", content.empty() ? json::Value() : json::Value(content));
    msg.set("refusal", refusal.empty() ? json::Value() : json::Value(refusal));
    json::Value choice = json::Value::object();
    choice.set("index", 0);
    choice.set("message", msg);
    choice.set("finish_reason", finish);
    json::Value choices = json::Value::array();
    choices.push(choice);
    json::Value r = json::Value::object();
    r.set("object", "chat.completion");
    r.set("choices", choices);
    return r.dump();
}

ai::Question openaiQuestion(const char* model = "") {
    ai::Question q = question();
    q.provider = ai::Provider::OpenAI;
    q.model = model;
    return q;
}

} // namespace

TEST(OpenAIRequestUsesStrictJsonSchemaAndImages) {
    ai::Question q = openaiQuestion("gpt-5");
    q.image = ai::Image{"q.jpg", "image/jpeg", "BBBB"};
    json::Value body;
    CHECK(json::parse(ai::requestBody(q, ai::initialMessages(q)), body));
    CHECK(body.str("model") == "gpt-5");
    CHECK(body["response_format"].str("type") == "json_schema");
    CHECK(body["response_format"]["json_schema"].flag("strict"));
    CHECK(body["response_format"]["json_schema"]["schema"].str("type") == "object");
    CHECK(body.str("reasoning_effort") == "high");
    CHECK(body.num("max_completion_tokens") > 0 && !body.has("max_tokens"));
    CHECK(!body.has("thinking") && !body.has("system") && !body.has("fallbacks"));
    const json::Value& messages = body["messages"];
    CHECK(messages[0].str("role") == "system" && !messages[0].str("content").empty());
    const json::Value& content = messages[1]["content"];
    CHECK(content[0].str("type") == "text" && content[0].str("text").find("10 m") != std::string::npos);
    CHECK(content[1].str("type") == "image_url");
    CHECK(content[1]["image_url"].str("url") == "data:image/jpeg;base64,BBBB");
}

TEST(OpenAIOlderChatModelsGetNoReasoningEffort) {
    const ai::Question q = openaiQuestion("gpt-4o");
    json::Value body;
    CHECK(json::parse(ai::requestBody(q, ai::initialMessages(q)), body));
    CHECK(!body.has("reasoning_effort"));
    const ai::Question o = openaiQuestion("o4-mini");
    CHECK(json::parse(ai::requestBody(o, ai::initialMessages(o)), body));
    CHECK(body.str("reasoning_effort") == "high");
}

TEST(OpenAIGenerateAcceptsAGoodAnswer) {
    Script s;
    s.replies = {{200, openaiReply(goodAnswer()), ""}};
    const ai::Outcome out = ai::generate(openaiQuestion(), s.transport());
    CHECK(out.spec && out.error.empty() && out.attempts == 1);
    CHECK_NEAR(out.check.result, kDropTime, 0.01);
    CHECK(s.requests[0].str("model") == ai::defaultModel(ai::Provider::OpenAI));
}

TEST(OpenAIGenerateRepairsWithPlainTextTurns) {
    Script s;
    s.replies = {{200, openaiReply(goodAnswer(2.0 * kDropTime)), ""}, {200, openaiReply(goodAnswer()), ""}};
    const ai::Outcome out = ai::generate(openaiQuestion("gpt-5"), s.transport());
    CHECK(out.spec && out.attempts == 2);
    const json::Value& messages = s.requests[1]["messages"];
    CHECK(messages.size() == 4); // system, question, the first answer, the feedback
    CHECK(messages[2].str("role") == "assistant" && messages[2].str("content").find("\"space\"") != std::string::npos);
    CHECK(messages[3].str("role") == "user" && messages[3].str("content").find("measured") != std::string::npos);
}

TEST(OpenAIGenerateReportsRefusalsTruncationAndQuota) {
    {
        Script s;
        s.replies = {{200, openaiReply("", "stop", "I can't help with that."), ""}};
        const ai::Outcome out = ai::generate(openaiQuestion("gpt-5"), s.transport());
        CHECK(!out.spec && out.error.find("declined") != std::string::npos && out.error.find("gpt-5") != std::string::npos);
    }
    {
        Script s;
        s.replies = {{200, openaiReply("{\"title\":", "length"), ""}};
        const ai::Outcome out = ai::generate(openaiQuestion(), s.transport());
        CHECK(!out.spec && out.error.find("ran out of room") != std::string::npos);
    }
    {
        Script s;
        s.replies = {{429, R"({"error":{"message":"You exceeded your current quota","type":"insufficient_quota","code":"insufficient_quota"}})", ""}};
        const ai::Outcome out = ai::generate(openaiQuestion(), s.transport());
        CHECK(!out.spec && out.error.find("no credit") != std::string::npos && s.requests.size() == 1);
    }
    {
        Script s;
        s.replies = {{401, R"({"error":{"message":"Incorrect API key provided","code":"invalid_api_key"}})", ""}};
        const ai::Outcome out = ai::generate(openaiQuestion(), s.transport());
        CHECK(!out.spec && out.error.find("OPENAI_API_KEY") != std::string::npos);
    }
    {
        Script s;
        s.replies = {{404, R"({"error":{"message":"The model `gpt-9` does not exist","code":"model_not_found"}})", ""}};
        const ai::Outcome out = ai::generate(openaiQuestion("gpt-9"), s.transport());
        CHECK(!out.spec && out.error.find("\"gpt-9\"") != std::string::npos);
    }
}

TEST(KeysAreRecognisedByTheirPrefix) {
    CHECK(ai::providerOfKey("sk-ant-api03-abc") == ai::Provider::Claude);
    CHECK(ai::providerOfKey("  sk-proj-abc\n") == ai::Provider::OpenAI);
    CHECK(ai::providerOfKey("sk-abc") == ai::Provider::OpenAI);
    CHECK(!ai::providerOfKey("hello"));
    CHECK(ai::trimKey(" sk-ant-x\r\n") == "sk-ant-x");
}
