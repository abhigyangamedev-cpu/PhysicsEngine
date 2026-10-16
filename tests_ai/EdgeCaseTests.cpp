// Edge cases in what the model sends back: JSON at the edges of the grammar, numbers no float can hold,
// mutated specs, and saved problems that collide or can't be read.
#include "Check.h"
#include "Generator.h"
#include "QuestionSpec.h"
#include "SpecHelpers.h"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <random>

using namespace spec_json;
namespace fs = std::filesystem;

namespace {

bool validUtf8(const std::string& s) {
    for (size_t i = 0; i < s.size();) {
        const unsigned char c = s[i];
        const int extra = c < 0x80 ? 0 : (c & 0xe0) == 0xc0 ? 1 : (c & 0xf0) == 0xe0 ? 2 : (c & 0xf8) == 0xf0 ? 3 : -1;
        if (extra < 0 || i + size_t(extra) >= s.size()) return false;
        unsigned cp = extra == 0 ? c : c & (0x7fu >> (extra + 1));
        for (int k = 1; k <= extra; ++k) {
            const unsigned char cc = s[i + k];
            if ((cc & 0xc0) != 0x80) return false;
            cp = cp << 6 | (cc & 0x3f);
        }
        static constexpr unsigned kMin[] = {0, 0x80, 0x800, 0x10000}; // shorter encodings are overlong
        if (cp < kMin[extra] || (cp >= 0xd800 && cp < 0xe000) || cp > 0x10ffff) return false;
        i += size_t(extra) + 1;
    }
    return true;
}

bool parses(const std::string& text) {
    json::Value v;
    return json::parse(text, v);
}

std::string parsedString(const std::string& literal) {
    json::Value v;
    if (!json::parse(literal, v) || !v.isString()) return "<not a string>";
    return v.asString();
}

std::string projectile2() {
    return answer2(object2("Ball", "ball", R"("position":[0,20.1],"radius":0.1,"velocity":[15,0],"no_spin":true)") + "," +
                       object2("Box", "block", R"("position":[4,0.5],"size":[1,1],"mass":2)"),
                   link2("spring", R"("a":"Box","b":"","b_offset":[8,0.5],"k":40)"), measure("lands", "Ball", 0, "range"),
                   30.3);
}

std::string pendulum3() {
    return answer3(object3("Bob", "ball", R"("position":[1,3,0],"radius":0.1)") + "," +
                       object3("Crate", "box", R"("position":[-2,0.5,0],"size":[1,1,1],"mass":3)"),
                   R"({"kind":"rod","a":"","a_offset":[0,4,0],"b":"Bob","b_offset":[0,0,0],"length":-1,"k":50,
                       "damping_ratio":0,"hinge_axis":[0,0,1],"swing_limit_deg":0})",
                   measure("reversals", "Bob", 3, "period", "x"), 2.84);
}

// Replaces one random leaf (or drops one key) somewhere in v with something hostile.
void mutate(json::Value& v, std::mt19937& rng) {
    static const double kNumbers[] = {0.0, -0.0, -1.0, 1e-30, -1e-30, 1e30, -1e30, 1e300, -1e300, 3.4e38, 2147483648.0,
                                      -2147483649.0, 1e9, 0.5, 1e-45};
    if (v.isObject() && !v.members().empty()) {
        auto& m = v.members();
        const size_t i = rng() % m.size();
        if (rng() % 12 == 0) {
            m.erase(m.begin() + long(i));
            return;
        }
        mutate(m[i].second, rng);
        return;
    }
    if (v.isArray() && !v.items().empty()) {
        auto& a = v.items();
        const size_t i = rng() % a.size();
        if (rng() % 10 == 0) {
            a.erase(a.begin() + long(i));
            return;
        }
        if (rng() % 10 == 0) {
            a.push_back(a[i]); // a duplicate object, link or coordinate
            return;
        }
        mutate(a[i], rng);
        return;
    }
    switch (rng() % 6) {
    case 0: v = json::Value(std::string(rng() % 3 == 0 ? "" : "Ball")); break;
    case 1: v = json::Value(); break;
    case 2: v = json::Value(rng() % 2 == 0); break;
    case 3: v = json::Value::array(); break;
    default: v = json::Value(kNumbers[rng() % std::size(kNumbers)]); break;
    }
}

} // namespace

// ---- JSON ----

TEST(EdgeJsonNumbersFollowTheGrammar) {
    for (const char* good : {"0", "-0", "1", "-1", "0.5", "-0.5", "1e3", "1E-3", "1.5e+3", "123456789012345678901234567890", "1e-400"})
        CHECK(parses(good));
    // strtod would take these: hex, infinities, NaN, and forms JSON doesn't have
    for (const char* bad : {"-", "+1", ".5", "1.", "01", "-01", "0x10", "-0x10", "1e", "1e+", "-inf", "-infinity", "-nan",
                            "1e400", "-1e400", "1..2", "--1", "1e5.5", "0b1"})
        if (parses(bad)) check::fail(__FILE__, __LINE__, std::string("accepted ") + bad);
    json::Value v;
    CHECK(json::parse("[1e308, -1e308, 4.9e-324]", v) && std::isfinite(v[0].asNumber()) && v[2].asNumber() > 0.0);
}

TEST(EdgeJsonSurrogatesAlwaysGiveValidUtf8) {
    CHECK(parsedString(R"("😀")") == "\xf0\x9f\x98\x80"); // a correct pair
    for (const char* s : {R"("\ud83d")", R"("\ud83dx")", R"("\ud83dA")", R"("\ude00")", R"("\ude00\ud83d")",
                          R"("􏿿")", R"("\ud800\ud800")", R"("a\ud83d\n")"}) {
        const std::string out = parsedString(s);
        if (out == "<not a string>" || !validUtf8(out)) check::fail(__FILE__, __LINE__, std::string("bad output for ") + s);
    }
    CHECK(parsedString(R"("\ud83dA")").find('A') != std::string::npos); // the character after a lone half survives
}

TEST(EdgeJsonFuzzNeverCrashesAndRoundTrips) {
    std::mt19937 rng(77);
    const std::string seeds[] = {projectile2(), pendulum3(), R"({"a":[1,2,{"b":null}],"c":"é\n"})"};
    const char alphabet[] = "{}[],:\"\\ -+.0123456789eEtrufalsn\x01\xff\xc3\xa9u";
    for (int trial = 0; trial < 4000; ++trial) {
        std::string text = seeds[trial % std::size(seeds)];
        const int edits = 1 + int(rng() % 8);
        for (int e = 0; e < edits && !text.empty(); ++e) {
            const size_t at = rng() % text.size();
            switch (rng() % 3) {
            case 0: text.erase(at, 1 + rng() % 4); break;
            case 1: text.insert(at, 1, alphabet[rng() % (sizeof alphabet - 1)]); break;
            default: text[at] = alphabet[rng() % (sizeof alphabet - 1)]; break;
            }
        }
        if (trial % 500 == 0) text = std::string(size_t(rng() % 50), '\0') + text;
        json::Value v;
        std::string error;
        if (!json::parse(text, v, &error)) {
            CHECK(!error.empty());
            continue;
        }
        // whatever parses writes back out as JSON that parses to the same thing
        const std::string once = v.dump(), pretty = v.dump(2);
        json::Value again, againPretty;
        CHECK(json::parse(once, again) && json::parse(pretty, againPretty));
        CHECK(again.dump() == once && againPretty.dump() == once);
        CHECK(validUtf8(once) || !validUtf8(text));
        std::string err;
        ai::readSpec(v, err); // and the reader copes with whatever shape it has
    }
}

TEST(EdgeJsonDeepNestingAndHugeInputs) {
    CHECK(!parses(std::string(100000, '[')));
    CHECK(!parses(std::string(100000, '[') + std::string(100000, ']')));
    std::string ok = std::string(150, '[') + std::string(150, ']');
    CHECK(parses(ok));
    std::string longString = "\"" + std::string(1 << 20, 'x') + "\"";
    CHECK(parses(longString));
    std::string many = "[";
    for (int i = 0; i < 200000; ++i) many += (i ? ",1" : "1");
    CHECK(parses(many + "]"));
    CHECK(!parses(many)); // unterminated
}

TEST(EdgeJsonNonFiniteNumbersWriteAsNull) {
    json::Value v = json::Value::array();
    v.push(std::numeric_limits<double>::infinity());
    v.push(std::nan(""));
    v.push(1.0 / 3.0);
    json::Value back;
    CHECK(json::parse(v.dump(), back));
    CHECK(back[0].isNull() && back[1].isNull());
    CHECK(back[2].asNumber() == 1.0 / 3.0); // a double comes back exactly
}

// ---- reading specs ----

TEST(EdgeSpecNumbersBeyondAFloatAreRejectedOrClamped) {
    // a double this size is finite but no float is; neither it nor a wild integer may reach a cast
    json::Value v = parsed(pendulum3());
    json::Value scene = v["scene_3d"], world = scene["world"], m = v["measure"];
    world.set("substeps", 1e300);
    scene.set("world", world);
    v.set("scene_3d", scene);
    m.set("event_value", 1e300);
    v.set("measure", m);
    std::string error;
    auto spec = ai::readSpec(v, error);
    if (spec) {
        CHECK(spec->scene3.world.substeps >= 1 && spec->scene3.world.substeps <= 32);
        CHECK(std::isfinite(spec->measure.value));
    }
    for (double big : {1e300, -1e300, 3.5e38, 1e39}) {
        json::Value w = parsed(projectile2());
        json::Value s2 = w["scene_2d"];
        json::Value objs = s2["objects"];
        json::Value ball = objs[0];
        ball.set("mass", big);
        ball.set("radius", big);
        ball.set("position", json::Value(json::Value::Array{big, big}));
        objs.items()[0] = ball;
        s2.set("objects", objs);
        w.set("scene_2d", s2);
        spec = ai::readSpec(w, error);
        if (!spec) continue;
        for (const auto& o : spec->scene2.objects) CHECK(std::isfinite(o.mass) && std::isfinite(o.radius) && std::isfinite(o.pos.x));
    }
}

TEST(EdgeSpecWithTooManyObjectsIsTurnedBack) {
    std::string objects;
    for (int i = 0; i < 2000; ++i) objects += (i ? "," : "") + object2("B" + std::to_string(i), "ball", R"("position":[0,)" + std::to_string(i) + "]");
    std::string error;
    auto spec = ai::readSpec(parsed(answer2(objects, "", measure("apex", "B0", 0, "y"), 1)), error);
    CHECK(!spec);
    CHECK(error.find("objects") != std::string::npos);
}

TEST(EdgeMutatedSpecsNeverCrashAndCheckRunsTerminate) {
    std::mt19937 rng(4242);
    const json::Value seeds[] = {parsed(projectile2()), parsed(pendulum3())};
    int accepted = 0;
    for (int trial = 0; trial < 300; ++trial) {
        json::Value v = seeds[trial % 2];
        for (int k = 0, n = 1 + int(rng() % 4); k < n; ++k) mutate(v, rng);
        std::string error;
        auto spec = ai::readSpec(v, error);
        if (!spec) {
            CHECK(!error.empty());
            continue;
        }
        ++accepted;
        spec->measure.timeout = std::min(spec->measure.timeout, 3.0f); // keep the run short
        const ai::Check c = ai::verify(*spec);
        CHECK(c.blewUp || !c.reached || std::isfinite(c.result));
        CHECK(c.time <= spec->measure.timeout + 0.1f);
        ai::toExample(spec);
        ai::toProblem(spec);
    }
    CHECK(accepted > 30); // the fuzz mostly makes small, still-valid changes
}

// ---- saved problems ----

TEST(EdgeSavingTheSameProblemTwiceKeepsBoth) {
    const fs::path dir = fs::temp_directory_path() / "pe_edge_saved";
    fs::remove_all(dir);
    std::string error;
    auto spec = ai::readSpec(parsed(projectile2()), error);
    CHECK(spec);
    if (!spec) return;
    std::string a, b, c;
    CHECK(ai::saveSpec(*spec, dir.string(), a));
    CHECK(ai::saveSpec(*spec, dir.string(), b));
    CHECK(ai::saveSpec(*spec, dir.string(), c));
    CHECK(a != b && b != c && a != c);
    CHECK(ai::loadSaved(dir.string()).size() == 3);
    fs::remove_all(dir);
}

TEST(EdgeLoadSavedSkipsJunkAndMissingFolders) {
    CHECK(ai::loadSaved((fs::temp_directory_path() / "pe_edge_no_such_dir").string()).empty());
    CHECK(ai::loadSaved("").empty());
    const fs::path dir = fs::temp_directory_path() / "pe_edge_junk";
    fs::remove_all(dir);
    fs::create_directories(dir / "folder.json"); // a directory with the extension
    std::ofstream(dir / "empty.json").close();
    std::ofstream(dir / "garbage.json") << "{\"title\": ";
    std::ofstream(dir / "wrong.json") << "[1,2,3]";
    std::ofstream(dir / "good.json") << projectile2();
    std::ofstream(dir / "good.JSON.bak") << projectile2();
    const auto saved = ai::loadSaved(dir.string());
    CHECK(saved.size() == 1);
    fs::remove_all(dir);
}
