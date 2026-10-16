// The JSON reader and writer the workspace uses for API traffic and saved problems.
#include "Check.h"
#include "Generator.h"
#include "Json.h"

using json::Value;

TEST(JsonParsesNestedValues) {
    Value v;
    std::string error;
    CHECK(json::parse(R"( {"a": [1, 2.5, -3e2, true, false, null], "b": {"c": "hi"}, "d": ""} )", v, &error));
    CHECK(error.empty());
    CHECK(v["a"].size() == 6);
    CHECK_NEAR(v["a"][1].asNumber(), 2.5, 1e-12);
    CHECK_NEAR(v["a"][2].asNumber(), -300.0, 1e-12);
    CHECK(v["a"][3].asBool() && !v["a"][4].asBool(true) && v["a"][5].isNull());
    CHECK(v["b"].str("c") == "hi");
    CHECK(v.has("d") && v.str("d", "x").empty());
    CHECK(v["missing"]["deeper"].isNull());
}

TEST(JsonDecodesEscapesAndUnicode) {
    Value v;
    CHECK(json::parse(R"(["line\nbreak \"q\" \\ ° 😀"])", v));
    CHECK(v[0].asString() == "line\nbreak \"q\" \\ \xc2\xb0 \xf0\x9f\x98\x80");
}

TEST(JsonRoundTrips) {
    Value v = Value::object();
    v.set("text", "tab\there \"quoted\"\n");
    v.set("n", 0.1);
    v.set("i", 42);
    Value list = Value::array();
    list.push(Value());
    list.push(true);
    v.set("list", list);
    for (int indent : {-1, 2}) {
        Value back;
        CHECK(json::parse(v.dump(indent), back));
        CHECK(back.str("text") == v.str("text"));
        CHECK_NEAR(back.num("n"), 0.1, 1e-12);
        CHECK(back.num("i") == 42 && back["list"][1].asBool() && back["list"][0].isNull());
    }
    CHECK(Value(3).dump() == "3");
}

TEST(JsonRejectsMalformedInput) {
    Value v;
    std::string error;
    for (const char* bad : {"", "{", "[1,]", "{\"a\" 1}", "\"open", "tru", "[1] 2", "{\"a\":01x}"}) {
        error.clear();
        CHECK(!json::parse(bad, v, &error));
        CHECK(!error.empty());
    }
}

TEST(JsonSetReplacesKeys) {
    Value v = Value::object();
    v.set("a", 1);
    v.set("a", 2);
    CHECK(v.size() == 1 && v.num("a") == 2);
}

TEST(Base64MatchesKnownVectors) {
    CHECK(ai::base64("") == "");
    CHECK(ai::base64("f") == "Zg==");
    CHECK(ai::base64("fo") == "Zm8=");
    CHECK(ai::base64("foo") == "Zm9v");
    CHECK(ai::base64("foobar") == "Zm9vYmFy");
    CHECK(ai::base64(std::string("\xff\x00\x10", 3)) == "/wAQ");
}
