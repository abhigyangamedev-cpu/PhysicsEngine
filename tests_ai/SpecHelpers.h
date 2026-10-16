#pragma once
// Builds the JSON the model would write, with every schema field filled, from the parts a test cares about.
#include "Json.h"
#include <string>

namespace spec_json {

inline std::string object2(const std::string& name, const std::string& kind, const std::string& extra) {
    json::Value o;
    json::parse(R"({"name":"","kind":"ball","position":[0,0],"angle_deg":0,"size":[1,1],"radius":0.5,"wedge_base":4,
        "wedge_slope_deg":30,"wedge_flip":false,"mass":1,"velocity":[0,0],"omega":0,"force":[0,0],"friction":0,
        "restitution":0,"charge":0,"fixed":false,"no_spin":false,"hold_velocity":false})",
                o);
    o.set("name", name);
    o.set("kind", kind);
    json::Value more;
    json::parse("{" + extra + "}", more);
    for (const auto& [k, v] : more.members()) o.set(k, v);
    return o.dump();
}

inline std::string object3(const std::string& name, const std::string& kind, const std::string& extra) {
    json::Value o;
    json::parse(R"({"name":"","kind":"ball","position":[0,0,0],"size":[1,1,1],"radius":0.5,"tilt_deg":[0,0,0],"mass":1,
        "velocity":[0,0,0],"spin":[0,0,0],"friction":0.5,"restitution":0,"rolling_resistance":0,"fixed":false})",
                o);
    o.set("name", name);
    o.set("kind", kind);
    json::Value more;
    json::parse("{" + extra + "}", more);
    for (const auto& [k, v] : more.members()) o.set(k, v);
    return o.dump();
}

inline std::string link2(const std::string& kind, const std::string& extra) {
    json::Value l;
    json::parse(R"({"kind":"rod","a":"","a_offset":[0,0],"b":"","b_offset":[0,0],"k":50,"damping":0,"length":-1,
        "pulley_wheel":[0,0],"pulley_radius":0.3,"pulley_ratio":1})",
                l);
    l.set("kind", kind);
    json::Value more;
    json::parse("{" + extra + "}", more);
    for (const auto& [k, v] : more.members()) l.set(k, v);
    return l.dump();
}

inline std::string measure(const std::string& event, const std::string& object, double value, const std::string& quantity,
                           const std::string& axis = "y") {
    json::Value m = json::Value::object();
    m.set("event", event);
    m.set("event_object", object);
    m.set("event_value", value);
    m.set("axis", axis);
    m.set("quantity", quantity);
    m.set("quantity_object", object);
    m.set("label", "Measured");
    m.set("timeout", 30);
    return m.dump();
}

// A complete answer in the 2D space; objects and links are JSON arrays' contents.
inline std::string answer2(const std::string& objects, const std::string& links, const std::string& measureJson,
                           double expected, const std::string& world = R"({"g":9.81,"ground":true,"ground_friction":1,
                           "drag":0,"electric_field":[0,0],"magnetic_field":0,"coulomb":false,"coulomb_k":8.99e9,
                           "mutual_gravity":false,"gravitational_constant":1})") {
    return R"({"cannot_simulate":"","title":"Test","topic":"Testing","question":"A test question.","solution":"Worked.",
        "expected_answer":)" +
           json::Value(expected).dump() + R"(,"unit":"m","space":"2d","measure":)" + measureJson +
           R"(,"view":{"center":[0,5,0],"size":30},"focus":"",
        "scene_2d":{"world":)" +
           world + R"(,"objects":[)" + objects + R"(],"links":[)" + links + R"(]},"scene_3d":null})";
}

inline std::string answer3(const std::string& objects, const std::string& links, const std::string& measureJson,
                           double expected) {
    return R"({"cannot_simulate":"","title":"Test 3D","topic":"Testing","question":"A test question.","solution":"Worked.",
        "expected_answer":)" +
           json::Value(expected).dump() + R"(,"unit":"m","space":"3d","measure":)" + measureJson +
           R"(,"view":{"center":[0,3,0],"size":12},"focus":"","scene_2d":null,
        "scene_3d":{"world":{"g":9.81,"ground":true,"ground_friction":0.6,"substeps":8},"objects":[)" +
           objects + R"(],"links":[)" + links + R"(]}})";
}

inline json::Value parsed(const std::string& text) {
    json::Value v;
    json::parse(text, v);
    return v;
}

} // namespace spec_json
