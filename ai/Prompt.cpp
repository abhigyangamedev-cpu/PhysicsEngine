// What the model is told about the engine, and the JSON schema its answer must follow.
#include "QuestionSpec.h"
#include <initializer_list>

namespace ai {

namespace {

using json::Value;

Value type(const char* t) {
    Value v = Value::object();
    v.set("type", t);
    return v;
}
Value number() { return type("number"); }
Value string() { return type("string"); }
Value boolean() { return type("boolean"); }
Value vector() {
    Value v = type("array");
    v.set("items", number());
    return v;
}
Value oneOf(std::initializer_list<const char*> words) {
    Value v = string();
    Value list = Value::array();
    for (const char* w : words) list.push(w);
    v.set("enum", list);
    return v;
}
Value listOf(Value item) {
    Value v = type("array");
    v.set("items", std::move(item));
    return v;
}
// Strict structured output: every property listed and required, nothing extra.
Value object(std::initializer_list<std::pair<const char*, Value>> props) {
    Value v = type("object");
    Value properties = Value::object(), required = Value::array();
    for (const auto& [name, schema] : props) {
        properties.set(name, schema);
        required.push(name);
    }
    v.set("properties", properties);
    v.set("required", required);
    v.set("additionalProperties", false);
    return v;
}
Value nullable(Value v) {
    Value any = Value::array();
    any.push(std::move(v));
    any.push(type("null"));
    Value out = Value::object();
    out.set("anyOf", any);
    return out;
}

Value buildSchema() {
    const Value world2 = object({{"g", number()},
                                 {"ground", boolean()},
                                 {"ground_friction", number()},
                                 {"drag", number()},
                                 {"electric_field", vector()},
                                 {"magnetic_field", number()},
                                 {"coulomb", boolean()},
                                 {"coulomb_k", number()},
                                 {"mutual_gravity", boolean()},
                                 {"gravitational_constant", number()}});
    const Value object2 = object({{"name", string()},
                                  {"kind", oneOf({"block", "ball", "wedge"})},
                                  {"position", vector()},
                                  {"angle_deg", number()},
                                  {"size", vector()},
                                  {"radius", number()},
                                  {"wedge_base", number()},
                                  {"wedge_slope_deg", number()},
                                  {"wedge_flip", boolean()},
                                  {"mass", number()},
                                  {"velocity", vector()},
                                  {"omega", number()},
                                  {"force", vector()},
                                  {"friction", number()},
                                  {"restitution", number()},
                                  {"charge", number()},
                                  {"fixed", boolean()},
                                  {"no_spin", boolean()},
                                  {"hold_velocity", boolean()}});
    const Value link2 = object({{"kind", oneOf({"spring", "rod", "rope", "hinge", "pulley"})},
                                {"a", string()},
                                {"a_offset", vector()},
                                {"b", string()},
                                {"b_offset", vector()},
                                {"k", number()},
                                {"damping", number()},
                                {"length", number()},
                                {"pulley_wheel", vector()},
                                {"pulley_radius", number()},
                                {"pulley_ratio", number()}});
    const Value world3 = object({{"g", number()}, {"ground", boolean()}, {"ground_friction", number()}, {"substeps", number()}});
    const Value object3 = object({{"name", string()},
                                  {"kind", oneOf({"box", "ball"})},
                                  {"position", vector()},
                                  {"size", vector()},
                                  {"radius", number()},
                                  {"tilt_deg", vector()},
                                  {"mass", number()},
                                  {"velocity", vector()},
                                  {"spin", vector()},
                                  {"friction", number()},
                                  {"restitution", number()},
                                  {"rolling_resistance", number()},
                                  {"fixed", boolean()}});
    const Value link3 = object({{"kind", oneOf({"rod", "rope", "spring", "ball_joint", "hinge"})},
                                {"a", string()},
                                {"a_offset", vector()},
                                {"b", string()},
                                {"b_offset", vector()},
                                {"length", number()},
                                {"k", number()},
                                {"damping_ratio", number()},
                                {"hinge_axis", vector()},
                                {"swing_limit_deg", number()}});
    const Value measure = object(
        {{"event", oneOf({"time", "lands", "crosses_x", "crosses_y", "crosses_z", "apex", "stops", "distance", "reversals"})},
         {"event_object", string()},
         {"event_value", number()},
         {"axis", oneOf({"x", "y", "z"})},
         {"quantity", oneOf({"time", "x", "y", "z", "speed", "vx", "vy", "vz", "distance", "range", "max_height", "max_speed",
                             "kinetic_energy", "period"})},
         {"quantity_object", string()},
         {"label", string()},
         {"timeout", number()}});
    return object({{"cannot_simulate", string()},
                   {"title", string()},
                   {"topic", string()},
                   {"question", string()},
                   {"solution", string()},
                   {"expected_answer", number()},
                   {"unit", string()},
                   {"space", oneOf({"2d", "3d"})},
                   {"measure", measure},
                   {"view", object({{"center", vector()}, {"size", number()}})},
                   {"focus", string()},
                   {"scene_2d", nullable(object({{"world", world2}, {"objects", listOf(object2)}, {"links", listOf(link2)}}))},
                   {"scene_3d", nullable(object({{"world", world3}, {"objects", listOf(object3)}, {"links", listOf(link3)}}))}});
}

} // namespace

const json::Value& outputSchema() {
    static const Value schema = buildSchema();
    return schema;
}

const char* systemPrompt() {
    return R"(You turn a physics question (typed text and/or a photo or screenshot of it) into a simulation for a custom rigid-body physics engine, and solve it on paper. The app builds your scene, runs it, measures the asked-for quantity at the asked-for moment, and shows the measured value next to your textbook answer. Your answer is one JSON object following the given schema.

## Units and axes
SI units throughout (m, kg, s, N, rad/s, C, V/m, T). y points up; gravity pulls along -y with magnitude g. 2D: x right, y up, angles in degrees counterclockwise. 3D: x right, y up, z toward the viewer. Positions are object centres (except wedges, below).

## Choosing 2D or 3D
Use "2d" for anything that happens in a plane: projectiles, inclines and wedges, pulleys and Atwood machines, springs and oscillators, pendulums, collisions along a line or in a plane, charges in electric/magnetic fields, orbits. Use "3d" only when the motion is truly three-dimensional (conical pendulums, tetherballs, motion that leaves the plane, spinning objects). Fill the scene for the chosen space and set the other to null. Only 2D has wedges, pulleys, constant applied forces, drag, charges, fields, mutual gravity and constant-velocity drives.

## 2D scene (scene_2d)
world: g; ground: true adds a fixed 60 m wide floor named "Ground" whose top surface is y = 0, centred on x = 0 (don't add your own floor object, and don't name an object "Ground"); ground_friction; drag: linear damping rate in 1/s (velocity decays like e^(-drag*t)), 0 for none; electric_field [Ex, Ey] in V/m; magnetic_field: Bz in tesla (out of the screen); coulomb: pairwise Coulomb forces between charged objects with constant coulomb_k (8.99e9); mutual_gravity: pairwise Newtonian gravity with gravitational_constant (set g = 0 and ground = false for orbits).
objects:
- kind "block": size [width, height]. kind "ball": radius. kind "wedge": a fixed right-triangle ramp with wedge_base (horizontal length) and wedge_slope_deg; its position is the BOTTOM-LEFT corner of its bounding box. Unflipped, the slope rises from left to right: the surface runs from (x0, y0) to (x0 + base, y0 + base*tan(slope)) and the vertical face is on the right. wedge_flip mirrors it so the slope falls from left to right. Make wedges fixed unless the question has a movable wedge.
- To rest a block of height h on an unflipped wedge's slope (angle q) a distance d up from its foot: centre = (x0 + d*cos q - (h/2)*sin q, y0 + d*sin q + (h/2)*cos q) and angle_deg = q. On a flipped wedge measured from its foot at (x0 + base, y0): centre = (x0 + base - d*cos q + (h/2)*sin q, y0 + d*sin q + (h/2)*cos q), angle_deg = -q.
- mass (kg); inertia is that of the uniform solid shape. velocity [vx, vy] initial; omega initial angular velocity (rad/s, counterclockwise); force [fx, fy] a constant force on the centre on top of gravity (thrust, a push, buoyancy); charge in coulombs; fixed: immovable; no_spin: locks rotation (use it for sliding blocks that must not tip, and for point masses); hold_velocity: a drive holds the object's speed along its initial velocity.
- friction: coefficient (static = kinetic). Two touching objects use the SMALLER of their two values, so a frictionless surface needs friction 0 on at least one of them. restitution 0..1 (bounciness); a contact uses the LARGER of the two values.
links (a and b are object names, or "" for a fixed point in the world; a_offset / b_offset are the attachment point in the object's own frame relative to its centre, or the world point itself when that end is ""):
- "spring": k (N/m), damping (N*s/m), length = natural length (-1 uses the initial distance).
- "rod": a rigid massless rod; length -1 keeps the initial distance. "rope": like a rod but can go slack.
- "hinge": a pin joint whose pivot is the a end (an object point, or a world point when a is ""); b is the hinged object.
- "pulley": an ideal massless pulley: a rope from a goes over a fixed wheel at pulley_wheel [x, y] with pulley_radius and down to b; pulley_ratio r keeps (length on a's side) + r*(length on b's side) constant (1 for a simple pulley). length -1 uses the initial rope length. Both ends must be objects.
Unused link fields: k 50, damping 0, length -1, pulley_wheel [0, 0], pulley_radius 0.3, pulley_ratio 1.

## 3D scene (scene_3d)
world: g; ground: true adds a fixed 80 m x 80 m floor whose top surface is y = 0; ground_friction; substeps 8 (16 for fast or stiff setups).
objects: kind "box": size [x, y, z] full edge lengths; kind "ball": radius; tilt_deg [x, y, z] rotations applied about x, then y, then z; mass; velocity [vx, vy, vz]; spin [wx, wy, wz] rad/s; friction (a contact uses sqrt(mu_a*mu_b)); restitution (the larger value is used); rolling_resistance 0..1 (0 for textbook rolling); fixed.
links: "rod", "rope", "spring" (k in N/m, damping_ratio dimensionless, length -1 = initial distance), "ball_joint" (pivot at the a end; swing_limit_deg cone limit, 0 = free), "hinge" (pivot at the a end, turning about hinge_axis, a world direction). Endpoints work as in 2D with [x, y, z] offsets.

## Building a faithful scene
- Model what the question describes with its numbers. Point masses and particles are small balls (radius 0.05 to 0.2 m, or as given) with no_spin in 2D. Strings and cords are ropes; light rods are rods.
- Idealisations: "frictionless" means friction 0; "no bounce" / "perfectly inelastic" means restitution 0; "elastic" means restitution 1; ignore air resistance unless asked (drag 0).
- Objects must not overlap at the start. Objects resting on something should touch it exactly (e.g. a ball of radius r on the ground has its centre at y = r).
- The timestep is 1/120 s. Keep sizes and speeds sane: nothing smaller than 1 cm, no springs stiffer than about 1e5 N/m for light masses.
- A drop from a cliff or a launch from a height: model the cliff or table as a fixed block whose top is at the given height, or start the object in the air, so it falls to the ground at y = 0.

## Measuring (measure)
The run stops at an event of event_object, then reads quantity off quantity_object at that exact moment (interpolated inside the step). Events:
- "time": at t = event_value seconds.
- "lands": the object's lowest point comes down to y = event_value while falling (0 for the ground).
- "crosses_x" / "crosses_y" / "crosses_z": its centre crosses that coordinate (= event_value).
- "apex": the top of its path (vertical velocity turns from up to down).
- "stops": it comes to rest after moving.
- "distance": its centre is event_value metres from where it started.
- "reversals": the axis component of its velocity has changed sign event_value times (turnarounds of an oscillation; use 5 or more for a period).
Quantities: "time"; "x", "y", "z" (centre); "speed"; "vx", "vy", "vz"; "distance" (straight-line distance of the centre from its start); "range" (horizontal distance from the start); "max_height" (highest centre y so far); "max_speed"; "kinetic_energy" (translational + rotational); "period" (only with event "reversals": the averaged full period).
The simulation measures object centres, so set the scene so the textbook quantity equals what is measured (e.g. a projectile's height is its centre's height above the ground; if the question gives heights for a point object, start the centre at that height and expected_answer is the centre's value). The engine can't read forces or accelerations directly: when the question asks for one, measure a kinematic consequence that pins it down (distance covered in a set time, speed after a set distance, time to cover a distance) and use that as the measured quantity, and explain the link in the solution, ending with the asked-for value. label names the measured quantity in a few words ("Time to land", "Speed at the bottom"). timeout: simulated seconds to wait for the event (well past when you expect it, at least 5 s, at most 600).

## Text fields
Plain ASCII only: write ^2, sqrt(), deg, theta, mu, pi. title: up to 32 characters. topic: up to 24 characters ("Projectile motion"). question: the full question restated cleanly with all given values (transcribe it from the image when one is given), plus any assumption you added (like g = 9.81 m/s^2). solution: a concise worked solution with the key equations and the final number with its unit. expected_answer: the textbook value of the measured quantity, in unit. unit: like "m/s", "s", "m", "J" ("" if dimensionless).
view: center [x, y, z] (z = 0 in 2D) and size, the width in metres the camera should show to frame the whole motion. focus: the object to select and graph when the problem opens (usually quantity_object).

## When it can't be simulated
If the question is not about motion this engine can show (circuits, optics, heat, fluids, waves on strings, quantum, pure maths), put a one-sentence reason in cannot_simulate and fill the rest with placeholders: empty strings, 0, scene_2d and scene_3d null, any valid enum values. Otherwise cannot_simulate is "".

If the user message asks for a 2D or 3D space, use it unless it is impossible for that question. When the app reports a problem with your previous answer, fix the cause and return the complete corrected JSON.)";
}

} // namespace ai
