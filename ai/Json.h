#pragma once
// A small JSON value with a parser and writer: enough for API requests, responses and saved problems.
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace json {

class Value {
public:
    enum class Type { Null, Bool, Number, String, Array, Object };
    using Array = std::vector<Value>;
    using Object = std::vector<std::pair<std::string, Value>>; // keeps insertion order

    Value() = default;
    Value(std::nullptr_t) {}
    Value(bool b) : type_(Type::Bool), bool_(b) {}
    Value(double n) : type_(Type::Number), num_(n) {}
    Value(float n) : Value(double(n)) {}
    Value(int n) : Value(double(n)) {}
    Value(const char* s) : type_(Type::String), str_(s) {}
    Value(std::string s) : type_(Type::String), str_(std::move(s)) {}
    Value(Array a) : type_(Type::Array), arr_(std::move(a)) {}
    Value(Object o) : type_(Type::Object), obj_(std::move(o)) {}
    static Value array() { return Value(Array{}); }
    static Value object() { return Value(Object{}); }

    Type type() const { return type_; }
    bool isNull() const { return type_ == Type::Null; }
    bool isBool() const { return type_ == Type::Bool; }
    bool isNumber() const { return type_ == Type::Number; }
    bool isString() const { return type_ == Type::String; }
    bool isArray() const { return type_ == Type::Array; }
    bool isObject() const { return type_ == Type::Object; }

    bool asBool(bool fallback = false) const { return isBool() ? bool_ : fallback; }
    double asNumber(double fallback = 0.0) const { return isNumber() ? num_ : fallback; }
    const std::string& asString() const; // empty unless a string
    const Array& items() const;          // empty unless an array
    const Object& members() const;       // empty unless an object
    Array& items();
    Object& members();

    // Object lookup; a missing key (or a non-object) gives a null value.
    const Value& operator[](const std::string& key) const;
    const Value& operator[](size_t index) const;
    bool has(const std::string& key) const;
    Value& set(const std::string& key, Value v); // replaces an existing key, returns the stored value
    void push(Value v);
    size_t size() const { return isArray() ? arr_.size() : isObject() ? obj_.size() : 0; }

    // convenience reads with defaults
    double num(const std::string& key, double fallback = 0.0) const { return (*this)[key].asNumber(fallback); }
    bool flag(const std::string& key, bool fallback = false) const { return (*this)[key].asBool(fallback); }
    std::string str(const std::string& key, const std::string& fallback = "") const {
        const Value& v = (*this)[key];
        return v.isString() ? v.str_ : fallback;
    }

    std::string dump(int indent = -1) const; // indent < 0: compact

private:
    Type type_ = Type::Null;
    bool bool_ = false;
    double num_ = 0.0;
    std::string str_;
    Array arr_;
    Object obj_;

    void write(std::string& out, int indent, int depth) const;
};

// Returns false and fills error (with a byte offset) on malformed input.
bool parse(const std::string& text, Value& out, std::string* error = nullptr);

std::string escape(const std::string& s); // the quoted JSON string literal

} // namespace json
