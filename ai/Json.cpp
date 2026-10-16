#include "Json.h"
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace json {

namespace {

const Value kNull;
const std::string kEmpty;
const Value::Array kEmptyArray;
const Value::Object kEmptyObject;

void appendUtf8(std::string& out, unsigned cp) {
    if (cp < 0x80) {
        out += char(cp);
    } else if (cp < 0x800) {
        out += char(0xc0 | (cp >> 6));
        out += char(0x80 | (cp & 0x3f));
    } else if (cp < 0x10000) {
        out += char(0xe0 | (cp >> 12));
        out += char(0x80 | ((cp >> 6) & 0x3f));
        out += char(0x80 | (cp & 0x3f));
    } else {
        out += char(0xf0 | (cp >> 18));
        out += char(0x80 | ((cp >> 12) & 0x3f));
        out += char(0x80 | ((cp >> 6) & 0x3f));
        out += char(0x80 | (cp & 0x3f));
    }
}

struct Parser {
    const std::string& s;
    size_t i = 0;
    std::string error;
    int depth = 0;

    bool fail(const char* what) {
        if (error.empty()) error = std::string(what) + " at byte " + std::to_string(i);
        return false;
    }
    void ws() {
        while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\n' || s[i] == '\r')) ++i;
    }
    bool literal(const char* word) {
        size_t n = std::char_traits<char>::length(word);
        if (s.compare(i, n, word) != 0) return fail("unexpected token");
        i += n;
        return true;
    }
    bool hex4(unsigned& out) {
        if (i + 4 > s.size()) return fail("short \\u escape");
        out = 0;
        for (int k = 0; k < 4; ++k) {
            char c = s[i++];
            out <<= 4;
            if (c >= '0' && c <= '9') out |= unsigned(c - '0');
            else if (c >= 'a' && c <= 'f') out |= unsigned(c - 'a' + 10);
            else if (c >= 'A' && c <= 'F') out |= unsigned(c - 'A' + 10);
            else return fail("bad \\u escape");
        }
        return true;
    }
    bool string(std::string& out) {
        ++i; // opening quote
        while (i < s.size()) {
            char c = s[i++];
            if (c == '"') return true;
            if (c != '\\') {
                out += c;
                continue;
            }
            if (i >= s.size()) break;
            char e = s[i++];
            switch (e) {
            case '"': out += '"'; break;
            case '\\': out += '\\'; break;
            case '/': out += '/'; break;
            case 'b': out += '\b'; break;
            case 'f': out += '\f'; break;
            case 'n': out += '\n'; break;
            case 'r': out += '\r'; break;
            case 't': out += '\t'; break;
            case 'u': {
                unsigned cp = 0;
                if (!hex4(cp)) return false;
                if (cp >= 0xd800 && cp < 0xdc00 && i + 1 < s.size() && s[i] == '\\' && s[i + 1] == 'u') {
                    const size_t at = i;
                    i += 2;
                    unsigned lo = 0;
                    if (!hex4(lo)) return false;
                    if (lo >= 0xdc00 && lo < 0xe000) cp = 0x10000 + ((cp - 0xd800) << 10) + (lo - 0xdc00);
                    else i = at; // not the second half: that escape stands on its own
                }
                appendUtf8(out, cp >= 0xd800 && cp < 0xe000 ? 0xfffd : cp); // half a pair has no UTF-8
                break;
            }
            default: return fail("bad escape");
            }
        }
        return fail("unterminated string");
    }
    bool value(Value& out) {
        if (++depth > 200) return fail("nested too deeply");
        ws();
        if (i >= s.size()) return fail("unexpected end");
        bool ok = true;
        char c = s[i];
        if (c == '{') {
            ++i;
            out = Value::object();
            ws();
            if (i < s.size() && s[i] == '}') {
                ++i;
            } else {
                while (ok) {
                    ws();
                    if (i >= s.size() || s[i] != '"') {
                        ok = fail("expected key");
                        break;
                    }
                    std::string key;
                    if (!(ok = string(key))) break;
                    ws();
                    if (i >= s.size() || s[i] != ':') {
                        ok = fail("expected ':'");
                        break;
                    }
                    ++i;
                    Value v;
                    if (!(ok = value(v))) break;
                    out.members().emplace_back(std::move(key), std::move(v));
                    ws();
                    if (i < s.size() && s[i] == ',') {
                        ++i;
                        continue;
                    }
                    if (i < s.size() && s[i] == '}') {
                        ++i;
                        break;
                    }
                    ok = fail("expected ',' or '}'");
                }
            }
        } else if (c == '[') {
            ++i;
            out = Value::array();
            ws();
            if (i < s.size() && s[i] == ']') {
                ++i;
            } else {
                while (ok) {
                    Value v;
                    if (!(ok = value(v))) break;
                    out.items().push_back(std::move(v));
                    ws();
                    if (i < s.size() && s[i] == ',') {
                        ++i;
                        continue;
                    }
                    if (i < s.size() && s[i] == ']') {
                        ++i;
                        break;
                    }
                    ok = fail("expected ',' or ']'");
                }
            }
        } else if (c == '"') {
            std::string str;
            ok = string(str);
            out = Value(std::move(str));
        } else if (c == 't') {
            ok = literal("true");
            out = Value(true);
        } else if (c == 'f') {
            ok = literal("false");
            out = Value(false);
        } else if (c == 'n') {
            ok = literal("null");
            out = Value();
        } else if (c == '-' || (c >= '0' && c <= '9')) {
            // JSON's grammar, checked before strtod, which would also take hex, inf and nan
            const size_t start = i;
            auto digits = [&] {
                const size_t from = i;
                while (i < s.size() && s[i] >= '0' && s[i] <= '9') ++i;
                return i > from;
            };
            if (s[i] == '-') ++i;
            if (i < s.size() && s[i] == '0') ++i;
            else if (!digits()) return fail("bad number");
            if (i < s.size() && s[i] == '.') {
                ++i;
                if (!digits()) return fail("bad number");
            }
            if (i < s.size() && (s[i] == 'e' || s[i] == 'E')) {
                ++i;
                if (i < s.size() && (s[i] == '+' || s[i] == '-')) ++i;
                if (!digits()) return fail("bad number");
            }
            const double d = std::strtod(s.c_str() + start, nullptr);
            if (!std::isfinite(d)) return fail("number out of range");
            out = Value(d);
        } else {
            ok = fail("unexpected character");
        }
        --depth;
        return ok;
    }
};

void newline(std::string& out, int indent, int depth) {
    if (indent < 0) return;
    out += '\n';
    out.append(size_t(indent * depth), ' ');
}

} // namespace

const std::string& Value::asString() const { return isString() ? str_ : kEmpty; }
const Value::Array& Value::items() const { return isArray() ? arr_ : kEmptyArray; }
const Value::Object& Value::members() const { return isObject() ? obj_ : kEmptyObject; }
Value::Array& Value::items() { return arr_; }
Value::Object& Value::members() { return obj_; }

const Value& Value::operator[](const std::string& key) const {
    if (!isObject()) return kNull;
    for (const auto& [k, v] : obj_)
        if (k == key) return v;
    return kNull;
}

const Value& Value::operator[](size_t index) const { return isArray() && index < arr_.size() ? arr_[index] : kNull; }

bool Value::has(const std::string& key) const {
    for (const auto& [k, v] : members())
        if (k == key) return true;
    return false;
}

Value& Value::set(const std::string& key, Value v) {
    if (!isObject()) *this = object();
    for (auto& [k, existing] : obj_)
        if (k == key) return existing = std::move(v);
    obj_.emplace_back(key, std::move(v));
    return obj_.back().second;
}

void Value::push(Value v) {
    if (!isArray()) *this = array();
    arr_.push_back(std::move(v));
}

std::string escape(const std::string& s) {
    std::string out = "\"";
    for (unsigned char c : s) {
        switch (c) {
        case '"': out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        case '\b': out += "\\b"; break;
        case '\f': out += "\\f"; break;
        default:
            if (c < 0x20) {
                char buf[8];
                std::snprintf(buf, sizeof buf, "\\u%04x", c);
                out += buf;
            } else {
                out += char(c);
            }
        }
    }
    return out + '"';
}

void Value::write(std::string& out, int indent, int depth) const {
    switch (type_) {
    case Type::Null: out += "null"; break;
    case Type::Bool: out += bool_ ? "true" : "false"; break;
    case Type::Number: {
        if (!std::isfinite(num_)) {
            out += "null";
            break;
        }
        char buf[32];
        if (num_ == std::floor(num_) && std::abs(num_) < 1e15) {
            std::snprintf(buf, sizeof buf, "%.0f", num_);
            out += buf;
        } else {
            out.append(buf, std::to_chars(buf, buf + sizeof buf, num_).ptr); // shortest text that reads back exactly
        }
        break;
    }
    case Type::String: out += escape(str_); break;
    case Type::Array:
        out += '[';
        for (size_t k = 0; k < arr_.size(); ++k) {
            if (k) out += ',';
            newline(out, indent, depth + 1);
            arr_[k].write(out, indent, depth + 1);
        }
        if (!arr_.empty()) newline(out, indent, depth);
        out += ']';
        break;
    case Type::Object:
        out += '{';
        for (size_t k = 0; k < obj_.size(); ++k) {
            if (k) out += ',';
            newline(out, indent, depth + 1);
            out += escape(obj_[k].first);
            out += indent < 0 ? ":" : ": ";
            obj_[k].second.write(out, indent, depth + 1);
        }
        if (!obj_.empty()) newline(out, indent, depth);
        out += '}';
        break;
    }
}

std::string Value::dump(int indent) const {
    std::string out;
    write(out, indent, 0);
    return out;
}

bool parse(const std::string& text, Value& out, std::string* error) {
    Parser p{text, 0, {}, 0};
    Value v;
    bool ok = p.value(v);
    if (ok) {
        p.ws();
        if (p.i != text.size()) ok = p.fail("trailing characters");
    }
    if (!ok) {
        if (error) *error = p.error;
        return false;
    }
    out = std::move(v);
    return true;
}

} // namespace json
