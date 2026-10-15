#include "Expr.h"
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <numbers>

namespace {

constexpr double kDeg = std::numbers::pi / 180.0;

struct Parser {
    const char* s;
    bool ok = true;

    void skip() {
        while (std::isspace(static_cast<unsigned char>(*s))) ++s;
    }
    bool eat(char c) {
        skip();
        if (*s != c) return false;
        ++s;
        return true;
    }

    double expr() {
        double v = term();
        for (;;) {
            if (eat('+')) v += term();
            else if (eat('-')) v -= term();
            else return v;
        }
    }
    double term() {
        double v = unary();
        for (;;) {
            if (eat('*')) v *= unary();
            else if (eat('/')) v /= unary();
            else return v;
        }
    }
    double unary() {
        if (eat('-')) return -unary();
        if (eat('+')) return unary();
        return power();
    }
    double power() {
        double v = primary();
        if (eat('^')) v = std::pow(v, unary());
        return v;
    }
    double primary() {
        skip();
        if (eat('(')) {
            double v = expr();
            if (!eat(')')) ok = false;
            return v;
        }
        if (std::isdigit(static_cast<unsigned char>(*s)) || *s == '.') {
            char* end = nullptr;
            double v = std::strtod(s, &end);
            if (end == s) ok = false;
            s = end;
            return v;
        }
        std::string name;
        while (std::isalpha(static_cast<unsigned char>(*s))) name += static_cast<char>(std::tolower(*s++));
        if (name == "pi") return std::numbers::pi;
        if (name == "g") return 9.81;
        if (name.empty() || !eat('(')) {
            ok = false;
            return 0.0;
        }
        double a = expr();
        if (!eat(')')) ok = false;
        if (name == "sqrt") return std::sqrt(a);
        if (name == "sin") return std::sin(a * kDeg);
        if (name == "cos") return std::cos(a * kDeg);
        if (name == "tan") return std::tan(a * kDeg);
        if (name == "asin") return std::asin(a) / kDeg;
        if (name == "acos") return std::acos(a) / kDeg;
        if (name == "atan") return std::atan(a) / kDeg;
        ok = false;
        return 0.0;
    }
};

} // namespace

std::optional<double> evaluate(const std::string& text) {
    Parser p{text.c_str()};
    double v = p.expr();
    p.skip();
    if (!p.ok || *p.s != '\0' || !std::isfinite(v)) return std::nullopt;
    return v;
}
