#include "Check.h"
#include "Expr.h"

namespace {

double eval(const char* s) { return evaluate(s).value_or(-12345.0); }
bool invalid(const char* s) { return !evaluate(s).has_value(); }

} // namespace

TEST(expr_numbers) {
    CHECK_NEAR(eval("42"), 42.0, 1e-12);
    CHECK_NEAR(eval("3.25"), 3.25, 1e-12);
    CHECK_NEAR(eval(".5"), 0.5, 1e-12);
    CHECK_NEAR(eval("2e-6"), 2e-6, 1e-18);
    CHECK_NEAR(eval("1.5E3"), 1500.0, 1e-9);
    CHECK_NEAR(eval("  7  "), 7.0, 1e-12);
}

TEST(expr_precedence) {
    CHECK_NEAR(eval("2+3*4"), 14.0, 1e-12);
    CHECK_NEAR(eval("(2+3)*4"), 20.0, 1e-12);
    CHECK_NEAR(eval("10-4-3"), 3.0, 1e-12);
    CHECK_NEAR(eval("12/3/2"), 2.0, 1e-12);
    CHECK_NEAR(eval("2^3^2"), 512.0, 1e-9); // right associative
    CHECK_NEAR(eval("-2^2"), -4.0, 1e-12);  // unary minus binds looser than ^
    CHECK_NEAR(eval("2*-3"), -6.0, 1e-12);
    CHECK_NEAR(eval("--4"), 4.0, 1e-12);
    CHECK_NEAR(eval("+5"), 5.0, 1e-12);
}

TEST(expr_trig_uses_degrees) {
    CHECK_NEAR(eval("sin(30)"), 0.5, 1e-12);
    CHECK_NEAR(eval("cos(60)"), 0.5, 1e-12);
    CHECK_NEAR(eval("tan(45)"), 1.0, 1e-12);
    CHECK_NEAR(eval("asin(0.5)"), 30.0, 1e-9);
    CHECK_NEAR(eval("acos(0)"), 90.0, 1e-9);
    CHECK_NEAR(eval("atan(1)"), 45.0, 1e-9);
    CHECK_NEAR(eval("10*cos(37)"), 7.986355, 1e-6);
}

TEST(expr_functions_and_constants) {
    CHECK_NEAR(eval("sqrt(16)"), 4.0, 1e-12);
    CHECK_NEAR(eval("SQRT(2*g*5)"), 9.904544, 1e-6);
    CHECK_NEAR(eval("pi"), 3.14159265358979, 1e-12);
    CHECK_NEAR(eval("2*PI"), 6.28318530717959, 1e-12);
    CHECK_NEAR(eval("g"), 9.81, 1e-12);
    CHECK_NEAR(eval("sqrt(sqrt(81))"), 3.0, 1e-12);
}

TEST(expr_rejects_bad_input) {
    CHECK(invalid(""));
    CHECK(invalid("2+"));
    CHECK(invalid("abc"));
    CHECK(invalid("sqrt("));
    CHECK(invalid("(1"));
    CHECK(invalid("1)"));
    CHECK(invalid("2 3"));
    CHECK(invalid("foo(2)"));
    CHECK(invalid("1/0"));
    CHECK(invalid("sqrt(-1)"));
    CHECK(invalid("2pi"));
}
