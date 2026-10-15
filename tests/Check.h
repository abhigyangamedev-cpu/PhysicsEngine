#pragma once
// Minimal test framework: TEST(name) { CHECK(...); CHECK_NEAR(...); }
#include <cmath>
#include <string>
#include <vector>

namespace check {

struct Test {
    const char* name;
    void (*fn)();
};

std::vector<Test>& registry();
void fail(const char* file, int line, const std::string& message);

struct Register {
    Register(const char* name, void (*fn)()) { registry().push_back({name, fn}); }
};

std::string near(const char* a, const char* b, double va, double vb, double tol);

} // namespace check

#define TEST(name)                                                                                                     \
    static void name();                                                                                                \
    static check::Register register_##name(#name, name);                                                              \
    static void name()

#define CHECK(cond)                                                                                                    \
    do {                                                                                                               \
        if (!(cond)) check::fail(__FILE__, __LINE__, #cond);                                                           \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                                                          \
    do {                                                                                                               \
        const double va_ = (a), vb_ = (b);                                                                             \
        if (!(std::abs(va_ - vb_) <= (tol))) check::fail(__FILE__, __LINE__, check::near(#a, #b, va_, vb_, (tol)));  \
    } while (0)

// relative tolerance, e.g. CHECK_REL(a, expected, 0.01) for 1%
#define CHECK_REL(a, b, rel) CHECK_NEAR(a, b, std::abs(double(b)) * (rel))
