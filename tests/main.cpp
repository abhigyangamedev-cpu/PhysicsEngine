#include "Check.h"
#include <cstdio>
#include <cstring>

namespace check {

namespace {
int g_failures = 0;
bool g_current = false;
} // namespace

std::vector<Test>& registry() {
    static std::vector<Test> tests;
    return tests;
}

void fail(const char* file, int line, const std::string& message) {
    ++g_failures;
    g_current = true;
    std::printf("    %s:%d: %s\n", file, line, message.c_str());
}

std::string near(const char* a, const char* b, double va, double vb, double tol) {
    char buf[512];
    std::snprintf(buf, sizeof buf, "%s = %.6g, expected %s = %.6g (tolerance %.3g)", a, va, b, vb, tol);
    return buf;
}

} // namespace check

// Usage: editor_tests [substring]  runs the tests whose name contains it.
int main(int argc, char** argv) {
    int run = 0, failed = 0;
    for (const auto& t : check::registry()) {
        if (argc > 1 && !std::strstr(t.name, argv[1])) continue;
        check::g_current = false;
        t.fn();
        ++run;
        if (check::g_current) ++failed;
        std::printf("%s %s\n", check::g_current ? "FAIL" : "  ok", t.name);
    }
    std::printf("\n%d tests, %d failed, %d checks failed\n", run, failed, check::g_failures);
    return failed ? 1 : 0;
}
