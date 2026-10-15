#pragma once
#include <optional>
#include <string>

// Evaluates typed values: + - * / ^, parentheses, sqrt sin cos tan (degrees), asin acos atan, pi, g.
std::optional<double> evaluate(const std::string& text);
