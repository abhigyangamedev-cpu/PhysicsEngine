#pragma once
// Blocking HTTPS POST. Implemented with WinHTTP on Windows; elsewhere it reports that it is unavailable.
#include <string>
#include <utility>
#include <vector>

namespace ai::http {

struct Response {
    int status = 0;     // HTTP status, 0 when the request never got an answer
    std::string body;
    std::string error;  // transport failure (DNS, TLS, timeout), empty otherwise
};

using Headers = std::vector<std::pair<std::string, std::string>>;

Response post(const std::string& host, const std::string& path, const Headers& headers, const std::string& body,
              int timeoutSeconds);

} // namespace ai::http
