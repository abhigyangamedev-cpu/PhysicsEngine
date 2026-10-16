#include "Http.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winhttp.h>
#endif

namespace ai::http {

#ifdef _WIN32

namespace {

std::wstring widen(const std::string& s) {
    if (s.empty()) return {};
    const int n = MultiByteToWideChar(CP_UTF8, 0, s.data(), int(s.size()), nullptr, 0);
    std::wstring w(size_t(n), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.data(), int(s.size()), w.data(), n);
    return w;
}

std::string lastError(const char* what) {
    const DWORD code = GetLastError();
    std::string msg = std::string(what) + " failed (error " + std::to_string(code) + ")";
    if (code == ERROR_WINHTTP_TIMEOUT) msg = std::string(what) + " timed out";
    else if (code == ERROR_WINHTTP_NAME_NOT_RESOLVED) msg = "could not resolve the server name: check the internet connection";
    else if (code == ERROR_WINHTTP_CANNOT_CONNECT) msg = "could not connect to the server: check the internet connection";
    return msg;
}

struct Handle {
    HINTERNET h = nullptr;
    ~Handle() {
        if (h) WinHttpCloseHandle(h);
    }
};

} // namespace

Response post(const std::string& host, const std::string& path, const Headers& headers, const std::string& body,
              int timeoutSeconds) {
    Response r;
    Handle session{WinHttpOpen(L"PhysicsEngine/1.0", WINHTTP_ACCESS_TYPE_AUTOMATIC_PROXY, WINHTTP_NO_PROXY_NAME,
                               WINHTTP_NO_PROXY_BYPASS, 0)};
    if (!session.h) {
        r.error = lastError("WinHttpOpen");
        return r;
    }
    const int ms = timeoutSeconds * 1000;
    WinHttpSetTimeouts(session.h, 15000, 15000, ms, ms);
    Handle connection{WinHttpConnect(session.h, widen(host).c_str(), INTERNET_DEFAULT_HTTPS_PORT, 0)};
    if (!connection.h) {
        r.error = lastError("connecting");
        return r;
    }
    Handle request{WinHttpOpenRequest(connection.h, L"POST", widen(path).c_str(), nullptr, WINHTTP_NO_REFERER,
                                      WINHTTP_DEFAULT_ACCEPT_TYPES, WINHTTP_FLAG_SECURE)};
    if (!request.h) {
        r.error = lastError("opening the request");
        return r;
    }
    std::wstring head;
    for (const auto& [name, value] : headers) head += widen(name) + L": " + widen(value) + L"\r\n";
    if (!WinHttpSendRequest(request.h, head.c_str(), DWORD(head.size()), const_cast<char*>(body.data()), DWORD(body.size()),
                            DWORD(body.size()), 0) ||
        !WinHttpReceiveResponse(request.h, nullptr)) {
        r.error = lastError("the request");
        return r;
    }
    DWORD status = 0, size = sizeof status;
    WinHttpQueryHeaders(request.h, WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER, WINHTTP_HEADER_NAME_BY_INDEX, &status,
                        &size, WINHTTP_NO_HEADER_INDEX);
    r.status = int(status);
    for (;;) {
        DWORD available = 0;
        if (!WinHttpQueryDataAvailable(request.h, &available)) {
            r.error = lastError("reading the response");
            break;
        }
        if (available == 0) break;
        const size_t at = r.body.size();
        r.body.resize(at + available);
        DWORD read = 0;
        if (!WinHttpReadData(request.h, r.body.data() + at, available, &read)) {
            r.error = lastError("reading the response");
            break;
        }
        r.body.resize(at + read);
    }
    return r;
}

#else

Response post(const std::string&, const std::string&, const Headers&, const std::string&, int) {
    Response r;
    r.error = "networking is only built for Windows (WinHTTP) in this version";
    return r;
}

#endif

} // namespace ai::http
