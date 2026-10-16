#include "Http.h"

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <winhttp.h>
#elif defined(PE_HAVE_CURL)
#include <curl/curl.h>
#include <mutex>
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

#elif defined(PE_HAVE_CURL)

namespace {

size_t append(char* data, size_t size, size_t count, void* out) {
    static_cast<std::string*>(out)->append(data, size * count);
    return size * count;
}

std::string describe(CURLcode code, const char* detail) {
    switch (code) {
    case CURLE_OPERATION_TIMEDOUT: return "the request timed out";
    case CURLE_COULDNT_RESOLVE_HOST: return "could not resolve the server name: check the internet connection";
    case CURLE_COULDNT_CONNECT: return "could not connect to the server: check the internet connection";
    default: return std::string("the request failed: ") + (*detail ? detail : curl_easy_strerror(code));
    }
}

} // namespace

Response post(const std::string& host, const std::string& path, const Headers& headers, const std::string& body,
              int timeoutSeconds) {
    Response r;
    // curl_global_init is not thread-safe and requests run on worker threads
    static std::once_flag once;
    std::call_once(once, [] { curl_global_init(CURL_GLOBAL_DEFAULT); });

    CURL* curl = curl_easy_init();
    if (!curl) {
        r.error = "could not start libcurl";
        return r;
    }
    curl_slist* list = nullptr;
    for (const auto& [name, value] : headers) list = curl_slist_append(list, (name + ": " + value).c_str());
    list = curl_slist_append(list, "Expect:"); // skip the 100-continue round trip on large bodies

    const std::string url = "https://" + host + path;
    char detail[CURL_ERROR_SIZE] = {};
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, list);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.data());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE_LARGE, curl_off_t(body.size()));
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "PhysicsEngine/1.0");
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 15L);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, long(timeoutSeconds));
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, append);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &r.body);
    curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, detail);

    const CURLcode code = curl_easy_perform(curl);
    if (code != CURLE_OK) r.error = describe(code, detail);
    long status = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
    r.status = int(status);

    curl_slist_free_all(list);
    curl_easy_cleanup(curl);
    return r;
}

#else

Response post(const std::string&, const std::string&, const Headers&, const std::string&, int) {
    Response r;
    r.error = "this build has no networking: rebuild with libcurl installed";
    return r;
}

#endif

} // namespace ai::http
