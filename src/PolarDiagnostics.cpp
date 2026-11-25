#include "qdevice_polar/PolarDiagnostics.hpp"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>

namespace qdevice_polar {

namespace {

std::mutex& logMutex() {
    static std::mutex mtx;
    return mtx;
}

LogCallback& logCallbackRef() {
    static LogCallback cb;
    return cb;
}

std::string getEnvString(const char* name) {
#ifdef _WIN32
    char* buf = nullptr;
    size_t len = 0;
    if (_dupenv_s(&buf, &len, name) == 0 && buf != nullptr) {
        std::string val(buf);
        std::free(buf);
        return val;
    }
    return {};
#else
    const char* val = std::getenv(name);
    return val ? std::string(val) : std::string();
#endif
}

std::string resolveLogFilePath() {
    const std::string env = getEnvString("QDEVICE_POLAR_LOG");
    if (env.empty() || env == "0" || env == "false" || env == "FALSE" || env == "off" || env == "OFF") {
        return {};
    }
    if (env == "1" || env == "true" || env == "TRUE" || env == "on" || env == "ON") {
#ifdef _WIN32
        std::string tmp = getEnvString("TEMP");
        if (tmp.empty()) tmp = getEnvString("TMP");
        if (tmp.empty()) tmp = ".";
        return tmp + "\\qdevice_polar_h10.log";
#else
        return "/tmp/qdevice_polar_h10.log";
#endif
    }
    return env;
}

std::string formatTimestampNow() {
    using namespace std::chrono;
    const auto now = system_clock::now();
    const auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
    const std::time_t t = system_clock::to_time_t(now);
    std::tm tm_local{};
#ifdef _WIN32
    localtime_s(&tm_local, &t);
#else
    localtime_r(&t, &tm_local);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm_local, "%Y-%m-%d %H:%M:%S")
        << '.' << std::setfill('0') << std::setw(3) << ms.count();
    return oss.str();
}

} // namespace

void setDiagnosticLogCallback(LogCallback cb) {
    std::lock_guard<std::mutex> lock(logMutex());
    logCallbackRef() = std::move(cb);
}

std::string activeDiagnosticLogPath() {
    return resolveLogFilePath();
}

void logDiagnostic(const std::string& msg) {
    const std::string line = "[" + formatTimestampNow() + "] [QDevice-Polar-H10] " + msg;

#ifdef _WIN32
    const std::string dbg = line + "\n";
    OutputDebugStringA(dbg.c_str());
#endif

    LogCallback cb_copy;
    {
        std::lock_guard<std::mutex> lock(logMutex());
        cb_copy = logCallbackRef();

        static std::string cached_path;
        static std::ofstream cached_ofs;

        const std::string path = resolveLogFilePath();
        if (path != cached_path) {
            if (cached_ofs.is_open()) {
                cached_ofs.close();
            }
            cached_path = path;
            if (!cached_path.empty()) {
                cached_ofs.open(cached_path, std::ios::app);
            }
        }
        if (cached_ofs.is_open()) {
            cached_ofs << line << '\n';
            cached_ofs.flush();
        }
    }

    if (cb_copy) {
        cb_copy(line);
    }
}

} // namespace qdevice_polar
