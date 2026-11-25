#pragma once

#include <functional>
#include <string>

namespace qdevice_polar {

using LogCallback = std::function<void(const std::string&)>;

/// Logs a diagnostic message:
/// - Always emits via OutputDebugStringA on Windows.
/// - Invokes the optional console/UI log callback if registered.
/// - Appends to a timestamped log file if environment variable QDEVICE_POLAR_LOG is set
///   ("1"/"true" -> %TEMP%/qdevice_polar_h10.log on Windows or /tmp/qdevice_polar_h10.log,
///    or any explicit file path).
void logDiagnostic(const std::string& msg);

/// Registers (or clears with nullptr) an optional callback receiving formatted log lines.
void setDiagnosticLogCallback(LogCallback cb);

/// Returns the resolved log file path if file logging is enabled via QDEVICE_POLAR_LOG, or "".
std::string activeDiagnosticLogPath();

} // namespace qdevice_polar
