#pragma once

#include "json.hpp"

#include <QString>
#include <complex>
#include <map>
#include <vector>

namespace LibreVNAHeadless {

struct DeviceInfo {
    QString label;
    QString serial;
    QString driver;
};

struct SweepRequest {
    QString cal_path;
    double f_start_hz = 0.0;
    double f_stop_hz = 0.0;
    int points = 0;
    double ifbw_hz = 0.0;
    double power_dbm = 0.0;
    double threshold_db = 0.0;
    double timeout_ms = 15000.0;
    std::vector<int> excited_ports;
    QString serial;
};

/// Standard type for calibration acquisition
enum class CalStandardType {
    Open,
    Short,
    Load,
    Through,
};

/// Result for a single calibration acquisition step
struct CalAcquisitionResult {
    bool success = false;
    QString error;
    QString standard;   // "open", "short", "load", "through"
    int port1 = 0;
    int port2 = 0;
    /// Points as JSON-compatible vector of maps
    nlohmann::json trace;
};

/// Request to run a complete SOLT/OSL calibration
struct CalibrationRequest {
    QString type = "SOLT";       // "SOLT" or "OSL"
    double f_start_hz = 0.0;
    double f_stop_hz = 0.0;
    int points = 0;
    double ifbw_hz = 0.0;
    double power_dbm = 0.0;
    double timeout_ms = 30000.0;
    QString output_path;          // .cal file save path
    std::vector<int> ports;       // e.g. {1} or {1,2}
    QString serial;
};

/// Result for a complete calibration run
struct CalibrationResult {
    bool success = false;
    QString error;
    QString cal_file;             // saved .cal path
    bool open_done = false;
    bool short_done = false;
    bool load_done = false;
    bool through_done = false;
    QString message;
};

struct SweepPoint {
    double frequency = 0.0;
    std::map<QString, std::complex<double>> params;
};

struct ParamResult {
    bool pass = true;
    double worst_db = -1e9;
    double fail_at_hz = 0.0;
    bool has_fail = false;
};

struct SweepResult {
    bool overall_pass = true;
    std::map<QString, ParamResult> results;
    std::vector<SweepPoint> trace;
};

std::vector<DeviceInfo> list_devices(bool *real_driver, QString *warning);
SweepResult run_sweep(const SweepRequest &request);
CalAcquisitionResult acquire_cal_standard(const QString &standard, int port1, int port2,
                                          const SweepRequest &sweep_params);
CalibrationResult run_calibration(const CalibrationRequest &request);

}  // namespace LibreVNAHeadless
