#pragma once

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

}  // namespace LibreVNAHeadless
