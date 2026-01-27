#pragma once

#include <complex>
#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace librevna::headless
{
struct SweepConfiguration
{
    double start_frequency_hz = 0.0;
    double stop_frequency_hz = 0.0;
    std::uint32_t points = 0;
    double if_bandwidth_hz = 0.0;
    double power_dbm = 0.0;
    double timeout_ms = 0.0;
    std::vector<int> excited_ports;
};

struct VNAMeasurement
{
    VNAMeasurement() = default;
    VNAMeasurement(double frequency_hz,
                   std::map<std::string, std::complex<double>> parameters)
        : frequency(frequency_hz),
          parameters(std::move(parameters))
    {
    }

    double frequency = 0.0;
    std::map<std::string, std::complex<double>> parameters;

    [[nodiscard]] std::complex<double> get(const std::string &key) const
    {
        auto it = parameters.find(key);
        if(it == parameters.end())
        {
            return {};
        }
        return it->second;
    }
};

class HostCore
{
public:
    HostCore();
    ~HostCore();

    bool connect(const std::string &serial);
    void disconnect();
    [[nodiscard]] bool is_connected() const;

    bool load_calibration(const std::filesystem::path &path);
    [[nodiscard]] std::filesystem::path calibration_file() const;

    std::vector<VNAMeasurement> run_sweep(const SweepConfiguration &config);

    [[nodiscard]] std::string last_error_message() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};

} // namespace librevna::headless
