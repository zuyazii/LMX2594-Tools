#include "librevna_headless/host_core.hpp"
#include "librevna_headless/calibration.hpp"

#include <cmath>
#include <exception>
#include <filesystem>
#include <random>
#include <utility>

namespace librevna::headless
{

struct HostCore::Impl
{
    bool connected = false;
    std::filesystem::path calibration_path;
    CalibrationApplier calibrator;
    std::string last_error;

    [[nodiscard]] std::vector<VNAMeasurement> run_stub_sweep(const SweepConfiguration &config)
    {
        std::vector<VNAMeasurement> result;
        if(config.points == 0)
        {
            return result;
        }

        const double step = (config.stop_frequency_hz - config.start_frequency_hz) /
                            static_cast<double>(config.points > 1 ? config.points - 1 : 1);
        std::mt19937_64 rng(static_cast<std::mt19937_64::result_type>(config.start_frequency_hz) ^
                            static_cast<std::mt19937_64::result_type>(config.stop_frequency_hz));
        std::normal_distribution<double> magnitude_noise(0.0, 0.02);
        std::normal_distribution<double> phase_noise(0.0, 0.01);

        for(std::uint32_t i = 0; i < config.points; ++i)
        {
            const double frequency = config.start_frequency_hz + step * static_cast<double>(i);
            auto make_point = [&](double base_db) {
                const double mag = std::pow(10.0, base_db / 20.0) * (1.0 + magnitude_noise(rng));
                const double phase = phase_noise(rng);
                return std::polar(mag, phase);
            };

            std::map<std::string, std::complex<double>> parameters = {
                {"S11", make_point(-15.0)},
                {"S21", make_point(-3.0)},
                {"S12", make_point(-30.0)},
                {"S22", make_point(-12.0)}
            };

            VNAMeasurement measurement(frequency, std::move(parameters));
            if(calibrator.has_calibration())
            {
                try
                {
                    calibrator.apply(measurement);
                }
                catch(const std::exception &ex)
                {
                    last_error = std::string("Calibration correction failed: ") + ex.what();
                    return {};
                }
            }
            result.push_back(std::move(measurement));
        }

        return result;
    }
};

HostCore::HostCore()
    : impl(std::make_unique<Impl>())
{
}

HostCore::~HostCore() = default;

bool HostCore::connect(const std::string &serial)
{
    impl->last_error.clear();
    (void)serial;
    impl->connected = true;
    return true;
}

void HostCore::disconnect()
{
    impl->connected = false;
}

bool HostCore::is_connected() const
{
    return impl->connected;
}

bool HostCore::load_calibration(const std::filesystem::path &path)
{
    impl->last_error.clear();
    if(!std::filesystem::exists(path))
    {
        impl->last_error = "Calibration file not found";
        return false;
    }

    std::string error_message;
    if(!impl->calibrator.load(path, &error_message))
    {
        impl->last_error = error_message.empty() ? "Failed to load calibration coefficients" : error_message;
        return false;
    }

    impl->calibration_path = path;
    return true;
}

std::filesystem::path HostCore::calibration_file() const
{
    return impl->calibration_path;
}

std::vector<VNAMeasurement> HostCore::run_sweep(const SweepConfiguration &config)
{
    return impl->run_stub_sweep(config);
}

std::string HostCore::last_error_message() const
{
    return impl->last_error;
}

} // namespace librevna::headless


