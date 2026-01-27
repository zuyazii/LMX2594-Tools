#include "librevna_headless/calibration.hpp"

#include <array>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <map>
#include <vector>
#include <utility>

#include <nlohmann/json.hpp>

namespace librevna::headless
{
namespace
{
using Complex = std::complex<double>;

constexpr double kFrequencyRelTol = 1e-9;
constexpr double kFrequencyAbsTol = 1e-3;
constexpr double kZeroThreshold = 1e-18;

struct ThroughPoint
{
    Complex s11{};
    Complex s21{};
    Complex s12{};
    Complex s22{};
};

struct CalibrationPoint
{
    double frequency = 0.0;
    std::vector<Complex> directivity;
    std::vector<Complex> reflection_tracking;
    std::vector<Complex> source_match;
    std::vector<std::vector<Complex>> receiver_match;
    std::vector<std::vector<Complex>> transmission_tracking;
    std::vector<std::vector<Complex>> isolation;

    [[nodiscard]] CalibrationPoint interpolate(const CalibrationPoint &other, double alpha) const
    {
        constexpr double kPi = 3.14159265358979323846264338327950288;

        CalibrationPoint result;
        result.frequency = frequency * (1.0 - alpha) + other.frequency * alpha;
        const auto interpolate_complex = [](const Complex &a, const Complex &b, double weight) -> Complex {
            if(weight <= 0.0)
            {
                return a;
            }
            if(weight >= 1.0)
            {
                return b;
            }
            const double mag_a = std::abs(a);
            const double mag_b = std::abs(b);
            if(mag_a == 0.0 && mag_b == 0.0)
            {
                return {0.0, 0.0};
            }
            double phase_a = std::atan2(a.imag(), a.real());
            double phase_b = std::atan2(b.imag(), b.real());
            double delta = phase_b - phase_a;
            while(delta > kPi)
            {
                delta -= 2.0 * kPi;
            }
            while(delta < -kPi)
            {
                delta += 2.0 * kPi;
            }
            const double magnitude = mag_a * (1.0 - weight) + mag_b * weight;
            const double phase = phase_a + weight * delta;
            return {magnitude * std::cos(phase), magnitude * std::sin(phase)};
        };

        result.directivity.reserve(directivity.size());
        result.reflection_tracking.reserve(reflection_tracking.size());
        result.source_match.reserve(source_match.size());

        for(std::size_t idx = 0; idx < directivity.size(); ++idx)
        {
            result.directivity.push_back(interpolate_complex(directivity[idx], other.directivity[idx], alpha));
            result.reflection_tracking.push_back(
                interpolate_complex(reflection_tracking[idx], other.reflection_tracking[idx], alpha));
            result.source_match.push_back(interpolate_complex(source_match[idx], other.source_match[idx], alpha));
        }

        const auto interpolate_matrix = [&](const std::vector<std::vector<Complex>> &lhs,
                                            const std::vector<std::vector<Complex>> &rhs) {
            std::vector<std::vector<Complex>> matrix(lhs.size(), std::vector<Complex>());
            for(std::size_t row = 0; row < lhs.size(); ++row)
            {
                matrix[row].reserve(lhs[row].size());
                for(std::size_t col = 0; col < lhs[row].size(); ++col)
                {
                    matrix[row].push_back(interpolate_complex(lhs[row][col], rhs[row][col], alpha));
                }
            }
            return matrix;
        };

        result.receiver_match = interpolate_matrix(receiver_match, other.receiver_match);
        result.transmission_tracking = interpolate_matrix(transmission_tracking, other.transmission_tracking);
        result.isolation = interpolate_matrix(isolation, other.isolation);
        return result;
    }
};

struct CalibrationCaptureResult
{
    std::vector<double> frequencies;
    std::map<int, std::vector<Complex>> open_measurements;
    std::map<int, std::vector<Complex>> short_measurements;
    std::map<int, std::vector<Complex>> load_measurements;
    std::map<std::pair<int, int>, std::vector<ThroughPoint>> through_measurements;
    std::map<std::pair<int, int>, std::vector<Complex>> isolation_measurements;

    void ensure_frequency_alignment() const
    {
        const std::size_t expected_points = frequencies.size();
        for(const auto &entry : open_measurements)
        {
            if(entry.second.size() != expected_points)
            {
                throw std::runtime_error("Open measurement point mismatch");
            }
        }
        for(const auto &entry : short_measurements)
        {
            if(entry.second.size() != expected_points)
            {
                throw std::runtime_error("Short measurement point mismatch");
            }
        }
        for(const auto &entry : load_measurements)
        {
            if(entry.second.size() != expected_points)
            {
                throw std::runtime_error("Load measurement point mismatch");
            }
        }
        for(const auto &entry : through_measurements)
        {
            if(entry.second.size() != expected_points)
            {
                throw std::runtime_error("Through measurement point mismatch");
            }
        }
        for(const auto &entry : isolation_measurements)
        {
            if(entry.second.size() != expected_points)
            {
                throw std::runtime_error("Isolation measurement point mismatch");
            }
        }
    }
};

Complex to_complex(const nlohmann::json &value)
{
    return {value.value("real", 0.0), value.value("imag", 0.0)};
}

std::vector<double> to_frequency_series(const nlohmann::json &points)
{
    std::vector<double> series;
    series.reserve(points.size());
    for(const auto &entry : points)
    {
        series.push_back(entry.value("frequency", 0.0));
    }
    return series;
}

void update_frequencies(std::optional<std::vector<double>> &frequencies,
                        const std::vector<double> &candidate,
                        const std::string &label)
{
    if(candidate.empty())
    {
        throw std::runtime_error("Calibration measurement '" + label + "' has no points");
    }
    if(!frequencies.has_value())
    {
        frequencies = candidate;
        return;
    }
    if(frequencies->size() != candidate.size())
    {
        throw std::runtime_error("Calibration measurement '" + label + "' has mismatched point count");
    }
    for(std::size_t idx = 0; idx < candidate.size(); ++idx)
    {
        const double ref = (*frequencies)[idx];
        const double value = candidate[idx];
        if(!std::isfinite(ref) || !std::isfinite(value))
        {
            throw std::runtime_error("Non-finite frequency in calibration data");
        }
        const double diff = std::abs(ref - value);
        if(diff > std::max(kFrequencyAbsTol, std::abs(ref) * kFrequencyRelTol))
        {
            throw std::runtime_error("Frequency mismatch in calibration data for '" + label + "'");
        }
    }
}

std::vector<Complex> extract_reflection_series(const nlohmann::json &points,
                                               std::optional<std::vector<double>> &frequencies,
                                               const std::string &label)
{
    std::vector<double> freq_series = to_frequency_series(points);
    update_frequencies(frequencies, freq_series, label);
    std::vector<Complex> values;
    values.reserve(points.size());
    for(const auto &entry : points)
    {
        values.emplace_back(entry.value("real", 0.0), entry.value("imag", 0.0));
    }
    return values;
}

ThroughPoint extract_through_point(const nlohmann::json &entry)
{
    ThroughPoint point;
    const auto &sparam = entry.value("Sparam", nlohmann::json::object());
    point.s11 = {sparam.value("m11_real", 0.0), sparam.value("m11_imag", 0.0)};
    point.s12 = {sparam.value("m12_real", 0.0), sparam.value("m12_imag", 0.0)};
    point.s21 = {sparam.value("m21_real", 0.0), sparam.value("m21_imag", 0.0)};
    point.s22 = {sparam.value("m22_real", 0.0), sparam.value("m22_imag", 0.0)};
    return point;
}

std::tuple<CalibrationCaptureResult, std::vector<int>> capture_from_payload(const nlohmann::json &payload)
{
    std::vector<int> ports;
    if(payload.contains("ports") && payload["ports"].is_array())
    {
        for(const auto &port : payload["ports"])
        {
            ports.push_back(port.get<int>());
        }
    }
    if(ports.empty())
    {
        ports = {1, 2};
    }
    if(ports.size() != 2)
    {
        throw std::runtime_error("Only two-port calibrations are supported");
    }

    CalibrationCaptureResult capture;
    std::optional<std::vector<double>> frequencies;

    for(const auto &measurement : payload.value("measurements", nlohmann::json::array()))
    {
        const std::string type = measurement.value("type", "");
        const auto &data = measurement.value("data", nlohmann::json::object());
        if(type == "Open" || type == "Short" || type == "Load")
        {
            const int port = data.value("port", 0);
            if(port == 0)
            {
                throw std::runtime_error("Calibration measurement '" + type + "' missing port information");
            }
            auto series = extract_reflection_series(data.value("points", nlohmann::json::array()), frequencies,
                                                    type + "_port_" + std::to_string(port));
            auto &target = (type == "Open"
                                ? capture.open_measurements
                                : (type == "Short" ? capture.short_measurements : capture.load_measurements));
            target[port] = std::move(series);
            continue;
        }
        if(type == "Through")
        {
            const int port1 = data.value("port1", ports[0]);
            const int port2 = data.value("port2", ports[1]);
            std::vector<double> freq_series;
            std::vector<ThroughPoint> forward;
            std::vector<ThroughPoint> reverse;
            for(const auto &entry : data.value("points", nlohmann::json::array()))
            {
                freq_series.push_back(entry.value("frequency", 0.0));
                auto point = extract_through_point(entry);
                forward.push_back(point);
                reverse.push_back({point.s22, point.s12, point.s21, point.s11});
            }
            update_frequencies(frequencies, freq_series, "through");
            capture.through_measurements[{port1, port2}] = forward;
            capture.through_measurements[{port2, port1}] = reverse;
            continue;
        }
        if(type == "Isolation")
        {
            const auto &points = data.value("points", nlohmann::json::array());
            std::vector<double> freq_series = to_frequency_series(points);
            update_frequencies(frequencies, freq_series, "isolation");
            for(const auto &entry : points)
            {
                const auto &matrix = entry.value("S", nlohmann::json::array());
                for(std::size_t row = 0; row < matrix.size(); ++row)
                {
                    const int dst_port = ports[row];
                    const auto &row_values = matrix[row];
                    for(std::size_t col = 0; col < row_values.size(); ++col)
                    {
                        if(col == row)
                        {
                            continue;
                        }
                        const int src_port = ports[col];
                        const auto &value = row_values[col];
                        Complex iso(value.value("real", 0.0), value.value("imag", 0.0));
                        capture.isolation_measurements[{src_port, dst_port}].push_back(iso);
                    }
                }
            }
            continue;
        }
    }

    if(!frequencies.has_value())
    {
        throw std::runtime_error("Calibration file did not contain any frequency information");
    }

    capture.frequencies = *frequencies;
    for(int port : ports)
    {
        if(!capture.open_measurements.count(port) || !capture.short_measurements.count(port)
           || !capture.load_measurements.count(port))
        {
            throw std::runtime_error("Missing SOL measurements for port " + std::to_string(port));
        }
    }
    const std::size_t expected_points = capture.frequencies.size();
    for(int src : ports)
    {
        for(int dst : ports)
        {
            if(src == dst)
            {
                continue;
            }
            if(!capture.through_measurements.count({src, dst}))
            {
                throw std::runtime_error("Missing through measurement for ports " + std::to_string(src) + "->"
                                         + std::to_string(dst));
            }
            auto &iso = capture.isolation_measurements[{src, dst}];
            if(iso.empty())
            {
                iso.assign(expected_points, Complex{0.0, 0.0});
            }
            else if(iso.size() != expected_points)
            {
                throw std::runtime_error("Isolation measurement point mismatch");
            }
        }
    }

    capture.ensure_frequency_alignment();
    return {capture, ports};
}

std::vector<CalibrationPoint> compute_coefficients(const CalibrationCaptureResult &capture,
                                                   const std::vector<int> &ports)
{
    const auto ideal_open = [](double) -> Complex { return {1.0, 0.0}; };
    const auto ideal_short = [](double) -> Complex { return {-1.0, 0.0}; };
    const auto ideal_load = [](double) -> Complex { return {0.0, 0.0}; };
    const auto ideal_through = [](double) {
        return std::array<Complex, 4>{Complex{0.0, 0.0}, Complex{1.0, 0.0}, Complex{1.0, 0.0}, Complex{0.0, 0.0}};
    }; // S11, S21, S12, S22

    const std::size_t point_count = capture.frequencies.size();
    std::vector<CalibrationPoint> points;
    points.reserve(point_count);

    auto port_index = [&](int port) -> std::size_t {
        for(std::size_t idx = 0; idx < ports.size(); ++idx)
        {
            if(ports[idx] == port)
            {
                return idx;
            }
        }
        throw std::runtime_error("Unknown port index");
    };

    for(std::size_t idx = 0; idx < point_count; ++idx)
    {
        const double frequency = capture.frequencies[idx];

        std::vector<Complex> directivity_terms;
        std::vector<Complex> source_match_terms;
        std::vector<Complex> reflection_tracking_terms;
        directivity_terms.reserve(ports.size());
        source_match_terms.reserve(ports.size());
        reflection_tracking_terms.reserve(ports.size());

        for(int port : ports)
        {
            const Complex short_measured = capture.short_measurements.at(port)[idx];
            const Complex open_measured = capture.open_measurements.at(port)[idx];
            const Complex load_measured = capture.load_measurements.at(port)[idx];
            const Complex short_actual = ideal_short(frequency);
            const Complex open_actual = ideal_open(frequency);
            const Complex load_actual = ideal_load(frequency);

            const Complex denom =
                load_actual * open_actual * (open_measured - load_measured)
                + load_actual * short_actual * (load_measured - short_measured)
                + open_actual * short_actual * (short_measured - open_measured);

            if(std::abs(denom) < kZeroThreshold)
            {
                throw std::runtime_error("Degenerate SOL coefficients");
            }

            const Complex directivity =
                (load_actual * open_measured
                     * (short_measured * (open_actual - short_actual) + load_measured * short_actual)
                 - load_actual * open_actual * load_measured * short_measured
                 + open_actual * load_measured * short_actual * (short_measured - open_measured))
                / denom;

            const Complex source_match =
                (load_actual * (open_measured - short_measured) + open_actual * (short_measured - load_measured)
                 + short_actual * (load_measured - open_measured))
                / denom;

            const Complex delta = (load_actual * load_measured * (open_measured - short_measured)
                                   + open_actual * open_measured * (short_measured - load_measured)
                                   + short_actual * short_measured * (load_measured - open_measured))
                                  / denom;

            const Complex reflection_tracking = directivity * source_match - delta;

            directivity_terms.push_back(directivity);
            source_match_terms.push_back(source_match);
            reflection_tracking_terms.push_back(reflection_tracking);
        }

        std::vector<std::vector<Complex>> receiver_match(ports.size(), std::vector<Complex>(ports.size(), {0.0, 0.0}));
        std::vector<std::vector<Complex>> transmission_tracking(ports.size(),
                                                                std::vector<Complex>(ports.size(), {0.0, 0.0}));
        std::vector<std::vector<Complex>> isolation(ports.size(), std::vector<Complex>(ports.size(), {0.0, 0.0}));

        for(int src_port : ports)
        {
            for(int dst_port : ports)
            {
                if(src_port == dst_port)
                {
                    continue;
                }
                const auto key = std::make_pair(src_port, dst_port);
                const auto &measurement = capture.through_measurements.at(key);
                const ThroughPoint &point = measurement[idx];
                std::array<Complex, 4> ideal = ideal_through(frequency);

                Complex measured_s11 = point.s11;
                Complex measured_s21 = point.s21;

                const Complex isolation_value = capture.isolation_measurements.at(key)[idx];

                const std::size_t src_index = port_index(src_port);
                const std::size_t dst_index = port_index(dst_port);

                const Complex delta_s = ideal[0] * ideal[3] - ideal[1] * ideal[2];

                const Complex numerator = (measured_s11 - directivity_terms[src_index])
                                          * (Complex{1.0, 0.0}
                                             - source_match_terms[src_index] * ideal[0])
                                          - ideal[0] * reflection_tracking_terms[src_index];

                const Complex denominator =
                    (measured_s11 - directivity_terms[src_index])
                        * (ideal[3] - source_match_terms[src_index] * delta_s)
                    - delta_s * reflection_tracking_terms[src_index];

                if(std::abs(denominator) < kZeroThreshold)
                {
                    throw std::runtime_error("Receiver match singularity in calibration");
                }

                receiver_match[src_index][dst_index] = numerator / denominator;

                transmission_tracking[src_index][dst_index] =
                    (measured_s21 - isolation_value)
                    * (Complex{1.0, 0.0} - source_match_terms[src_index] * ideal[0]
                       - receiver_match[src_index][dst_index] * ideal[3]
                       + source_match_terms[src_index] * receiver_match[src_index][dst_index] * delta_s)
                    / ideal[1];

                isolation[src_index][dst_index] = isolation_value;
            }
        }

        points.push_back(CalibrationPoint{
            frequency,
            std::move(directivity_terms),
            std::move(reflection_tracking_terms),
            std::move(source_match_terms),
            std::move(receiver_match),
            std::move(transmission_tracking),
            std::move(isolation),
        });
    }

    return points;
}

std::vector<std::vector<Complex>> invert_2x2(const std::vector<std::vector<Complex>> &matrix)
{
    if(matrix.size() != 2 || matrix[0].size() != 2 || matrix[1].size() != 2)
    {
        throw std::runtime_error("Only 2x2 matrices are supported for inversion");
    }

    const Complex det = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
    if(std::abs(det) < kZeroThreshold)
    {
        throw std::runtime_error("Calibration matrix is singular");
    }
    std::vector<std::vector<Complex>> inverse(2, std::vector<Complex>(2));
    inverse[0][0] = matrix[1][1] / det;
    inverse[1][1] = matrix[0][0] / det;
    inverse[0][1] = -matrix[0][1] / det;
    inverse[1][0] = -matrix[1][0] / det;
    return inverse;
}

std::vector<std::vector<Complex>> multiply_2x2(const std::vector<std::vector<Complex>> &lhs,
                                               const std::vector<std::vector<Complex>> &rhs)
{
    std::vector<std::vector<Complex>> result(2, std::vector<Complex>(2));
    for(std::size_t row = 0; row < 2; ++row)
    {
        for(std::size_t col = 0; col < 2; ++col)
        {
            Complex value{0.0, 0.0};
            for(std::size_t k = 0; k < 2; ++k)
            {
                value += lhs[row][k] * rhs[k][col];
            }
            result[row][col] = value;
        }
    }
    return result;
}

class LoadedCalibration
{
public:
    LoadedCalibration(std::vector<int> ports, std::vector<CalibrationPoint> points, std::filesystem::path source_path)
        : ports_(std::move(ports)),
          points_(std::move(points)),
          source_path_(std::move(source_path))
    {
    }

    [[nodiscard]] bool valid() const
    {
        return points_.size() >= 2 && ports_.size() == 2;
    }

    void apply(VNAMeasurement &measurement) const
    {
        if(!valid())
        {
            return;
        }
        const CalibrationPoint point = locate_point(measurement.frequency);
        const std::size_t n = ports_.size();
        std::vector<std::vector<Complex>> s_matrix(n, std::vector<Complex>(n, {0.0, 0.0}));

        for(std::size_t col = 0; col < n; ++col)
        {
            const int src_port = ports_[col];
            for(std::size_t row = 0; row < n; ++row)
            {
                const int dst_port = ports_[row];
                const std::string key = "S" + std::to_string(dst_port) + std::to_string(src_port);
                Complex value = {};
                if(const auto it = measurement.parameters.find(key); it != measurement.parameters.end())
                {
                    value = it->second;
                }
                if(row != col)
                {
                    value -= point.isolation[col][row];
                }
                s_matrix[row][col] = value;
            }
        }

        std::vector<std::vector<Complex>> a(n, std::vector<Complex>(n, {0.0, 0.0}));
        std::vector<std::vector<Complex>> b(n, std::vector<Complex>(n, {0.0, 0.0}));

        for(std::size_t col = 0; col < n; ++col)
        {
            for(std::size_t row = 0; row < n; ++row)
            {
                if(row == col)
                {
                    const Complex numerator = s_matrix[row][col] - point.directivity[col];
                    const Complex denom = point.reflection_tracking[col];
                    if(std::abs(denom) < kZeroThreshold)
                    {
                        throw std::runtime_error("Reflection tracking coefficient is zero");
                    }
                    b[row][col] = numerator / denom;
                    a[row][col] = Complex{1.0, 0.0} + point.source_match[col] * b[row][col];
                }
                else
                {
                    const Complex denom = point.transmission_tracking[col][row];
                    if(std::abs(denom) < kZeroThreshold)
                    {
                        throw std::runtime_error("Transmission tracking coefficient is zero");
                    }
                    b[row][col] = s_matrix[row][col] / denom;
                    a[row][col] = point.receiver_match[col][row] * b[row][col];
                }
            }
        }

        const auto inv_a = invert_2x2(a);
        const auto corrected_matrix = multiply_2x2(b, inv_a);

        measurement.parameters.clear();
        for(std::size_t col = 0; col < n; ++col)
        {
            const int src_port = ports_[col];
            for(std::size_t row = 0; row < n; ++row)
            {
                const int dst_port = ports_[row];
                const std::string key = "S" + std::to_string(dst_port) + std::to_string(src_port);
                measurement.parameters[key] = corrected_matrix[row][col];
            }
        }
    }

    [[nodiscard]] const std::filesystem::path &source_path() const
    {
        return source_path_;
    }

private:
    CalibrationPoint locate_point(double frequency) const
    {
        if(frequency <= points_.front().frequency)
        {
            return points_.front();
        }
        if(frequency >= points_.back().frequency)
        {
            return points_.back();
        }
        std::size_t low = 0;
        std::size_t high = points_.size() - 1;
        while(high - low > 1)
        {
            const std::size_t mid = (low + high) / 2;
            if(points_[mid].frequency <= frequency)
            {
                low = mid;
            }
            else
            {
                high = mid;
            }
        }
        const CalibrationPoint &start = points_[low];
        const CalibrationPoint &end = points_[high];
        const double span = end.frequency - start.frequency;
        const double alpha = span == 0.0 ? 0.0 : (frequency - start.frequency) / span;
        return start.interpolate(end, alpha);
    }

    std::vector<int> ports_;
    std::vector<CalibrationPoint> points_;
    std::filesystem::path source_path_;
};

std::unique_ptr<LoadedCalibration> load_calibration_file(const std::filesystem::path &path)
{
    std::ifstream stream(path);
    if(!stream)
    {
        throw std::runtime_error("Failed to open calibration file: " + path.string());
    }
    nlohmann::json payload = nlohmann::json::parse(stream, nullptr, true, true);
    bool supported_format = false;
    if(payload.contains("format"))
    {
        const auto &format_field = payload["format"];
        if(format_field.is_number_integer())
        {
            supported_format = (format_field.get<int>() == 3);
        }
        else if(format_field.is_string())
        {
            const std::string value = format_field.get<std::string>();
            supported_format = (value == "3" || value == "solt12-calibration-v1");
        }
    }
    if(!supported_format)
    {
        throw std::runtime_error("Unsupported calibration file format");
    }

    const auto [capture, ports] = capture_from_payload(payload);
    auto points = compute_coefficients(capture, ports);
    return std::make_unique<LoadedCalibration>(ports, std::move(points), path);
}

} // namespace

struct CalibrationApplier::Impl
{
    std::unique_ptr<LoadedCalibration> calibration;
};

CalibrationApplier::CalibrationApplier() : impl_(std::make_unique<Impl>()) {}
CalibrationApplier::~CalibrationApplier() = default;

bool CalibrationApplier::load(const std::filesystem::path &path, std::string *error_out)
{
    try
    {
        impl_->calibration = load_calibration_file(path);
        return true;
    }
    catch(const std::exception &ex)
    {
        impl_->calibration.reset();
        if(error_out)
        {
            *error_out = ex.what();
        }
        return false;
    }
}

void CalibrationApplier::reset()
{
    impl_->calibration.reset();
}

void CalibrationApplier::apply(VNAMeasurement &measurement) const
{
    if(impl_->calibration)
    {
        impl_->calibration->apply(measurement);
    }
}

bool CalibrationApplier::has_calibration() const
{
    return static_cast<bool>(impl_->calibration);
}

std::filesystem::path CalibrationApplier::source_path() const
{
    if(impl_->calibration)
    {
        return impl_->calibration->source_path();
    }
    return {};
}

} // namespace librevna::headless






