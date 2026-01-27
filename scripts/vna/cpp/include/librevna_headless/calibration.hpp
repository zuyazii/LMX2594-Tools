#pragma once

#include "host_core.hpp"

#include <complex>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace librevna::headless
{

/**
 * @brief Simple loader and applier for LibreVNA two-port SOLT calibration files.
 *
 * Only format 3 (LibreVNA JSON) SOLT_12 calibrations are supported today.
 * The implementation mirrors the algorithms used in the original GUI and the
 * Python helper contained in this repository.
 */
class CalibrationApplier
{
public:
    CalibrationApplier();
    ~CalibrationApplier();

    /**
     * Load a calibration from disk. On success the parsed coefficients are kept
     * in memory and can be applied to future measurements.
     *
     * @param path The path to the `.cal` file.
     * @param error_out Optional pointer that receives a human readable failure reason.
     *
     * @return true when the calibration was loaded successfully, false otherwise.
     */
    bool load(const std::filesystem::path &path, std::string *error_out = nullptr);

    /**
     * Clear the current calibration, reverting to uncorrected measurements.
     */
    void reset();

    /**
     * Apply the currently loaded calibration to a measurement in-place.
     * If no calibration is available the measurement is left untouched.
     */
    void apply(VNAMeasurement &measurement) const;

    [[nodiscard]] bool has_calibration() const;
    [[nodiscard]] std::filesystem::path source_path() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace librevna::headless

