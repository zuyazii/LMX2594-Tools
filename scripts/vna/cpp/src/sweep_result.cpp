#include "sweep_result.hpp"

#include <cmath>

namespace librevna::headless
{
namespace
{
double magnitude_to_db(std::complex<double> value)
{
    double mag = std::abs(value);
    if(mag <= 0.0)
    {
        return -300.0;
    }
    return 20.0 * std::log10(mag);
}

} // namespace

SweepSummary evaluate_sweep(const CLIOptions &options,
                            const std::vector<VNAMeasurement> &measurements)
{
    SweepSummary summary;
    summary.parameter_results = {
        {"S11", {}},
        {"S21", {}},
        {"S12", {}},
        {"S22", {}}
    };

    for(const auto &entry : summary.parameter_results)
    {
        summary.parameter_results[entry.first].worst_db = -300.0;
        summary.parameter_results[entry.first].fail_frequency = 0.0;
        summary.parameter_results[entry.first].pass = true;
    }

    for(const auto &measurement : measurements)
    {
        for(auto &[parameter, result] : summary.parameter_results)
        {
            const auto value = measurement.get(parameter);
            const double db = magnitude_to_db(value);
            if(db > result.worst_db)
            {
                result.worst_db = db;
                result.fail_frequency = measurement.frequency;
            }
            if(db > options.threshold_db)
            {
                result.pass = false;
                summary.overall_pass = false;
            }
        }
    }

    return summary;
}

} // namespace librevna::headless
