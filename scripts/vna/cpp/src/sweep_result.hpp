#pragma once

#include "cli_options.hpp"
#include "librevna_headless/host_core.hpp"

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace librevna::headless
{
struct ParameterSummary
{
    bool pass = true;
    double worst_db = 0.0;
    double fail_frequency = 0.0;
};

struct SweepSummary
{
    std::map<std::string, ParameterSummary> parameter_results;
    bool overall_pass = true;
};

SweepSummary evaluate_sweep(const CLIOptions &options,
                            const std::vector<VNAMeasurement> &measurements);

} // namespace librevna::headless
