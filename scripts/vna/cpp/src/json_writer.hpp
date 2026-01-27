#pragma once

#include "cli_options.hpp"
#include "sweep_result.hpp"

#include <nlohmann/json.hpp>

#include <filesystem>
#include <optional>
#include <string>

namespace librevna::headless
{
struct JsonOutput
{
    nlohmann::json payload;
    int exit_code = 0;
};

JsonOutput build_json_payload(const CLIOptions &options,
                              const SweepSummary &summary,
                              const std::vector<VNAMeasurement> &measurements);

bool write_json_to_file(const nlohmann::json &payload, const std::filesystem::path &path, std::string &error);

} // namespace librevna::headless
