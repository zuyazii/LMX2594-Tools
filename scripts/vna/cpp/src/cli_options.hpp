#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace librevna::headless
{
struct CLIOptions
{
    std::optional<std::string> serial;
    std::string cal_path;
    double f_start_hz = 0.0;
    double f_stop_hz = 0.0;
    std::uint32_t points = 0;
    double ifbw_hz = 0.0;
    double power_dbm = 0.0;
    double threshold_db = -10.0;
    double timeout_ms = 15000.0;
    std::vector<int> excited_ports{1, 2};
    bool progress_ndjson = false;
    std::optional<std::string> logs_path;
    std::optional<std::string> json_out_path;
};

bool parse_cli_options(int argc, char **argv, CLIOptions &options, std::string &error_message);

} // namespace librevna::headless
