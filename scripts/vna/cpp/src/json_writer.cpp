#include "json_writer.hpp"

#include <fstream>
#include <utility>

namespace librevna::headless
{
JsonOutput build_json_payload(const CLIOptions &options,
                              const SweepSummary &summary,
                              const std::vector<VNAMeasurement> &measurements)
{
    nlohmann::json payload;
    payload["device"] = {
        {"serial", options.serial.value_or("unknown")},
        {"transport", "USB"}
    };
    payload["config"] = {
        {"f_start", options.f_start_hz},
        {"f_stop", options.f_stop_hz},
        {"points", options.points},
        {"ifbw", options.ifbw_hz},
        {"power_dBm", options.power_dbm}
    };
    payload["threshold_db"] = options.threshold_db;

    nlohmann::json result_json = nlohmann::json::object();
    for(const auto &[key, value] : summary.parameter_results)
    {
        nlohmann::json entry = {
            {"pass", value.pass},
            {"worst_db", value.worst_db}
        };
        if(!value.pass)
        {
            entry["fail_at_hz"] = value.fail_frequency;
        }
        result_json[key] = entry;
    }
    payload["results"] = result_json;
    payload["overall_pass"] = summary.overall_pass;

    nlohmann::json trace = nlohmann::json::array();
    for(const auto &measurement : measurements)
    {
        nlohmann::json point;
        point["frequency"] = measurement.frequency;
        for(const auto &parameter : summary.parameter_results)
        {
            const auto value = measurement.get(parameter.first);
            point[parameter.first] = {
                {"real", value.real()},
                {"imag", value.imag()}
            };
        }
        trace.push_back(point);
    }
    payload["trace"] = trace;

    JsonOutput output;
    output.payload = std::move(payload);
    output.exit_code = summary.overall_pass ? 0 : 1;
    return output;
}

bool write_json_to_file(const nlohmann::json &payload, const std::filesystem::path &path, std::string &error)
{
    std::ofstream ofs(path);
    if(!ofs)
    {
        error = "Failed to open JSON output path";
        return false;
    }
    ofs << payload.dump(2);
    if(!ofs)
    {
        error = "Failed to write JSON payload";
        return false;
    }
    return true;
}

} // namespace librevna::headless
