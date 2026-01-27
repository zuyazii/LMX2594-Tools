#include "cli_runner.hpp"

#include "json_writer.hpp"
#include "sweep_result.hpp"

#include <chrono>
#include <filesystem>
#include <iostream>
#include <thread>

namespace librevna::headless
{
namespace
{
void emit_progress(std::ostream &stream,
                   std::size_t current,
                   std::size_t total,
                   double frequency)
{
    nlohmann::json progress = {
        {"type", "progress"},
        {"current_point", current},
        {"total_points", total},
        {"frequency", frequency}
    };
    stream << progress.dump() << '\n';
}

} // namespace

int run_cli(const CLIOptions &options)
{
    HostCore host;
    if(!host.connect(options.serial.value_or("")))
    {
        nlohmann::json error = {
            {"error", "connection_failed"},
            {"message", host.last_error_message()}
        };
        std::cout << error.dump(2) << std::endl;
        return 2;
    }

    if(!host.load_calibration(options.cal_path))
    {
        nlohmann::json error = {
            {"error", "calibration_load_failed"},
            {"message", host.last_error_message()},
            {"calibration_path", options.cal_path}
        };
        std::cout << error.dump(2) << std::endl;
        return 3;
    }

    SweepConfiguration configuration;
    configuration.start_frequency_hz = options.f_start_hz;
    configuration.stop_frequency_hz = options.f_stop_hz;
    configuration.points = options.points;
    configuration.if_bandwidth_hz = options.ifbw_hz;
    configuration.power_dbm = options.power_dbm;
    configuration.timeout_ms = options.timeout_ms;
    configuration.excited_ports = options.excited_ports;

    auto measurements = host.run_sweep(configuration);

    if(options.progress_ndjson)
    {
        for(std::size_t index = 0; index < measurements.size(); ++index)
        {
            emit_progress(std::cerr, index + 1, measurements.size(), measurements[index].frequency);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    auto summary = evaluate_sweep(options, measurements);
    auto json_output = build_json_payload(options, summary, measurements);

    std::cout << json_output.payload.dump(2) << std::endl;

    if(options.json_out_path)
    {
        std::string error_message;
        if(!write_json_to_file(json_output.payload, *options.json_out_path, error_message))
        {
            nlohmann::json error_json = {
                {"error", "json_write_failed"},
                {"message", error_message}
            };
            std::cerr << error_json.dump() << std::endl;
            return 10;
        }
    }

    return json_output.exit_code;
}

std::string usage()
{
    return R"(librevna-cli - Headless LibreVNA sweep runner

Required arguments:
  --cal <path>         Path to calibration file (.cal)
  --fstart <Hz>        Sweep start frequency (Hz)
  --fstop <Hz>         Sweep stop frequency (Hz)
  --points <N>         Number of sweep points
  --ifbw <Hz>          IF bandwidth (Hz)
  --power <dBm>        Source power level (dBm)
  --threshold <dB>     Pass/fail threshold (dB)

Optional arguments:
  --serial <serial>    Connect to the specific LibreVNA serial number
  --timeout-ms <ms>    Timeout in milliseconds (default 15000)
  --excite <list>      Comma separated list of excited ports (default 1,2)
  --logs <path>        Write driver logs to the provided path
  --json-out <path>    Persist JSON results to disk
  --progress-ndjson    Emit NDJSON progress updates to stderr
  --help               Show this message
)";
}

} // namespace librevna::headless
