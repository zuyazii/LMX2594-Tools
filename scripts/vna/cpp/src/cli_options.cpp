#include "cli_options.hpp"

#include <algorithm>
#include <charconv>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <sstream>

namespace librevna::headless
{
namespace
{
bool parse_double(std::string_view value, double &out)
{
    if(value.empty())
    {
        return false;
    }
    std::string s(value);
    char *end = nullptr;
    out = std::strtod(s.c_str(), &end);
    return end != s.c_str() && *end == '\0';
}

bool parse_uint(std::string_view value, std::uint32_t &out)
{
    if(value.empty())
    {
        return false;
    }
    std::string s(value);
    char *end = nullptr;
    unsigned long val = std::strtoul(s.c_str(), &end, 10);
    if(end == s.c_str() || *end != '\0')
    {
        return false;
    }
    if(val > std::numeric_limits<std::uint32_t>::max())
    {
        return false;
    }
    out = static_cast<std::uint32_t>(val);
    return true;
}

std::vector<std::string_view> split_list(std::string_view text)
{
    std::vector<std::string_view> parts;
    size_t start = 0;
    while(start < text.size())
    {
        size_t comma = text.find(',', start);
        if(comma == std::string_view::npos)
        {
            parts.emplace_back(text.substr(start));
            break;
        }
        parts.emplace_back(text.substr(start, comma - start));
        start = comma + 1;
    }
    return parts;
}

} // namespace

bool parse_cli_options(int argc, char **argv, CLIOptions &options, std::string &error_message)
{
    options = {};
    for(int i = 1; i < argc; ++i)
    {
        std::string_view arg(argv[i]);
        auto require_value = [&](std::string_view name) -> std::string_view {
            if(i + 1 >= argc)
            {
                std::ostringstream oss;
                oss << "Missing value for option '" << name << "'";
                error_message = oss.str();
                return {};
            }
            return std::string_view(argv[++i]);
        };

        if(arg == "--serial")
        {
            auto value = require_value(arg);
            if(value.empty())
            {
                return false;
            }
            options.serial = std::string(value);
        }
        else if(arg == "--cal")
        {
            auto value = require_value(arg);
            if(value.empty())
            {
                return false;
            }
            options.cal_path = std::string(value);
        }
        else if(arg == "--fstart")
        {
            auto value = require_value(arg);
            if(value.empty() || !parse_double(value, options.f_start_hz))
            {
                error_message = "Invalid value for --fstart";
                return false;
            }
        }
        else if(arg == "--fstop")
        {
            auto value = require_value(arg);
            if(value.empty() || !parse_double(value, options.f_stop_hz))
            {
                error_message = "Invalid value for --fstop";
                return false;
            }
        }
        else if(arg == "--points")
        {
            auto value = require_value(arg);
            if(value.empty() || !parse_uint(value, options.points))
            {
                error_message = "Invalid value for --points";
                return false;
            }
        }
        else if(arg == "--ifbw")
        {
            auto value = require_value(arg);
            if(value.empty() || !parse_double(value, options.ifbw_hz))
            {
                error_message = "Invalid value for --ifbw";
                return false;
            }
        }
        else if(arg == "--power")
        {
            auto value = require_value(arg);
            if(value.empty() || !parse_double(value, options.power_dbm))
            {
                error_message = "Invalid value for --power";
                return false;
            }
        }
        else if(arg == "--threshold")
        {
            auto value = require_value(arg);
            if(value.empty() || !parse_double(value, options.threshold_db))
            {
                error_message = "Invalid value for --threshold";
                return false;
            }
        }
        else if(arg == "--timeout-ms")
        {
            auto value = require_value(arg);
            if(value.empty() || !parse_double(value, options.timeout_ms))
            {
                error_message = "Invalid value for --timeout-ms";
                return false;
            }
        }
        else if(arg == "--excite")
        {
            auto value = require_value(arg);
            if(value.empty())
            {
                error_message = "Invalid value for --excite";
                return false;
            }
            options.excited_ports.clear();
            for(auto part : split_list(value))
            {
                std::uint32_t port = 0;
                if(!parse_uint(part, port))
                {
                    error_message = "Invalid port in --excite";
                    return false;
                }
                options.excited_ports.push_back(static_cast<int>(port));
            }
        }
        else if(arg == "--logs")
        {
            auto value = require_value(arg);
            if(value.empty())
            {
                error_message = "Invalid value for --logs";
                return false;
            }
            options.logs_path = std::string(value);
        }
        else if(arg == "--json-out")
        {
            auto value = require_value(arg);
            if(value.empty())
            {
                error_message = "Invalid value for --json-out";
                return false;
            }
            options.json_out_path = std::string(value);
        }
        else if(arg == "--progress-ndjson")
        {
            options.progress_ndjson = true;
        }
        else if(arg == "--help" || arg == "-h")
        {
            error_message.clear();
            return false;
        }
        else
        {
            std::ostringstream oss;
            oss << "Unknown argument: " << arg;
            error_message = oss.str();
            return false;
        }
    }

    if(options.cal_path.empty())
    {
        error_message = "--cal is required";
        return false;
    }
    if(options.f_start_hz <= 0.0 || options.f_stop_hz <= options.f_start_hz)
    {
        error_message = "Invalid frequency range";
        return false;
    }
    if(options.points == 0)
    {
        error_message = "--points must be greater than zero";
        return false;
    }
    if(options.ifbw_hz <= 0.0)
    {
        error_message = "--ifbw must be greater than zero";
        return false;
    }

    return true;
}

} // namespace librevna::headless
