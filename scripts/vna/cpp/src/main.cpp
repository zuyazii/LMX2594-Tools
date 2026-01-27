#include "cli_options.hpp"
#include "cli_runner.hpp"

#include <iostream>

int main(int argc, char **argv)
{
    librevna::headless::CLIOptions options;
    std::string error_message;
    if(!librevna::headless::parse_cli_options(argc, argv, options, error_message))
    {
        if(!error_message.empty())
        {
            std::cerr << "Error: " << error_message << "\n\n";
        }
        std::cerr << librevna::headless::usage() << std::endl;
        return 64;
    }

    try
    {
        return librevna::headless::run_cli(options);
    }
    catch(const std::exception &ex)
    {
        std::cerr << "Unhandled exception: " << ex.what() << std::endl;
    }
    catch(...)
    {
        std::cerr << "Unhandled unknown exception" << std::endl;
    }
    return 70;
}
