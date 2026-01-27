#pragma once

#include "device_ids.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace librevna::headless
{

struct DiscoveredDevice
{
    std::uint16_t vendor_id = 0;
    std::uint16_t product_id = 0;
    std::string label;
    std::string serial;
};

std::vector<DiscoveredDevice> discover_devices(std::string &error_message);

bool real_driver_available();

} // namespace librevna::headless

