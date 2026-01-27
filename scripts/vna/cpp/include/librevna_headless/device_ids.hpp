#pragma once

#include <array>
#include <cstdint>
#include <string_view>

namespace librevna::headless
{

struct DeviceIdentifier
{
    std::uint16_t vendor_id;
    std::uint16_t product_id;
    std::string_view label;
};

inline constexpr std::array<DeviceIdentifier, 3> kKnownLibreVNADevices = {
    DeviceIdentifier{0x0483, 0x564E, "LibreVNA"},
    DeviceIdentifier{0x0483, 0x4121, "LibreVNA"},
    DeviceIdentifier{0x1209, 0x4121, "LibreVNA"},
};

} // namespace librevna::headless

