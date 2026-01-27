#include "librevna_headless/device_discovery.hpp"

#include <algorithm>
#include <sstream>

#if LIBREVNA_HEADLESS_HAS_REAL_DRIVER
#include <libusb-1.0/libusb.h>
#endif

namespace librevna::headless
{
namespace
{

std::string format_libusb_error(int code)
{
#if LIBREVNA_HEADLESS_HAS_REAL_DRIVER
    const char *name = libusb_error_name(code);
    if(name)
    {
        return std::string(name);
    }
    return "libusb_error_" + std::to_string(code);
#else
    (void)code;
    return {};
#endif
}

} // namespace

std::vector<DiscoveredDevice> discover_devices(std::string &error_message)
{
    error_message.clear();
    std::vector<DiscoveredDevice> devices;

#if LIBREVNA_HEADLESS_HAS_REAL_DRIVER
    libusb_context *context = nullptr;
    int rc = libusb_init(&context);
    if(rc != LIBUSB_SUCCESS)
    {
        error_message = "Failed to initialise libusb: " + format_libusb_error(rc);
        return devices;
    }

#if defined(LIBUSB_OPTION_LOG_LEVEL)
    libusb_set_option(context, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_NONE);
#else
    libusb_set_debug(context, LIBUSB_LOG_LEVEL_NONE);
#endif

    libusb_device **list = nullptr;
    const ssize_t count = libusb_get_device_list(context, &list);
    if(count < 0)
    {
        error_message = "Failed to enumerate USB devices: " + format_libusb_error(static_cast<int>(count));
        libusb_exit(context);
        return devices;
    }

    auto cleanup = [&]() {
        libusb_free_device_list(list, 1);
        libusb_exit(context);
    };

    for(ssize_t index = 0; index < count; ++index)
    {
        libusb_device *device = list[index];
        libusb_device_descriptor descriptor{};
        if(libusb_get_device_descriptor(device, &descriptor) != LIBUSB_SUCCESS)
        {
            continue;
        }

        const auto match = std::find_if(kKnownLibreVNADevices.begin(),
                                        kKnownLibreVNADevices.end(),
                                        [&](const DeviceIdentifier &info) {
                                            return descriptor.idVendor == info.vendor_id &&
                                                   descriptor.idProduct == info.product_id;
                                        });
        if(match == kKnownLibreVNADevices.end())
        {
            continue;
        }

        libusb_device_handle *handle = nullptr;
        if(libusb_open(device, &handle) != LIBUSB_SUCCESS)
        {
            continue;
        }

        std::string serial;
        if(descriptor.iSerialNumber != 0)
        {
            unsigned char buffer[256] = {0};
            const int serial_len = libusb_get_string_descriptor_ascii(handle,
                                                                      descriptor.iSerialNumber,
                                                                      buffer,
                                                                      sizeof(buffer));
            if(serial_len > 0)
            {
                serial.assign(reinterpret_cast<char *>(buffer), static_cast<std::size_t>(serial_len));
            }
        }

        devices.push_back(DiscoveredDevice{
            descriptor.idVendor,
            descriptor.idProduct,
            std::string(match->label),
            serial,
        });

        libusb_close(handle);
    }

    cleanup();

    std::sort(devices.begin(), devices.end(), [](const DiscoveredDevice &lhs, const DiscoveredDevice &rhs) {
        if(lhs.label != rhs.label)
        {
            return lhs.label < rhs.label;
        }
        if(lhs.serial.empty() != rhs.serial.empty())
        {
            return rhs.serial.empty();
        }
        return lhs.serial < rhs.serial;
    });
// #else
//     devices.push_back(DiscoveredDevice{
//         0xFFFF,
//         0xFFFF,
//         "LibreVNA (simulation)",
//         "STUB-0001",
//     });
#endif

    return devices;
}

bool real_driver_available()
{
#if LIBREVNA_HEADLESS_HAS_REAL_DRIVER
    return true;
#else
    return false;
#endif
}

} // namespace librevna::headless

