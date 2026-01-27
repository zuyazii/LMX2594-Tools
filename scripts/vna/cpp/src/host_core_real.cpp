#include "librevna_headless/host_core.hpp"
#include "librevna_headless/calibration.hpp"
#include "librevna_headless/device_ids.hpp"

#include "Protocol.hpp"

#ifndef NOMINMAX
#define NOMINMAX 1
#endif
#include <libusb-1.0/libusb.h>

#include <algorithm>
#include <exception>
#include <array>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace librevna::headless
{
namespace
{
constexpr std::uint8_t kEndpointOut = 0x01;
constexpr std::uint8_t kEndpointInData = 0x81;
// Log endpoint (0x82) is currently unused but left for completeness

constexpr int kControlTimeoutMs = 1000;
constexpr int kBulkReadTimeoutMs = 100;
constexpr std::size_t kBulkReadBufferSize = 4096;

std::string libusb_error_to_string(int code)
{
    const char *name = libusb_error_name(code);
    if(name)
    {
        return std::string(name);
    }
    return "libusb_error_" + std::to_string(code);
}

bool is_valid_complex(const std::complex<double> &value)
{
    return !std::isnan(value.real()) && !std::isnan(value.imag());
}

} // namespace

struct HostCore::Impl
{
    libusb_context *context = nullptr;
    libusb_device_handle *handle = nullptr;
    bool connected = false;

    std::filesystem::path calibration_path;
    CalibrationApplier calibrator;
    std::string last_error;

    Protocol::DeviceInfo device_info{};
    bool have_device_info = false;

    ~Impl()
    {
        disconnect();
    }

    bool ensure_context()
    {
        if(context)
        {
            return true;
        }
        const int rc = libusb_init(&context);
        if(rc != LIBUSB_SUCCESS)
        {
            last_error = "Failed to initialise libusb: " + libusb_error_to_string(rc);
            context = nullptr;
            return false;
        }
        libusb_set_option(context, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_NONE);
        return true;
    }

    bool connect(const std::string &serial_filter)
    {
        last_error.clear();

        disconnect();

        if(!ensure_context())
        {
            return false;
        }

        libusb_device **list = nullptr;
        const ssize_t count = libusb_get_device_list(context, &list);
        if(count < 0)
        {
            last_error = "Failed to enumerate USB devices: " + libusb_error_to_string(static_cast<int>(count));
            return false;
        }

        libusb_device_handle *selected = nullptr;
        std::string selected_serial;

        for(ssize_t index = 0; index < count; ++index)
        {
            libusb_device *device = list[index];
            libusb_device_descriptor descriptor{};
            if(libusb_get_device_descriptor(device, &descriptor) != LIBUSB_SUCCESS)
            {
                continue;
            }

            const auto match = std::find_if(kKnownLibreVNADevices.begin(), kKnownLibreVNADevices.end(),
                                            [&](const DeviceIdentifier &info) {
                                                return descriptor.idVendor == info.vendor_id &&
                                                       descriptor.idProduct == info.product_id;
                                            });
            if(match == kKnownLibreVNADevices.end())
            {
                continue;
            }

            libusb_device_handle *candidate = nullptr;
            if(libusb_open(device, &candidate) != LIBUSB_SUCCESS)
            {
                continue;
            }

            std::string candidate_serial;
            if(descriptor.iSerialNumber != 0)
            {
                unsigned char serial_buffer[256] = {0};
                const int serial_len = libusb_get_string_descriptor_ascii(candidate,
                                                                          descriptor.iSerialNumber,
                                                                          serial_buffer,
                                                                          sizeof(serial_buffer));
                if(serial_len > 0)
                {
                    candidate_serial.assign(reinterpret_cast<char *>(serial_buffer), serial_len);
                }
            }

            const bool serial_matches = serial_filter.empty() || serial_filter == candidate_serial;
            if(serial_matches)
            {
                selected = candidate;
                selected_serial = std::move(candidate_serial);
                break;
            }

            libusb_close(candidate);
        }

        libusb_free_device_list(list, 1);

        if(!selected)
        {
            last_error = serial_filter.empty()
                             ? "No LibreVNA device found"
                             : "LibreVNA device with serial '" + serial_filter + "' not found";
            return false;
        }

#ifdef LIBUSB_API_VERSION
#if LIBUSB_API_VERSION >= 0x01000106
        libusb_set_auto_detach_kernel_driver(selected, 1);
#endif
#endif

        const int claim_rc = libusb_claim_interface(selected, 0);
        if(claim_rc != LIBUSB_SUCCESS)
        {
            last_error = "Failed to claim LibreVNA interface: " + libusb_error_to_string(claim_rc);
            libusb_close(selected);
            return false;
        }

        handle = selected;
        connected = true;
        have_device_info = false;
        calibration_path.clear();
        last_error.clear();
        return true;
    }

    void disconnect()
    {
        if(handle)
        {
            set_idle();
            libusb_release_interface(handle, 0);
            libusb_close(handle);
            handle = nullptr;
        }
        connected = false;
        have_device_info = false;
        if(context)
        {
            libusb_exit(context);
            context = nullptr;
        }
    }

    bool load_calibration(const std::filesystem::path &path)
    {
        last_error.clear();
        if(!std::filesystem::exists(path))
        {
            last_error = "Calibration file not found";
            return false;
        }
        std::string error_message;
        if(!calibrator.load(path, &error_message))
        {
            last_error = error_message.empty() ? "Failed to load calibration coefficients" : error_message;
            return false;
        }
        calibration_path = path;
        return true;
    }

    std::filesystem::path calibration_file() const
    {
        return calibration_path;
    }

    std::string last_error_message() const
    {
        return last_error;
    }

    bool send_packet(const Protocol::PacketInfo &packet)
    {
        if(!handle)
        {
            last_error = "Device not connected";
            return false;
        }

        std::array<std::uint8_t, 512> buffer{};
        const std::uint16_t encoded = Protocol::EncodePacket(packet, buffer.data(), static_cast<std::uint16_t>(buffer.size()));
        if(encoded == 0)
        {
            last_error = "Failed to encode packet";
            return false;
        }

        int transferred = 0;
        const int rc = libusb_bulk_transfer(handle,
                                            kEndpointOut,
                                            buffer.data(),
                                            encoded,
                                            &transferred,
                                            kControlTimeoutMs);
        if(rc != LIBUSB_SUCCESS || transferred != encoded)
        {
            last_error = "Failed to send packet: " + libusb_error_to_string(rc);
            return false;
        }
        return true;
    }

    bool send_command(Protocol::PacketType type)
    {
        Protocol::PacketInfo packet{};
        packet.type = type;
        return send_packet(packet);
    }

    void set_idle()
    {
        if(!connected || !handle)
        {
            return;
        }
        send_command(Protocol::PacketType::SetIdle);
    }

    bool request_device_info()
    {
        if(have_device_info)
        {
            return true;
        }

        if(!send_command(Protocol::PacketType::RequestDeviceInfo))
        {
            return false;
        }

        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
        std::vector<std::uint8_t> buffer;
        buffer.reserve(512);
        std::array<std::uint8_t, kBulkReadBufferSize> read_buffer{};

        while(std::chrono::steady_clock::now() < deadline)
        {
            int transferred = 0;
            const int rc = libusb_bulk_transfer(handle,
                                                kEndpointInData,
                                                read_buffer.data(),
                                                static_cast<int>(read_buffer.size()),
                                                &transferred,
                                                kBulkReadTimeoutMs);
            if(rc == LIBUSB_ERROR_TIMEOUT)
            {
                continue;
            }
            if(rc != LIBUSB_SUCCESS)
            {
                last_error = "USB read failed while waiting for device info: " + libusb_error_to_string(rc);
                return false;
            }
            if(transferred <= 0)
            {
                continue;
            }

            buffer.insert(buffer.end(), read_buffer.begin(), read_buffer.begin() + transferred);

            while(!buffer.empty())
            {
                Protocol::PacketInfo info{};
                const std::uint16_t consumed = Protocol::DecodeBuffer(buffer.data(), static_cast<std::uint16_t>(buffer.size()), &info);
                if(consumed == 0)
                {
                    break;
                }
                buffer.erase(buffer.begin(), buffer.begin() + consumed);

                if(info.type == Protocol::PacketType::DeviceInfo)
                {
                    device_info = info.info;
                    have_device_info = true;
                    return true;
                }

                if(info.type == Protocol::PacketType::Nack)
                {
                    last_error = "Device reported NACK to info request";
                    return false;
                }

                if(info.type == Protocol::PacketType::VNADatapoint && info.VNAdatapoint)
                {
                    delete info.VNAdatapoint;
                }
            }
        }

        last_error = "Timed out waiting for device info";
        return false;
    }

    unsigned int detected_port_count() const
    {
        if(have_device_info && device_info.num_ports != 0)
        {
            return std::max<unsigned int>(2u, device_info.num_ports);
        }
        return 2;
    }

    Protocol::SweepSettings build_sweep_settings(const SweepConfiguration &config) const
    {
        Protocol::SweepSettings settings{};
        settings.f_start = static_cast<std::uint64_t>(config.start_frequency_hz);
        settings.f_stop = static_cast<std::uint64_t>(config.stop_frequency_hz);
        settings.points = static_cast<std::uint16_t>(config.points);
        settings.if_bandwidth = static_cast<std::uint32_t>(config.if_bandwidth_hz);

        const auto centi_dbm = static_cast<std::int16_t>(std::llround(config.power_dbm * 100.0));
        settings.cdbm_excitation_start = centi_dbm;
        settings.cdbm_excitation_stop = centi_dbm;

        settings.standby = 0;
        settings.syncMaster = 0;
        settings.suppressPeaks = 0;
        settings.fixedPowerSetting = 1;
        settings.logSweep = 0;
        settings.syncMode = 0;
        settings.dwell_time = 0;

        const std::size_t stage_count = config.excited_ports.empty() ? 0 : (config.excited_ports.size() - 1);
        settings.stages = static_cast<std::uint16_t>(stage_count);

        auto stage_for_port = [&](int port) -> std::uint16_t {
            const auto it = std::find(config.excited_ports.begin(), config.excited_ports.end(), port);
            if(it == config.excited_ports.end())
            {
                return static_cast<std::uint16_t>(config.excited_ports.size());
            }
            return static_cast<std::uint16_t>(std::distance(config.excited_ports.begin(), it));
        };

        settings.port1Stage = stage_for_port(1);
        settings.port2Stage = stage_for_port(2);
        settings.port3Stage = stage_for_port(3);
        settings.port4Stage = stage_for_port(4);

        return settings;
    }

    VNAMeasurement convert_datapoint(Protocol::VNADatapoint<32> &datapoint,
                                     const SweepConfiguration &config,
                                     unsigned int port_count) const
    {
        std::map<std::string, std::complex<double>> parameters;
        std::map<int, std::uint8_t> stage_mapping;
        for(std::size_t idx = 0; idx < config.excited_ports.size(); ++idx)
        {
            const int port = config.excited_ports[idx];
            stage_mapping[port] = static_cast<std::uint8_t>(idx);
        }

        for(const auto &[port, stage] : stage_mapping)
        {
            const auto reference = datapoint.getValue(stage, static_cast<std::uint8_t>(port - 1), true);
            if(!is_valid_complex(reference) || std::abs(reference) < 1e-12)
            {
                continue;
            }

            for(unsigned int to_port = 1; to_port <= port_count; ++to_port)
            {
                const auto input = datapoint.getValue(stage, static_cast<std::uint8_t>(to_port - 1), false);
                if(!is_valid_complex(input))
                {
                    continue;
                }
                const std::complex<double> ratio = input / reference;
                parameters["S" + std::to_string(to_port) + std::to_string(port)] = ratio;
            }
        }

        const double frequency = static_cast<double>(datapoint.frequency);
        return VNAMeasurement(frequency, std::move(parameters));
    }

    bool handle_packet(Protocol::PacketInfo &packet,
                       std::vector<VNAMeasurement> &measurements,
                       const SweepConfiguration &config,
                       unsigned int port_count,
                       bool &ack_received)
    {
        switch(packet.type)
        {
        case Protocol::PacketType::Ack:
            ack_received = true;
            break;
        case Protocol::PacketType::Nack:
            last_error = "Device returned NACK";
            return false;
        case Protocol::PacketType::DeviceInfo:
            device_info = packet.info;
            have_device_info = true;
            break;
        case Protocol::PacketType::VNADatapoint:
            if(packet.VNAdatapoint)
            {
                auto measurement = convert_datapoint(*packet.VNAdatapoint, config, port_count);
                if(calibrator.has_calibration())
                {
                    try
                    {
                        calibrator.apply(measurement);
                    }
                    catch(const std::exception &ex)
                    {
                        last_error = std::string("Calibration correction failed: ") + ex.what();
                        delete packet.VNAdatapoint;
                        packet.VNAdatapoint = nullptr;
                        return false;
                    }
                }
                measurements.push_back(std::move(measurement));
                delete packet.VNAdatapoint;
                packet.VNAdatapoint = nullptr;
            }
            break;
        default:
            break;
        }
        return true;
    }

    std::vector<VNAMeasurement> run_sweep(const SweepConfiguration &config)
    {
        std::vector<VNAMeasurement> results;

        bool sweep_active = false;
        struct SweepGuard
        {
            Impl *impl;
            bool *active;
            ~SweepGuard()
            {
                if(impl && active && *active)
                {
                    impl->set_idle();
                }
            }
        } sweep_guard{this, &sweep_active};

        if(!connected || !handle)
        {
            last_error = "Device not connected";
            return results;
        }
        if(config.points == 0)
        {
            last_error = "Sweep must contain at least one point";
            return results;
        }
        if(config.excited_ports.empty())
        {
            last_error = "At least one excited port is required";
            return results;
        }

        if(!request_device_info())
        {
            return results;
        }

        Protocol::PacketInfo sweep_packet{};
        sweep_packet.type = Protocol::PacketType::SweepSettings;
        sweep_packet.settings = build_sweep_settings(config);

        if(!send_packet(sweep_packet))
        {
            return results;
        }

        sweep_active = true;

        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(
            static_cast<long long>(config.timeout_ms > 0.0 ? config.timeout_ms : 15000.0));

        std::vector<std::uint8_t> buffer;
        buffer.reserve(kBulkReadBufferSize * 2);
        std::array<std::uint8_t, kBulkReadBufferSize> read_buffer{};

        bool ack_received = false;
        const unsigned int port_count = detected_port_count();

        while(std::chrono::steady_clock::now() < deadline)
        {
            if(results.size() >= config.points)
            {
                break;
            }

            int transferred = 0;
            const int rc = libusb_bulk_transfer(handle,
                                                kEndpointInData,
                                                read_buffer.data(),
                                                static_cast<int>(read_buffer.size()),
                                                &transferred,
                                                kBulkReadTimeoutMs);
            if(rc == LIBUSB_ERROR_TIMEOUT)
            {
                continue;
            }
            if(rc != LIBUSB_SUCCESS)
            {
                last_error = "USB read failed: " + libusb_error_to_string(rc);
                return {};
            }
            if(transferred <= 0)
            {
                continue;
            }

            buffer.insert(buffer.end(), read_buffer.begin(), read_buffer.begin() + transferred);

            while(!buffer.empty())
            {
                Protocol::PacketInfo packet{};
                const auto available = static_cast<std::uint16_t>(std::min<std::size_t>(buffer.size(), std::numeric_limits<std::uint16_t>::max()));
                const std::uint16_t consumed = Protocol::DecodeBuffer(buffer.data(), available, &packet);
                if(consumed == 0)
                {
                    break;
                }
                buffer.erase(buffer.begin(), buffer.begin() + consumed);

                if(!handle_packet(packet, results, config, port_count, ack_received))
                {
                    return {};
                }
            }
        }

        if(results.size() < config.points)
        {
            last_error = "Sweep timed out before receiving all datapoints";
            return {};
        }

        if(!ack_received)
        {
            last_error = "Device did not acknowledge sweep configuration";
            return {};
        }

        set_idle();
        sweep_active = false;

        return results;
    }
};

HostCore::HostCore()
    : impl(std::make_unique<Impl>())
{
}

HostCore::~HostCore() = default;

bool HostCore::connect(const std::string &serial)
{
    return impl->connect(serial);
}

void HostCore::disconnect()
{
    impl->disconnect();
}

bool HostCore::is_connected() const
{
    return impl->connected;
}

bool HostCore::load_calibration(const std::filesystem::path &path)
{
    return impl->load_calibration(path);
}

std::filesystem::path HostCore::calibration_file() const
{
    return impl->calibration_file();
}

std::vector<VNAMeasurement> HostCore::run_sweep(const SweepConfiguration &config)
{
    return impl->run_sweep(config);
}

std::string HostCore::last_error_message() const
{
    return impl->last_error_message();
}

} // namespace librevna::headless


