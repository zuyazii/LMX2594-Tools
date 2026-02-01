#include "librevna_headless.h"

#include "Calibration/calibration.h"
#include "Device/devicedriver.h"
#include "CustomWidgets/informationbox.h"

#include <QEventLoop>
#include <QElapsedTimer>
#include <QCoreApplication>
#include <QThread>
#include <QFileInfo>
#include <QMetaType>
#include <QDateTime>
#include <QTimer>
#include <QtMath>
#include <QTextStream>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace LibreVNAHeadless {

namespace {
struct Selection {
    DeviceDriver *driver = nullptr;
    QString serial;
};

bool debug_enabled() {
    const QString value = qEnvironmentVariable("LIBREVNA_IPC_DEBUG").trimmed().toLower();
    return value == "1" || value == "true" || value == "yes";
}

void log_line(const QString &message) {
    QTextStream err(stderr);
    err << QDateTime::currentDateTime().toString("HH:mm:ss.zzz")
        << " " << message << "\n";
    err.flush();
    fflush(stderr);
}

Selection select_device(const QString &requested_serial) {
    auto drivers = DeviceDriver::getDrivers();

    if (!requested_serial.isEmpty()) {
        for (auto *driver : drivers) {
            const auto serials = driver->GetAvailableDevices();
            if (serials.count(requested_serial)) {
                return {driver, requested_serial};
            }
        }
        for (auto *driver : drivers) {
            const auto serials = driver->GetAvailableDevices();
            if (!serials.empty() && driver->getDriverName().contains(requested_serial, Qt::CaseInsensitive)) {
                return {driver, *serials.begin()};
            }
        }
    }

    for (auto *driver : drivers) {
        const auto serials = driver->GetAvailableDevices();
        if (!serials.empty()) {
            return {driver, *serials.begin()};
        }
    }

    return {};
}

double magnitude_db(const std::complex<double> &value) {
    const double mag = std::abs(value);
    if (mag <= 0.0) {
        return -300.0;
    }
    return 20.0 * std::log10(mag);
}
}  // namespace

std::vector<DeviceInfo> list_devices(bool *real_driver, QString *warning) {
    InformationBox::setGUI(false);
    std::vector<DeviceInfo> devices;

#if defined(LIBREVNA_HEADLESS_HAS_REAL_DRIVER)
    const bool real = LIBREVNA_HEADLESS_HAS_REAL_DRIVER != 0;
#else
    const bool real = true;
#endif

    if (real_driver) {
        *real_driver = real;
    }
    if (!real && warning) {
        *warning = "libusb not detected; USB devices may be unavailable";
    }

    auto drivers = DeviceDriver::getDrivers();
    for (auto *driver : drivers) {
        const auto serials = driver->GetAvailableDevices();
        for (const auto &serial : serials) {
            DeviceInfo info;
            info.label = "LibreVNA";
            info.serial = serial;
            info.driver = driver->getDriverName();
            devices.push_back(info);
        }
    }

    return devices;
}

SweepResult run_sweep(const SweepRequest &request) {
    InformationBox::setGUI(false);
    qRegisterMetaType<DeviceDriver::VNAMeasurement>("DeviceDriver::VNAMeasurement");

    if (debug_enabled()) {
        log_line("run_sweep v2 - with device info wait");
        log_line(QStringLiteral("headless sweep requested f_start=%1 f_stop=%2 points=%3 ifbw=%4 power=%5 timeout_ms=%6 cal=%7 serial=%8")
                     .arg(request.f_start_hz, 0, 'f', 0)
                     .arg(request.f_stop_hz, 0, 'f', 0)
                     .arg(request.points)
                     .arg(request.ifbw_hz, 0, 'f', 0)
                     .arg(request.power_dbm, 0, 'f', 1)
                     .arg(request.timeout_ms, 0, 'f', 0)
                     .arg(request.cal_path.isEmpty() ? "none" : request.cal_path)
                     .arg(request.serial.isEmpty() ? "auto" : request.serial));
    }

    if (request.points <= 0) {
        throw std::runtime_error("points must be > 0");
    }

    if (!request.cal_path.isEmpty() && !QFileInfo::exists(request.cal_path)) {
        throw std::runtime_error("calibration file not found");
    }

    Selection selection = select_device(request.serial.trimmed());
    if (!selection.driver) {
        throw std::runtime_error("no LibreVNA device detected");
    }
    if (debug_enabled()) {
        log_line(QStringLiteral("selected driver=%1 serial=%2")
                     .arg(selection.driver->getDriverName())
                     .arg(selection.serial));
    }

    if (debug_enabled()) {
        log_line("loading calibration file...");
    }
    Calibration calibration;
    const bool has_cal = !request.cal_path.isEmpty() && calibration.fromFile(request.cal_path);
    if (debug_enabled()) {
        log_line(QStringLiteral("calibration.fromFile returned=%1").arg(has_cal ? "true" : "false"));
    }
    if (!request.cal_path.isEmpty() && !has_cal) {
        throw std::runtime_error("failed to load calibration file");
    }
    if (debug_enabled()) {
        log_line(QStringLiteral("calibration loaded=%1").arg(has_cal ? "true" : "false"));
    }

    const QString serial = selection.serial;
    if (!selection.driver->connectDevice(serial)) {
        throw std::runtime_error("failed to connect to LibreVNA device");
    }
    if (debug_enabled()) {
        log_line("device connected");
    }

    // Wait for device info to be received.
    // The USB driver sends RequestDeviceInfo during connectTo() but the response
    // arrives asynchronously via QueuedConnection signals. We must pump events
    // so that handleReceivedPacket runs and populates device capabilities.
    if (debug_enabled()) {
        log_line("waiting for device info...");
    }
    {
        QElapsedTimer infoTimer;
        infoTimer.start();
        const int infoTimeoutMs = 5000;
        bool infoReceived = false;

        // Connect a flag so we know when InfoUpdated fires
        auto infoConn = QObject::connect(selection.driver, &DeviceDriver::InfoUpdated, selection.driver, [&]() {
            infoReceived = true;
            if (debug_enabled()) {
                log_line("device info received via InfoUpdated signal");
            }
        }, Qt::DirectConnection);

        int lastLoggedSec = -1;
        while (!infoReceived && infoTimer.elapsed() < infoTimeoutMs) {
            // Process events with a reasonable timeout
            QCoreApplication::processEvents(QEventLoop::AllEvents, 100);

            // Check if info is available
            if (selection.driver->supports(DeviceDriver::Feature::VNA)) {
                infoReceived = true;
                if (debug_enabled()) {
                    log_line("device info detected via supports() check");
                }
            }

            // Log progress once per second
            int currentSec = infoTimer.elapsed() / 1000;
            if (debug_enabled() && currentSec > lastLoggedSec) {
                lastLoggedSec = currentSec;
                log_line(QStringLiteral("still waiting... elapsed=%1ms").arg(infoTimer.elapsed()));
            }

            // Small sleep to avoid spinning too fast
            if (!infoReceived) {
                QThread::msleep(10);
            }
        }

        // Disconnect to avoid dangling reference to local infoReceived
        QObject::disconnect(infoConn);

        if (!infoReceived) {
            if (debug_enabled()) {
                log_line(QStringLiteral("timeout waiting for device info after %1 ms").arg(infoTimer.elapsed()));
            }
            selection.driver->disconnectDevice();
            throw std::runtime_error("timeout waiting for device info");
        }
    }

    if (debug_enabled()) {
        log_line("device ready");
    }

    DeviceDriver::VNASettings settings;
    settings.freqStart = request.f_start_hz;
    settings.freqStop = request.f_stop_hz;
    settings.dBmStart = request.power_dbm;
    settings.dBmStop = request.power_dbm;
    settings.IFBW = request.ifbw_hz;
    settings.points = request.points;
    settings.logSweep = false;
    settings.dwellTime = 0.0;
    settings.excitedPorts = request.excited_ports;
    if (settings.excitedPorts.empty()) {
        settings.excitedPorts = {1, 2};
    }
    if (debug_enabled()) {
        QStringList port_list;
        for (int port : settings.excitedPorts) {
            port_list << QString::number(port);
        }
        log_line(QStringLiteral("starting sweep points=%1 ports=%2")
                     .arg(settings.points)
                     .arg(port_list.join(",")));
    }

    QString error;
    std::vector<DeviceDriver::VNAMeasurement> measurements;

    QEventLoop loop;
    QTimer timeout;
    timeout.setSingleShot(true);
    QObject::connect(&timeout, &QTimer::timeout, &loop, [&]() {
        if (error.isEmpty()) {
            error = "sweep timeout";
        }
        if (debug_enabled()) {
            log_line(QStringLiteral("sweep timeout after %1 ms (received=%2)")
                         .arg(request.timeout_ms, 0, 'f', 0)
                         .arg(static_cast<int>(measurements.size())));
        }
        loop.quit();
    });

    QObject::connect(selection.driver, &DeviceDriver::VNAmeasurementReceived, &loop,
                     [&](DeviceDriver::VNAMeasurement measurement) {
                         if (has_cal) {
                             calibration.correctMeasurement(measurement);
                         }
                         measurements.push_back(measurement);
                         if (debug_enabled()) {
                             const int count = static_cast<int>(measurements.size());
                             if (count == 1 || (count % 50) == 0 || count == request.points) {
                                 log_line(QStringLiteral("measurement received %1/%2")
                                              .arg(count)
                                              .arg(request.points));
                             }
                         }
                         if (static_cast<int>(measurements.size()) >= request.points) {
                             loop.quit();
                         }
                     });

    if (debug_enabled()) {
        log_line("setVNA starting");
    }
    const bool configured = selection.driver->setVNA(settings, [&](bool ok) {
        if (debug_enabled()) {
            log_line(QStringLiteral("setVNA callback ok=%1").arg(ok ? "true" : "false"));
        }
        if (!ok && error.isEmpty()) {
            error = "failed to configure VNA sweep";
            loop.quit();
        }
    });
    if (debug_enabled()) {
        log_line(QStringLiteral("setVNA returned=%1").arg(configured ? "true" : "false"));
    }

    if (!configured) {
        selection.driver->disconnectDevice();
        throw std::runtime_error("failed to start VNA sweep");
    }

    timeout.start(static_cast<int>(request.timeout_ms));
    if (debug_enabled()) {
        log_line("event loop starting");
    }
    loop.exec();
    if (debug_enabled()) {
        log_line(QStringLiteral("event loop ended (error=%1, received=%2)")
                     .arg(error.isEmpty() ? "none" : error)
                     .arg(static_cast<int>(measurements.size())));
    }

    selection.driver->setIdle(nullptr);
    selection.driver->disconnectDevice();
    if (debug_enabled()) {
        log_line("device disconnected");
    }

    if (!error.isEmpty()) {
        if (debug_enabled()) {
            log_line(QStringLiteral("sweep failed: %1").arg(error));
        }
        throw std::runtime_error(error.toStdString());
    }

    if (static_cast<int>(measurements.size()) < request.points) {
        throw std::runtime_error("sweep ended early");
    }

    if (debug_enabled()) {
        log_line(QStringLiteral("sweep complete (points=%1)").arg(static_cast<int>(measurements.size())));
    }

    SweepResult result;
    result.overall_pass = true;

    for (const auto &measurement : measurements) {
        SweepPoint point;
        point.frequency = measurement.frequency;
        for (const auto &entry : measurement.measurements) {
            const QString param = entry.first;
            const std::complex<double> value = entry.second;
            point.params[param] = value;

            if (param.startsWith("S")) {
                auto &param_result = result.results[param];
                const double mag_db = magnitude_db(value);
                if (mag_db > param_result.worst_db) {
                    param_result.worst_db = mag_db;
                    param_result.fail_at_hz = point.frequency;
                }
            }
        }
        result.trace.push_back(point);
    }

    for (auto &entry : result.results) {
        auto &param_result = entry.second;
        param_result.pass = param_result.worst_db <= request.threshold_db;
        if (!param_result.pass) {
            param_result.has_fail = true;
            result.overall_pass = false;
        }
    }

    return result;
}

}  // namespace LibreVNAHeadless
