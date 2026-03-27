#include "librevna_headless.h"

#include "Calibration/calibration.h"
#include "Calibration/calibrationmeasurement.h"
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
    // Retry connect up to 3 times, waiting for the device to become available again
    // after a previous sweep. The LibreVNA can take ~30s to be re-detected after a sweep.
    const int maxRetries = 3;
    const int retryDelayMs = 10000;  // 10 seconds between retries
    bool connected = false;
    for (int attempt = 1; attempt <= maxRetries; ++attempt) {
        if (debug_enabled()) {
            log_line(QStringLiteral("connect attempt %1/%2").arg(attempt).arg(maxRetries));
        }
        if (selection.driver->connectDevice(serial)) {
            connected = true;
            break;
        }
        if (attempt < maxRetries) {
            if (debug_enabled()) {
                log_line(QStringLiteral("connect failed, waiting %1 ms before retry...").arg(retryDelayMs));
            }
            QElapsedTimer delay;
            delay.start();
            while (delay.elapsed() < retryDelayMs) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
                QThread::msleep(50);
            }
        }
    }
    if (!connected) {
        throw std::runtime_error("failed to connect to LibreVNA device (tried multiple times)");
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

// ---------------------------------------------------------------------------
// Helper: convert CalAcquisitionResult to JSON
// ---------------------------------------------------------------------------
static nlohmann::json cal_acquisition_to_json(const CalAcquisitionResult &r) {
    nlohmann::json j;
    j["success"] = r.success;
    j["error"] = r.error.toStdString();
    j["standard"] = r.standard.toStdString();
    j["port1"] = r.port1;
    j["port2"] = r.port2;
    j["trace"] = r.trace;
    return j;
}

// ---------------------------------------------------------------------------
// Helper: convert CalStandardType from string
// ---------------------------------------------------------------------------
static CalStandardType standard_type_from_string(const QString &s) {
    const QString lower = s.toLower();
    if (lower == "open")     return CalStandardType::Open;
    if (lower == "short")    return CalStandardType::Short;
    if (lower == "load")     return CalStandardType::Load;
    if (lower == "through")  return CalStandardType::Through;
    return CalStandardType::Open;
}

// ---------------------------------------------------------------------------
// Helper: build VNASettings from a generic SweepRequest
// ---------------------------------------------------------------------------
static DeviceDriver::VNASettings build_vna_settings(const SweepRequest &req) {
    DeviceDriver::VNASettings s;
    s.freqStart = req.f_start_hz;
    s.freqStop = req.f_stop_hz;
    s.dBmStart = req.power_dbm;
    s.dBmStop = req.power_dbm;
    s.IFBW = req.ifbw_hz;
    s.points = req.points;
    s.logSweep = false;
    s.dwellTime = 0.0;
    s.excitedPorts = req.excited_ports;
    if (s.excitedPorts.empty()) {
        s.excitedPorts = {1, 2};
    }
    return s;
}

// ---------------------------------------------------------------------------
// Helper: connect to device and wait for info
// ---------------------------------------------------------------------------
static bool connect_and_wait(DeviceDriver *driver, const QString &serial,
                            const QString &label, int timeout_ms = 5000) {
    if (!driver->connectDevice(serial)) {
        if (debug_enabled()) {
            log_line(QStringLiteral("[%1] connect failed").arg(label));
        }
        return false;
    }
    if (debug_enabled()) {
        log_line(QStringLiteral("[%1] connected, waiting for info...").arg(label));
    }

    QElapsedTimer timer;
    timer.start();
    while (timer.elapsed() < timeout_ms) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
        QThread::msleep(10);
        if (driver->supports(DeviceDriver::Feature::VNA)) {
            if (debug_enabled()) {
                log_line(QStringLiteral("[%1] device ready").arg(label));
            }
            return true;
        }
    }
    if (debug_enabled()) {
        log_line(QStringLiteral("[%1] device info timeout").arg(label));
    }
    driver->disconnectDevice();
    return false;
}

// ---------------------------------------------------------------------------
// Helper: run a single acquisition sweep and collect trace data
// ---------------------------------------------------------------------------
static std::pair<bool, std::vector<DeviceDriver::VNAMeasurement>>
run_acquisition_sweep(DeviceDriver *driver,
                       const DeviceDriver::VNASettings &settings,
                       const QString &label,
                       double timeout_ms)
{
    std::vector<DeviceDriver::VNAMeasurement> measurements;
    QString error;
    QEventLoop loop;
    QTimer timeout;
    timeout.setSingleShot(true);

    QObject::connect(&timeout, &QTimer::timeout, &loop, [&]() {
        if (error.isEmpty()) error = "timeout";
        if (debug_enabled()) {
            log_line(QStringLiteral("[%1] sweep timeout (received=%2)").arg(label).arg(
                         static_cast<int>(measurements.size())));
        }
        loop.quit();
    });

    auto conn = QObject::connect(driver, &DeviceDriver::VNAmeasurementReceived,
                                &loop,
                                [&](DeviceDriver::VNAMeasurement m) {
                                    measurements.push_back(m);
                                    if (debug_enabled()) {
                                        const int cnt = static_cast<int>(measurements.size());
                                        if (cnt == 1 || (cnt % 100) == 0 || cnt == settings.points) {
                                            log_line(QStringLiteral("[%1] point %2/%3")
                                                         .arg(label).arg(cnt).arg(settings.points));
                                        }
                                    }
                                    if (static_cast<int>(measurements.size()) >= settings.points) {
                                        loop.quit();
                                    }
                                });

    bool configured = driver->setVNA(settings, [&](bool ok) {
        if (!ok && error.isEmpty()) {
            error = "setVNA failed";
            loop.quit();
        }
    });

    if (!configured) {
        QObject::disconnect(conn);
        if (debug_enabled()) {
            log_line(QStringLiteral("[%1] setVNA returned false").arg(label));
        }
        return {false, {}};
    }

    timeout.start(static_cast<int>(timeout_ms));
    loop.exec();
    QObject::disconnect(conn);
    driver->setIdle(nullptr);

    if (debug_enabled()) {
        log_line(QStringLiteral("[%1] sweep done (error='%2' received=%3)")
                     .arg(label).arg(error).arg(static_cast<int>(measurements.size())));
    }

    if (!error.isEmpty() && error != "timeout") {
        return {false, {}};
    }
    return {true, measurements};
}

// ---------------------------------------------------------------------------
// acquire_cal_standard
// Performs a single standard acquisition (Open / Short / Load / Through).
// ---------------------------------------------------------------------------
CalAcquisitionResult acquire_cal_standard(const QString &standard,
                                         int port1, int port2,
                                         const SweepRequest &sweep_params)
{
    InformationBox::setGUI(false);
    CalAcquisitionResult result;
    result.standard = standard.toLower();
    result.port1 = port1;
    result.port2 = port2;
    qRegisterMetaType<DeviceDriver::VNAMeasurement>("DeviceDriver::VNAMeasurement");

    if (debug_enabled()) {
        log_line(QStringLiteral("acquire_cal_standard: standard=%1 port1=%2 port2=%3 f_start=%4 f_stop=%5 points=%6")
                     .arg(standard).arg(port1).arg(port2)
                     .arg(sweep_params.f_start_hz, 0, 'f', 0)
                     .arg(sweep_params.f_stop_hz, 0, 'f', 0)
                     .arg(sweep_params.points));
    }

    if (sweep_params.points <= 0) {
        result.error = "points must be > 0";
        return result;
    }

    Selection sel = select_device(sweep_params.serial.trimmed());
    if (!sel.driver) {
        result.error = "no LibreVNA device detected";
        return result;
    }

    if (!connect_and_wait(sel.driver, sel.serial, standard)) {
        result.error = "failed to connect to device";
        return result;
    }

    // Build port list: for Open/Short/Load excite port1 only (reflectometry),
    // for Through excite both ports
    std::vector<int> ports;
    CalStandardType stype = standard_type_from_string(standard);
    if (stype == CalStandardType::Through) {
        ports = (sweep_params.excited_ports.empty() || sweep_params.excited_ports.size() == 1)
                    ? std::vector<int>{port1, port2}
                    : sweep_params.excited_ports;
    } else {
        ports = {port1};
    }

    DeviceDriver::VNASettings settings = build_vna_settings(sweep_params);
    settings.excitedPorts = ports;

    if (debug_enabled()) {
        log_line(QStringLiteral("acquire_cal_standard: running sweep for %1").arg(standard));
    }

    auto [ok, measurements] = run_acquisition_sweep(
        sel.driver, settings, standard,
        sweep_params.timeout_ms > 0 ? sweep_params.timeout_ms : 30000.0);

    sel.driver->disconnectDevice();

    if (!ok || measurements.empty()) {
        result.error = measurements.empty() ? "no measurements received" : "acquisition failed";
        return result;
    }

    if (static_cast<int>(measurements.size()) < sweep_params.points) {
        if (debug_enabled()) {
            log_line(QStringLiteral("acquire_cal_standard: incomplete sweep %1/%2")
                         .arg(static_cast<int>(measurements.size())).arg(sweep_params.points));
        }
    }

    result.success = true;

    // Build trace array as nlohmann::json
    for (const auto &m : measurements) {
        nlohmann::json row;
        row["frequency"] = m.frequency;
        for (const auto &entry : m.measurements) {
            nlohmann::json complex_val;
            complex_val["real"] = entry.second.real();
            complex_val["imag"] = entry.second.imag();
            row[entry.first.toStdString()] = complex_val;
        }
        result.trace.push_back(row);
    }

    if (debug_enabled()) {
        log_line(QStringLiteral("acquire_cal_standard: done, %1 points").arg(
                     static_cast<int>(result.trace.size())));
    }

    return result;
}

// ---------------------------------------------------------------------------
// run_calibration
// Performs a complete SOLT/OSL calibration sequence:
//   1. Acquire Open (port 1 and optionally port 2)
//   2. Acquire Short (port 1 and optionally port 2)
//   3. Acquire Load (port 1 and optionally port 2)
//   4. Acquire Through (port 1 <-> port 2, if 2-port)
//   5. Compute error terms
//   6. Save .cal file
// ---------------------------------------------------------------------------
CalibrationResult run_calibration(const CalibrationRequest &request)
{
    InformationBox::setGUI(false);
    CalibrationResult result;
    qRegisterMetaType<DeviceDriver::VNAMeasurement>("DeviceDriver::VNAMeasurement");

    const QString calType = request.type.toUpper();
    if (debug_enabled()) {
        log_line(QStringLiteral("run_calibration: type=%1 f_start=%2 f_stop=%3 points=%4 output=%5")
                     .arg(calType)
                     .arg(request.f_start_hz, 0, 'f', 0)
                     .arg(request.f_stop_hz, 0, 'f', 0)
                     .arg(request.points)
                     .arg(request.output_path));
    }

    if (request.points <= 0) {
        result.error = "points must be > 0";
        return result;
    }
    if (request.output_path.isEmpty()) {
        result.error = "output_path is required";
        return result;
    }

    // Build the base sweep request
    SweepRequest base;
    base.f_start_hz = request.f_start_hz;
    base.f_stop_hz = request.f_stop_hz;
    base.points = request.points;
    base.ifbw_hz = request.ifbw_hz;
    base.power_dbm = request.power_dbm;
    base.timeout_ms = request.timeout_ms;
    base.excited_ports = request.ports.empty() ? std::vector<int>{1, 2} : request.ports;
    base.serial = request.serial;

    std::vector<int> ports = base.excited_ports;
    if (ports.empty()) ports = {1, 2};

    // ------------------------------------------------------------------------
    // Step 1: Select device and connect once for the entire sequence
    // ------------------------------------------------------------------------
    Selection sel = select_device(request.serial.trimmed());
    if (!sel.driver) {
        result.error = "no LibreVNA device detected";
        return result;
    }

    if (!connect_and_wait(sel.driver, sel.serial, "cal")) {
        result.error = "failed to connect to device";
        return result;
    }

    // Build sweep settings for 1-port (Open/Short/Load)
    DeviceDriver::VNASettings settings_1port = build_vna_settings(base);
    settings_1port.excitedPorts = {ports[0]};

    // Build sweep settings for 2-port (Through)
    DeviceDriver::VNASettings settings_2port = build_vna_settings(base);
    settings_2port.excitedPorts = (ports.size() >= 2) ? std::vector<int>{ports[0], ports[1]}
                                                       : std::vector<int>{ports[0]};

    // ------------------------------------------------------------------------
    // Step 2: Acquire Open on all ports
    // ------------------------------------------------------------------------
    std::vector<DeviceDriver::VNAMeasurement> open_measurements;
    for (int port : ports) {
        if (debug_enabled()) {
            log_line(QStringLiteral("run_calibration: acquiring Open on port %1").arg(port));
        }
        auto [ok, pts] = run_acquisition_sweep(sel.driver, settings_1port, "Open",
                                               base.timeout_ms);
        if (!ok || pts.empty()) {
            result.error = QString("Open acquisition failed on port %1").arg(port);
            sel.driver->disconnectDevice();
            return result;
        }
        open_measurements.insert(open_measurements.end(), pts.begin(), pts.end());
    }
    result.open_done = true;
    if (debug_enabled()) {
        log_line(QStringLiteral("run_calibration: Open done (%1 points)").arg(
                     static_cast<int>(open_measurements.size())));
    }

    // ------------------------------------------------------------------------
    // Step 3: Acquire Short on all ports
    // ------------------------------------------------------------------------
    std::vector<DeviceDriver::VNAMeasurement> short_measurements;
    for (int port : ports) {
        if (debug_enabled()) {
            log_line(QStringLiteral("run_calibration: acquiring Short on port %1").arg(port));
        }
        auto [ok, pts] = run_acquisition_sweep(sel.driver, settings_1port, "Short",
                                               base.timeout_ms);
        if (!ok || pts.empty()) {
            result.error = QString("Short acquisition failed on port %1").arg(port);
            sel.driver->disconnectDevice();
            return result;
        }
        short_measurements.insert(short_measurements.end(), pts.begin(), pts.end());
    }
    result.short_done = true;
    if (debug_enabled()) {
        log_line(QStringLiteral("run_calibration: Short done"));
    }

    // ------------------------------------------------------------------------
    // Step 4: Acquire Load on all ports
    // ------------------------------------------------------------------------
    std::vector<DeviceDriver::VNAMeasurement> load_measurements;
    for (int port : ports) {
        if (debug_enabled()) {
            log_line(QStringLiteral("run_calibration: acquiring Load on port %1").arg(port));
        }
        auto [ok, pts] = run_acquisition_sweep(sel.driver, settings_1port, "Load",
                                               base.timeout_ms);
        if (!ok || pts.empty()) {
            result.error = QString("Load acquisition failed on port %1").arg(port);
            sel.driver->disconnectDevice();
            return result;
        }
        load_measurements.insert(load_measurements.end(), pts.begin(), pts.end());
    }
    result.load_done = true;
    if (debug_enabled()) {
        log_line(QStringLiteral("run_calibration: Load done"));
    }

    // ------------------------------------------------------------------------
    // Step 5: Acquire Through (only for 2-port)
    // ------------------------------------------------------------------------
    std::vector<DeviceDriver::VNAMeasurement> through_measurements;
    if (ports.size() >= 2) {
        if (debug_enabled()) {
            log_line(QStringLiteral("run_calibration: acquiring Through"));
        }
        auto [ok, pts] = run_acquisition_sweep(sel.driver, settings_2port, "Through",
                                                base.timeout_ms);
        if (!ok || pts.empty()) {
            result.error = "Through acquisition failed";
            sel.driver->disconnectDevice();
            return result;
        }
        through_measurements = std::move(pts);
        result.through_done = true;
        if (debug_enabled()) {
            log_line(QStringLiteral("run_calibration: Through done"));
        }
    }

    // ------------------------------------------------------------------------
    // Step 6: Disconnect before compute/save
    // ------------------------------------------------------------------------
    sel.driver->disconnectDevice();
    if (debug_enabled()) {
        log_line("run_calibration: disconnected");
    }

    // ------------------------------------------------------------------------
    // Step 7: Build Calibration object, add measurements, compute
    // ------------------------------------------------------------------------
    if (debug_enabled()) {
        log_line("run_calibration: building Calibration object");
    }

    Calibration calibration;
    calibration.reset();

    // Determine calibration type
    Calibration::Type cal_type;
    if (calType == "OSL") {
        cal_type = Calibration::Type::SOLT;
    } else {
        cal_type = Calibration::Type::SOLT;
    }

    // Create default measurement set
    Calibration::DefaultMeasurements dm;
    if (ports.size() >= 2) {
        dm = Calibration::DefaultMeasurements::SOLT2Port;
    } else {
        dm = Calibration::DefaultMeasurements::SOL1Port;
    }
    calibration.createDefaultMeasurements(dm);

    // Build CalType
    Calibration::CalType ct;
    ct.type = cal_type;
    ct.usedPorts.clear();
    std::transform(ports.begin(), ports.end(), std::back_inserter(ct.usedPorts),
                   [](int p) { return static_cast<unsigned int>(p); });

    // Add Open measurements (one per port)
    auto openSet = calibration.findMeasurements(CalibrationMeasurement::Base::Type::Open);
    auto shortSet = calibration.findMeasurements(CalibrationMeasurement::Base::Type::Short);
    auto loadSet = calibration.findMeasurements(CalibrationMeasurement::Base::Type::Load);
    auto throughSet = calibration.findMeasurements(CalibrationMeasurement::Base::Type::Through);

    auto add_meas = [&](CalibrationMeasurement::Base *m,
                        const std::vector<DeviceDriver::VNAMeasurement> &data) {
        for (const auto &pt : data) {
            m->addPoint(pt);
        }
    };

    if (!openSet.empty() && !open_measurements.empty()) {
        add_meas(openSet[0], open_measurements);
    }
    if (!shortSet.empty() && !short_measurements.empty()) {
        add_meas(shortSet[0], short_measurements);
    }
    if (!loadSet.empty() && !load_measurements.empty()) {
        add_meas(loadSet[0], load_measurements);
    }
    if (!throughSet.empty() && !through_measurements.empty()) {
        add_meas(throughSet[0], through_measurements);
    }

    if (debug_enabled()) {
        log_line("run_calibration: calling compute()");
    }

    bool computed = calibration.compute(ct);
    if (!computed) {
        result.error = "calibration compute failed (insufficient measurements?)";
        return result;
    }

    // ------------------------------------------------------------------------
    // Step 8: Save to file
    // ------------------------------------------------------------------------
    if (debug_enabled()) {
        log_line(QStringLiteral("run_calibration: saving to %1").arg(request.output_path));
    }

    bool saved = calibration.toFile(request.output_path);
    if (!saved) {
        result.error = QString("failed to save calibration to %1").arg(request.output_path);
        return result;
    }

    result.success = true;
    result.cal_file = request.output_path;
    result.message = QString("Calibration saved to %1").arg(request.output_path);

    if (debug_enabled()) {
        log_line(QStringLiteral("run_calibration: SUCCESS - %1").arg(request.output_path));
    }

    return result;
}

}  // namespace LibreVNAHeadless
