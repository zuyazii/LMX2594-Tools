#include "librevna_headless.h"

#include "json.hpp"

#include <QApplication>
#include <QCommandLineParser>
#include <QFileInfo>
#include <QTextStream>

#include <iostream>

using json = nlohmann::json;

namespace {
json to_json(const LibreVNAHeadless::SweepResult &result) {
    json payload;
    payload["overall_pass"] = result.overall_pass;

    json results = json::object();
    for (const auto &entry : result.results) {
        json item;
        item["pass"] = entry.second.pass;
        item["worst_db"] = entry.second.worst_db;
        if (entry.second.has_fail) {
            item["fail_at_hz"] = entry.second.fail_at_hz;
        }
        results[entry.first.toStdString()] = item;
    }
    payload["results"] = results;

    json trace = json::array();
    for (const auto &point : result.trace) {
        json row;
        row["frequency"] = point.frequency;
        for (const auto &param : point.params) {
            json entry;
            entry["real"] = param.second.real();
            entry["imag"] = param.second.imag();
            row[param.first.toStdString()] = entry;
        }
        trace.push_back(row);
    }
    payload["trace"] = trace;

    return payload;
}
}  // namespace

int main(int argc, char **argv) {
    QApplication app(argc, argv);
    QCoreApplication::setApplicationName("librevna-cli");

    QCommandLineParser parser;
    parser.setApplicationDescription("LibreVNA headless CLI");
    parser.addHelpOption();

    QCommandLineOption calOpt("cal", "Calibration file path", "path");
    QCommandLineOption fstartOpt("fstart", "Start frequency (Hz)", "hz");
    QCommandLineOption fstopOpt("fstop", "Stop frequency (Hz)", "hz");
    QCommandLineOption pointsOpt("points", "Number of points", "count");
    QCommandLineOption ifbwOpt("ifbw", "IF bandwidth (Hz)", "hz");
    QCommandLineOption powerOpt("power", "Output power (dBm)", "dbm");
    QCommandLineOption thresholdOpt("threshold", "Pass/fail threshold (dB)", "db");
    QCommandLineOption timeoutOpt("timeout-ms", "Timeout (ms)", "ms", "15000");
    QCommandLineOption serialOpt("serial", "Device serial", "serial");
    QCommandLineOption exciteOpt("excite", "Comma-separated excited ports", "ports", "1,2");
    QCommandLineOption jsonOutOpt("json-out", "Write JSON output to file", "path");
    QCommandLineOption progressOpt("progress-ndjson", "Emit NDJSON progress");

    parser.addOption(calOpt);
    parser.addOption(fstartOpt);
    parser.addOption(fstopOpt);
    parser.addOption(pointsOpt);
    parser.addOption(ifbwOpt);
    parser.addOption(powerOpt);
    parser.addOption(thresholdOpt);
    parser.addOption(timeoutOpt);
    parser.addOption(serialOpt);
    parser.addOption(exciteOpt);
    parser.addOption(jsonOutOpt);
    parser.addOption(progressOpt);

    parser.process(app);

    if (!parser.isSet(calOpt) || !parser.isSet(fstartOpt) || !parser.isSet(fstopOpt) ||
        !parser.isSet(pointsOpt) || !parser.isSet(ifbwOpt) || !parser.isSet(powerOpt) ||
        !parser.isSet(thresholdOpt)) {
        QTextStream(stderr) << "Missing required arguments. Use --help for details.\n";
        return 2;
    }

    const QString calPath = parser.value(calOpt);
    if (!QFileInfo::exists(calPath)) {
        QTextStream(stderr) << "Calibration file not found: " << calPath << "\n";
        return 2;
    }

    LibreVNAHeadless::SweepRequest request;
    request.cal_path = calPath;
    request.f_start_hz = parser.value(fstartOpt).toDouble();
    request.f_stop_hz = parser.value(fstopOpt).toDouble();
    request.points = parser.value(pointsOpt).toInt();
    request.ifbw_hz = parser.value(ifbwOpt).toDouble();
    request.power_dbm = parser.value(powerOpt).toDouble();
    request.threshold_db = parser.value(thresholdOpt).toDouble();
    request.timeout_ms = parser.value(timeoutOpt).toDouble();
    request.serial = parser.value(serialOpt);

    const QStringList ports = parser.value(exciteOpt).split(',', Qt::SkipEmptyParts);
    for (const auto &port : ports) {
        request.excited_ports.push_back(port.trimmed().toInt());
    }

    try {
        LibreVNAHeadless::SweepResult result = LibreVNAHeadless::run_sweep(request);
        json payload = to_json(result);
        const std::string output = payload.dump();
        std::cout << output;

        if (parser.isSet(jsonOutOpt)) {
            QFile file(parser.value(jsonOutOpt));
            if (file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
                file.write(output.c_str(), static_cast<qint64>(output.size()));
                file.close();
            }
        }

        return result.overall_pass ? 0 : 1;
    } catch (const std::exception &exc) {
        QTextStream(stderr) << "librevna-cli error: " << exc.what() << "\n";
        return 2;
    }
}
