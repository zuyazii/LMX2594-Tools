#include "librevna_headless.h"

#include "json.hpp"

#include <QApplication>
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QHash>
#include <QLocalServer>
#include <QLocalSocket>
#include <QFileInfo>
#include <QMutex>
#include <QTextStream>
#include <exception>

using json = nlohmann::json;

namespace {
bool debug_enabled() {
    const QString value = qEnvironmentVariable("LIBREVNA_IPC_DEBUG").trimmed().toLower();
    return value == "1" || value == "true" || value == "yes";
}

QString log_path() {
    const QString env_path = qEnvironmentVariable("LIBREVNA_IPC_LOG").trimmed();
    if (!env_path.isEmpty()) {
        return env_path;
    }
    return QDir::current().filePath("librevna-ipc.log");
}

void log_line(const QString &message) {
    const QString line = QDateTime::currentDateTime().toString("HH:mm:ss.zzz") + " " + message + "\n";
    QTextStream(stderr) << line;
    static QMutex mutex;
    QMutexLocker locker(&mutex);
    QFile file(log_path());
    if (file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
        QTextStream out(&file);
        out << line;
    }
}

json sweep_to_json(const LibreVNAHeadless::SweepResult &result) {
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

json devices_to_json(const std::vector<LibreVNAHeadless::DeviceInfo> &devices, bool real_driver, const QString &warning) {
    json payload;
    payload["devices"] = json::array();
    for (const auto &device : devices) {
        json entry;
        entry["label"] = device.label.toStdString();
        entry["serial"] = device.serial.toStdString();
        entry["driver"] = device.driver.toStdString();
        payload["devices"].push_back(entry);
    }
    payload["real_driver"] = real_driver;
    if (!warning.isEmpty()) {
        payload["warning"] = warning.toStdString();
    }
    return payload;
}

class IpcServer : public QObject {
    Q_OBJECT
public:
    explicit IpcServer(const QString &name, QObject *parent = nullptr)
        : QObject(parent) {
        const bool removed = QLocalServer::removeServer(name);
        if (debug_enabled()) {
            log_line(QStringLiteral("Removed stale server %1: %2").arg(name, removed ? "true" : "false"));
        }
        if (!server_.listen(name)) {
            log_line(QStringLiteral("Failed to listen on %1: %2").arg(name, server_.errorString()));
        } else {
            log_line(QStringLiteral("Listening on %1").arg(name));
        }
        connect(&server_, &QLocalServer::newConnection, this, &IpcServer::handleNewConnection);
        QTextStream(stdout) << "LibreVNA IPC listening on " << name << "\n";
    }

private:
    void handleNewConnection() {
        while (auto *socket = server_.nextPendingConnection()) {
            buffers_[socket] = QByteArray();
            if (debug_enabled()) {
                log_line("Client connected");
            }
            connect(socket, &QLocalSocket::readyRead, this, [this, socket]() { handleReadyRead(socket); });
            connect(socket, &QLocalSocket::disconnected, this, [this, socket]() {
                buffers_.remove(socket);
                if (debug_enabled()) {
                    log_line("Client disconnected");
                }
                socket->deleteLater();
            });
        }
    }

    void handleReadyRead(QLocalSocket *socket) {
        auto &buffer = buffers_[socket];
        buffer.append(socket->readAll());
        while (true) {
            const int index = buffer.indexOf('\n');
            if (index < 0) {
                break;
            }
            const QByteArray line = buffer.left(index).trimmed();
            buffer.remove(0, index + 1);
            if (!line.isEmpty()) {
                handleRequest(socket, line);
            }
        }
    }

    void handleRequest(QLocalSocket *socket, const QByteArray &line) {
        json response;
        response["ok"] = false;

        try {
            const json request = json::parse(line.constData());
            const std::string cmd = request.value("cmd", "");
            if (debug_enabled()) {
                log_line(QStringLiteral("Request cmd=%1").arg(QString::fromStdString(cmd)));
            }
            if (cmd == "list_devices") {
                bool real_driver = true;
                QString warning;
                auto devices = LibreVNAHeadless::list_devices(&real_driver, &warning);
                response["ok"] = true;
                response["result"] = devices_to_json(devices, real_driver, warning);
            } else if (cmd == "run_sweep") {
                QElapsedTimer timer;
                timer.start();
                LibreVNAHeadless::SweepRequest sweep;
                sweep.cal_path = QString::fromStdString(request.value("cal_path", ""));
                sweep.f_start_hz = request.value("f_start_hz", 0.0);
                sweep.f_stop_hz = request.value("f_stop_hz", 0.0);
                sweep.points = request.value("points", 0);
                sweep.ifbw_hz = request.value("ifbw_hz", 0.0);
                sweep.power_dbm = request.value("power_dbm", 0.0);
                sweep.threshold_db = request.value("threshold_db", 0.0);
                sweep.timeout_ms = request.value("timeout_ms", 15000.0);
                sweep.serial = QString::fromStdString(request.value("serial", ""));
                sweep.excited_ports.clear();
                if (request.contains("excited_ports") && request["excited_ports"].is_array()) {
                    for (const auto &item : request["excited_ports"]) {
                        sweep.excited_ports.push_back(item.get<int>());
                    }
                }

                if (!sweep.cal_path.isEmpty() && !QFileInfo::exists(sweep.cal_path)) {
                    throw std::runtime_error("calibration file not found");
                }

                if (debug_enabled()) {
                    log_line(QStringLiteral("run_sweep start f_start=%1 f_stop=%2 points=%3 ifbw=%4 power=%5 timeout_ms=%6")
                                 .arg(sweep.f_start_hz, 0, 'f', 0)
                                 .arg(sweep.f_stop_hz, 0, 'f', 0)
                                 .arg(sweep.points)
                                 .arg(sweep.ifbw_hz, 0, 'f', 0)
                                 .arg(sweep.power_dbm, 0, 'f', 1)
                                 .arg(sweep.timeout_ms, 0, 'f', 0));
                }
                const auto result = LibreVNAHeadless::run_sweep(sweep);
                response["ok"] = true;
                response["result"] = sweep_to_json(result);
                if (debug_enabled()) {
                    log_line(QStringLiteral("run_sweep done in %1 ms").arg(timer.elapsed()));
                }
            } else if (cmd == "shutdown") {
                log_line("Shutdown requested");
                response["ok"] = true;
                response["result"] = json::object();
                socket->write((response.dump() + "\n").c_str());
                socket->flush();
                QCoreApplication::quit();
                return;
            } else {
                throw std::runtime_error("unknown cmd");
            }
        } catch (const std::exception &exc) {
            response["error"] = exc.what();
            log_line(QStringLiteral("Request failed: %1").arg(exc.what()));
        }

        socket->write((response.dump() + "\n").c_str());
        socket->flush();
    }

    QLocalServer server_;
    QHash<QLocalSocket *, QByteArray> buffers_;
};
}  // namespace

int main(int argc, char **argv) {
    std::set_terminate([]() {
        log_line("std::terminate called");
        std::_Exit(1);
    });
    if (qEnvironmentVariable("LIBREVNA_DISABLE_PACKET_LOG").trimmed().isEmpty()) {
        qputenv("LIBREVNA_DISABLE_PACKET_LOG", "1");
        log_line("Packet log disabled by default (set LIBREVNA_DISABLE_PACKET_LOG=0 to enable)");
    }
    QApplication app(argc, argv);
    QCoreApplication::setApplicationName("librevna-ipc");

    const QString name = qEnvironmentVariable("LIBREVNA_IPC_NAME", "librevna-ipc");
    log_line("LibreVNA IPC starting");
    log_line(QStringLiteral("PID=%1").arg(QCoreApplication::applicationPid()));
    log_line(QStringLiteral("Qt=%1").arg(QT_VERSION_STR));
    log_line(QStringLiteral("App=%1").arg(QCoreApplication::applicationFilePath()));
    log_line(QStringLiteral("CWD=%1").arg(QDir::currentPath()));
    log_line(QStringLiteral("Server name=%1").arg(name));
    log_line(QStringLiteral("Log file=%1").arg(log_path()));
    if (debug_enabled()) {
        log_line(QStringLiteral("PATH=%1").arg(qEnvironmentVariable("PATH")));
    }
    IpcServer server(name);
    return app.exec();
}

#include "librevna_ipc_main.moc"
