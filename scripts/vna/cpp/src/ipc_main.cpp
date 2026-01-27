#include "cli_options.hpp"
#include "json_writer.hpp"
#include "librevna_headless/device_discovery.hpp"
#include "librevna_headless/host_core.hpp"
#include "sweep_result.hpp"

#include <QByteArray>
#include <QCoreApplication>
#include <QHash>
#include <QLocalServer>
#include <QLocalSocket>

#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace
{
std::vector<int> parse_ports(const nlohmann::json &value)
{
    std::vector<int> ports;
    if(!value.is_array())
    {
        return ports;
    }
    for(const auto &entry : value)
    {
        if(entry.is_number_integer())
        {
            ports.push_back(entry.get<int>());
        }
    }
    return ports;
}

} // namespace

class IpcServer : public QObject
{
public:
    explicit IpcServer(const QString &name, QObject *parent = nullptr)
        : QObject(parent), server_(new QLocalServer(this))
    {
        QLocalServer::removeServer(name);
        if(!server_->listen(name))
        {
            std::cerr << "Failed to listen on " << name.toStdString() << std::endl;
        }

        connect(server_, &QLocalServer::newConnection, this, [this]() {
            while(QLocalSocket *socket = server_->nextPendingConnection())
            {
                buffers_.insert(socket, {});
                connect(socket, &QLocalSocket::readyRead, this, [this, socket]() { onReadyRead(socket); });
                connect(socket, &QLocalSocket::disconnected, this, [this, socket]() {
                    buffers_.remove(socket);
                    socket->deleteLater();
                });
            }
        });
    }

private:
    void onReadyRead(QLocalSocket *socket)
    {
        auto &buffer = buffers_[socket];
        buffer.append(socket->readAll());
        int index = buffer.indexOf('\n');
        while(index >= 0)
        {
            QByteArray line = buffer.left(index);
            buffer.remove(0, index + 1);
            handleLine(socket, line);
            index = buffer.indexOf('\n');
        }
    }

    void handleLine(QLocalSocket *socket, const QByteArray &line)
    {
        nlohmann::json response;
        try
        {
            const auto request = nlohmann::json::parse(line.constData());
            const auto cmd = request.value("cmd", "");
            if(cmd.empty())
            {
                throw std::runtime_error("Missing cmd");
            }
            response["ok"] = true;
            response["result"] = handleCommand(cmd, request);
            if(request.contains("id"))
            {
                response["id"] = request["id"];
            }
        }
        catch(const std::exception &ex)
        {
            response["ok"] = false;
            response["error"] = ex.what();
        }

        const auto payload = response.dump() + "\n";
        socket->write(payload.c_str(), static_cast<qint64>(payload.size()));
        socket->flush();
    }

    nlohmann::json handleCommand(const std::string &cmd, const nlohmann::json &request)
    {
        using namespace librevna::headless;
        if(cmd == "ping")
        {
            return {{"message", "pong"}};
        }
        if(cmd == "shutdown")
        {
            QCoreApplication::quit();
            return {{"message", "shutting_down"}};
        }
        if(cmd == "list_devices")
        {
            std::string error;
            auto devices = discover_devices(error);
            nlohmann::json device_json = nlohmann::json::array();
            for(const auto &device : devices)
            {
                device_json.push_back({
                    {"vendor_id", device.vendor_id},
                    {"product_id", device.product_id},
                    {"label", device.label},
                    {"serial", device.serial},
                });
            }
            nlohmann::json payload = {
                {"devices", device_json},
                {"real_driver", real_driver_available()},
            };
            if(!error.empty())
            {
                payload["warning"] = error;
            }
            return payload;
        }
        if(cmd == "run_sweep")
        {
            CLIOptions options;
            const auto cal_path = request.value("cal_path", "");
            if(cal_path.empty())
            {
                throw std::runtime_error("cal_path is required");
            }
            if(!std::filesystem::exists(cal_path))
            {
                throw std::runtime_error("Calibration file not found");
            }
            options.cal_path = cal_path;
            if(request.contains("serial") && request["serial"].is_string())
            {
                options.serial = request["serial"].get<std::string>();
            }
            options.f_start_hz = request.value("f_start_hz", 0.0);
            options.f_stop_hz = request.value("f_stop_hz", 0.0);
            options.points = request.value("points", 0);
            options.ifbw_hz = request.value("ifbw_hz", 1000.0);
            options.power_dbm = request.value("power_dbm", -10.0);
            options.threshold_db = request.value("threshold_db", -10.0);
            options.timeout_ms = request.value("timeout_ms", 15000.0);

            const auto requested_ports = parse_ports(request.value("excited_ports", nlohmann::json::array()));
            if(!requested_ports.empty())
            {
                options.excited_ports = requested_ports;
            }

            HostCore host;
            if(!host.connect(options.serial.value_or("")))
            {
                throw std::runtime_error(host.last_error_message());
            }
            if(!host.load_calibration(std::filesystem::path(options.cal_path)))
            {
                host.disconnect();
                throw std::runtime_error(host.last_error_message());
            }

            SweepConfiguration config;
            config.start_frequency_hz = options.f_start_hz;
            config.stop_frequency_hz = options.f_stop_hz;
            config.points = options.points;
            config.if_bandwidth_hz = options.ifbw_hz;
            config.power_dbm = options.power_dbm;
            config.timeout_ms = options.timeout_ms;
            config.excited_ports = options.excited_ports;

            const auto measurements = host.run_sweep(config);
            host.disconnect();

            const auto summary = evaluate_sweep(options, measurements);
            auto output = build_json_payload(options, summary, measurements);
            return output.payload;
        }

        throw std::runtime_error("Unknown command");
    }

    QLocalServer *server_;
    QHash<QLocalSocket *, QByteArray> buffers_;
};

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    const QString name = QString::fromLocal8Bit(qgetenv("LIBREVNA_IPC_NAME"));
    const QString server_name = name.isEmpty() ? QStringLiteral("librevna-ipc") : name;
    IpcServer server(server_name);
    return app.exec();
}
