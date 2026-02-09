#include "librevnausbdriver.h"

#include "CustomWidgets/informationbox.h"
#include "devicepacketlog.h"
#include "Util/usbinbuffer.h"

#include <QTimer>
#include <QThread>
#include <QByteArray>
#include <QMetaObject>
#include <QTextStream>
#include <cstring>
#include <cstdio>
#ifdef _WIN32
#include <windows.h>
#endif

using namespace std;

using USBID = struct {
    int VID;
    int PID;
};
static constexpr USBID IDs[] = {
    {0x0483, 0x564e},
    {0x0483, 0x4121},
    {0x1209, 0x4121},
};

static int bulkTimeoutMs()
{
    static int cached = -2;
    if (cached != -2) {
        return cached;
    }
    bool ok = false;
    const int value = qEnvironmentVariableIntValue("LIBREVNA_USB_BULK_TIMEOUT_MS", &ok);
    if (!ok || value < 0) {
        cached = 3000;
    } else {
        cached = value;
    }
    QTextStream(stderr) << "LibreVNA USB bulk timeout set to " << cached << " ms\n";
    return cached;
}

static bool debug_enabled()
{
    const QString value = qEnvironmentVariable("LIBREVNA_IPC_DEBUG").trimmed().toLower();
    return value == "1" || value == "true" || value == "yes";
}

static void dbg(const QString &message)
{
    if (!debug_enabled()) {
        return;
    }
    QTextStream(stderr) << message << "\n";
}

// Raw fprintf debug for use from non-Qt threads
static void raw_dbg(const char *msg)
{
    if (!debug_enabled()) {
        return;
    }
    fprintf(stderr, "[USBDriver] %s\n", msg);
    fflush(stderr);
}

static bool packet_log_enabled()
{
    const QString value = qEnvironmentVariable("LIBREVNA_DISABLE_PACKET_LOG").trimmed().toLower();
    if (value == "1" || value == "true" || value == "yes") {
        return false;
    }
    return true;
}

namespace {
struct OutTransferContext {
    LibreVNAUSBDriver *driver = nullptr;
    unsigned char *buffer = nullptr;
    int expected_length = 0;
};

void LIBUSB_CALL OutTransferCallbackTrampoline(libusb_transfer *transfer)
{
    raw_dbg("OutTransferCallbackTrampoline called");
    auto *ctx = static_cast<OutTransferContext *>(transfer->user_data);
    if (!ctx || !ctx->driver) {
        raw_dbg("OutTransferCallbackTrampoline: invalid context");
        if (transfer) {
            libusb_free_transfer(transfer);
        }
        return;
    }
    LibreVNAUSBDriver *driver = ctx->driver;
    const int status = transfer->status;
    const int actual = transfer->actual_length;
    const int expected = ctx->expected_length;
    if (debug_enabled()) {
        char msg[256];
        snprintf(msg, sizeof(msg), "OutTransferCallback: status=%d actual=%d expected=%d", status, actual, expected);
        raw_dbg(msg);
    }
    if (ctx->buffer) {
        delete[] ctx->buffer;
        ctx->buffer = nullptr;
    }
    delete ctx;
    libusb_free_transfer(transfer);
    raw_dbg("OutTransferCallback: invoking handleOutTransferResult");
    QMetaObject::invokeMethod(driver, "handleOutTransferResult", Qt::QueuedConnection,
                              Q_ARG(int, status), Q_ARG(int, actual), Q_ARG(int, expected));
}
} // namespace

LibreVNAUSBDriver::LibreVNAUSBDriver()
    : LibreVNADriver()
{
    connected = false;
    m_handle = nullptr;
    m_context = nullptr;
    dataBuffer = nullptr;
    logBuffer = nullptr;
    m_receiveThread = nullptr;
    lastTimestamp = QDateTime::currentDateTime();
    byteCnt = 0;

    specificSettings.push_back(Savable::SettingDescription(&captureRawReceiverValues, "LibreVNAUSBDriver.captureRawReceiverValues", false));
    specificSettings.push_back(Savable::SettingDescription(&harmonicMixing, "LibreVNAUSBDriver.harmonicMixing", false));
    specificSettings.push_back(Savable::SettingDescription(&SASignalID, "LibreVNAUSBDriver.signalID", true));
    specificSettings.push_back(Savable::SettingDescription(&VNASuppressInvalidPeaks, "LibreVNAUSBDriver.suppressInvalidPeaks", true));
    specificSettings.push_back(Savable::SettingDescription(&VNAAdjustPowerLevel, "LibreVNAUSBDriver.adjustPowerLevel", false));
    specificSettings.push_back(Savable::SettingDescription(&SAUseDFT, "LibreVNAUSBDriver.useDFT", true));
    specificSettings.push_back(Savable::SettingDescription(&SARBWLimitForDFT, "LibreVNAUSBDriver.RBWlimitDFT", 3000));
}

QString LibreVNAUSBDriver::getDriverName()
{
    return "LibreVNA/USB";
}

std::set<QString> LibreVNAUSBDriver::GetAvailableDevices()
{
    std::set<QString> serials;

    libusb_context *ctx;
    libusb_init(&ctx);
#if LIBUSB_API_VERSION >= 0x01000106
    libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_INFO);
#endif

    SearchDevices([&serials](libusb_device_handle *, QString serial) -> bool {
        serials.insert(serial);
        return true;
    }, ctx, true);

    libusb_exit(ctx);

    return serials;
}

bool LibreVNAUSBDriver::connectTo(QString serial)
{
    if(connected) {
        disconnect();
    }

//    info = defaultInfo;
//    status = {};

    m_handle = nullptr;
//    infoValid = false;
    libusb_init(&m_context);
#if LIBUSB_API_VERSION >= 0x01000106
    libusb_set_option(m_context, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_INFO);
#endif

    SearchDevices([=](libusb_device_handle *handle, QString found_serial) -> bool {
        if(serial.isEmpty() || serial == found_serial) {
            // accept connection to this device
            this->serial = found_serial;
            m_handle = handle;
            // abort device search
            return false;
        } else {
            // not the requested device, continue search
            return true;
        }
    }, m_context, false);

    if(!m_handle) {
        QString message =  "No device found";
        InformationBox::ShowError("Error opening device", message);
        libusb_exit(m_context);
        throw std::runtime_error(message.toStdString());
    }

    // Found the correct device, now connect
    /* claim the interface */
    int ret = libusb_claim_interface(m_handle, 0);
    if (ret < 0) {
        libusb_close(m_handle);
        /* Failed to open */
        QString message =  "Failed to claim interface: \"";
        message.append(libusb_strerror((libusb_error) ret));
        message.append("\" Maybe you are already connected to this device?");
        qWarning() << message;
        InformationBox::ShowError("Error opening device", message);
        libusb_exit(m_context);
        throw std::runtime_error(message.toStdString());
    }
    qInfo() << "USB connection established" << Qt::flush;
    connected = true;
    {
        lock_guard<mutex> lock(transmissionMutex);
        transmissionQueue.clear();
        transmissionActive = false;
    }
    dbg(QStringLiteral("transmission reset active=%1").arg(transmissionActive ? "true" : "false"));
    // Create buffers and connect signals BEFORE starting the USB thread.
    // USBInBuffer constructors submit async transfers via libusb_submit_transfer(),
    // but they won't be processed until someone calls libusb_handle_events().
    // By connecting signals first, we ensure no callbacks are missed.
    dataBuffer = new USBInBuffer(m_handle, EP_Data_In_Addr, 65536);
    logBuffer = new USBInBuffer(m_handle, EP_Log_In_Addr, 65536);
    // Use direct callbacks instead of signals to avoid Qt's signal/slot issues
    // when emitting from non-Qt threads (libusb callback runs in USB thread)
    dataBuffer->setDirectCallback([this]() { ReceivedData(); });
    logBuffer->setDirectCallback([this]() { ReceivedLog(); });
    connect(dataBuffer, &USBInBuffer::TransferError, this, &LibreVNAUSBDriver::ConnectionLost);
    connect(&transmissionTimer, &QTimer::timeout, this, &LibreVNAUSBDriver::transmissionTimeout, Qt::UniqueConnection);
    connect(this, &LibreVNAUSBDriver::receivedAnswer, this, &LibreVNAUSBDriver::transmissionFinished, static_cast<Qt::ConnectionType>(Qt::QueuedConnection | Qt::UniqueConnection));
    connect(this, &LibreVNAUSBDriver::receivedPacket, this, &LibreVNAUSBDriver::handleReceivedPacket, static_cast<Qt::ConnectionType>(Qt::QueuedConnection | Qt::UniqueConnection));
    transmissionTimer.setSingleShot(true);
    // Now start the USB event handling thread. It will call libusb_handle_events()
    // which processes the pending transfers and invokes callbacks with signals connected.
    m_receiveThread = new std::thread(&LibreVNAUSBDriver::USBHandleThread, this);

    dbg("sending RequestDeviceInfo");
    sendWithoutPayload(Protocol::PacketType::RequestDeviceInfo);
    dbg("sending RequestDeviceStatus");
    sendWithoutPayload(Protocol::PacketType::RequestDeviceStatus);
    dbg("connectTo returning true");
//    updateIFFrequencies();
    return true;
}

void LibreVNAUSBDriver::disconnect()
{
    if(connected) {
        setIdle();
        delete dataBuffer;
        delete logBuffer;
        connected = false;
        serial = "";
        for (int if_num = 0; if_num < 1; if_num++) {
            int ret = libusb_release_interface(m_handle, if_num);
            if (ret < 0) {
                qCritical() << "Error releasing interface" << libusb_error_name(ret);
            }
        }
        libusb_release_interface(m_handle, 0);
        libusb_close(m_handle);
        m_receiveThread->join();
        libusb_exit(m_context);
        delete m_receiveThread;
        m_handle = nullptr;
        m_context = nullptr;
        m_receiveThread = nullptr;
        dataBuffer = nullptr;
        logBuffer = nullptr;
    }
}

void LibreVNAUSBDriver::ReceivedData()
{
    if (debug_enabled()) {
        char msg[256];
        snprintf(msg, sizeof(msg), "ReceivedData() called, bytes available: %d", dataBuffer->getReceived());
        raw_dbg(msg);
    }
    Protocol::PacketInfo packet;
    uint16_t handled_len;
//    qDebug() << "Received data";
    do {
        // qDebug() << "Decoding" << dataBuffer->getReceived() << "Bytes";
        handled_len = Protocol::DecodeBuffer(dataBuffer->getBuffer(), dataBuffer->getReceived(), &packet);
        if (debug_enabled()) {
            char msg[256];
            snprintf(msg, sizeof(msg), "Decoded packet type: %d handled_len: %d", (int)packet.type, handled_len);
            raw_dbg(msg);
        }
        if(handled_len > 0 && packet_log_enabled()) {
            auto &log = DevicePacketLog::getInstance();
            if(packet.type != Protocol::PacketType::None) {
                log.addPacket(packet, serial);
            } else {
                log.addInvalidBytes(dataBuffer->getBuffer(), handled_len, serial);
            }
        }
        dataBuffer->removeBytes(handled_len);
        if(packet.type == Protocol::PacketType::SetTrigger) {
            qDebug() << "Incoming set trigger from " << serial;
        }
        if(packet.type == Protocol::PacketType::ClearTrigger) {
            qDebug() << "Incoming clear trigger from " << serial;
        }
        switch(packet.type) {
        case Protocol::PacketType::Ack:
            raw_dbg("ReceivedData: Emitting receivedAnswer(Ack)");
            emit receivedAnswer(TransmissionResult::Ack);
            break;
        case Protocol::PacketType::Nack:
            raw_dbg("ReceivedData: Emitting receivedAnswer(Nack)");
            emit receivedAnswer(TransmissionResult::Nack);
            break;
        case Protocol::PacketType::SetTrigger:
            emit receivedTrigger(this, true);
            break;
        case Protocol::PacketType::ClearTrigger:
            emit receivedTrigger(this, false);
            break;
        case Protocol::PacketType::None:
            break;
        default:
            // pass on to LibreVNADriver class
            if (debug_enabled()) {
                char msg[256];
                snprintf(msg, sizeof(msg), "ReceivedData: Emitting receivedPacket for type: %d", (int)packet.type);
                raw_dbg(msg);
            }
            emit receivedPacket(packet);
            break;
        }
        // byteCnt += handled_len;
        // auto now = QDateTime::currentDateTime();
        // if(lastTimestamp.time().msecsTo(now.time()) > 1000) {
        //     lastTimestamp = now;
        //     constexpr unsigned int maxThroughput = 12000000 / 8;
        //     qDebug() << "USB throughput: " << byteCnt << "(" << (double) byteCnt * 100.0 / maxThroughput << "%)";
        //     byteCnt = 0;
        // }
    } while (handled_len > 0);
}

void LibreVNAUSBDriver::ReceivedLog()
{
    uint16_t handled_len;
    do {
        handled_len = 0;
        auto firstLinebreak = (uint8_t*) memchr(logBuffer->getBuffer(), '\n', logBuffer->getReceived());
        if(firstLinebreak) {
            handled_len = firstLinebreak - logBuffer->getBuffer();
            auto line = QString::fromLatin1((const char*) logBuffer->getBuffer(), handled_len - 1);
            emit LogLineReceived(line);
            logBuffer->removeBytes(handled_len + 1);
        }
    } while(handled_len > 0);
}

void LibreVNAUSBDriver::transmissionFinished(LibreVNADriver::TransmissionResult result)
{
    lock_guard<mutex> lock(transmissionMutex);
    // remove transmitted packet
    if(transmissionQueue.empty()) {
        qWarning() << "transmissionFinished with empty transmission queue, stray Ack? Result:" << result;
        return;
    }
    auto t = transmissionQueue.dequeue();
    // qDebug() << "Transmission finsished (packet type" << (int) t.packet.type <<",result" << result << "), queue at " << transmissionQueue.size();
    if(result == TransmissionResult::Timeout) {
        qWarning() << "transmissionFinished with timeout, packettype:" << (int) t.packet.type << "Device:" << serial;
    }
    if(result == TransmissionResult::Nack) {
        qWarning() << "transmissionFinished with NACK";
    }
    if(t.callback) {
        t.callback(result);
    }
    transmissionTimer.stop();
    bool success = false;
    while(!transmissionQueue.isEmpty() && !success) {
        success = startNextTransmission();
        if(!success) {
            // failed to send this packet
            auto t = transmissionQueue.dequeue();
            if(t.callback) {
                t.callback(TransmissionResult::InternalError);
            }
        }
    }
    if(transmissionQueue.isEmpty()) {
        transmissionActive = false;
    }
}

bool LibreVNAUSBDriver::SendPacket(const Protocol::PacketInfo &packet, std::function<void (LibreVNADriver::TransmissionResult)> cb, unsigned int timeout)
{
    Transmission t;
    t.packet = packet;
    t.timeout = timeout;
    t.callback = cb;
    lock_guard<mutex> lock(transmissionMutex);
    transmissionQueue.enqueue(t);
    dbg(QStringLiteral("SendPacket queued type=%1 queue=%2 active=%3")
            .arg(static_cast<int>(packet.type))
            .arg(transmissionQueue.size())
            .arg(transmissionActive ? "true" : "false"));
//    qDebug() << "Enqueued packet, queue at " << transmissionQueue.size();
    if(!transmissionActive) {
        dbg("SendPacket starting transmission");
        startNextTransmission();
    } else {
        dbg("SendPacket did not start transmission (already active)");
    }
    return true;
}

void LibreVNAUSBDriver::USBHandleThread()
{
    raw_dbg("USBHandleThread started");
    int iteration = 0;
    while (connected) {
        timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 10000;
        libusb_lock_events(m_context);
        const int ret = libusb_handle_events_timeout_completed(m_context, &tv, nullptr);
        libusb_unlock_events(m_context);
        if (ret < 0) {
            char msg[256];
            snprintf(msg, sizeof(msg), "USBHandleThread: error %d", ret);
            raw_dbg(msg);
            QThread::msleep(10);
        }
        iteration++;
        if (debug_enabled() && (iteration == 1 || (iteration % 100) == 0)) {
            char msg[256];
            snprintf(msg, sizeof(msg), "USBHandleThread: iteration %d, connected=%d", iteration, connected ? 1 : 0);
            raw_dbg(msg);
        }
    }
    raw_dbg("USBHandleThread exiting");
}

void LibreVNAUSBDriver::handleOutTransferResult(int status, int actual_length, int expected_length)
{
    if (debug_enabled()) {
        char msg[256];
        snprintf(msg, sizeof(msg), "handleOutTransferResult: status=%d actual=%d expected=%d", status, actual_length, expected_length);
        raw_dbg(msg);
    }
    if (status != LIBUSB_TRANSFER_COMPLETED || actual_length != expected_length) {
        qCritical() << "USB OUT transfer failed" << status << "actual" << actual_length << "expected" << expected_length;
        dbg(QStringLiteral("USB OUT transfer failed status=%1 actual=%2 expected=%3")
                .arg(status)
                .arg(actual_length)
                .arg(expected_length));
        Transmission failed;
        bool has_failed = false;
        {
            lock_guard<mutex> lock(transmissionMutex);
            if (!transmissionQueue.isEmpty()) {
                failed = transmissionQueue.dequeue();
                has_failed = true;
            }
        }
        if (has_failed && failed.callback) {
            failed.callback(TransmissionResult::InternalError);
        }
        bool success = false;
        {
            lock_guard<mutex> lock(transmissionMutex);
            while (!transmissionQueue.isEmpty() && !success) {
                success = startNextTransmission();
                if (!success) {
                    auto t = transmissionQueue.dequeue();
                    if (t.callback) {
                        t.callback(TransmissionResult::InternalError);
                    }
                }
            }
            if (transmissionQueue.isEmpty()) {
                transmissionActive = false;
            }
        }
        return;
    }
    raw_dbg("handleOutTransferResult: transfer successful, starting timer");

    dbg(QStringLiteral("USB OUT transfer ok actual=%1 expected=%2")
            .arg(actual_length)
            .arg(expected_length));
    lock_guard<mutex> lock(transmissionMutex);
    if (transmissionQueue.isEmpty()) {
        transmissionActive = false;
        return;
    }
    auto t = transmissionQueue.head();
    transmissionTimer.start(t.timeout);
}

void LibreVNAUSBDriver::SearchDevices(std::function<bool (libusb_device_handle *, QString)> foundCallback, libusb_context *context, bool ignoreOpenError)
{
    libusb_device **devList;
    auto ndevices = libusb_get_device_list(context, &devList);

    for (ssize_t idx = 0; idx < ndevices; idx++) {
        int ret;
        libusb_device *device = devList[idx];
        libusb_device_descriptor desc = {};

        ret = libusb_get_device_descriptor(device, &desc);
        if (ret) {
            /* some error occured */
            qCritical() << "Failed to get device descriptor: "
                    << libusb_strerror((libusb_error) ret);
            continue;
        }

        bool correctID = false;
        int numIDs = sizeof(IDs)/sizeof(IDs[0]);
        for(int i=0;i<numIDs;i++) {
            if(desc.idVendor == IDs[i].VID && desc.idProduct == IDs[i].PID) {
                correctID = true;
                break;
            }
        }
        if(!correctID) {
            continue;
        }

        /* Try to open the device */
        libusb_device_handle *handle = nullptr;
        ret = libusb_open(device, &handle);
        if (ret) {
            qDebug() << libusb_strerror((enum libusb_error) ret);
            /* Failed to open */
            if(!ignoreOpenError) {
                QString message =  "Found potential device but failed to open usb connection: \"";
                message.append(libusb_strerror((libusb_error) ret));
                message.append("\" On Linux this is most likely caused by a missing udev rule. "
                               "On Windows this most likely means that you are already connected to "
                               "this device (is another instance of the application already runnning?)");
                qWarning() << message;
                InformationBox::ShowMessage("Error opening device", message);
            }
            continue;
        }

        char c_product[256];
        char c_serial[256];
        libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber,
                (unsigned char*) c_serial, sizeof(c_serial));
        ret = libusb_get_string_descriptor_ascii(handle, desc.iProduct,
                (unsigned char*) c_product, sizeof(c_product));
        if (ret > 0) {
            /* managed to read the product string */
            QString product(c_product);
            if (product == "VNA") {
                // this is a match
                if(!foundCallback(handle, QString(c_serial))) {
                    // abort search
                    break;
                }
            }
        } else {
            qWarning() << "Failed to get product descriptor: "
                    << libusb_strerror((libusb_error) ret);
        }
        libusb_close(handle);
    }
    libusb_free_device_list(devList, 1);
}

bool LibreVNAUSBDriver::startNextTransmission()
{
    dbg("startNextTransmission entered");
    if(transmissionQueue.isEmpty() || !connected) {
        // nothing more to transmit
        transmissionActive = false;
        return false;
    }
    transmissionActive = true;
    auto t = transmissionQueue.head();
    unsigned char buffer[1024];
    dbg("encode packet start");
    unsigned int length = Protocol::EncodePacket(t.packet, buffer, sizeof(buffer));
    dbg(QStringLiteral("encode packet done len=%1").arg(length));
    if(!length) {
        qCritical() << "Failed to encode packet";
        return false;
    }
    if (packet_log_enabled()) {
        dbg("packet log add start");
        auto &log = DevicePacketLog::getInstance();
        log.addPacket(t.packet);
        dbg("packet log add done");
    } else {
        dbg("packet log disabled");
    }
    const int timeout_ms = bulkTimeoutMs();
    dbg(QStringLiteral("startNextTransmission type=%1 len=%2 timeout=%3")
            .arg(static_cast<int>(t.packet.type))
            .arg(length)
            .arg(timeout_ms));
    auto *ctx = new OutTransferContext();
    ctx->driver = this;
    ctx->expected_length = static_cast<int>(length);
    ctx->buffer = new unsigned char[length];
    memcpy(ctx->buffer, buffer, length);
    libusb_transfer *transfer = libusb_alloc_transfer(0);
    if (!transfer) {
        delete[] ctx->buffer;
        delete ctx;
        qCritical() << "Failed to allocate libusb transfer";
        return false;
    }
    libusb_fill_bulk_transfer(transfer, m_handle, EP_Data_Out_Addr, ctx->buffer, length, OutTransferCallbackTrampoline, ctx, timeout_ms);
    const int submit = libusb_submit_transfer(transfer);
    if (submit < 0) {
        qCritical() << "Error submitting data transfer:" << libusb_strerror((libusb_error) submit);
        dbg(QStringLiteral("libusb_submit_transfer failed ret=%1").arg(submit));
        libusb_free_transfer(transfer);
        delete[] ctx->buffer;
        delete ctx;
        return false;
    }
    dbg(QStringLiteral("USB OUT transfer submitted len=%1").arg(length));
    // qDebug() << "Transmission started (packet type" << (int) t.packet.type << "), queue at " << transmissionQueue.size();
    return true;
}
