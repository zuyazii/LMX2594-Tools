#include "usbinbuffer.h"

#include <mutex>
#include <cstdio>

#include <QDebug>

using namespace std;

static bool usbinbuffer_debug_enabled()
{
    static int cached = -1;
    if (cached == -1) {
        const char *val = std::getenv("LIBREVNA_IPC_DEBUG");
        cached = (val && (strcmp(val, "1") == 0 || strcmp(val, "true") == 0)) ? 1 : 0;
    }
    return cached == 1;
}

static void usbinbuffer_dbg(const char *msg)
{
    if (!usbinbuffer_debug_enabled()) return;
    fprintf(stderr, "[USBInBuffer] %s\n", msg);
    fflush(stderr);
}

USBInBuffer::USBInBuffer(libusb_device_handle *handle, unsigned char endpoint, int buffer_size) :
    buffer_size(buffer_size),
    received_size(0),
    inCallback(false),
    cancelling(false)
{
    usbinbuffer_dbg("USBInBuffer constructor start");
    buffer = new unsigned char[buffer_size];
    memset(buffer, 0, buffer_size);
    transfer = libusb_alloc_transfer(0);
    libusb_fill_bulk_transfer(transfer, handle, endpoint, buffer, buffer_size, CallbackTrampoline, this, 0);
    int ret = libusb_submit_transfer(transfer);
    if (usbinbuffer_debug_enabled()) {
        char msg[256];
        snprintf(msg, sizeof(msg), "USBInBuffer constructor: endpoint=0x%02x submit_ret=%d", endpoint, ret);
        usbinbuffer_dbg(msg);
    }
}

USBInBuffer::~USBInBuffer()
{
    if(transfer) {
        cancelling = true;
        libusb_cancel_transfer(transfer);
        // wait for cancellation to complete
        mutex mtx;
        unique_lock<mutex> lck(mtx);
        using namespace std::chrono_literals;
        if(cv.wait_for(lck, 100ms) == cv_status::timeout) {
            qWarning() << "Timed out waiting for mutex acquisition during disconnect";
        }
    }
    delete[] buffer;
}

void USBInBuffer::removeBytes(int handled_bytes)
{
    if(!inCallback) {
        throw runtime_error("Removing of bytes is only allowed from within receive callback");
    }
    if(handled_bytes >= received_size) {
        received_size = 0;
    } else {
        // not removing all bytes, have to move remaining data to the beginning of the buffer
        memmove(buffer, &buffer[handled_bytes], received_size - handled_bytes);
        received_size -= handled_bytes;
    }
}

int USBInBuffer::getReceived() const
{
    return received_size;
}

void USBInBuffer::Callback(libusb_transfer *transfer)
{
    if (usbinbuffer_debug_enabled()) {
        char msg[256];
        snprintf(msg, sizeof(msg), "Callback: status=%d actual_length=%d cancelling=%d",
                 transfer->status, transfer->actual_length, cancelling ? 1 : 0);
        usbinbuffer_dbg(msg);
    }
    if(cancelling || (transfer->status == LIBUSB_TRANSFER_CANCELLED)) {
        // destructor called, do not resubmit
        libusb_free_transfer(transfer);
        this->transfer = nullptr;
        cv.notify_all();
        return;
    }
    switch(transfer->status) {
    case LIBUSB_TRANSFER_COMPLETED:
    case LIBUSB_TRANSFER_TIMED_OUT:
        if(transfer->actual_length > 0) {
            received_size += transfer->actual_length;
            inCallback = true;
            // Use direct callback if set (bypasses Qt signal system for non-Qt threads)
            if (directCallback) {
                usbinbuffer_dbg("Callback: calling directCallback");
                directCallback();
                usbinbuffer_dbg("Callback: directCallback returned");
            } else {
                usbinbuffer_dbg("Callback: emitting DataReceived");
                emit DataReceived();
                usbinbuffer_dbg("Callback: DataReceived emitted");
            }
            inCallback = false;
        }
        break;
    case LIBUSB_TRANSFER_NO_DEVICE:
        qCritical() << "LIBUSB_TRANSFER_NO_DEVICE";
        libusb_free_transfer(transfer);
        return;
    case LIBUSB_TRANSFER_ERROR:
    case LIBUSB_TRANSFER_OVERFLOW:
    case LIBUSB_TRANSFER_STALL:
        qCritical() << "LIBUSB_ERROR" << transfer->status;
        libusb_free_transfer(transfer);
        this->transfer = nullptr;
        emit TransferError();
        return;
        break;
    case LIBUSB_TRANSFER_CANCELLED:
        // already handled before switch-case
        break;
    }
    // Resubmit the transfer
    transfer->buffer = &buffer[received_size];
    transfer->length = buffer_size - received_size;
    transfer->length = (transfer->length / 512) * 512;
    int ret = libusb_submit_transfer(transfer);
    if (usbinbuffer_debug_enabled()) {
        char msg[256];
        snprintf(msg, sizeof(msg), "Callback: resubmit ret=%d", ret);
        usbinbuffer_dbg(msg);
    }
}

void USBInBuffer::CallbackTrampoline(libusb_transfer *transfer)
{
    usbinbuffer_dbg("CallbackTrampoline called");
    auto usb = (USBInBuffer*) transfer->user_data;
    usb->Callback(transfer);
}

uint8_t *USBInBuffer::getBuffer() const
{
    return buffer;
}
