"""
USB2ANY SPI Interface for LMX2594 Frequency Synthesizer

Adapted from examples/kekelele-pyusb2any/usb2anyapi.py
Provides SPI communication functions for LMX2594 programming via USB2ANY adapter.
"""

import ctypes as ct
from enum import IntEnum
import time
import os

# API Version V2.8.2
OPEN_ERROR = False
handle = 0
dll = None

def _load_dll():
    """Load USB2ANY.dll on demand"""
    global dll
    if dll is not None:
        return dll

    try:
        # Try loading from current directory first, then parent directory
        dll_path = 'USB2ANY.dll'
        if not os.path.exists(dll_path):
            # Try parent directory
            parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            dll_path = os.path.join(parent_dir, 'USB2ANY.dll')

        dll = ct.windll.LoadLibrary(dll_path)
        return dll
    except Exception as e:
        raise RuntimeError(f"Failed to load USB2ANY.dll: {e}\nMake sure USB2ANY.dll is in the project root directory")

ErrorCodes = {
    0: "No error",
    -1: "Receiver overflowed",
    -2: "Receive buffer is empty",
    -3: "Transmit buffer is full",
    -4: "Transmit is stalled",
    -5: "Transmit failed",
    -6: "Failed to open communications port",
    -7: "Communications port is not open",
    -8: "Communications port is open",
    -9: "Receive timeout",
    -10: "Communications port read error",
    -11: "Communications port write error",
    -12: "Communications device not found",
    -13: "Communications CRC failed",
    -20: "Invalid port",
    -21: "Address is out of accepted range",
    -22: "Invalid function code",
    -23: "Invalid packet size",
    -24: "Invalid handle",
    -25: "Operation failed",
    -26: "Parameter is out of range",
    -27: "Packet is out of sequence",
    -28: "Invalid packet header",
    -29: "Function not implemented",
    -30: "Too much data",
    -31: "Invalid device",
    -32: "Unsupported firmware version",
    -33: "Buffer is too small",
    -34: "No data available",
    -35: "Resource conflict",
    -36: "EVM is required for external power",
    -37: "Command is busy",
    -38: "Adjustable power supply failure",
    -39: "Interface or mode is not enabled",
    -40: "I2C initialization failed",
    -41: "I2C read error",
    -42: "I2C write error",
    -43: "I2C busy (transfer is pending)",
    -44: "Address not acknowledged (NAK)",
    -45: "Data not acknowledged (NAK)",
    -46: "Read timeout",
    -47: "Read data timeout",
    -48: "Timeout waiting for read complete",
    -49: "Write timeout",
    -50: "Write data timeout",
    -51: "Timeout waiting for write complete",
    -52: "I2C not in Master mode",
    -53: "I2C arbitration lost",
    -54: "I2C pullups require the 3.3V EXT power to be on",
    -60: "SPI initialization failed",
    -61: "SPI write/read error",
    -70: "Data write error",
    -71: "Data read error",
    -72: "Operation timeout",
    -73: "Data CRC failed"
}

def openError():
    global OPEN_ERROR
    OPEN_ERROR = True

def printret(ret):
    if ret < 0 and OPEN_ERROR:
        print("Error Code=%d, %s" % (ret, ErrorCodes.get(ret, "Unknown error")))

# Controller functions
def u2aFindControllers():
    dll = _load_dll()
    ret = dll.u2aFindControllers()
    printret(ret)
    return ret

def u2aOpen(SerialNumber):
    dll = _load_dll()
    dll.u2aOpen.argtypes = [ct.c_char_p]
    ret = dll.u2aOpen(SerialNumber.encode('UTF-8'))
    printret(ret)
    return ret

def u2aClose(handle):
    dll = _load_dll()
    dll.u2aClose.argtypes = [ct.c_long]
    ret = dll.u2aClose(handle)
    printret(ret)
    return ret

def GetSerialNumberFromHandle(handle):
    dll = _load_dll()
    dll.GetSerialNumberFromHandle.argtypes = [ct.c_long, ct.c_char_p, ct.c_int]
    szSerialNumber = ct.create_string_buffer(80)
    ret = dll.GetSerialNumberFromHandle(handle, szSerialNumber, 80)
    printret(ret)
    return ret, szSerialNumber.value

# SPI functions

class SPI_ClockPhase(IntEnum):
    SPI_Capture_On_Trailing_Edge = 0
    SPI_Change_On_First_Edge = 0
    SPI_Capture_On_Leading_Edge = 1
    SPI_Change_On_Following_Edge = 1

class SPI_ClockPolarity(IntEnum):
    SPI_Inactive_State_Low = 0
    SPI_Inactive_State_High = 1

class SPI_BitDirection(IntEnum):
    SPI_LSB_First = 0
    SPI_MSB_First = 1

class SPI_CharacterLength(IntEnum):
    SPI__8_Bit = 0
    SPI__7_Bit = 1

class SPI_ChipSelectType(IntEnum):
    SPI_With_Every_Byte = 0
    SPI_With_Every_Packet = 1
    SPI_With_Every_Word = 2
    SPI_No_CS = 3
    SPI_MOSI_High_When_Idle = 4
    SPI_Pulse_After_Packet = 255

class SPI_ChipSelectPolarity(IntEnum):
    SPI_CS_Active_Low = 0
    SPI_CS_Active_High = 1

def u2aSPI_Control(handle, _SPI_ClockPhase, _SPI_ClockPolarity, _SPI_BitDirection,
                   _SPI_CharacterLength, _SPI_CSType, _SPI_CSPolarity, _DividerHigh, _DividerLow):
    dll = _load_dll()
    dll.u2aSPI_Control.argtypes = [ct.c_long, ct.c_int, ct.c_int, ct.c_int, ct.c_int, ct.c_int, ct.c_int, ct.c_ubyte, ct.c_ubyte]
    ret = dll.u2aSPI_Control(handle, _SPI_ClockPhase, _SPI_ClockPolarity, _SPI_BitDirection,
                           _SPI_CharacterLength, _SPI_CSType, _SPI_CSPolarity, _DividerHigh, _DividerLow)
    printret(ret)
    return ret

def u2aSPI_WriteAndRead(handle, nBytes, Data):
    dll = _load_dll()
    dll.u2aSPI_WriteAndRead.argtypes = [ct.c_long, ct.c_ubyte, ct.c_char_p]
    tempData = ct.create_string_buffer(nBytes)
    tempData.raw = Data[:nBytes]
    ret = dll.u2aSPI_WriteAndRead(handle, nBytes, tempData)
    printret(ret)
    return ret, tempData.raw

# GPIO functions (for reading lock detect from MUXout pin)
def u2aDigital_Read(handle, pin):
    """Read digital pin state"""
    dll = _load_dll()
    dll.u2aDigital_Read.argtypes = [ct.c_long, ct.c_ubyte]
    ret = dll.u2aDigital_Read(handle, pin)
    printret(ret)
    return ret

def u2aDigital_Write(handle, pin, value):
    """Write digital pin state"""
    dll = _load_dll()
    dll.u2aDigital_Write.argtypes = [ct.c_long, ct.c_ubyte, ct.c_ubyte]
    ret = dll.u2aDigital_Write(handle, pin, value)
    printret(ret)
    return ret

def u2aPower_Enable(handle, enable, vccio, vddio):
    """Enable/disable power supplies"""
    dll = _load_dll()
    dll.u2aPower_Enable.argtypes = [ct.c_long, ct.c_ubyte, ct.c_ubyte, ct.c_ubyte]
    ret = dll.u2aPower_Enable(handle, enable, vccio, vddio)
    printret(ret)
    return ret

def GetSerialNumberFromHandle(index):
    """Get serial number from device index"""
    dll = _load_dll()
    serial_buffer = ct.create_string_buffer(256)
    dll.GetSerialNumberFromHandle.argtypes = [ct.c_int, ct.c_char_p]
    ret = dll.GetSerialNumberFromHandle(index, serial_buffer)
    return ret, serial_buffer.value

# High-level interface for LMX2594 SPI communication
class USB2ANYInterface:
    def __init__(self, serial_number=None):
        self.handle = None
        self.serial_number = serial_number
        self.connected = False
        self.CE_PIN = 7

    def connect(self):
        # Find device
        count = u2aFindControllers()
        if count <= 0:
            raise RuntimeError("No USB2ANY devices found")

        # Open device
        if self.serial_number:
            self.handle = u2aOpen(self.serial_number)
        else:
            self.handle = u2aOpen("")  # Open first available device

        if self.handle < 0:
            raise RuntimeError(f"Failed to open USB2ANY: {self.handle}")

        self.connected = True

        # Try to enable power (may not be available in all DLL versions)
        try:
            ret = u2aPower_Enable(self.handle, 1, 0, 0)
            if ret >= 0:
                # Wait for power to stabilize
                time.sleep(0.1)  # 100ms
            else:
                print("Warning: Could not enable power (function may not be available)")
        except Exception as e:
            print(f"Warning: Power enable failed: {e} (continuing without power control)")

        # Configure SPI
        self.configure_spi()

        # Try to enable CE pin (may not be available in all DLL versions)
        try:
            ret = self.write_gpio(self.CE_PIN, 1)
            if ret < 0:
                print("Warning: Could not enable CE pin (GPIO functions may not be available)")
        except Exception as e:
            print(f"Warning: CE pin control failed: {e} (continuing without CE control)")

        print(f"Connected to USB2ANY (handle={self.handle})")


    def disconnect(self):
        """Disconnect from USB2ANY device"""
        if self.connected and self.handle is not None:
            u2aClose(self.handle)
            self.connected = False

    def configure_spi(self, clock_freq=400000):
        """
        Configure SPI for LMX2594 communication

        Based on TI E2E forum example:
        - SPI capture on leading edge (CPHA=1)
        - SPI inactive state low (CPOL=0)
        - MSB first
        - 8-bit (but sending 24-bit packets)
        - CS with every packet
        - CS active low
        - 400 kHz clock
        """
        if not self.connected:
            raise RuntimeError("Not connected to USB2ANY device")

        # Try SPI Mode 0: CPOL=0, CPHA=0 (standard SPI mode)
        # LMX2594 datasheet may specify Mode 0 instead of Mode 1
        clock_phase = SPI_ClockPhase.SPI_Capture_On_Trailing_Edge  # 0
        clock_polarity = SPI_ClockPolarity.SPI_Inactive_State_Low  # 0

        # MSB first, 8-bit, CS with every packet, CS active low
        bit_direction = SPI_BitDirection.SPI_MSB_First
        char_length = SPI_CharacterLength.SPI__8_Bit
        cs_type = SPI_ChipSelectType.SPI_With_Every_Packet
        cs_polarity = SPI_ChipSelectPolarity.SPI_CS_Active_Low

        # Calculate divider for bit rate (TI example method)
        bitRateKbps = clock_freq // 1000
        divider = int(24000 / bitRateKbps)
        dividerHigh = (divider >> 8) & 0xFF
        dividerLow = (divider & 0xFF)

        ret = u2aSPI_Control(self.handle, clock_phase, clock_polarity,
                           bit_direction, char_length, cs_type, cs_polarity,
                           dividerHigh, dividerLow)
        if ret < 0:
            raise RuntimeError(f"Failed to configure SPI: {ret}")

        print(f"SPI configured: {bitRateKbps} kbps clock, Mode 0 (CPOL=0, CPHA=0)")

    def spi_write_read(self, data):
        """
        Write data to SPI and read response

        Args:
            data: bytes to write

        Returns:
            tuple: (return_code, response_bytes)
        """
        if not self.connected:
            raise RuntimeError("Not connected to USB2ANY device")

        return u2aSPI_WriteAndRead(self.handle, len(data), data)

    def read_gpio(self, pin):
        """Read GPIO pin state"""
        if not self.connected:
            raise RuntimeError("Not connected to USB2ANY device")

        return u2aDigital_Read(self.handle, pin)

    def write_gpio(self, pin, value):
        """Write GPIO pin state"""
        if not self.connected:
            raise RuntimeError("Not connected to USB2ANY device")

        return u2aDigital_Write(self.handle, pin, value)

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
