# LMX2594 Frequency Generator

Python scripts to program the LMX2594 frequency synthesizer via USB2ANY SPI interface to generate continuous of a specified frequency output with automatic lock detection and VCO recalibration.

## Hardware Requirements

- **LMX2594 Evaluation Board** with proper power supply (5V)
- **USB2ANY Programmer** for SPI communication
- **50MHz Reference Oscillator** (default) or custom reference
- **Spectrum Analyzer** (optional, for verification) - tinySA Ultra recommended

## Hardware Connections

Using a 10-pin SPI wire to connect the USB2ANY and the LMX2594, and connect the LMX2594 with a +5v power supply:

```
USB2ANY <-> LMX2594 Module
-------------------------
PA3      <-> LMX2594-CLK   (SPI Clock)
PA6      <-> LMX2594-LE    (SPI Latch Enable / CS)
PA2      <-> LMX2594-DATA  (SPI MOSI)
PA1      <-> LMX2594-MUX   (Lock Detect output from MUXout)
PA7      <-> LMX2594-CE    (Chip Enable)

Power Connections:
5V       <-> LMX2594 +5V
GND      <-> LMX2594 GND

Reference Input:
50MHz    <-> LMX2594 FINC/FDEC (Reference Clock Input, usually on-board)
```

## Installation

1. Ensure USB2ANY.dll is available (located in root directory)
2. Install 32-bit Python 3.6+ if not already installed, creating a venv is suggested

## Usage

### Basic Usage

Generate 3.6 GHz continuous output:

```bash
python scripts/frequency_generator.py --freq 3.6GHz
```

### Advanced Usage

```bash
# Custom reference oscillator and PFD frequency
python scripts/frequency_generator.py --freq 2400MHz --fosc 50MHz --pfd 25MHz

# Debug mode with detailed output
python frequency_generator.py --freq 3.6GHz --debug

# Dry run to see computed values without programming
python frequency_generator.py --freq 3.6GHz --dry-run

# Disable auto-recalibration
python frequency_generator.py --freq 3.6GHz --no-auto-recal

# Custom timeouts and monitoring intervals
python frequency_generator.py --freq 3.6GHz --lock-timeout 10.0 --monitor-interval 0.5

# Force programming even if verification fails
python frequency_generator.py --freq 3.6GHz --force
```

### Command Line Options

- `--freq, -f`: Target output frequency (required)

  - Supports units: Hz, kHz, MHz, GHz (case insensitive)
  - Examples: `3.6GHz`, `2400MHz`, `1000000000`

- `--fosc`: Reference oscillator frequency (default: 50MHz)

- `--pfd`: Target PFD frequency (default: 50MHz)

- `--serial`: USB2ANY serial number (if multiple devices)

- `--dry-run`: Show computed values without programming device

- `--debug`: Enable detailed debug output

- `--auto-recal`: Enable automatic recalibration (default: enabled)

- `--no-auto-recal`: Disable automatic recalibration

- `--force`: Continue even if register verification fails

- `--lock-timeout`: Initial lock detection timeout in seconds (default: 5.0)

- `--monitor-interval`: Lock monitoring interval in seconds (default: 1.0)

## Frequency Planning Algorithm

The scripts implement the LMX2594 frequency calculation with these constraints:

- **VCO Range**: 7.5 - 15 GHz
- **PFD Limits**: Integer mode ≤400MHz, Fractional mode ≤300MHz
- **Channel Dividers**: Only values 2, 4, 6, 8, 12, 16, 24, 32, 48, 64, 96, 128, 192
- **N Minimum**: N ≥28 when VCO >12.8 GHz

The algorithm automatically selects the optimal channel divider to place VCO frequency within valid range.

## Programming Sequence

### Initial Programming

1. Write R0 with RESET=1
2. Write R0 with RESET=0
3. Write registers R112→R1 (reverse order)
4. Write R0 with FCAL_EN=1
5. Wait ≥20 µs

### Re-calibration (on unlock)

1. Write R0 with FCAL_EN=1
2. Wait ≥20 µs
3. Check lock status

## Lock Detection

- MUXout pin is configured to output lock detect signal
- Lock status is continuously monitored via GPIO read
- Automatic recalibration triggers on unlock detection
- Maximum 10 recalibration attempts before failure

## Troubleshooting

### Testing Without Hardware

The scripts include testing options to verify functionality without USB2ANY hardware:

- `--test-imports`: Verify all Python modules can be imported
- `--test-registers`: Test frequency calculation and register generation
- `--mock-hardware`: Run full programming sequence with simulated hardware

### Common Issues

1. **"ImportError: attempted relative import with no known parent package"**

   - Run the script from the LMX2594 project root directory, not from within the scripts folder
   - Use: `python scripts/frequency_generator.py` instead of `cd scripts && python frequency_generator.py`

2. **"No USB2ANY controllers found"**

   - Check USB connection to USB2ANY device
   - Ensure USB2ANY drivers are installed
   - Try different USB port

3. **"SPI write/read error"**

   - Verify SPI connections (CLK, DATA, LE/CS)
   - Check LMX2594 power (red LED should be on)
   - Confirm SPI mode 0 configuration

4. **"Device failed to lock"**

   - Check reference oscillator (50MHz default)
   - Verify frequency calculation constraints
   - Check spectrum analyzer for actual output
   - Try different PFD frequency

5. **"Register write verification failed"**
   - Check SPI connections and timing
   - Use `--force` to bypass verification
   - Check for LMX2594 power issues

### Debug Mode

Use `--debug` flag for detailed output including:

- Register write operations and read-back values
- Lock status polling
- Recalibration attempts
- Timing information

### Verification

Compare with TICS Pro settings in `examples/3600_Blue_LED_ON.tcs` and spectrum capture in `examples/3600_Blue_LED_ON_result.json` for validation.

## Files

- `usb2anyapi.py`: USB2ANY SPI interface wrapper
- `lmx2594.py`: LMX2594 register driver and frequency calculation
- `frequency_generator.py`: Main CLI application
- `requirements.txt`: Python dependencies (minimal)
- `README.md`: This documentation

## Safety Notes

- Ensure proper RF shielding when using high frequencies
- Verify power supply voltage and current limits
- Use appropriate SMA connectors and cables for RF output
- Monitor device temperature during operation
