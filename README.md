# LMX2594 Frequency Generator

Python scripts to program the LMX2594 frequency synthesizer via USB2ANY SPI interface to generate continuous of a specified frequency output with automatic lock detection and VCO recalibration.

## Hardware Requirements

- **LMX2594 Evaluation Board** with proper power supply (5V)
- **USB2ANY Programmer** for SPI communication
- **50MHz Reference Oscillator** (default) or custom reference
- **Spectrum Analyzer** (optional, for verification) - tinySA Ultra recommended

## Hardware Connections

Using a 10-pin SPI wire to connect the USB2ANY and the LMX2594, and connect the LMX2594 with a +5v power supply. Check official technical documents of the two devices for verification.

## Installation

1. Ensure USB2ANY.dll is available (located in root directory)
2. Install 32-bit Python 3.6+ if not already installed, creating a venv is suggested

## Usage

### Basic Usage

Generate 3.6 GHz continuous output:

```bash
python scripts/frequency_generator.py --freq 3.6GHz
```

### tinySA Antenna Automation

Automate tinySA measurements with the LMX2594 to capture a golden sample and test antennas
against it.

Requirements:

```bash
pip install tsapython pyserial
```

Examples:

```bash
# List available serial ports
python scripts/tinysa_antenna_test.py list-ports

# Capture a golden sample across 3.4-3.6 GHz in 10 MHz steps
python scripts/tinysa_antenna_test.py golden --start-freq 3.4GHz --stop-freq 3.6GHz --step 10MHz \
  --out-json output/golden_sample.json --out-csv output/golden_sample.csv

# Direct capture for LMX->tinySA validation (no pass/fail logic)
python scripts/tinysa_antenna_test.py direct --start-freq 3.4GHz --stop-freq 3.6GHz --step 10MHz \
  --out-json output/direct_capture.json --out-csv output/direct_capture.csv

# Test another antenna with a +/-5 dB tolerance
python scripts/tinysa_antenna_test.py test --golden-json output/golden_sample.json --tolerance-db 5 \
  --out-json output/test_result.json --out-csv output/test_result.csv
```

Notes:
- Use `--port COM#` to specify the tinySA serial port manually.
- Use `--no-lmx` if an external signal source is already configured.
- Adjust `--settle-us` to match the required LMX settling time.
- For `direct` mode, use proper RF attenuation between LMX and tinySA.

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

# Sweep with per-step lock waits and dwell time
python frequency_generator.py --start-freq 3.2GHz --end-freq 3.6GHz --points 200 --wait-lock --dwell-ms 10

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

- `--wait-lock`: In sweep mode, wait for PLL lock after each frequency update

- `--dwell-ms`: In sweep mode, dwell time after each frequency update in milliseconds (default: 0)

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

## Files

- `usb2anyapi.py`: USB2ANY SPI interface wrapper
- `lmx2594.py`: LMX2594 register driver and frequency calculation
- `frequency_generator.py`: Main CLI application
- `README.md`: This documentation

## Safety Notes

- Ensure proper RF shielding when using high frequencies
- Verify power supply voltage and current limits
- Use appropriate SMA connectors and cables for RF output
- Monitor device temperature during operation
