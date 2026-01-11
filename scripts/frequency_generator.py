#!/usr/bin/env python3
"""
LMX2594 Frequency Generator CLI

Command-line tool to program LMX2594 frequency synthesizer via USB2ANY SPI interface.
Generates continuous frequency output with automatic lock detection and VCO recalibration.
"""

import argparse
import sys
import time
import signal
import os

# Add scripts directory to Python path
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)
from usb2anyapi import USB2ANYInterface
from lmx2594 import LMX2594Driver, LMX2594Error, build_registers_from_template

def parse_frequency(freq_str):
    """Parse frequency string with optional units (Hz, kHz, MHz, GHz)"""
    freq_str = freq_str.strip().lower()

    if freq_str.endswith('ghz'):
        return float(freq_str[:-3]) * 1e9
    elif freq_str.endswith('mhz'):
        return float(freq_str[:-3]) * 1e6
    elif freq_str.endswith('khz'):
        return float(freq_str[:-3]) * 1e3
    elif freq_str.endswith('hz'):
        return float(freq_str[:-2])
    else:
        # Assume Hz if no unit specified
        return float(freq_str)

def parse_register_values(path, debug=False):
    """
    Parse register values file with lines like: R112 0x700000
    Returns (register_list, r0_value, reg_map)
    """
    import re
    register_list = []
    reg_map = {}
    r0_value = None

    with open(path, 'r', encoding='utf-8', errors='ignore') as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            match = re.match(r'^R(\d+)\s+0x([0-9A-Fa-f]+)$', line)
            if not match:
                continue
            addr = int(match.group(1))
            full = int(match.group(2), 16)
            data = full & 0xFFFF
            register_list.append((addr, data))
            reg_map[addr] = data
            if addr == 0:
                r0_value = data
            if debug and ((full >> 16) & 0x7F) not in (addr, 0):
                print(f"Warning: R{addr} value 0x{full:06X} has mismatched address field")

    if not register_list:
        raise ValueError(f"No register values found in {path}")

    return register_list, r0_value, reg_map

def compute_pfd_from_registers(f_osc, reg_map):
    """
    Compute fPD using fPD = fOSC * (1 + OSC_2X) * MULT / (PLL_R_PRE * PLL_R)
    """
    osc_2x = (reg_map.get(9, 0) >> 12) & 0x1
    mult = (reg_map.get(10, 0) >> 7) & 0x1F
    pll_r = (reg_map.get(11, 0) >> 4) & 0xFF
    pll_r_pre = reg_map.get(12, 0) & 0xFF

    if mult == 0:
        mult = 1
    if pll_r == 0:
        pll_r = 1
    if pll_r_pre == 0:
        pll_r_pre = 1

    f_pd = f_osc * (1 + osc_2x) * mult / (pll_r_pre * pll_r)
    return f_pd, osc_2x, mult, pll_r_pre, pll_r


def reg_map_to_list(reg_map):
    """Convert a register map dict {addr: data} to list[(addr, data)]"""
    return [(addr, data & 0xFFFF) for addr, data in reg_map.items()]

def main():
    parser = argparse.ArgumentParser(
        description="LMX2594 Frequency Generator via USB2ANY SPI",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate 3.6 GHz with default settings
  python frequency_generator.py --freq 3.6GHz

  # Generate 2.4 GHz with custom reference and PFD
  python frequency_generator.py --freq 2400MHz --fosc 50MHz --pfd 125MHz

  # Dry run to see register values without programming
  python frequency_generator.py --freq 3.6GHz --dry-run

  # Debug mode with detailed output
  python frequency_generator.py --freq 3.6GHz --debug

  # Disable auto-recalibration
  python frequency_generator.py --freq 3.6GHz --no-auto-recal
        """
    )

    parser.add_argument(
        '--freq', '-f',
        required=False,
        help='Target output frequency (e.g., 3.6GHz, 2400MHz, 1000000000)'
    )

    parser.add_argument(
        '--fosc',
        default='50MHz',
        help='Reference oscillator frequency (default: 50MHz)'
    )

    parser.add_argument(
        '--pfd',
        default='50MHz',
        help='Target PFD frequency (default: 50MHz)'
    )

    parser.add_argument(
        '--serial',
        help='USB2ANY serial number (if multiple devices connected)'
    )

    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Show computed register values without programming device'
    )

    parser.add_argument(
        '--debug',
        action='store_true',
        help='Enable debug output'
    )

    parser.add_argument(
        '--readback',
        action='store_true',
        help='Enable SPI readback checks (requires MUXout wired for readback)'
    )

    parser.add_argument(
        '--auto-recal',
        action='store_true',
        default=True,
        help='Enable automatic VCO recalibration on unlock (default: enabled)'
    )

    parser.add_argument(
        '--no-auto-recal',
        action='store_false',
        dest='auto_recal',
        help='Disable automatic VCO recalibration'
    )

    parser.add_argument(
        '--force',
        action='store_true',
        help='Force programming even if register verification fails'
    )

    parser.add_argument(
        '--lock-timeout',
        type=float,
        default=5.0,
        help='Timeout for initial lock detection (seconds, default: 5.0)'
    )

    parser.add_argument(
        '--monitor-interval',
        type=float,
        default=1.0,
        help='Lock monitoring interval (seconds, default: 1.0)'
    )

    parser.add_argument(
        '--test-imports',
        action='store_true',
        help='Test that all imports work correctly and exit'
    )

    parser.add_argument(
        '--test-registers',
        action='store_true',
        help='Test register list generation without hardware and exit'
    )

    parser.add_argument(
        '--mock-hardware',
        action='store_true',
        help='Use mock hardware interface for testing (bypasses USB2ANY)'
    )

    parser.add_argument(
        '--register-values',
        help='Path to a register values file (e.g., examples/register-values/HexRegisterValues3600.txt)'
    )

    parser.add_argument(
        '--template-registers',
        help='Template register file used for non-PLL defaults (default: 3.6 GHz template)'
    )

    args = parser.parse_args()

    # Import required modules
    try:
        from usb2anyapi import USB2ANYInterface
        from lmx2594 import LMX2594Driver, LMX2594Error
    except ImportError as e:
        print(f"ERROR: Import error: {e}")
        return 1

    # Test imports and exit if requested
    if args.test_imports:
        print("SUCCESS: All imports successful!")
        print("SUCCESS: usb2anyapi module loaded")
        print("SUCCESS: lmx2594 module loaded")
        print("SUCCESS: LMX2594Driver and LMX2594Error classes available")
        return 0

    # Parse frequencies
    try:
        f_target = parse_frequency(args.freq) if args.freq else None
        f_osc = parse_frequency(args.fosc)
        pfd_target = parse_frequency(args.pfd)
    except ValueError as e:
        print(f"Error parsing frequency: {e}")
        return 1

    # Test register generation and exit if requested
    default_template = args.template_registers
    if not default_template:
        default_template = os.path.abspath(os.path.join(script_dir, '..', 'examples', 'register-values', 'HexRegisterValues3600.txt'))

    if args.test_registers:
        if f_target is None:
            print("ERROR: --freq is required for --test-registers when using the PLL calculator")
            return 1
        try:
            template_list, template_r0, template_map = parse_register_values(default_template, debug=args.debug)
            reg_map, plan = build_registers_from_template(f_target, f_osc, template_map)
            register_list = reg_map_to_list(reg_map)
            print(f"Frequency plan: {plan}")
            print(f"Generated {len(register_list)} registers from template {default_template}")

            invalid_addrs = [addr for addr, data in register_list if addr < 0 or addr > 127]
            if invalid_addrs:
                print(f"ERROR: Invalid register addresses found: {invalid_addrs}")
                return 1
            else:
                print("SUCCESS: All register addresses are valid (0-127)")
                return 0

        except Exception as e:
            print(f"ERROR: Register test failed: {e}")
            import traceback
            traceback.print_exc()
            return 1

    print(f"LMX2594 Frequency Generator")
    if f_target is not None:
        print(f"Target frequency: {f_target/1e9:.3f} GHz")
    print(f"Reference oscillator: {f_osc/1e6:.1f} MHz")
    print(f"Template defaults: {default_template}")
    print(f"Auto-recalibration: {'enabled' if args.auto_recal else 'disabled'}")
    print()
    print("Hardware Requirements Check:")
    print("- Red power LED on LMX2594 module should be ON")
    print("- 50MHz reference clock should be connected to LMX2594")
    print("- SPI connections (CLK, DATA, LE) should be properly wired")
    print("- +5V power supply should be connected to LMX2594")
    print("- Blue lock LED should turn ON after successful programming")
    print()

    if not args.register_values and f_target is None:
        print("ERROR: --freq is required unless you supply --register-values")
        return 1

    # Setup signal handler for clean exit
    running = True
    def signal_handler(signum, frame):
        nonlocal running
        print("\nReceived signal, shutting down...")
        running = False

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        # Connect to USB2ANY or use mock interface
        if args.mock_hardware:
            print("Using mock hardware interface...")

            class MockUSB2ANY:
                def spi_write_read(self, data):
                    # Return success and dummy read data
                    return 0, b'\x00\x00\x00'
                def read_gpio(self, pin):
                    # Simulate locked state
                    return 1
                def write_gpio(self, pin, value):
                    return 0

            usb2any = MockUSB2ANY()
        else:
            print("Connecting to USB2ANY...")
            usb2any = USB2ANYInterface(args.serial, debug=args.debug)
            usb2any.connect()

        # Create LMX2594 driver
        lmx = LMX2594Driver(usb2any, readback_enabled=args.readback, debug=args.debug)

        plan = None
        register_list = None
        r0_value = None
        reg_map = None
        if args.register_values:
            print("Using register values file...")
            register_list, r0_value, reg_map = parse_register_values(args.register_values, debug=args.debug)
            f_pd, osc_2x, mult, pll_r_pre, pll_r = compute_pfd_from_registers(f_osc, reg_map)
            print(f"Computed fPD (from file): {f_pd/1e6:.1f} MHz")
            print(f"OSC_2X: {osc_2x} MULT: {mult} PLL_R_PRE: {pll_r_pre} PLL_R: {pll_r}")
            print(f"Loaded {len(register_list)} registers")
            print()
        else:
            if f_target is None:
                print("ERROR: --freq is required when computing registers from a template")
                return 1
            print("Computing PLL fields from template defaults...")
            _, r0_value, template_map = parse_register_values(default_template, debug=args.debug)
            reg_map, plan = build_registers_from_template(f_target, f_osc, template_map)
            register_list = reg_map_to_list(reg_map)

            chdiv_desc = f"{plan['chdiv_value']} (code {plan['chdiv_code']})" if plan['chdiv_code'] is not None else "bypass"
            print(f"fPD: {plan['f_pd']/1e6:.1f} MHz (OSC_2X={plan['osc_2x']} MULT={plan['mult']} PLL_R_PRE={plan['pll_r_pre']} PLL_R={plan['pll_r']})")
            print(f"VCO frequency: {plan['f_vco']/1e9:.3f} GHz via CHDIV {chdiv_desc}")
            print(f"N: {plan['n_int']}  NUM/DEN: {plan['num']}/{plan['den']}")
            print(f"Generated {len(register_list)} registers from template defaults")
            if args.pfd and abs(plan['f_pd'] - pfd_target) > 1:
                print(f"Note: template-derived fPD ({plan['f_pd']/1e6:.1f} MHz) differs from requested --pfd ({pfd_target/1e6:.1f} MHz)")
            print()

        if args.dry_run:
            print("Dry run - exiting without programming device")
            return 0

        # Program the device
        print("Programming LMX2594...")
        try:
            if register_list is not None:
                lmx.program_registers(register_list, r0_value=r0_value)
            else:
                lmx.program_frequency(f_target, f_osc, pfd_target)
        except LMX2594Error as e:
            if not args.force:
                print(f"Programming failed: {e}")
                return 1
            else:
                print(f"Programming error (continuing due to --force): {e}")

        # Wait for initial lock
        print(f"Waiting for lock (timeout: {args.lock_timeout}s)...")
        if not lmx.wait_for_lock(args.lock_timeout):
            print("ERROR: Device failed to lock within timeout period")
            if not args.auto_recal:
                return 1
            else:
                print("Attempting recalibration...")

        # Continuous monitoring and auto-recalibration
        recal_attempts = 0
        max_recal_attempts = 10

        print("Monitoring lock status...")
        print("Press Ctrl+C to exit")

        while running:
            if not lmx.is_locked():
                print(f"WARNING: Lock lost! (attempt {recal_attempts + 1}/{max_recal_attempts})")

                if not args.auto_recal:
                    print("Auto-recalibration disabled, exiting")
                    return 1

                if recal_attempts >= max_recal_attempts:
                    print("ERROR: Maximum recalibration attempts exceeded")
                    return 1

                # Perform recalibration
                try:
                    if lmx.recalibrate_vco():
                        print("Recalibration successful - lock restored")
                        recal_attempts = 0  # Reset counter on success
                    else:
                        print("Recalibration failed - lock not restored")
                        recal_attempts += 1
                except LMX2594Error as e:
                    print(f"Recalibration error: {e}")
                    recal_attempts += 1
            else:
                if args.debug:
                    print(f"[{time.strftime('%H:%M:%S')}] Lock OK")

            time.sleep(args.monitor_interval)

        print("Shutting down...")
        return 0

    except KeyboardInterrupt:
        print("\nInterrupted by user")
        return 0
    except Exception as e:
        print(f"Error: {e}")
        if args.debug:
            import traceback
            traceback.print_exc()
        return 1
    finally:
        try:
            usb2any.disconnect()
        except:
            pass

if __name__ == '__main__':
    sys.exit(main())
