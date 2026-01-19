#!/usr/bin/env python3
"""
LMX2594 Frequency Generator CLI

Command-line tool to program LMX2594 frequency synthesizer via USB2ANY SPI interface.
Generates continuous frequency output with automatic lock detection and VCO recalibration.
"""

import argparse
import csv
import json
import os
import signal
import sys
import time

# Add scripts directory to Python path
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)
from usb2anyapi import USB2ANYInterface
from lmx2594 import (
    LMX2594Driver,
    LMX2594Error,
    OUTA_PWR_DEFAULT,
    apply_outa_pwr,
    build_registers_from_template,
)

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

def build_frequency_sweep(start_freq, end_freq, points):
    """Build a list of sweep frequencies including endpoints."""
    if points < 1:
        raise ValueError("points must be >= 1")
    if points == 1:
        return [start_freq]
    step = (end_freq - start_freq) / (points - 1)
    return [start_freq + step * i for i in range(points)]

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


def load_outa_pwr_map(path):
    """Load OUTA_PWR mapping from CSV or JSON."""
    _, ext = os.path.splitext(path.lower())
    entries = []

    if ext == ".json":
        with open(path, "r", encoding="utf-8") as handle:
            data = json.load(handle)
        if isinstance(data, dict):
            for key, value in data.items():
                entries.append((float(key), int(value)))
        elif isinstance(data, list):
            for item in data:
                if not isinstance(item, dict):
                    continue
                if "dbm" in item and "outa_pwr" in item:
                    entries.append((float(item["dbm"]), int(item["outa_pwr"])))
        else:
            raise ValueError("Unsupported JSON format for OUTA_PWR mapping")
    else:
        with open(path, "r", newline="", encoding="utf-8") as handle:
            reader = csv.reader(handle)
            for row in reader:
                if not row or len(row) < 2:
                    continue
                try:
                    dbm = float(row[0])
                    code = int(row[1])
                except ValueError:
                    continue
                entries.append((dbm, code))

    if not entries:
        raise ValueError("OUTA_PWR mapping file contains no entries")

    for _dbm, code in entries:
        if not 0 <= code <= 63:
            raise ValueError("OUTA_PWR mapping contains code outside 0..63")

    return sorted(entries, key=lambda item: item[0])


def map_dbm_to_outa_pwr(dbm, mapping):
    """Linearly interpolate OUTA_PWR code from a dBm mapping table."""
    if not mapping:
        raise ValueError("OUTA_PWR mapping is empty")

    if dbm <= mapping[0][0]:
        return mapping[0][1]
    if dbm >= mapping[-1][0]:
        return mapping[-1][1]

    for (dbm_lo, code_lo), (dbm_hi, code_hi) in zip(mapping, mapping[1:]):
        if dbm_lo <= dbm <= dbm_hi:
            if dbm_hi == dbm_lo:
                return code_lo
            ratio = (dbm - dbm_lo) / (dbm_hi - dbm_lo)
            code = code_lo + (code_hi - code_lo) * ratio
            return int(round(code))

    return mapping[-1][1]

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

  # Set output power code directly (OUTA_PWR 0-63)
  python frequency_generator.py --freq 3.6GHz --outa-pwr 50

  # Map desired dBm to OUTA_PWR using a mapping file
  python frequency_generator.py --freq 3.6GHz --outa-db -3 --outa-map outa_map.csv

  # Sweep a range quickly
  python frequency_generator.py --start-freq 2.0GHz --end-freq 2.2GHz --points 5

  # Raise SPI clock for faster programming
  python frequency_generator.py --freq 3.6GHz --spi-clock 2MHz

  # Sweep with delta register updates only
  python frequency_generator.py --start-freq 3.2GHz --end-freq 3.6GHz --points 200 --delta-update

  # Sweep with per-step lock wait and dwell time
  python frequency_generator.py --start-freq 3.2GHz --end-freq 3.6GHz --points 200 --wait-lock --dwell-ms 10
        """
    )

    parser.add_argument(
        '--freq', '-f',
        required=False,
        help='Target output frequency (e.g., 3.6GHz, 2400MHz, 1000000000)'
    )

    parser.add_argument(
        '--start-freq',
        help='Sweep start frequency (e.g., 2.0GHz)'
    )

    parser.add_argument(
        '--end-freq',
         help='Sweep end frequency (e.g., 2.2GHz)'
     )

    parser.add_argument(
        '--points',
        type=int,
        help='Number of points in sweep range (inclusive of endpoints)'
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

    outa_group = parser.add_mutually_exclusive_group(required=False)
    outa_group.add_argument(
        '--outa-pwr',
        type=int,
        help='OUTA_PWR code (0-63). Overrides mapping.'
    )
    outa_group.add_argument(
        '--outa-db',
        type=float,
        help='Desired output power in dBm (requires --outa-map)'
    )
    parser.add_argument(
        '--outa-map',
        help='CSV or JSON mapping file for --outa-db (columns: dbm,outa_pwr)'
    )

    parser.add_argument(
        '--serial',
        help='USB2ANY serial number (if multiple devices connected)'
    )

    parser.add_argument(
        '--spi-clock',
        default='400kHz',
        help='USB2ANY SPI clock (e.g., 400kHz, 1MHz, 2MHz; default: 400kHz)'
    )

    parser.add_argument(
        '--delta-update',
        action='store_true',
        help='In sweep mode, only write registers that change between points'
    )

    parser.add_argument(
        '--wait-lock',
        action='store_true',
        help='In sweep mode, wait for PLL lock after each frequency update'
    )

    parser.add_argument(
        '--dwell-ms',
        type=float,
        default=0.0,
        help='In sweep mode, dwell time after each frequency update in milliseconds (default: 0)'
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
        f_start = parse_frequency(args.start_freq) if args.start_freq else None
        f_end = parse_frequency(args.end_freq) if args.end_freq else None
        f_osc = parse_frequency(args.fosc)
        pfd_target = parse_frequency(args.pfd)
        spi_clock = parse_frequency(args.spi_clock)
    except ValueError as e:
        print(f"Error parsing frequency: {e}")
        return 1

    outa_pwr_override = None
    outa_pwr_source = "default"
    if args.outa_pwr is not None:
        if not 0 <= args.outa_pwr <= 63:
            print("ERROR: --outa-pwr must be between 0 and 63")
            return 1
        outa_pwr_override = int(args.outa_pwr)
        outa_pwr_source = "outa-pwr"
    elif args.outa_db is not None:
        if not args.outa_map:
            print("ERROR: --outa-db requires --outa-map")
            return 1
        try:
            mapping = load_outa_pwr_map(args.outa_map)
            outa_pwr_override = map_dbm_to_outa_pwr(args.outa_db, mapping)
            outa_pwr_source = "outa-db"
        except Exception as exc:
            print(f"ERROR: Failed to load OUTA_PWR mapping: {exc}")
            return 1
    elif args.outa_map:
        print("ERROR: --outa-map requires --outa-db")
        return 1

    if spi_clock < 10e3 or spi_clock > 8e6:
        print("ERROR: --spi-clock must be between 10kHz and 8MHz (USB2ANY SPI limits)")
        return 1

    sweep_requested = any([args.start_freq, args.end_freq, args.points is not None])
    sweep_freqs = None
    if sweep_requested:
        if args.freq:
            print("ERROR: --freq cannot be used with --start-freq/--end-freq/--points")
            return 1
        if f_start is None or f_end is None or args.points is None:
            print("ERROR: --start-freq, --end-freq, and --points are required for a sweep")
            return 1
        if args.register_values:
            print("ERROR: --register-values cannot be used with sweep mode")
            return 1
        try:
            sweep_freqs = build_frequency_sweep(f_start, f_end, args.points)
        except ValueError as e:
            print(f"ERROR: {e}")
            return 1

    # Test register generation and exit if requested
    default_template = args.template_registers
    if not default_template:
        default_template = os.path.abspath(os.path.join(script_dir, '..', 'examples', 'register-values', 'HexRegisterValues3600.txt'))

    outa_pwr_value = outa_pwr_override if outa_pwr_override is not None else OUTA_PWR_DEFAULT

    if args.test_registers:
        test_freq = f_target if f_target is not None else (sweep_freqs[0] if sweep_freqs else None)
        if test_freq is None:
            print("ERROR: --freq is required for --test-registers when using the PLL calculator")
            return 1
        try:
            template_list, template_r0, template_map = parse_register_values(default_template, debug=args.debug)
            reg_map, plan = build_registers_from_template(
                test_freq, f_osc, template_map, outa_pwr=outa_pwr_value
            )
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
    if sweep_freqs:
        print(f"Sweep start: {sweep_freqs[0]/1e9:.3f} GHz")
        print(f"Sweep end: {sweep_freqs[-1]/1e9:.3f} GHz")
        print(f"Sweep points: {len(sweep_freqs)}")
        if args.wait_lock:
            print(f"Sweep lock wait: enabled (timeout {args.lock_timeout}s)")
        if args.dwell_ms:
            print(f"Sweep dwell: {args.dwell_ms:.1f} ms")
    print(f"Reference oscillator: {f_osc/1e6:.1f} MHz")
    print(f"Template defaults: {default_template}")
    if outa_pwr_override is not None:
        print(f"OUTA_PWR: {outa_pwr_override} ({outa_pwr_source})")
    print(f"Auto-recalibration: {'enabled' if args.auto_recal else 'disabled'}")
    print()
    print("Hardware Requirements Check:")
    print("- Red power LED on LMX2594 module should be ON")
    print("- 50MHz reference clock should be connected to LMX2594")
    print("- SPI connections (CLK, DATA, LE) should be properly wired")
    print("- +5V power supply should be connected to LMX2594")
    print("- Blue lock LED should turn ON after successful programming")
    print()

    if not args.register_values and f_target is None and not sweep_freqs:
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
            usb2any = USB2ANYInterface(args.serial, debug=args.debug, clock_freq=spi_clock)
            usb2any.connect()

        # Create LMX2594 driver
        lmx = LMX2594Driver(usb2any, readback_enabled=args.readback, debug=args.debug, f_osc=f_osc)

        plan = None
        register_list = None
        r0_value = None
        reg_map = None
        if args.register_values:
            print("Using register values file...")
            register_list, r0_value, reg_map = parse_register_values(args.register_values, debug=args.debug)
            if outa_pwr_override is not None:
                apply_outa_pwr(reg_map, outa_pwr_override)
                register_list = reg_map_to_list(reg_map)
            f_pd, osc_2x, mult, pll_r_pre, pll_r = compute_pfd_from_registers(f_osc, reg_map)
            print(f"Computed fPD (from file): {f_pd/1e6:.1f} MHz")
            print(f"OSC_2X: {osc_2x} MULT: {mult} PLL_R_PRE: {pll_r_pre} PLL_R: {pll_r}")
            print(f"Loaded {len(register_list)} registers")
            print()
        else:
            if not sweep_freqs:
                if f_target is None:
                    print("ERROR: --freq is required when computing registers from a template")
                    return 1
                print("Computing PLL fields from template defaults...")
                _, r0_value, template_map = parse_register_values(default_template, debug=args.debug)
                reg_map, plan = build_registers_from_template(
                    f_target, f_osc, template_map, outa_pwr=outa_pwr_value
                )
                register_list = reg_map_to_list(reg_map)

                chdiv_desc = f"{plan['chdiv_value']} (code {plan['chdiv_code']})" if plan['chdiv_code'] is not None else "bypass"
                print(f"fPD: {plan['f_pd']/1e6:.1f} MHz (OSC_2X={plan['osc_2x']} MULT={plan['mult']} PLL_R_PRE={plan['pll_r_pre']} PLL_R={plan['pll_r']})")
                print(f"VCO frequency: {plan['f_vco']/1e9:.3f} GHz via CHDIV {chdiv_desc}")
                print(f"N: {plan['n_int']}  NUM/DEN: {plan['num']}/{plan['den']}")
                print(f"Generated {len(register_list)} registers from template defaults")
                if args.pfd and abs(plan['f_pd'] - pfd_target) > 1:
                    print(f"Note: template-derived fPD ({plan['f_pd']/1e6:.1f} MHz) differs from requested --pfd ({pfd_target/1e6:.1f} MHz)")
                print()

        if args.dry_run and not sweep_freqs:
            print("Dry run - exiting without programming device")
            return 0

        if sweep_freqs:
            print("Computing PLL fields from template defaults...")
            _, r0_value, template_map = parse_register_values(default_template, debug=args.debug)
            dwell_s = max(0.0, args.dwell_ms / 1000.0)
            wait_lock = args.wait_lock
            if wait_lock and args.lock_timeout <= 0:
                print("Note: --wait-lock requested but --lock-timeout <= 0; skipping per-step lock waits.")
                wait_lock = False

            if args.dry_run:
                for freq in sweep_freqs:
                    reg_map, plan = build_registers_from_template(
                        freq, f_osc, template_map, outa_pwr=outa_pwr_value
                    )
                    chdiv_desc = f"{plan['chdiv_value']} (code {plan['chdiv_code']})" if plan['chdiv_code'] is not None else "bypass"
                    print(f"{freq/1e9:.3f} GHz -> VCO {plan['f_vco']/1e9:.3f} GHz CHDIV {chdiv_desc} N {plan['n_int']} NUM/DEN {plan['num']}/{plan['den']}")
                print("Dry run - exiting without programming device")
                return 0

            # Program first frequency fully, then fast update for the rest.
            first_freq = sweep_freqs[0]
            reg_map, plan = build_registers_from_template(
                first_freq, f_osc, template_map, outa_pwr=outa_pwr_value
            )
            register_list = reg_map_to_list(reg_map)
            print(f"Programming initial frequency {first_freq/1e9:.3f} GHz...")
            try:
                lmx.program_registers(register_list, r0_value=r0_value)
            except LMX2594Error as e:
                if not args.force:
                    print(f"Programming failed: {e}")
                    return 1
                else:
                    print(f"Programming error (continuing due to --force): {e}")

            lmx.configure_muxout_lock_detect()

            if args.lock_timeout > 0:
                print(f"Waiting for lock (timeout: {args.lock_timeout}s)...")
                if not lmx.wait_for_lock(args.lock_timeout):
                    print("ERROR: Device failed to lock within timeout period")
                    if not args.auto_recal:
                        return 1
                    else:
                        print("Attempting recalibration...")

            prev_reg_map = reg_map
            if dwell_s > 0:
                time.sleep(dwell_s)

            for freq in sweep_freqs[1:]:
                reg_map, plan = build_registers_from_template(
                    freq, f_osc, template_map, outa_pwr=outa_pwr_value
                )
                if args.delta_update:
                    update_addrs = (31, 34, 36, 38, 39, 42, 43, 44, 75)
                    write_addrs = [addr for addr in update_addrs if reg_map.get(addr) != prev_reg_map.get(addr)]
                    if not write_addrs:
                        print(f"Updating to {freq/1e9:.3f} GHz... (no register changes)")
                    else:
                        print(f"Updating to {freq/1e9:.3f} GHz...")
                        lmx.fast_update_frequency(reg_map, write_addrs=write_addrs)
                else:
                    print(f"Updating to {freq/1e9:.3f} GHz...")
                    lmx.fast_update_frequency(reg_map)

                if wait_lock:
                    if not lmx.wait_for_lock(args.lock_timeout):
                        print("ERROR: Device failed to lock within timeout period")
                        if not args.auto_recal:
                            return 1
                        else:
                            print("Attempting recalibration...")
                            if not lmx.recalibrate_vco():
                                print("ERROR: Recalibration failed - lock not restored")
                                return 1

                if dwell_s > 0:
                    time.sleep(dwell_s)
                prev_reg_map = reg_map

            print("Sweep complete")
            return 0

        # Program the device (single frequency)
        print("Programming LMX2594...")
        try:
            if register_list is not None:
                lmx.program_registers(register_list, r0_value=r0_value)
            else:
                lmx.program_frequency(f_target, f_osc, pfd_target, outa_pwr=outa_pwr_value)
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
