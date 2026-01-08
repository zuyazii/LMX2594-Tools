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
from lmx2594 import LMX2594Driver, LMX2594Error

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
        f_target = parse_frequency(args.freq)
        f_osc = parse_frequency(args.fosc)
        pfd_target = parse_frequency(args.pfd)
    except ValueError as e:
        print(f"Error parsing frequency: {e}")
        return 1

    # Test register generation and exit if requested
    if args.test_registers:
        try:
            from usb2anyapi import USB2ANYInterface
            from lmx2594 import LMX2594Driver, LMX2594Error

            # Create a mock LMX2594 driver to test register generation
            class MockUSB2ANY:
                def spi_write_read(self, data):
                    return 0, b'\x00\x00\x00'

            mock_usb2any = MockUSB2ANY()
            lmx = LMX2594Driver(mock_usb2any)

            # Test frequency plan
            plan = lmx.compute_frequency_plan(f_target, f_osc, pfd_target)
            print(f"Frequency plan: {plan}")

            # Test register list generation
            register_list = lmx._generate_register_list(plan)
            print(f"Generated {len(register_list)} registers")

            # Check for invalid addresses
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
    print(f"Target frequency: {f_target/1e9:.3f} GHz")
    print(f"Reference oscillator: {f_osc/1e6:.1f} MHz")
    print(f"Target PFD: {pfd_target/1e6:.1f} MHz")
    print(f"Auto-recalibration: {'enabled' if args.auto_recal else 'disabled'}")
    print()
    print("Hardware Requirements Check:")
    print("- Red power LED on LMX2594 module should be ON")
    print("- 50MHz reference clock should be connected to LMX2594")
    print("- SPI connections (CLK, DATA, LE) should be properly wired")
    print("- +5V power supply should be connected to LMX2594")
    print("- Blue lock LED should turn ON after successful programming")
    print()

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
            usb2any = USB2ANYInterface(args.serial)
            usb2any.connect()

        # Create LMX2594 driver
        lmx = LMX2594Driver(usb2any)

        # Compute frequency plan
        print("Computing frequency plan...")
        plan = lmx.compute_frequency_plan(f_target, f_osc, pfd_target)

        print(f"VCO frequency: {plan['f_vco']/1e9:.3f} GHz")
        print(f"Channel divider: {plan['chdiv']}")
        print(f"R divider: {plan['r_div']}")
        print(f"PFD frequency: {plan['pfd_freq']/1e6:.1f} MHz")
        print(f"N integer: {plan['n_int']}")
        print(f"N fractional: {plan['n_frac']}")
        print()

        if args.dry_run:
            print("Dry run - exiting without programming device")
            return 0

        # Program the device
        print("Programming LMX2594...")
        try:
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
