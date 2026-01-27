#!/usr/bin/env python3
"""Minimal command-line interface for running LibreVNA headless sweeps."""

from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

import click
from click.core import ParameterSource

from src.librevna.device.headless_cli import HeadlessLibreVNA, HeadlessLibreVNAResult
from src.librevna.output_config import OutputConfig
from src.librevna.calibration import (
    CalibrationCaptureResult,
    CalibrationConfiguration,
    SOLT12Calibration,
    SOLT12Calibrator,
)


try:  # pragma: no cover - optional convenience wrapper around libusb
    import libusb_package  # type: ignore
except Exception:  # pragma: no cover
    libusb_package = None  # type: ignore[assignment]


KNOWN_VNA_IDS = (
    (0x0483, 0x564E, "LibreVNA"),
    (0x0483, 0x4121, "LibreVNA"),
    (0x1209, 0x4121, "LibreVNA"),
)


def _discover_vna_devices() -> List[dict]:
    if usb is None:
        raise RuntimeError("PyUSB is required for device enumeration. Install 'pyusb'.")

    backend = None
    if libusb_package is not None:
        backend = libusb_package.get_libusb1_backend()

    devices: List[dict] = []
    for vendor, product, label in KNOWN_VNA_IDS:
        found = usb.core.find(  # type: ignore[attr-defined]
            find_all=True,
            idVendor=vendor,
            idProduct=product,
            backend=backend,
        )
        if not found:
            continue
        for dev in found:
            serial = None
            try:
                if getattr(dev, "iSerialNumber", None):
                    serial = usb.util.get_string(dev, dev.iSerialNumber)  # type: ignore[attr-defined]
            except Exception:
                serial = None
            devices.append(
                {
                    "vendor": vendor,
                    "product": product,
                    "label": label,
                    "serial": serial,
                }
            )
    return devices


def _print_discovered_devices() -> None:
    try:
        devices = _discover_vna_devices()
    except RuntimeError as exc:
        click.echo(f"{exc}", err=True)
        return

    if not devices:
        click.echo("No LibreVNA devices detected.")
        return

    click.echo("Connected LibreVNA devices:")
    for index, dev in enumerate(devices, start=1):
        serial = dev["serial"] or "<no-serial>"
        click.echo(f"  [{index}] {dev['label']} (VID:PID {dev['vendor']:04X}:{dev['product']:04X}) serial={serial}")


def _default_serial_number() -> Optional[str]:
    try:
        devices = _discover_vna_devices()
    except RuntimeError:
        return None
    for dev in devices:
        if dev["serial"]:
            return dev["serial"]
    return None

try:  # pragma: no cover - optional dependency
    import usb  # type: ignore
    import usb.core  # noqa: F401
    import usb.util  # noqa: F401
except ImportError:
    usb = None  # type: ignore[assignment]


class FrequencyParamType(click.ParamType):
    """Parse frequencies provided in GHz and return the value in Hz."""

    name = "frequency"

    def convert(self, value, param, ctx):
        try:
            if isinstance(value, (int, float)):
                return float(value) * 1e9
            return float(str(value)) * 1e9
        except (TypeError, ValueError):
            self.fail(
                f"'{value}' is not a valid frequency. Use decimal GHz (e.g. 2.45).",
                param,
                ctx,
            )


@click.group(invoke_without_command=True)
@click.version_option(version="0.2.0", prog_name="LibreVNA CLI")
@click.option('-l', '--list', 'list_devices_flag', is_flag=True,
              help='List connected LibreVNA devices and exit')
@click.pass_context
def cli(ctx: click.Context, list_devices_flag: bool) -> None:
    """Control LibreVNA hardware through the native headless sweep binary."""

    if list_devices_flag:
        _print_discovered_devices()
        ctx.exit(0)

    if ctx.invoked_subcommand is None:
        click.echo(ctx.get_help())
        ctx.exit(0)


def _parse_ports(raw: str) -> List[int]:
    ports: List[int] = []
    for item in raw.split(','):
        item = item.strip()
        if not item:
            continue
        try:
            ports.append(int(item))
        except ValueError as exc:
            raise click.BadParameter(f"Invalid port '{item}' in --excite") from exc
    if not ports:
        raise click.BadParameter("At least one port must be provided via --excite")
    return ports


def _decode_complex(value: Mapping[str, object]) -> complex:
    if not isinstance(value, Mapping):
        return 0.0 + 0.0j
    try:
        real = float(value.get("real", 0.0))
        imag = float(value.get("imag", 0.0))
    except (TypeError, ValueError):
        return 0.0 + 0.0j
    return complex(real, imag)


def _extract_complex_trace(result: HeadlessLibreVNAResult) -> Tuple[List[float], List[Dict[str, complex]]]:
    frequencies: List[float] = []
    parameters: List[Dict[str, complex]] = []
    for entry in result.trace:
        frequency = float(entry.get("frequency", 0.0))
        frequencies.append(frequency)
        point: Dict[str, complex] = {}
        for key, value in entry.items():
            if key.startswith("S") and isinstance(value, Mapping):
                point[key] = _decode_complex(value)
        parameters.append(point)
    return frequencies, parameters


def _ensure_frequency_grid(
    reference: Optional[List[float]],
    candidate: List[float],
) -> List[float]:
    if reference is None:
        return candidate
    if len(reference) != len(candidate):
        raise click.ClickException("Calibration sweeps returned differing point counts")
    for index, (left, right) in enumerate(zip(reference, candidate)):
        if abs(left - right) > max(1.0, abs(left)) * 1e-9:
            raise click.ClickException(
                f"Frequency mismatch at point {index}: {left} Hz vs {right} Hz"
            )
    return reference


def _extract_parameter_series(
    data: List[Dict[str, complex]],
    key: str,
    label: str,
) -> List[complex]:
    series: List[complex] = []
    for index, point in enumerate(data):
        if key not in point:
            raise click.ClickException(
                f"Missing parameter {key} while capturing {label} (point {index})"
            )
        series.append(point[key])
    return series


def _run_capture_sweep(
    runner: HeadlessLibreVNA,
    cal_path: Path,
    configuration: CalibrationConfiguration,
    threshold_db: float,
    serial: Optional[str],
    timeout_ms: float,
    excited_ports: Sequence[int],
) -> Tuple[List[float], List[Dict[str, complex]]]:
    result = runner.run_sweep(
        cal=cal_path,
        f_start=configuration.start_frequency_hz,
        f_stop=configuration.stop_frequency_hz,
        points=configuration.points,
        if_bandwidth=configuration.if_bandwidth_hz,
        power_dbm=configuration.power_dbm,
        threshold_db=threshold_db,
        serial=serial,
        timeout_ms=timeout_ms,
        excited_ports=list(excited_ports),
        progress=False,
    )
    return _extract_complex_trace(result)


def _capture_solt12_standards(
    runner: HeadlessLibreVNA,
    cal_path: Path,
    configuration: CalibrationConfiguration,
    threshold_db: float,
    serial: Optional[str],
    timeout_ms: float,
) -> CalibrationCaptureResult:
    ports = [1, 2]
    frequencies: Optional[List[float]] = None
    open_measurements: Dict[int, List[complex]] = {}
    short_measurements: Dict[int, List[complex]] = {}
    load_measurements: Dict[int, List[complex]] = {}
    through_measurements: Dict[Tuple[int, int], List[Dict[str, complex]]] = {}

    for port in ports:
        click.echo(f"Port {port}: OPEN")
        click.pause(
            f"Connect the OPEN standard to port {port} and press Enter to capture."
        )
        sweep_freqs, sweep_data = _run_capture_sweep(
            runner,
            cal_path,
            configuration,
            threshold_db,
            serial,
            timeout_ms,
            [port],
        )
        if frequencies is None:
            frequencies = sweep_freqs
        else:
            _ensure_frequency_grid(frequencies, sweep_freqs)
        open_measurements[port] = _extract_parameter_series(
            sweep_data,
            f"S{port}{port}",
            f"open (port {port})",
        )

    for port in ports:
        click.echo(f"Port {port}: SHORT")
        click.pause(
            f"Connect the SHORT standard to port {port} and press Enter to capture."
        )
        sweep_freqs, sweep_data = _run_capture_sweep(
            runner,
            cal_path,
            configuration,
            threshold_db,
            serial,
            timeout_ms,
            [port],
        )
        _ensure_frequency_grid(frequencies, sweep_freqs)
        short_measurements[port] = _extract_parameter_series(
            sweep_data,
            f"S{port}{port}",
            f"short (port {port})",
        )

    for port in ports:
        click.echo(f"Port {port}: LOAD")
        click.pause(
            f"Connect the LOAD standard to port {port} and press Enter to capture."
        )
        sweep_freqs, sweep_data = _run_capture_sweep(
            runner,
            cal_path,
            configuration,
            threshold_db,
            serial,
            timeout_ms,
            [port],
        )
        _ensure_frequency_grid(frequencies, sweep_freqs)
        load_measurements[port] = _extract_parameter_series(
            sweep_data,
            f"S{port}{port}",
            f"load (port {port})",
        )

    click.echo("Through measurement: port 1 -> port 2")
    click.pause("Connect the THRU between ports 1 and 2, press Enter to capture forward.")
    sweep_freqs, forward_data = _run_capture_sweep(
        runner,
        cal_path,
        configuration,
        threshold_db,
        serial,
        timeout_ms,
        [ports[0]],
    )
    _ensure_frequency_grid(frequencies, sweep_freqs)
    through_measurements[(ports[0], ports[1])] = forward_data

    click.echo("Through measurement: port 2 -> port 1")
    click.pause("Keep the THRU connected, press Enter to capture reverse.")
    sweep_freqs, reverse_data = _run_capture_sweep(
        runner,
        cal_path,
        configuration,
        threshold_db,
        serial,
        timeout_ms,
        [ports[1]],
    )
    _ensure_frequency_grid(frequencies, sweep_freqs)
    through_measurements[(ports[1], ports[0])] = reverse_data

    if frequencies is None:
        raise click.ClickException("Calibration capture produced no data")

    return CalibrationCaptureResult(
        frequencies=frequencies,
        open_measurements=open_measurements,
        short_measurements=short_measurements,
        load_measurements=load_measurements,
        through_measurements=through_measurements,
        isolation_measurements={},
    )


def _load_solt12_calibration(path: Path) -> Optional[SOLT12Calibration]:
    try:
        return SOLT12Calibration.load(path)
    except FileNotFoundError:
        return None
    except Exception as exc:
        click.echo(f"Warning: failed to load calibration {path}: {exc}", err=True)
        return None


def _recompute_summary(
    trace: List[Dict[str, object]],
    threshold_db: float,
) -> Tuple[Dict[str, Dict[str, float]], bool]:
    parameters = sorted({key for point in trace for key in point if key.startswith("S")})
    overall_pass = True
    summary: Dict[str, Dict[str, float]] = {}
    for name in parameters:
        worst_db = -300.0
        fail_frequency = 0.0
        passes = True
        for point in trace:
            value = point.get(name)
            if not isinstance(value, Mapping):
                continue
            coefficient = _decode_complex(value)
            magnitude = abs(coefficient)
            magnitude_db = -300.0 if magnitude <= 0.0 else 20.0 * math.log10(magnitude)
            if magnitude_db > worst_db:
                worst_db = magnitude_db
                fail_frequency = float(point.get("frequency", 0.0))
            if magnitude_db > threshold_db:
                passes = False
        entry: Dict[str, float] = {"pass": passes, "worst_db": worst_db}
        if not passes:
            entry["fail_at_hz"] = fail_frequency
        summary[name] = entry
        overall_pass = overall_pass and passes
    return summary, overall_pass


def _apply_calibration_to_result(
    result: HeadlessLibreVNAResult,
    calibration: SOLT12Calibration,
    threshold_db: float,
) -> None:
    trace = result.raw.get("trace", [])
    calibrated_trace = calibration.apply_trace(trace)
    result.raw["trace"] = calibrated_trace
    summary, overall_pass = _recompute_summary(calibrated_trace, threshold_db)
    result.raw["results"] = summary
    result.raw["overall_pass"] = overall_pass
    calibration_meta = result.raw.setdefault("calibration", {})
    calibration_meta.update(
        {
            "format": calibration.FORMAT,
            "ports": calibration.ports,
        }
    )


@cli.command(name="calibrate-solt12")
@click.option('-c', '--cal', type=click.Path(path_type=Path),
              default=Path('calibration/solt12.cal'), show_default=True,
              help='Output calibration file (.cal)')
@click.option('-s', '--serial', type=str, help='Device serial number to target')
@click.option('-f', '--start-freq', type=FrequencyParamType(), required=True,
              help='Start frequency in GHz (converted to Hz)')
@click.option('-F', '--stop-freq', type=FrequencyParamType(), required=True,
              help='Stop frequency in GHz (converted to Hz)')
@click.option('-p', '--points', type=int, required=True, help='Number of calibration points')
@click.option('-i', '--ifbw', type=float, default=1000.0, show_default=True,
              help='IF bandwidth in Hz')
@click.option('-P', '--power', type=float, default=-10.0, show_default=True,
              help='Source power in dBm')
@click.option('-t', '--threshold', type=float, default=-10.0, show_default=True,
              help='Threshold in dB used during calibration capture')
@click.option('-T', '--timeout-ms', type=float, default=20000.0, show_default=True,
              help='Timeout in milliseconds for each capture sweep')
def calibrate_solt12(
    *,
    cal: Path,
    serial: Optional[str],
    start_freq: float,
    stop_freq: float,
    points: int,
    ifbw: float,
    power: float,
    threshold: float,
    timeout_ms: float,
) -> None:
    """Capture SOLT_12 calibration coefficients using LibreVNA headless sweeps."""

    if serial is None:
        default_serial = _default_serial_number()
        if default_serial:
            serial = default_serial
            click.echo(f"Using LibreVNA serial: {serial}")

    try:
        runner = HeadlessLibreVNA()
    except FileNotFoundError as exc:
        click.echo(f"Error: {exc}", err=True)
        sys.exit(2)

    cal = cal.resolve()
    cal.parent.mkdir(parents=True, exist_ok=True)
    if not cal.exists():
        cal.write_text('{}\n')

    configuration = CalibrationConfiguration(
        start_frequency_hz=start_freq,
        stop_frequency_hz=stop_freq,
        points=points,
        if_bandwidth_hz=ifbw,
        power_dbm=power,
    )

    click.echo('Starting SOLT_12 calibration capture...')
    capture = _capture_solt12_standards(
        runner,
        cal,
        configuration,
        threshold,
        serial,
        timeout_ms,
    )

    calibrator = SOLT12Calibrator([1, 2])
    metadata = {}
    if serial:
        metadata['device_serial'] = serial
    calibration = calibrator.compute(capture, configuration, metadata)
    calibration.save(cal)
    click.echo(f'Saved SOLT_12 calibration to {cal}')


@cli.command(name="headless-sweep")
@click.option('-c', '--cal', type=click.Path(exists=True, path_type=Path),
              default=Path('calibration/calibrate.cal'), show_default=True,
              help='Calibration file (.cal)')
@click.option('-s', '--serial', type=str, help='Device serial number to target')
@click.option('-f', '--start-freq', type=FrequencyParamType(), required=True,
              help='Start frequency in GHz (converted to Hz)')
@click.option('-F', '--stop-freq', type=FrequencyParamType(), required=True,
              help='Stop frequency in GHz (converted to Hz)')
@click.option('-p', '--points', type=int, required=True, help='Number of sweep points')
@click.option('-i', '--ifbw', type=float, default=1000.0, show_default=True,
              help='IF bandwidth in Hz')
@click.option('-P', '--power', type=float, default=-10.0, show_default=True,
              help='Source power in dBm')
@click.option('-t', '--threshold', type=float, default=-10.0, show_default=True,
              help='Pass/fail threshold in dB')
@click.option('-e', '--excite', type=str, default='1,2', show_default=True,
              help='Comma-separated excited ports')
@click.option('-T', '--timeout-ms', type=float, default=15000.0, show_default=True,
              help='Timeout in milliseconds')
@click.option('-g', '--progress', is_flag=True, help='Stream NDJSON progress to stderr')
@click.option('-j', '--json-dir', type=click.Path(path_type=Path),
              help='Directory to store generated output files')
@click.option('-G', '--plot', type=str, metavar='NAME', default='plot', show_default=True,
              help='Generate magnitude-vs-frequency plot saved with sweep artifacts. NAME overrides file stem.')
def headless_sweep(
    *,
    cal: Path,
    serial: Optional[str],
    start_freq: float,
    stop_freq: float,
    points: int,
    ifbw: float,
    power: float,
    threshold: float,
    excite: str,
    timeout_ms: float,
    progress: bool,
    json_dir: Optional[Path],
    plot: Optional[str],
):
    """Run a sweep using the native ``librevna-cli`` binary."""

    ports = _parse_ports(excite)

    if serial is None:
        default_serial = _default_serial_number()
        if default_serial:
            serial = default_serial
            click.echo(f"Using LibreVNA serial: {serial}")

    try:
        runner = HeadlessLibreVNA()
    except FileNotFoundError as exc:
        click.echo(f"Error: {exc}", err=True)
        sys.exit(2)

    try:
        result = runner.run_sweep(
            cal=cal,
            f_start=start_freq,
            f_stop=stop_freq,
            points=points,
            if_bandwidth=ifbw,
            power_dbm=power,
            threshold_db=threshold,
            serial=serial,
            timeout_ms=timeout_ms,
            excited_ports=ports,
            progress=progress,
        )
    except Exception as exc:
        click.echo(f"Headless sweep failed: {exc}", err=True)
        sys.exit(3)

    calibration = _load_solt12_calibration(cal)
    if calibration is not None:
        try:
            _apply_calibration_to_result(result, calibration, threshold)
        except Exception as exc:
            click.echo(f"Warning: failed to apply calibration: {exc}", err=True)

    base_output_dir = json_dir if json_dir else Path("output")
    csv_path, result_dir = _store_headless_outputs(result, base_output_dir)

    ctx = click.get_current_context()
    plot_source = ctx.get_parameter_source('plot')
    if plot_source != ParameterSource.DEFAULT:
        try:
            desired = Path(plot)
            desired_name = desired.name
            if not desired_name or desired_name == '.':
                desired_name = f"{csv_path.stem}_magnitude"
            plot_name = desired_name if Path(desired_name).suffix else f"{desired_name}.png"
            plot_path = result_dir / plot_name
            _plot_from_csv(csv_path, plot_path)
        except Exception as exc:
            click.echo(f"Plot generation failed: {exc}", err=True)

    summary = _summarize_headless_result(result)
    click.echo(json.dumps(summary))

    sys.exit(0 if result.overall_pass else 1)


def _store_headless_outputs(result: HeadlessLibreVNAResult, base_output_dir: Path) -> tuple[Path, Path]:
    """Persist headless sweep outputs as JSON and CSV in structured folders."""

    output_config = OutputConfig.create_with_shared_timestamp(
        output_directory=str(base_output_dir),
        file_prefix="headless_sweep",
    )

    output_config.ensure_base_directory()

    timestamp = output_config.shared_timestamp
    base_name = "result"
    subdir_name = f"{output_config.file_prefix}_{base_name}_{timestamp}"
    subdir_path = Path(output_config.output_directory) / subdir_name
    subdir_path.mkdir(parents=True, exist_ok=True)

    filename_base = f"{output_config.file_prefix}_{base_name}_{timestamp}"
    json_path = subdir_path / f"{filename_base}.json"
    csv_path = subdir_path / f"{filename_base}.csv"

    with json_path.open('w', encoding='utf-8') as json_file:
        json.dump(result.raw, json_file, indent=2)

    _write_headless_csv(result, csv_path)

    return csv_path, subdir_path


def _write_headless_csv(result: HeadlessLibreVNAResult, csv_path: Path) -> None:
    """Write CSV representation of headless sweep trace data."""

    parameter_names = sorted(result.parameter_results.keys())

    with csv_path.open('w', newline='', encoding='utf-8') as csv_file:
        writer = csv.writer(csv_file)

        header = ["Frequency (Hz)"]
        for name in parameter_names:
            header.extend([
                f"{name}_Real",
                f"{name}_Imag",
                f"{name}_Magnitude_dB",
                f"{name}_Phase_deg",
            ])
        writer.writerow(header)

        for point in result.trace:
            frequency = point.get("frequency", "")
            row = [frequency]
            for name in parameter_names:
                value = point.get(name, {})
                real = value.get("real")
                imag = value.get("imag")
                if real is None or imag is None:
                    row.extend(["", "", "", ""])
                    continue

                magnitude = math.hypot(real, imag)
                magnitude_db = 20 * math.log10(max(magnitude, 1e-12))
                phase_deg = math.degrees(math.atan2(imag, real))
                row.extend([real, imag, magnitude_db, phase_deg])

            writer.writerow(row)


def _summarize_headless_result(result: HeadlessLibreVNAResult) -> dict:
    """Build concise summary of pass/fail status and failed points."""

    failures = []
    for name, data in result.parameter_results.items():
        if not data.get("pass", False):
            failure_entry = {"parameter": name}
            if "fail_at_hz" in data and data["fail_at_hz"] is not None:
                failure_entry["frequency_hz"] = data["fail_at_hz"]
            if "worst_db" in data and data["worst_db"] is not None:
                failure_entry["worst_db"] = data["worst_db"]
            failures.append(failure_entry)

    summary = {"status": "pass" if result.overall_pass else "fail"}
    if failures:
        summary["failed_points"] = failures

    return summary


def _plot_from_csv(csv_path: Path, output_path: Path) -> None:
    import matplotlib.pyplot as plt
    import numpy as np

    data = np.genfromtxt(csv_path, delimiter=',', names=True)
    columns = list(data.dtype.names or [])
    if not columns:
        raise RuntimeError('CSV appears to be empty.')

    def _normalise(name: str) -> str:
        return ''.join(ch for ch in name.lower() if ch.isalnum())

    freq_column = None
    for name in columns:
        if 'frequency' in _normalise(name):
            freq_column = name
            break
    if freq_column is None:
        freq_column = columns[0]

    freq = data[freq_column]

    magnitude_columns = [name for name in data.dtype.names if name.endswith('_Magnitude_dB')]
    if not magnitude_columns:
        raise RuntimeError('CSV does not contain any magnitude columns to plot.')

    plt.figure(figsize=(10, 6))
    for column in magnitude_columns:
        label = column.replace('_Magnitude_dB', '')
        plt.plot(freq, data[column], label=label)

    plt.title('Magnitude vs Frequency')
    plt.xlabel(freq_column.replace('_', ' '))
    plt.ylabel('Magnitude (dB)')
    plt.grid(True)
    plt.legend()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_path, bbox_inches='tight')
    plt.close()


if __name__ == "__main__":
    cli()
