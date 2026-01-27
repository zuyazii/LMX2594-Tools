from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np


__all__ = [
    "CalibrationConfiguration",
    "CalibrationPoint",
    "CalibrationCaptureResult",
    "SOLT12Calibration",
    "SOLT12Calibrator",
]

Complex = complex

_FREQUENCY_REL_TOL = 1e-9
_FREQUENCY_ABS_TOL = 1e-3


def _as_complex(value: Mapping[str, float]) -> complex:
    return complex(float(value.get("real", 0.0)), float(value.get("imag", 0.0)))


def _serialize_complex(value: complex) -> Dict[str, float]:
    return {"real": float(value.real), "imag": float(value.imag)}


def _interpolate_complex(a: complex, b: complex, alpha: float) -> complex:
    if alpha <= 0.0:
        return a
    if alpha >= 1.0:
        return b
    mag_a = abs(a)
    mag_b = abs(b)
    if mag_a == 0.0 and mag_b == 0.0:
        return 0.0j
    phase_a = math.atan2(a.imag, a.real)
    phase_b = math.atan2(b.imag, b.real)
    delta = phase_b - phase_a
    while delta > math.pi:
        delta -= 2.0 * math.pi
    while delta < -math.pi:
        delta += 2.0 * math.pi
    magnitude = mag_a * (1.0 - alpha) + mag_b * alpha
    phase = phase_a + alpha * delta
    return complex(math.cos(phase) * magnitude, math.sin(phase) * magnitude)

@dataclass
class CalibrationConfiguration:
    start_frequency_hz: float
    stop_frequency_hz: float
    points: int
    if_bandwidth_hz: float
    power_dbm: float

    def to_dict(self) -> Dict[str, float]:
        return {
            "start_frequency_hz": self.start_frequency_hz,
            "stop_frequency_hz": self.stop_frequency_hz,
            "points": self.points,
            "if_bandwidth_hz": self.if_bandwidth_hz,
            "power_dbm": self.power_dbm,
        }


@dataclass
class CalibrationPoint:
    frequency: float
    directivity: np.ndarray
    reflection_tracking: np.ndarray
    source_match: np.ndarray
    receiver_match: np.ndarray
    transmission_tracking: np.ndarray
    isolation: np.ndarray

    def interpolate(self, other: "CalibrationPoint", alpha: float) -> "CalibrationPoint":
        directivity = np.array(
            [_interpolate_complex(a, b, alpha) for a, b in zip(self.directivity, other.directivity)],
            dtype=np.complex128,
        )
        reflection_tracking = np.array(
            [_interpolate_complex(a, b, alpha) for a, b in zip(self.reflection_tracking, other.reflection_tracking)],
            dtype=np.complex128,
        )
        source_match = np.array(
            [_interpolate_complex(a, b, alpha) for a, b in zip(self.source_match, other.source_match)],
            dtype=np.complex128,
        )
        receiver_match = np.empty_like(self.receiver_match)
        transmission_tracking = np.empty_like(self.transmission_tracking)
        isolation = np.empty_like(self.isolation)
        for idx in range(self.receiver_match.shape[0]):
            for jdx in range(self.receiver_match.shape[1]):
                receiver_match[idx, jdx] = _interpolate_complex(
                    self.receiver_match[idx, jdx], other.receiver_match[idx, jdx], alpha
                )
                transmission_tracking[idx, jdx] = _interpolate_complex(
                    self.transmission_tracking[idx, jdx], other.transmission_tracking[idx, jdx], alpha
                )
                isolation[idx, jdx] = _interpolate_complex(
                    self.isolation[idx, jdx], other.isolation[idx, jdx], alpha
                )
        return CalibrationPoint(
            frequency=self.frequency * (1.0 - alpha) + other.frequency * alpha,
            directivity=directivity,
            reflection_tracking=reflection_tracking,
            source_match=source_match,
            receiver_match=receiver_match,
            transmission_tracking=transmission_tracking,
            isolation=isolation,
        )

    def to_dict(self) -> Dict[str, object]:
        return {
            "frequency": self.frequency,
            "directivity": [_serialize_complex(value) for value in self.directivity],
            "reflection_tracking": [_serialize_complex(value) for value in self.reflection_tracking],
            "source_match": [_serialize_complex(value) for value in self.source_match],
            "receiver_match": [
                [_serialize_complex(value) for value in row] for row in self.receiver_match
            ],
            "transmission_tracking": [
                [_serialize_complex(value) for value in row] for row in self.transmission_tracking
            ],
            "isolation": [
                [_serialize_complex(value) for value in row] for row in self.isolation
            ],
        }

    @classmethod
    def from_dict(cls, payload: Mapping[str, object]) -> "CalibrationPoint":
        def _to_complex_list(entries: Iterable[Mapping[str, float]]) -> np.ndarray:
            return np.array([_as_complex(entry) for entry in entries], dtype=np.complex128)

        def _to_complex_matrix(entries: Iterable[Iterable[Mapping[str, float]]]) -> np.ndarray:
            return np.array([
                [_as_complex(value) for value in row] for row in entries
            ], dtype=np.complex128)

        return cls(
            frequency=float(payload["frequency"]),
            directivity=_to_complex_list(payload["directivity"]),
            reflection_tracking=_to_complex_list(payload["reflection_tracking"]),
            source_match=_to_complex_list(payload["source_match"]),
            receiver_match=_to_complex_matrix(payload["receiver_match"]),
            transmission_tracking=_to_complex_matrix(payload["transmission_tracking"]),
            isolation=_to_complex_matrix(payload["isolation"]),
        )

@dataclass
class CalibrationCaptureResult:
    frequencies: List[float]
    open_measurements: Dict[int, List[complex]]
    short_measurements: Dict[int, List[complex]]
    load_measurements: Dict[int, List[complex]]
    through_measurements: Dict[Tuple[int, int], List[Dict[str, complex]]]
    isolation_measurements: Dict[Tuple[int, int], List[complex]]

    def ensure_frequency_alignment(self) -> None:
        reference = self.frequencies
        expected_points = len(reference)
        for entry in (
            *self.open_measurements.values(),
            *self.short_measurements.values(),
            *self.load_measurements.values(),
        ):
            if len(entry) != expected_points:
                raise ValueError("Measurement point mismatch across SOL standards")
        for through in self.through_measurements.values():
            if len(through) != expected_points:
                raise ValueError("Through measurement point count mismatch")
        for isolation in self.isolation_measurements.values():
            if len(isolation) != expected_points:
                raise ValueError("Isolation measurement point count mismatch")

class SOLT12Calibration:
    """SOLT two-port calibration coefficients and application helpers."""

    FORMAT = "solt12-calibration-v1"

    def __init__(
        self,
        ports: Sequence[int],
        points: List[CalibrationPoint],
        configuration: CalibrationConfiguration,
        metadata: Optional[Mapping[str, object]] = None,
    ) -> None:
        if len(ports) != 2:
            raise ValueError("SOLT_12 calibration requires exactly two ports")
        self.ports = list(ports)
        self.points = sorted(points, key=lambda item: item.frequency)
        self.configuration = configuration
        self.metadata = dict(metadata or {})

    @property
    def port_count(self) -> int:
        return len(self.ports)

    def _locate_point(self, frequency: float) -> CalibrationPoint:
        if not self.points:
            raise ValueError("Calibration contains no points")
        if frequency <= self.points[0].frequency:
            return self.points[0]
        if frequency >= self.points[-1].frequency:
            return self.points[-1]
        low_index = 0
        high_index = len(self.points) - 1
        while high_index - low_index > 1:
            mid = (low_index + high_index) // 2
            if self.points[mid].frequency <= frequency:
                low_index = mid
            else:
                high_index = mid
        low = self.points[low_index]
        high = self.points[high_index]
        span = high.frequency - low.frequency
        alpha = 0.0 if span == 0.0 else (frequency - low.frequency) / span
        return low.interpolate(high, alpha)

    def apply(self, parameters: Mapping[str, complex], frequency: float) -> Dict[str, complex]:
        point = self._locate_point(frequency)
        n = self.port_count
        s_matrix = np.zeros((n, n), dtype=np.complex128)
        for col, src_port in enumerate(self.ports):
            for row, dst_port in enumerate(self.ports):
                key = f"S{dst_port}{src_port}"
                value = complex(parameters.get(key, 0.0 + 0.0j))
                if row != col:
                    value -= point.isolation[col, row]
                s_matrix[row, col] = value
        a = np.zeros_like(s_matrix)
        b = np.zeros_like(s_matrix)
        for col in range(n):
            for row in range(n):
                if row == col:
                    numerator = s_matrix[row, col] - point.directivity[col]
                    if point.reflection_tracking[col] == 0.0:
                        raise ZeroDivisionError("Reflection tracking coefficient is zero")
                    b[row, col] = numerator / point.reflection_tracking[col]
                    a[row, col] = 1.0 + point.source_match[col] * b[row, col]
                else:
                    if point.transmission_tracking[col, row] == 0.0:
                        raise ZeroDivisionError("Transmission tracking coefficient is zero")
                    b[row, col] = s_matrix[row, col] / point.transmission_tracking[col, row]
                    a[row, col] = point.receiver_match[col, row] * b[row, col]
        try:
            corrected = b @ np.linalg.inv(a)
        except np.linalg.LinAlgError as exc:
            raise RuntimeError(
                f"Calibration matrix is singular at {frequency:.3f} Hz"
            ) from exc
        result: Dict[str, complex] = {}
        for col, src_port in enumerate(self.ports):
            for row, dst_port in enumerate(self.ports):
                result[f"S{dst_port}{src_port}"] = corrected[row, col]
        return result

    def apply_trace(self, trace: Sequence[Mapping[str, object]]) -> List[Dict[str, object]]:
        calibrated: List[Dict[str, object]] = []
        for item in trace:
            frequency = float(item.get("frequency", 0.0))
            parameters = {
                key: _as_complex(value)
                for key, value in item.items()
                if key.startswith("S") and isinstance(value, Mapping)
            }
            corrected = self.apply(parameters, frequency)
            entry: Dict[str, object] = {"frequency": frequency}
            for key, value in corrected.items():
                entry[key] = _serialize_complex(value)
            calibrated.append(entry)
        return calibrated

    def to_dict(self) -> Dict[str, object]:
        return {
            "format": self.FORMAT,
            "ports": self.ports,
            "configuration": self.configuration.to_dict(),
            "points": [point.to_dict() for point in self.points],
            "metadata": self.metadata,
        }

    def save(self, path: Path) -> None:
        payload = self.to_dict()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(payload, indent=2))

    @classmethod
    def load(cls, path: Path) -> "SOLT12Calibration":
        payload = json.loads(path.read_text())
        fmt = payload.get("format")
        if fmt == cls.FORMAT:
            configuration = CalibrationConfiguration(**payload["configuration"])
            points = [CalibrationPoint.from_dict(item) for item in payload["points"]]
            metadata = payload.get("metadata", {})
            return cls(payload["ports"], points, configuration, metadata)
        if fmt == 3 or fmt == "3":
            return cls._from_librevna_payload(payload, path)
        raise ValueError(f"Unsupported calibration format: {fmt}")

    @classmethod
    def _from_librevna_payload(cls, payload: Mapping[str, object], source_path: Path) -> "SOLT12Calibration":
        ports = list(payload.get("ports") or [1, 2])
        if len(ports) != 2:
            raise ValueError("LibreVNA calibration requires exactly two ports for SOLT_12")

        capture, configuration, metadata = _capture_from_librevna_payload(payload, ports)
        metadata.setdefault("source_path", str(source_path))
        metadata.setdefault("source_format", payload.get("format"))
        if payload.get("device"):
            metadata.setdefault("device", payload.get("device"))
        calkit = payload.get("calkit")
        if isinstance(calkit, Mapping):
            if calkit.get("Description"):
                metadata.setdefault("calkit_description", calkit.get("Description"))
            if calkit.get("version"):
                metadata.setdefault("calkit_version", calkit.get("version"))
        if payload.get("type"):
            metadata.setdefault("original_type", payload.get("type"))

        calibrator = SOLT12Calibrator(ports)
        return calibrator.compute(capture, configuration, metadata)

class SOLT12Calibrator:
    """Compute SOLT_12 calibration coefficients from captured standards."""

    def __init__(self, ports: Sequence[int]) -> None:
        if len(ports) != 2:
            raise ValueError("SOLT_12 calibration requires exactly two ports")
        self.ports = list(ports)
        self._port_index = {port: index for index, port in enumerate(self.ports)}

    @staticmethod
    def _ideal_open(_: float) -> complex:
        return 1.0 + 0.0j

    @staticmethod
    def _ideal_short(_: float) -> complex:
        return -1.0 + 0.0j

    @staticmethod
    def _ideal_load(_: float) -> complex:
        return 0.0 + 0.0j

    @staticmethod
    def _ideal_through(_: float) -> Dict[str, complex]:
        return {
            "S11": 0.0 + 0.0j,
            "S22": 0.0 + 0.0j,
            "S21": 1.0 + 0.0j,
            "S12": 1.0 + 0.0j,
        }

    def compute(
        self,
        capture: CalibrationCaptureResult,
        configuration: CalibrationConfiguration,
        metadata: Optional[Mapping[str, object]] = None,
    ) -> SOLT12Calibration:
        capture.ensure_frequency_alignment()
        metadata = dict(metadata or {})
        points: List[CalibrationPoint] = []
        for idx, frequency in enumerate(capture.frequencies):
            directivity_terms: List[complex] = []
            source_match_terms: List[complex] = []
            reflection_tracking_terms: List[complex] = []
            for port in self.ports:
                short_measured = capture.short_measurements[port][idx]
                open_measured = capture.open_measurements[port][idx]
                load_measured = capture.load_measurements[port][idx]
                short_actual = self._ideal_short(frequency)
                open_actual = self._ideal_open(frequency)
                load_actual = self._ideal_load(frequency)
                denom = (
                    load_actual * open_actual * (open_measured - load_measured)
                    + load_actual * short_actual * (load_measured - short_measured)
                    + open_actual * short_actual * (short_measured - open_measured)
                )
                if denom == 0:
                    raise ZeroDivisionError(
                        f"Degenerate SOL coefficients at {frequency:.3f} Hz on port {port}"
                    )
                directivity = (
                    load_actual
                    * open_measured
                    * (short_measured * (open_actual - short_actual) + load_measured * short_actual)
                    - load_actual * open_actual * load_measured * short_measured
                    + open_actual * load_measured * short_actual * (short_measured - open_measured)
                ) / denom
                source_match = (
                    load_actual * (open_measured - short_measured)
                    + open_actual * (short_measured - load_measured)
                    + short_actual * (load_measured - open_measured)
                ) / denom
                delta = (
                    load_actual * load_measured * (open_measured - short_measured)
                    + open_actual * open_measured * (short_measured - load_measured)
                    + short_actual * short_measured * (load_measured - open_measured)
                ) / denom
                reflection_tracking = directivity * source_match - delta
                directivity_terms.append(directivity)
                source_match_terms.append(source_match)
                reflection_tracking_terms.append(reflection_tracking)
            directivity_arr = np.array(directivity_terms, dtype=np.complex128)
            source_match_arr = np.array(source_match_terms, dtype=np.complex128)
            reflection_tracking_arr = np.array(reflection_tracking_terms, dtype=np.complex128)
            receiver_match = np.zeros((2, 2), dtype=np.complex128)
            transmission_tracking = np.zeros((2, 2), dtype=np.complex128)
            isolation = np.zeros((2, 2), dtype=np.complex128)
            for src_port in self.ports:
                for dst_port in self.ports:
                    if src_port == dst_port:
                        continue
                    key = (src_port, dst_port)
                    measurement = capture.through_measurements.get(key)
                    orientation = "forward"
                    if measurement is None:
                        measurement = capture.through_measurements.get((dst_port, src_port))
                        orientation = "reverse"
                    if measurement is None:
                        raise ValueError(
                            f"Missing through measurement for ports {src_port}->{dst_port}"
                        )
                    point = measurement[idx]
                    ideal = self._ideal_through(frequency)
                    if orientation == "forward":
                        measured_s11 = point.get("S11", 0.0 + 0.0j)
                        measured_s21 = point.get("S21", 0.0 + 0.0j)
                    else:
                        measured_s11 = point.get("S22", 0.0 + 0.0j)
                        measured_s21 = point.get("S12", 0.0 + 0.0j)
                        ideal = {
                            "S11": ideal["S22"],
                            "S22": ideal["S11"],
                            "S21": ideal["S12"],
                            "S12": ideal["S21"],
                        }
                    isolation_track = capture.isolation_measurements.get(key)
                    if isolation_track is None and orientation == "reverse":
                        isolation_track = capture.isolation_measurements.get((dst_port, src_port))
                    isolation_value = (
                        isolation_track[idx] if isolation_track is not None else 0.0 + 0.0j
                    )
                    src_index = self._port_index[src_port]
                    dst_index = self._port_index[dst_port]
                    delta_s = ideal["S11"] * ideal["S22"] - ideal["S21"] * ideal["S12"]
                    numerator = (
                        (measured_s11 - directivity_arr[src_index])
                        * (1.0 - source_match_arr[src_index] * ideal["S11"])
                        - ideal["S11"] * reflection_tracking_arr[src_index]
                    )
                    denominator = (
                        (measured_s11 - directivity_arr[src_index])
                        * (ideal["S22"] - source_match_arr[src_index] * delta_s)
                        - delta_s * reflection_tracking_arr[src_index]
                    )
                    if denominator == 0:
                        raise ZeroDivisionError(
                            f"Receiver match denominator zero at {frequency:.3f} Hz"
                        )
                    receiver_match[src_index, dst_index] = numerator / denominator
                    transmission_tracking[src_index, dst_index] = (
                        (measured_s21 - isolation_value)
                        * (
                            1.0
                            - source_match_arr[src_index] * ideal["S11"]
                            - receiver_match[src_index, dst_index] * ideal["S22"]
                            + source_match_arr[src_index]
                            * receiver_match[src_index, dst_index]
                            * delta_s
                        )
                        / ideal["S21"]
                    )
                    isolation[src_index, dst_index] = isolation_value
            points.append(
                CalibrationPoint(
                    frequency=frequency,
                    directivity=directivity_arr,
                    reflection_tracking=reflection_tracking_arr,
                    source_match=source_match_arr,
                    receiver_match=receiver_match,
                    transmission_tracking=transmission_tracking,
                    isolation=isolation,
                )
            )
        metadata.setdefault("ports", self.ports)
        return SOLT12Calibration(self.ports, points, configuration, metadata)


def _capture_from_librevna_payload(
    payload: Mapping[str, object],
    ports: Sequence[int],
) -> Tuple[CalibrationCaptureResult, CalibrationConfiguration, Dict[str, object]]:
    frequencies: Optional[List[float]] = None

    def update_frequencies(candidate: List[float], label: str) -> None:
        nonlocal frequencies
        if not candidate:
            raise ValueError(f"Calibration measurement '{label}' has no points")
        candidate_list = list(candidate)
        if frequencies is None:
            frequencies = candidate_list
            return
        if len(frequencies) != len(candidate_list):
            raise ValueError(
                f"Calibration measurement '{label}' has mismatched point count: "
                f"{len(candidate_list)} vs {len(frequencies)}"
            )
        for idx, (ref, value) in enumerate(zip(frequencies, candidate_list)):
            if not math.isclose(ref, value, rel_tol=_FREQUENCY_REL_TOL, abs_tol=_FREQUENCY_ABS_TOL):
                raise ValueError(
                    f"Frequency mismatch for '{label}' at index {idx}: {ref} Hz vs {value} Hz"
                )

    def extract_reflection_series(points: List[Mapping[str, object]], label: str) -> List[complex]:
        freq_series = [float(entry.get("frequency", 0.0)) for entry in points]
        values = [
            complex(float(entry.get("real", 0.0)), float(entry.get("imag", 0.0)))
            for entry in points
        ]
        update_frequencies(freq_series, label)
        return values

    open_measurements: Dict[int, List[complex]] = {}
    short_measurements: Dict[int, List[complex]] = {}
    load_measurements: Dict[int, List[complex]] = {}
    through_measurements: Dict[Tuple[int, int], List[Dict[str, complex]]] = {}
    isolation_measurements: Dict[Tuple[int, int], List[complex]] = {}

    for measurement in payload.get('measurements', []):
        m_type = measurement.get('type')
        data = measurement.get('data', {})
        if m_type in {'Open', 'Short', 'Load'}:
            port = data.get('port')
            if port is None:
                raise ValueError(f"Calibration measurement '{m_type}' is missing port information")
            series = extract_reflection_series(data.get('points', []), f"{m_type.lower()}_port_{port}")
            target = {
                'Open': open_measurements,
                'Short': short_measurements,
                'Load': load_measurements,
            }[m_type]
            target[int(port)] = series
            continue
        if m_type == 'Through':
            port1 = int(data.get('port1', ports[0]))
            port2 = int(data.get('port2', ports[1]))
            freq_series: List[float] = []
            forward: List[Dict[str, complex]] = []
            reverse: List[Dict[str, complex]] = []
            for entry in data.get('points', []):
                freq = float(entry.get('frequency', 0.0))
                sparam = entry.get('Sparam', {})
                s11 = complex(float(sparam.get('m11_real', 0.0)), float(sparam.get('m11_imag', 0.0)))
                s12 = complex(float(sparam.get('m12_real', 0.0)), float(sparam.get('m12_imag', 0.0)))
                s21 = complex(float(sparam.get('m21_real', 0.0)), float(sparam.get('m21_imag', 0.0)))
                s22 = complex(float(sparam.get('m22_real', 0.0)), float(sparam.get('m22_imag', 0.0)))
                freq_series.append(freq)
                forward.append({'S11': s11, 'S21': s21, 'S12': s12, 'S22': s22})
                reverse.append({'S11': s22, 'S21': s12, 'S12': s21, 'S22': s11})
            update_frequencies(freq_series, 'through')
            through_measurements[(port1, port2)] = forward
            through_measurements[(port2, port1)] = reverse
            continue
        if m_type == 'Isolation':
            freq_series = [float(entry.get('frequency', 0.0)) for entry in data.get('points', [])]
            update_frequencies(freq_series, 'isolation')
            for entry in data.get('points', []):
                matrix = entry.get('S') or []
                for dst_index, dst_port in enumerate(ports):
                    if dst_index >= len(matrix):
                        continue
                    row = matrix[dst_index]
                    for src_index, src_port in enumerate(ports):
                        if src_index == dst_index or src_index >= len(row):
                            continue
                        value = row[src_index]
                        iso_value = complex(float(value.get('real', 0.0)), float(value.get('imag', 0.0)))
                        isolation_measurements.setdefault((src_port, dst_port), []).append(iso_value)
            continue

    if frequencies is None:
        raise ValueError('Calibration file did not contain frequency information')

    for port in ports:
        if port not in open_measurements or port not in short_measurements or port not in load_measurements:
            raise ValueError(f"Missing SOL standards for port {port} in calibration file")

    for src in ports:
        for dst in ports:
            if src == dst:
                continue
            if (src, dst) not in through_measurements:
                raise ValueError(f"Missing through measurement for ports {src}->{dst}")
            if (src, dst) not in isolation_measurements:
                isolation_measurements[(src, dst)] = [0.0 + 0.0j] * len(frequencies)
            else:
                if len(isolation_measurements[(src, dst)]) != len(frequencies):
                    raise ValueError(
                        f"Isolation measurement for ports {src}->{dst} has mismatched point count"
                    )

    capture = CalibrationCaptureResult(
        frequencies=list(frequencies),
        open_measurements=open_measurements,
        short_measurements=short_measurements,
        load_measurements=load_measurements,
        through_measurements=through_measurements,
        isolation_measurements=isolation_measurements,
    )

    configuration = CalibrationConfiguration(
        start_frequency_hz=frequencies[0],
        stop_frequency_hz=frequencies[-1],
        points=len(frequencies),
        if_bandwidth_hz=float(payload.get('ifBandwidthHz', 0.0)),
        power_dbm=float(payload.get('powerDbm', 0.0)),
    )

    metadata: Dict[str, object] = {}
    if payload.get('version'):
        metadata['librevna_version'] = payload.get('version')
    return capture, configuration, metadata
