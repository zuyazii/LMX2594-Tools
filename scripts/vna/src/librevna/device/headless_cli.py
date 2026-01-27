"""Headless LibreVNA CLI adapter.

This module provides a small adapter that invokes the native ``librevna-cli``
binary introduced by the ``cpp`` project. It keeps the Python layer focused on
orchestration while deferring the instrument protocol and calibration logic to
the C++ implementation.
"""

from __future__ import annotations

import json
import logging
import os
import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Dict, Iterable, List, Optional

logger = logging.getLogger(__name__)


class HeadlessLibreVNAResult:
    """Container for the results reported by the native CLI."""

    def __init__(self, raw: Dict):
        self.raw = raw

    @property
    def overall_pass(self) -> bool:
        return bool(self.raw.get("overall_pass", False))

    @property
    def parameter_results(self) -> Dict:
        return dict(self.raw.get("results", {}))

    @property
    def trace(self) -> List[Dict]:
        return list(self.raw.get("trace", []))


class HeadlessLibreVNA:
    """Executes sweeps through the native ``librevna-cli`` binary."""

    def __init__(self, binary_path: Optional[Path] = None):
        self.binary_path = binary_path or locate_binary()
        if not self.binary_path:
            raise FileNotFoundError(
                "librevna-cli binary not found. Please build the C++ project or"
                " set the LIBREVNA_CLI_BIN environment variable."
            )

    def run_sweep(
        self,
        *,
        cal: Path,
        f_start: float,
        f_stop: float,
        points: int,
        if_bandwidth: float,
        power_dbm: float,
        threshold_db: float,
        serial: Optional[str] = None,
        timeout_ms: float = 15000,
        excited_ports: Optional[Iterable[int]] = None,
        progress: bool = False,
        output_dir: Optional[Path] = None,
    ) -> HeadlessLibreVNAResult:
        """Execute a sweep via the CLI and return parsed results."""

        if not cal.exists():
            raise FileNotFoundError(f"Calibration file not found: {cal}")

        args = [str(self.binary_path)]
        args.extend(["--cal", str(cal)])
        args.extend(["--fstart", str(f_start)])
        args.extend(["--fstop", str(f_stop)])
        args.extend(["--points", str(points)])
        args.extend(["--ifbw", str(if_bandwidth)])
        args.extend(["--power", str(power_dbm)])
        args.extend(["--threshold", str(threshold_db)])
        args.extend(["--timeout-ms", str(timeout_ms)])

        if serial:
            args.extend(["--serial", serial])

        ports = list(excited_ports or [1, 2])
        args.extend(["--excite", ",".join(str(p) for p in ports)])

        with tempfile.NamedTemporaryFile(delete=False, suffix=".json") as tmp_file:
            tmp_path = Path(tmp_file.name)

        args.extend(["--json-out", str(tmp_path)])

        if progress:
            args.append("--progress-ndjson")

        logger.debug("Executing librevna-cli: %s", " ".join(args))

        completed = subprocess.run(
            args,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            text=True,
        )

        stdout = completed.stdout.strip()
        stderr = completed.stderr.strip()

        if stderr:
            logger.debug("librevna-cli stderr: %s", stderr)

        if completed.returncode not in {0, 1}:
            raise RuntimeError(
                f"librevna-cli failed with exit code {completed.returncode}: {stderr or stdout}"
            )

        if not stdout:
            raise RuntimeError("librevna-cli produced no JSON output")

        try:
            payload = json.loads(stdout)
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"Failed to parse CLI JSON output: {exc}") from exc

        if output_dir:
            output_dir.mkdir(parents=True, exist_ok=True)
            target = output_dir / tmp_path.name
            shutil.move(str(tmp_path), target)
        else:
            tmp_path.unlink(missing_ok=True)

        return HeadlessLibreVNAResult(payload)


def locate_binary() -> Optional[Path]:
    env_override = os.environ.get("LIBREVNA_CLI_BIN")
    if env_override:
        path = Path(env_override)
        if path.exists():
            return path
        raise FileNotFoundError(f"LIBREVNA_CLI_BIN is set but file does not exist: {path}")

    binary = shutil.which("librevna-cli")
    if binary:
        return Path(binary)

    repo_candidate = Path(__file__).resolve().parents[3] / "cpp" / "build" / "librevna-cli"
    if repo_candidate.exists():
        return repo_candidate

    return None


__all__ = ["HeadlessLibreVNA", "HeadlessLibreVNAResult", "locate_binary"]
