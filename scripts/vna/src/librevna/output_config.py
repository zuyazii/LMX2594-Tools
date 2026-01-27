"""Minimal output configuration helpers for headless sweep storage."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional


@dataclass
class OutputConfig:
    """Configuration for organising sweep outputs on disk."""

    output_directory: str = "output"
    file_prefix: str = "headless_sweep"
    timestamp_format: str = "%Y%m%d_%H%M%S"
    shared_timestamp: Optional[str] = None

    @classmethod
    def create_with_shared_timestamp(
        cls,
        *,
        output_directory: str = "output",
        file_prefix: str = "headless_sweep",
        timestamp_format: str = "%Y%m%d_%H%M%S",
    ) -> "OutputConfig":
        return cls(
            output_directory=output_directory,
            file_prefix=file_prefix,
            timestamp_format=timestamp_format,
            shared_timestamp=datetime.now().strftime(timestamp_format),
        )

    def ensure_base_directory(self) -> Path:
        path = Path(self.output_directory)
        path.mkdir(parents=True, exist_ok=True)
        return path

