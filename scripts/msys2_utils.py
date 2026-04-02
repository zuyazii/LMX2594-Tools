#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MSYS2 auto-detection utilities for LMX2594 GUI.
Automatically finds MSYS2 installation and Qt6 plugins.
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Optional, Tuple


def find_msys2_installation() -> Optional[Tuple[Path, Path, Path]]:
    """
    Auto-detect MSYS2 UCRT64 installation.
    
    Returns:
        Tuple of (msys2_root, ucrt64_bin, qt6_plugins) or None if not found.
    """
    # Check environment variables first
    if msys2_root_env := os.environ.get("MSYS2_ROOT"):
        candidates = [Path(msys2_root_env)]
    else:
        candidates = [
            Path("C:/msys64"),
            Path("C:/msys64_old"),
            Path("C:/tools/msys64"),
            Path("D:/msys64"),
            Path("D:/tools/msys64"),
        ]
    
    for msys2_root in candidates:
        ucrt64_bin = msys2_root / "ucrt64" / "bin"
        qt6_plugins = msys2_root / "ucrt64" / "share" / "qt6" / "plugins"
        
        # Verify it's a valid MSYS2 installation
        if ucrt64_bin.exists() and (ucrt64_bin / "gcc.exe").exists():
            return (msys2_root, ucrt64_bin, qt6_plugins)
    
    return None


def get_msys2_paths() -> Tuple[str, str]:
    """
    Get MSYS2 paths, with auto-detection fallback.
    
    Returns:
        Tuple of (ucrt64_bin, qt6_plugins) paths as strings.
    """
    # Check environment variables first
    if ucrt64_bin_env := os.environ.get("MSYS2_UCRT64_BIN"):
        ucrt64_bin = Path(ucrt64_bin_env)
        qt6_plugins = ucrt64_bin.parent / "share" / "qt6" / "plugins"
        return (str(ucrt64_bin), str(qt6_plugins))
    
    if qt6_plugins_env := os.environ.get("MSYS2_QT_PLUGIN_PATH"):
        return ("", qt6_plugins_env)
    
    # Auto-detect MSYS2
    result = find_msys2_installation()
    if result:
        _, ucrt64_bin, qt6_plugins = result
        return (str(ucrt64_bin), str(qt6_plugins))
    
    # Fallback to old hardcoded paths
    return (r"C:\msys64\ucrt64\bin", r"C:\msys64\ucrt64\share\qt6\plugins")


def is_msys2_available() -> bool:
    """Check if MSYS2 UCRT64 is available."""
    return find_msys2_installation() is not None


def get_qt_plugin_paths() -> list[str]:
    """
    Get all possible Qt plugin paths (internal + MSYS2 fallback).
    Returns paths in order of preference.
    """
    paths = []
    
    # Bundled PySide6 plugins (in PyInstaller _internal)
    if hasattr(os, '_MEIPASS'):
        meipass = getattr(os, '_MEIPASS', '')
        bundled = Path(meipass)
        for subdir in ['PySide6', 'PyQt6', '']:
            plugins = bundled / subdir / 'plugins' if subdir else bundled / 'plugins'
            if plugins.exists():
                paths.append(str(plugins))
            platforms = bundled / subdir / 'plugins' / 'platforms' if subdir else bundled / 'plugins' / 'platforms'
            if platforms.exists() and str(plugins) not in paths:
                paths.insert(0, str(plugins))
    
    # MSYS2 Qt6 plugins
    msys2_paths = get_msys2_paths()
    if msys2_paths[1]:
        paths.append(msys2_paths[1])
    
    return paths
