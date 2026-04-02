#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
LMX2594 打包构建脚本
=====================
将 LMX2594 Calibration GUI 和 32位 worker 打包为可直接双击运行的发行版。

特点
--------
- 自动检测 MSYS2 UCRT64 安装（支持 MSYS2_ROOT 环境变量自定义路径）
- 自动查找 Qt6 工具链
- 无需手动配置即可构建

使用方法
--------
    python build.py              # 完整构建（自动检测 MSYS2）
    python build.py --gui-only   # 仅构建 GUI
    python build.py --worker-only # 仅构建 worker
    python build.py --skip-vna    # 跳过 LibreVNA IPC 构建
    python build.py --clean      # 清理 dist/ 并重新构建

MSYS2 安装（如需构建 LibreVNA IPC）
--------
    1. 下载 MSYS2: https://www.msys2.org/
    2. 安装后运行 MSYS2 UCRT64 终端：
       pacman -S mingw-w64-ucrt-x86_64-qt6-base
       pacman -S mingw-w64-ucrt-x86_64-cmake
       pacman -S mingw-w64-ucrt-x86_64-ninja
    或设置自定义路径：set MSYS2_ROOT=C:\msys64_old

输出目录: dist/LMX2594-Release/
"""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import urllib.request
import hashlib
from pathlib import Path
from typing import Optional

# ----------------------------------------------------------------------
# 全局配置
# ----------------------------------------------------------------------
REPO_ROOT = Path(__file__).parent.resolve()
BUILD_DIR = REPO_ROOT / "build"
DIST_DIR = REPO_ROOT / "dist"
RELEASE_DIR = DIST_DIR / "LMX2594-Release"
VENV_X64 = REPO_ROOT / ".venv-x64"
VENV_X86 = REPO_ROOT / ".venv-x86"

# 检测系统 Python
SYSTEM_PYTHON = shutil.which("python") or shutil.which("python3") or ""
PY313 = Path(shutil.which("python") or "")
if PY313 and PY313.name.startswith("python"):
    try:
        v = subprocess.run(
            [str(PY313), "-c", "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}')"],
            capture_output=True, text=True, errors="replace"
        )
        if v.returncode == 0 and v.stdout.strip().startswith("3.13"):
            SYSTEM_PYTHON = str(PY313)
    except Exception:
        pass

# ----------------------------------------------------------------------
# 辅助函数
# ----------------------------------------------------------------------

def log(msg: str) -> None:
    print(f"\n>>> {msg}")


def run(cmd: list[str], env: Optional[dict] = None, cwd: Optional[Path] = None,
        check: bool = True) -> subprocess.CompletedProcess[bytes]:
    """使用 text=False 以便正确处理 Windows 路径和输出。"""
    merged_env = os.environ.copy()
    if env:
        merged_env.update(env)
    merged_env.setdefault("PYTHONDONTWRITEBYTECODE", "1")
    return subprocess.run(
        cmd,
        cwd=cwd or REPO_ROOT,
        env=merged_env,
        check=check,
        stderr=subprocess.STDOUT,
    )


def python_exe(venv: Path) -> Path:
    """返回 venv 中 python.exe 的路径。"""
    py = venv / "Scripts" / "python.exe"
    if not py.exists():
        raise FileNotFoundError(f"Python not found in {venv}")
    return py


def pip_install(venv: Path, *packages: str, upgrade: bool = False) -> None:
    """在指定 venv 中安装包。"""
    py = python_exe(venv)
    cmd = [str(py), "-m", "pip", "install"]
    if upgrade:
        cmd.append("-U")
    cmd.extend(packages)
    run(cmd)


def ensure_pyinstaller(python: Path) -> None:
    """确保 PyInstaller 已安装。"""
    log(f"检查 PyInstaller...")
    try:
        subprocess.run(
            [str(python), "-c", "import PyInstaller; print(PyInstaller.__version__)"],
            capture_output=True, check=True,
        )
        print("  PyInstaller 已安装")
    except subprocess.CalledProcessError:
        log(f"正在安装 PyInstaller（使用 {python}）...")
        run([str(python), "-m", "pip", "install", "pyinstaller"])


def ensure_pipreqs(python: Path) -> None:
    """确保 pipreqs 已安装（用于精简依赖）。"""
    try:
        subprocess.run(
            [str(python), "-c", "import pipreqs; print(pipreqs.__version__)"],
            capture_output=True, check=True,
        )
    except subprocess.CalledProcessError:
        log("正在安装 pipreqs...")
        run([str(python), "-m", "pip", "install", "pipreqs"])


# ----------------------------------------------------------------------
# MSYS2 自动检测
# ----------------------------------------------------------------------

def _find_msys2_installation() -> Optional[tuple[Path, Path]]:
    """
    自动检测 MSYS2 安装位置。
    返回 (msys2_bin, msys2_lib) 元组，或 None。
    搜索顺序：
    1. 环境变量 MSYS2_ROOT / MSYS2_BIN
    2. 常见安装位置
    """
    # 环境变量优先级
    if msys2_root_env := os.environ.get("MSYS2_ROOT"):
        msys2_root = Path(msys2_root_env)
    elif msys2_bin_env := os.environ.get("MSYS2_BIN"):
        msys2_root = Path(msys2_bin_env).parent
    else:
        # 常见安装位置
        candidates = [
            Path("C:/msys64"),
            Path("C:/msys64_old"),
            Path("C:/tools/msys64"),
            Path("D:/msys64"),
            Path("D:/tools/msys64"),
        ]
        for cand in candidates:
            ucrt_bin = cand / "ucrt64" / "bin"
            if ucrt_bin.exists() and (ucrt_bin / "gcc.exe").exists():
                msys2_root = cand
                break
        else:
            return None

    msys2_bin = msys2_root / "ucrt64" / "bin"
    msys2_lib = msys2_root / "ucrt64" / "lib"

    if not msys2_bin.exists() or not msys2_lib.exists():
        return None

    return (msys2_bin, msys2_lib)


def _find_qt6_dir() -> Optional[Path]:
    """搜索常见的 Qt6 安装路径，优先 MSYS2 UCRT64。"""
    candidates = [
        # MSYS2 UCRT64（最常见）- 动态检测
        None,  # 占位符，由 _find_msys2_installation 填充
        # 官方 Qt 安装
        Path("C:/Qt/6.6.3/msvc2022_64/lib/cmake/Qt6"),
        Path("C:/Qt/6.7.0/msvc2022_64/lib/cmake/Qt6"),
        Path("C:/Qt/6.8.0/msvc2022_64/lib/cmake/Qt6"),
        Path("C:/Qt/6.8.1/msvc2022_64/lib/cmake/Qt6"),
        Path("C:/Qt/6.8.2/msvc2022_64/lib/cmake/Qt6"),
        Path("C:/Qt/6.8.3/msvc2022_64/lib/cmake/Qt6"),
        Path("C:/Qt/6.9.0/msvc2022_64/lib/cmake/Qt6"),
        Path("C:/Qt/6.10.0/msvc2022_64/lib/cmake/Qt6"),
        Path("C:/Qt/6.10.1/msvc2022_64/lib/cmake/Qt6"),
    ]

    # 动态添加 MSYS2 Qt6 路径
    msys2_info = _find_msys2_installation()
    if msys2_info:
        msys2_bin, msys2_lib = msys2_info
        candidates[0] = msys2_lib / "cmake" / "Qt6"

    for p in candidates:
        if p and p.exists():
            return p
    return None


def _get_msys2_info() -> tuple[Path, Path]:
    """
    获取 MSYS2 路径，自动检测或抛出友好错误。
    返回 (msys2_bin, msys2_lib)
    """
    msys2_info = _find_msys2_installation()
    if not msys2_info:
        raise RuntimeError(
            "未找到 MSYS2 UCRT64 安装！\n"
            "请安装 MSYS2：https://www.msys2.org/\n"
            "安装后运行: pacman -S mingw-w64-ucrt-x86_64-qt6-base mingw-w64-ucrt-x86_64-cmake mingw-w64-ucrt-x86_64-ninja\n"
            "或设置环境变量 MSYS2_ROOT=C:\\msys64"
        )
    return msys2_info


# ----------------------------------------------------------------------
# 构建 LibreVNA IPC（可选）
# ----------------------------------------------------------------------

def build_librevna_ipc() -> bool:
    """
    Build LibreVNA IPC CLI 工具。
    自动检测 MSYS2 和 Qt6，收集 Qt DLL 并复制到源码 build 目录。
    如果源码不存在或构建失败，静默跳过。
    """
    cmake_dir = REPO_ROOT / "scripts" / "vna" / "cpp"
    build_dir = cmake_dir / "build"
    ipc_exe = build_dir / "librevna-ipc.exe"

    if not cmake_dir.exists():
        log("跳过 LibreVNA IPC：源码目录不存在")
        return False

    # 检查是否已构建且 Qt DLL 已在同目录
    if ipc_exe.exists() and (build_dir / "Qt6Core.dll").exists():
        log(f"LibreVNA IPC 已构建: {ipc_exe}")
        return True

    log("构建 LibreVNA IPC（可选）...")

    # 自动检测 MSYS2 UCRT64 环境
    try:
        msys2_bin, msys2_lib = _get_msys2_info()
        log(f"  使用 MSYS2: {msys2_bin.parent.parent}")
    except RuntimeError as e:
        log(str(e))
        return False

    windeployqt = msys2_bin / "windeployqt6.exe"
    cmake_exe = msys2_bin / "cmake.exe"
    ninja_exe = msys2_bin / "ninja.exe"

    # 构建时 PATH 包含 MSYS2 工具链
    build_env = os.environ.copy()
    build_env["PATH"] = f"{msys2_bin};{msys2_lib};" + build_env.get("PATH", "")

    # 检测工具链
    missing_tools = []
    if not cmake_exe.exists():
        missing_tools.append("cmake (pacman -S mingw-w64-ucrt-x86_64-cmake)")
    if not ninja_exe.exists():
        missing_tools.append("ninja (pacman -S mingw-w64-ucrt-x86_64-ninja)")
    if not windeployqt.exists():
        missing_tools.append("Qt6 (pacman -S mingw-w64-ucrt-x86_64-qt6-base)")

    if missing_tools:
        log(f"  警告：缺少工具: {', '.join(missing_tools)}")
        log("  安装命令: pacman -S mingw-w64-ucrt-x86_64-cmake mingw-w64-ucrt-x86_64-ninja mingw-w64-ucrt-x86_64-qt6-base")
        return False

    # 检测 Qt6
    qt6_dir_env = os.environ.get("Qt6_DIR", "")
    qt6_dir = Path(qt6_dir_env) if qt6_dir_env else None
    if not qt6_dir or not qt6_dir.exists():
        qt6_dir = _find_qt6_dir()
        if qt6_dir:
            log(f"  找到 Qt6: {qt6_dir}")
        else:
            log("  警告：未找到 Qt6，跳过 LibreVNA IPC 构建")
            log("  安装命令: pacman -S mingw-w64-ucrt-x86_64-qt6-base")
            return False

    # CMake 配置
    cmake_cmd = [
        "cmake",
        "-S", str(cmake_dir),
        "-B", str(build_dir),
        "-G", "Ninja",
        "-DCMAKE_BUILD_TYPE=Release",
        f"-DQt6_DIR={qt6_dir}",
    ]

    # 添加 libusb（如果存在）
    vcpkg_installed = REPO_ROOT.parent / "vcpkg" / "installed"
    if vcpkg_installed.exists():
        libusb_include = vcpkg_installed / "x64-windows" / "include"
        libusb_lib = vcpkg_installed / "x64-windows" / "lib" / "libusb-1.0.lib"
        if libusb_include.exists() and libusb_lib.exists():
            cmake_cmd += [
                f"-DLIBUSB_INCLUDE_DIR={libusb_include}",
                f"-DLIBUSB_LIBRARY={libusb_lib}",
            ]

    try:
        log("  CMake 配置...")
        subprocess.run(cmake_cmd, env=build_env, check=True, cwd=REPO_ROOT,
                       stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        log("  编译 librevna-ipc...")
        subprocess.run(
            ["cmake", "--build", str(build_dir), "--target", "librevna-ipc"],
            env=build_env, check=True, cwd=REPO_ROOT,
            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
        )
    except subprocess.CalledProcessError as e:
        log(f"  LibreVNA IPC 构建失败: {e}")
        return False

    if not ipc_exe.exists():
        log("  错误：librevna-ipc.exe 未生成")
        return False

    # windeployqt 收集 Qt 运行时 DLL
    if windeployqt.exists():
        log("  收集 Qt 运行时 DLL（windeployqt6）...")
        subprocess.run(
            [str(windeployqt), "--no-translations", str(ipc_exe)],
            env=build_env, cwd=str(build_dir),
            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
        )

    log(f"  LibreVNA IPC 构建完成: {ipc_exe}")
    return True


# ----------------------------------------------------------------------
# 核心构建步骤
# ----------------------------------------------------------------------

def build_gui() -> bool:
    """构建 64位 GUI 可执行文件（PyInstaller）。"""
    log("===== 构建 GUI (64位 PyInstaller) =====")

    # 选择 Python
    gui_python_env = os.environ.get("GUI_PYTHON")
    if gui_python_env:
        gui_py = Path(gui_python_env)
    elif VENV_X64.exists():
        gui_py = python_exe(VENV_X64)
    elif SYSTEM_PYTHON:
        gui_py = Path(SYSTEM_PYTHON)
    else:
        gui_py = Path(shutil.which("python") or "python")

    log(f"  使用 Python: {gui_py}")

    ensure_pyinstaller(gui_py)

    spec = BUILD_DIR / "lmx2594_gui.spec"
    if not spec.exists():
        log(f"  错误：找不到 spec 文件: {spec}")
        return False

    gui_dist = DIST_DIR / "lmx2594_gui"

    # 清理旧的构建产物
    if gui_dist.exists():
        log("  清理旧 GUI 构建...")
        shutil.rmtree(gui_dist, ignore_errors=True)

    log(f"  运行 PyInstaller ({gui_py} -m PyInstaller)...")
    try:
        run([str(gui_py), "-m", "PyInstaller", str(spec),
             "--distpath", str(DIST_DIR),
             "--workpath", str(BUILD_DIR / "lmx2594_gui-build"),
             "--noconfirm"],
            env={**os.environ, "REPO_ROOT": str(REPO_ROOT)})
    except subprocess.CalledProcessError as e:
        log(f"  GUI 构建失败: {e}")
        return False

    if not gui_dist.exists():
        log("  错误：PyInstaller 未生成输出目录")
        return False

    # 收集 Qt 插件（PyInstaller hooks 有时会遗漏）
    log("  收集 Qt 插件...")
    _collect_qt_plugins(gui_dist, gui_py)

    log(f"  GUI 构建完成: {gui_dist / 'LMX2594-Calibration.exe'}")
    return True


def _collect_qt_plugins(gui_dist: Path, gui_py: Path) -> None:
    """
    收集 Qt 运行时插件到 PyInstaller 输出目录。
    使用 windeployqt6（来自 MSYS2）收集所有必要的 Qt 文件。
    """
    gui_exe = gui_dist / "LMX2594-Calibration.exe"
    internal_dir = gui_dist / "_internal"

    # 尝试使用 windeployqt6（来自 MSYS2）
    msys2_info = _find_msys2_installation()
    if msys2_info:
        msys2_bin, _ = msys2_info
        windeployqt = msys2_bin / "windeployqt6.exe"
        if windeployqt.exists():
            try:
                build_env = os.environ.copy()
                build_env["PATH"] = f"{msys2_bin};" + build_env.get("PATH", "")
                log(f"    使用 windeployqt6 收集 Qt 插件...")
                subprocess.run(
                    [str(windeployqt), "--no-translations", "--no-compiler",
                     "--dir", str(internal_dir), str(gui_exe)],
                    env=build_env, check=True,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True
                )
                log("    windeployqt6 成功收集 Qt 插件")
                return
            except subprocess.CalledProcessError as e:
                log(f"    windeployqt6 失败: {e}")

    # 备用方法：手动复制
    _copy_qt_plugins_manual(internal_dir, gui_py)


def _copy_qt_plugins_manual(internal_dir: Path, gui_py: Path) -> None:
    """手动复制 Qt 插件到 _internal 目录。"""
    import site

    # 首先尝试从 venv 的 site-packages 获取
    venv_path = gui_py.parent.parent  # Scripts -> venv root
    venv_site_packages = venv_path / "Lib" / "site-packages"
    if venv_site_packages.exists():
        site_packages = str(venv_site_packages)
    else:
        site_packages = next((p for p in site.getsitepackages() if 'site-packages' in p), "")

    if not site_packages:
        log("    警告：无法找到 site-packages，跳过 Qt 插件收集")
        return

    log(f"    搜索 Qt 插件在: {site_packages}")

    # 查找 PySide6 插件位置
    pyside6_dir = Path(site_packages) / "PySide6"
    plugins_src = pyside6_dir / "plugins"

    if not plugins_src.exists():
        log(f"    警告：找不到 Qt 插件目录: {plugins_src}")
        return

    # 复制 plugins 目录到 _internal/PySide6/plugins
    plugins_dst = internal_dir / "PySide6" / "plugins"
    if plugins_dst.exists():
        log(f"    Qt 插件已存在，跳过复制")
        return

    try:
        plugins_dst.parent.mkdir(parents=True, exist_ok=True)
        _copy_dir_safe(plugins_src, plugins_dst)
        log(f"    复制 Qt 插件: {plugins_src} -> {plugins_dst}")

        # 也复制 PySide6 根目录下的 DLL 文件
        for dll in pyside6_dir.glob("*.dll"):
            shutil.copy2(dll, internal_dir / dll.name)
        log(f"    复制 PySide6 DLL 文件到 _internal")

    except Exception as e:
        log(f"    警告：复制 Qt 插件失败: {e}")


def _copy_dir_safe(src: Path, dst: Path) -> None:
    """安全复制目录。"""
    import shutil
    if dst.exists():
        log(f"    目标已存在: {dst}")
        return
    shutil.copytree(src, dst)


def build_worker() -> bool:
    """构建 32位 worker 可执行文件（PyInstaller）。"""
    log("===== 构建 Worker (32位 PyInstaller) =====")

    # 选择 32位 Python
    if not VENV_X86.exists():
        log("  错误：32位 venv 不存在: .venv-x86/")
        log("  请先创建 32位环境：")
        log("    uv python install cpython-3.10.11-windows-x86-none")
        log("    uv venv --python cpython-3.10.11-windows-x86-none .venv-x86")
        log("    .venv-x86\\Scripts\\activate")
        log("    pip install pyserial")
        return False

    worker_py = python_exe(VENV_X86)
    log(f"  使用 Python: {worker_py}")

    ensure_pyinstaller(worker_py)

    spec = BUILD_DIR / "lmx2594_worker.spec"
    if not spec.exists():
        log(f"  错误：找不到 spec 文件: {spec}")
        return False

    worker_dist = DIST_DIR / "lmx2594_worker"

    # 清理旧的构建产物
    if worker_dist.exists():
        log("  清理旧 Worker 构建...")
        shutil.rmtree(worker_dist, ignore_errors=True)

    log(f"  运行 PyInstaller (32位)...")
    try:
        run([str(worker_py), "-m", "PyInstaller", str(spec),
             "--distpath", str(DIST_DIR),
             "--workpath", str(BUILD_DIR / "lmx2594_worker-build"),
             "--noconfirm"],
            env={**os.environ, "REPO_ROOT": str(REPO_ROOT)})
    except subprocess.CalledProcessError as e:
        log(f"  Worker 构建失败: {e}")
        return False

    if not worker_dist.exists():
        log("  错误：PyInstaller 未生成输出目录")
        return False

    log(f"  Worker 构建完成: {worker_dist / 'lmx2594_worker.exe'}")
    return True


# ----------------------------------------------------------------------
# 复制 / 整理最终发行目录
# ----------------------------------------------------------------------

def prepare_release_dir() -> None:
    """准备最终发行目录 dist/LMX2594-Release/。"""
    log("===== 准备发行目录 =====")

    if RELEASE_DIR.exists():
        log(f"  清理旧发行目录: {RELEASE_DIR}")
        shutil.rmtree(RELEASE_DIR, ignore_errors=True)

    RELEASE_DIR.mkdir(parents=True, exist_ok=True)

    # 1. 复制 GUI 产物
    gui_dist = DIST_DIR / "lmx2594_gui"
    if gui_dist.exists():
        _copy_dir(gui_dist, RELEASE_DIR)
        log(f"  复制 GUI -> {RELEASE_DIR}")
    else:
        log("  警告：未找到 GUI 构建产物，跳过")

    # 2. 复制 Worker 产物到子目录（必须与 GUI 分离：两套 _internal 不能合并，
    #    否则 32 位 worker 的 .pyd 会覆盖 64 位 GUI，导致
    #    “Failed to start embedded python interpreter”）
    worker_dist = DIST_DIR / "lmx2594_worker"
    worker_dest = RELEASE_DIR / "lmx2594_worker"
    if worker_dist.exists():
        _copy_dir(worker_dist, worker_dest)
        log(f"  复制 Worker -> {worker_dest}")
    else:
        log("  警告：未找到 Worker 构建产物，跳过")

    # 3. 复制 USB2ANY.dll
    usb2any_src = REPO_ROOT / "USB2ANY.dll"
    if usb2any_src.exists():
        shutil.copy2(usb2any_src, RELEASE_DIR / "USB2ANY.dll")
        log(f"  复制 USB2ANY.dll")
    else:
        log("  警告：USB2ANY.dll 未找到")

    # 4. 复制校准目录
    cal_src = REPO_ROOT / "calibration"
    if cal_src.exists():
        _copy_dir(cal_src, RELEASE_DIR / "calibration")
        log(f"  复制 calibration/")

    # 5. 复制 examples 目录
    examples_src = REPO_ROOT / "examples"
    if examples_src.exists():
        _copy_dir(examples_src, RELEASE_DIR / "examples")
        log(f"  复制 examples/")

    # 6. 复制 LibreVNA IPC 及 Qt6 运行时库（如果存在）
    ipc_candidates = [
        REPO_ROOT / "scripts" / "vna" / "cpp" / "build" / "Release" / "librevna-ipc.exe",
        REPO_ROOT / "scripts" / "vna" / "cpp" / "build" / "RelWithDebInfo" / "librevna-ipc.exe",
        REPO_ROOT / "scripts" / "vna" / "cpp" / "build" / "librevna-ipc.exe",
    ]
    ipc_found = False
    ipc_dir = None
    for ipc_path in ipc_candidates:
        if ipc_path.exists():
            ipc_dir = ipc_path.parent
            target = RELEASE_DIR / "scripts" / "vna" / "cpp" / "build" / ipc_path.name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(ipc_path, target)
            log(f"  复制 librevna-ipc.exe -> {target}")
            ipc_found = True

            # MinGW Qt6：DLL + platforms，且 windeployqt6 与 PySide 插件不可混用
            _copy_qt6_dlls_for_ipc(ipc_dir, target.parent)
            _windeploy_qt6_for_exe(target)
            break
    if not ipc_found:
        log("  提示：未找到 librevna-ipc.exe（可选）")

    # 7. 复制 requirements 文件
    for req_file in ["requirements.txt", "requirements-x86.txt"]:
        src = REPO_ROOT / req_file
        if src.exists():
            shutil.copy2(src, RELEASE_DIR / req_file)
            log(f"  复制 {req_file}")

    # 8. 写入 README
    _write_readme()
    log("  生成 README.md")

    # 9. 生成 run_gui.bat
    _write_run_bat()
    log("  生成 run_gui.bat")


def _copy_dir(src: Path, dst: Path) -> None:
    """递归复制目录，跳过 .git 等特殊目录。"""
    skip_dirs = {".git", ".svn", ".hg", "__pycache__", ".venv", ".eggs"}
    for item in src.rglob("*"):
        # 跳过 .git 等目录
        if any(part in skip_dirs for part in item.parts):
            continue
        if item.is_file():
            rel = item.relative_to(src)
            dst_file = dst / rel
            dst_file.parent.mkdir(parents=True, exist_ok=True)
            try:
                shutil.copy2(item, dst_file)
            except PermissionError:
                # 忽略只读/锁定的文件
                pass


def _windeploy_qt6_for_exe(exe_path: Path) -> None:
    """对 MinGW 构建的 Qt 可执行文件运行 windeployqt6，收集匹配的插件与 DLL。"""
    msys2_info = _find_msys2_installation()
    if not msys2_info:
        return
    msys2_bin, _ = msys2_info
    windeployqt = msys2_bin / "windeployqt6.exe"
    if not windeployqt.exists():
        return
    env = os.environ.copy()
    env["PATH"] = f"{msys2_bin};{env.get('PATH', '')}"
    try:
        r = subprocess.run(
            [str(windeployqt), "--no-translations", str(exe_path)],
            env=env,
            cwd=str(exe_path.parent),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )
        if r.returncode == 0:
            log(f"    windeployqt6 完成: {exe_path.name}")
        else:
            log(f"    windeployqt6 退出码 {r.returncode}（可忽略若 IPC 仍能运行）: {exe_path.name}")
    except Exception as exc:
        log(f"    windeployqt6 失败: {exc}")


def _msys2_share_qt6_plugins() -> Optional[Path]:
    info = _find_msys2_installation()
    if not info:
        return None
    msys2_bin, _ = info
    p = msys2_bin.parent / "share" / "qt6" / "plugins"
    return p if p.is_dir() else None


def _copy_qt6_dlls_for_ipc(ipc_src_dir: Path, ipc_dest_dir: Path) -> None:
    """复制 Qt6 运行时 DLL 与平台插件到 IPC 目标目录（须与 MinGW 可执行文件匹配）。"""
    copied_any = False

    # 源码 build 目录中 windeployqt 生成的 platforms/（与 exe 同级）
    platforms_src = ipc_src_dir / "platforms"
    if platforms_src.is_dir():
        platforms_dst = ipc_dest_dir / "platforms"
        try:
            shutil.copytree(platforms_src, platforms_dst, dirs_exist_ok=True)
            log(f"    已复制 platforms <- {platforms_src}")
            copied_any = True
        except Exception:
            pass

    plugins_src = ipc_src_dir / "plugins"
    if plugins_src.is_dir():
        plugins_dst = ipc_dest_dir / "plugins"
        try:
            shutil.copytree(plugins_src, plugins_dst, dirs_exist_ok=True)
            log(f"    已复制 plugins <- {plugins_src}")
            copied_any = True
        except Exception:
            pass

    qwindows = ipc_dest_dir / "platforms" / "qwindows.dll"
    if not qwindows.is_file():
        share_plugins = _msys2_share_qt6_plugins()
        if share_plugins and (share_plugins / "platforms").is_dir():
            dst_plat = ipc_dest_dir / "platforms"
            try:
                shutil.copytree(share_plugins / "platforms", dst_plat, dirs_exist_ok=True)
                log(f"    已从 MSYS2 复制 platforms -> {dst_plat}")
                copied_any = True
            except Exception:
                pass

    # Qt6 DLL 通常在 MSYS2 的 ucrt64/bin 目录
    msys2_info = _find_msys2_installation()
    if msys2_info:
        msys2_bin, _ = msys2_info
        qt6_dlls = list(msys2_bin.glob("Qt6*.dll"))
        if qt6_dlls:
            log(f"    复制 Qt6 DLLs 从 MSYS2 -> {ipc_dest_dir}")
            for dll in qt6_dlls:
                try:
                    shutil.copy2(dll, ipc_dest_dir / dll.name)
                    copied_any = True
                except Exception:
                    pass

            # 也复制必要的依赖 DLL (mingw runtime, ssl, etc.)
            for dll_name in ["libgcc_s_seh-1.dll", "libstdc++-6.dll", "libwinpthread-1.dll",
                            "libssl-3.dll", "libcrypto-3.dll", "zlib1.dll", "libpng16-16.dll",
                            "libfreetype-6.dll", "libharfbuzz-0.dll", "libglib-2.0-0.dll",
                            "libgthread-2.0-0.dll", "glib-2.dll", "pcre2-8.dll"]:
                dll_path = msys2_bin / dll_name
                if dll_path.exists():
                    try:
                        shutil.copy2(dll_path, ipc_dest_dir / dll_name)
                    except Exception:
                        pass

    if copied_any:
        log(f"    Qt6 运行时 DLL 已复制到 {ipc_dest_dir}")
    else:
        log(f"    警告：未找到 Qt6 DLL，IPC 可能无法运行（需要安装 MSYS2 Qt6）")


def _write_readme() -> None:
    """生成发行版 README.md。"""
    readme = RELEASE_DIR / "README.md"
    readme.write_text(
        "# LMX2594 Calibration Tools\n\n"
        "Python tools for the LMX2594 synthesizer with a tinySA Ultra "
        "calibration GUI and CLI utilities.\n\n"
        "## 运行\n\n"
        "直接双击 `run_gui.bat` 启动 GUI。\n\n"
        "首次运行时，GUI 会自动发现 `lmx2594_worker/lmx2594_worker.exe`\n"
        "（与 GUI 同级的子目录）作为 USB2ANY 控制进程。\n\n"
        "## 硬件要求\n\n"
        "- LMX2594 评估板（5V）\n"
        "- USB2ANY 编程器（SPI）\n"
        "- 50 MHz 参考（默认）\n"
        "- tinySA Ultra（可选，用于校准）\n\n"
        "## 目录结构\n\n"
        "```\n"
        "LMX2594-Calibration.exe   GUI 主程序（64位）\n"
        "_internal/                GUI 运行时（勿与 worker 合并）\n"
        "lmx2594_worker/           32位 Worker 及其 _internal/\n"
        "USB2ANY.dll               USB2ANY 驱动 DLL\n"
        "calibration/              校准数据\n"
        "examples/                 示例数据\n"
        "scripts/vna/cpp/build/    LibreVNA IPC 工具（可选）\n"
        "```\n\n"
        "## 提示\n\n"
        "- tinySA Ultra 需在 CONFIG -> Connection -> USB 模式下使用 USB 线连接。\n"
        "- LibreVNA IPC 工具为可选组件，需要单独构建 Qt6 版本。\n"
        "- 如 USB2ANY 无法识别，检查 USB 驱动和 SPI 连线。\n",
        encoding="utf-8",
    )


def _write_run_bat() -> None:
    """生成可直接双击的启动批处理。"""
    bat = RELEASE_DIR / "run_gui.bat"
    bat.write_text(
        '@echo off\n'
        'chcp 65001 >nul\n'
        'setlocal EnableDelayedExpansion\n'
        '\n'
        'set "RELEASE=%~dp0"\n'
        '\n'
        'REM Worker exe (packaged layout: lmx2594_worker\\\\ subfolder)\n'
        'if exist "%RELEASE%lmx2594_worker\\lmx2594_worker.exe" (\n'
        '    set "WORKER_PYTHON=%RELEASE%lmx2594_worker\\lmx2594_worker.exe"\n'
        ') else (\n'
        '    if exist "%RELEASE%lmx2594_worker.exe" (\n'
        '        set "WORKER_PYTHON=%RELEASE%lmx2594_worker.exe"\n'
        '    ) else (\n'
        '        set "WORKER_PYTHON=%RELEASE%.venv-x86\\Scripts\\python.exe"\n'
        '    )\n'
        ')\n'
        '\n'
        'REM Qt 插件路径设置 - 优先使用 PyInstaller 内部插件\n'
        'if exist "%RELEASE%_internal\\PySide6\\plugins\\platforms\\qwindows.dll" (\n'
        '    set "QT_PLUGIN_PATH=%RELEASE%_internal\\PySide6\\plugins"\n'
        '    set "QT_QPA_PLATFORM_PLUGIN_PATH=%RELEASE%_internal\\PySide6\\plugins\\platforms"\n'
        ') else if exist "%RELEASE%_internal\\platforms\\qwindows.dll" (\n'
        '    set "QT_PLUGIN_PATH=%RELEASE%_internal"\n'
        '    set "QT_QPA_PLATFORM_PLUGIN_PATH=%RELEASE%_internal\\platforms"\n'
        ') else if exist "%RELEASE%_internal\\PyQt6\\plugins\\platforms\\qwindows.dll" (\n'
        '    set "QT_PLUGIN_PATH=%RELEASE%_internal\\PyQt6\\plugins"\n'
        '    set "QT_QPA_PLATFORM_PLUGIN_PATH=%RELEASE%_internal\\PyQt6\\plugins\\platforms"\n'
        ')\n'
        '\n'
        'REM 强制使用 Windows 平台插件\n'
        'set "QT_QPA_PLATFORM=windows"\n'
        '\n'
        'REM 设置 PyInstaller 内部路径（供运行时使用）\n'
        'set "PYINSTALLER_LMX_INTERNAL=%RELEASE%_internal"\n'
        '\n'
        'start "" "%RELEASE%LMX2594-Calibration.exe"\n'
        '\n'
        'endlocal\n',
        encoding="utf-8",
    )


# ----------------------------------------------------------------------
# 清理
# ----------------------------------------------------------------------

def clean() -> None:
    """清理所有构建产物。"""
    log("===== 清理构建产物 =====")

    for d in [
        BUILD_DIR / "lmx2594_gui",
        BUILD_DIR / "lmx2594_gui-build",
        BUILD_DIR / "lmx2594_worker",
        BUILD_DIR / "lmx2594_worker-build",
        DIST_DIR / "lmx2594_gui",
        DIST_DIR / "lmx2594_worker",
    ]:
        if d.exists():
            log(f"  删除 {d}")
            shutil.rmtree(d, ignore_errors=True)

    # 清理 PyInstaller 生成的 .spec.bak 文件
    for spec_file in BUILD_DIR.glob("*.spec"):
        bak = spec_file.with_suffix(".spec.bak")
        if bak.exists():
            bak.unlink()

    # 清理 Python __pycache__
    for cache in [
        *REPO_ROOT.rglob("__pycache__"),
        *REPO_ROOT.rglob("*.pyc"),
    ]:
        try:
            if cache.is_file():
                cache.unlink()
            elif cache.is_dir():
                shutil.rmtree(cache, ignore_errors=True)
        except Exception:
            pass

    log("清理完成")


# ----------------------------------------------------------------------
# 主入口
# ----------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="LMX2594 打包构建脚本")
    parser.add_argument("--gui-only", action="store_true",
                        help="仅构建 GUI")
    parser.add_argument("--worker-only", action="store_true",
                        help="仅构建 Worker")
    parser.add_argument("--clean", action="store_true",
                        help="构建前清理所有产物")
    parser.add_argument("--skip-vna", action="store_true",
                        help="跳过 LibreVNA IPC 构建")
    args = parser.parse_args()

    print("=" * 60)
    print("  LMX2594 Calibration 打包构建")
    print(f"  仓库根目录: {REPO_ROOT}")
    print(f"  输出目录:   {RELEASE_DIR}")
    print("=" * 60)

    if args.clean:
        clean()
        print()
        if not (args.gui_only or args.worker_only):
            print("  清理完成，现在运行: python build.py")
            return

    # 构建 LibreVNA IPC（可选）
    if not args.skip_vna:
        build_librevna_ipc()

    gui_ok = True
    worker_ok = True

    if args.worker_only:
        worker_ok = build_worker()
    elif args.gui_only:
        gui_ok = build_gui()
    else:
        gui_ok = build_gui()
        if gui_ok:
            worker_ok = build_worker()

    # 整理发行目录
    prepare_release_dir()

    # 最终状态
    print()
    print("=" * 60)
    if gui_ok and worker_ok:
        print("  构建完成！")
        print(f"  发行目录: {RELEASE_DIR}")
        print()
        print("  复制整个 LMX2594-Release/ 文件夹到目标电脑，")
        print("  双击 run_gui.bat 即可运行。")
    else:
        print("  构建有错误，请检查上面的输出。")
        if not gui_ok:
            print("  - GUI 构建失败")
        if not worker_ok:
            print("  - Worker 构建失败")
    print("=" * 60)


if __name__ == "__main__":
    main()
