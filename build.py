#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
LMX2594 打包构建脚本
=====================
将 LMX2594 Calibration GUI 和 32位 worker 打包为可直接双击运行的发行版。

前置条件
--------
1. 在 repo 根目录（LMX2594/）激活 64位 Python 环境（GUI 依赖）:
       .venv-x64\Scripts\activate
   或手动指定 Python:
       set GUI_PYTHON=C:\Python313\python.exe

2. 确认存在 32位 Python venv（用于 USB2ANY worker）:
       .venv-x86\Scripts\python.exe
   可通过以下方式创建:
       uv python install cpython-3.10.11-windows-x86-none
       uv venv --python cpython-3.10.11-windows-x86-none .venv-x86
       .venv-x86\Scripts\activate && pip install -r requirements-x86.txt

使用方法
--------
    python build.py              # 完整构建（GUI + worker）
    python build.py --gui-only   # 仅构建 GUI
    python build.py --worker-only # 仅构建 worker
    python build.py --clean      # 清理 dist/ 并重新构建

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
# 构建 LibreVNA IPC（可选）
# ----------------------------------------------------------------------

def build_librevna_ipc() -> bool:
    """
    Build LibreVNA IPC CLI 工具。
    使用 MSYS2 UCRT64 Qt6，收集 Qt DLL 并复制到源码 build 目录。
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

    # MSYS2 UCRT64 环境
    msys2_bin = r"C:\msys64\ucrt64\bin"
    msys2_lib = r"C:\msys64\ucrt64\lib"
    windeployqt = Path(msys2_bin) / "windeployqt6.exe"

    # 构建时 PATH 包含 MSYS2 工具链
    build_env = os.environ.copy()
    build_env["PATH"] = f"{msys2_bin};{msys2_lib};" + build_env.get("PATH", "")

    # 检测 Qt6
    qt6_dir_env = os.environ.get("Qt6_DIR", "")
    qt6_dir = Path(qt6_dir_env) if qt6_dir_env else None
    if not qt6_dir or not qt6_dir.exists():
        qt6_dir = _find_qt6_dir()
        if qt6_dir:
            log(f"  找到 Qt6: {qt6_dir}")
        else:
            log("  警告：未找到 Qt6，跳过 LibreVNA IPC 构建")
            log("  如需此功能，请安装 Qt6（MSYS2: pacman -S mingw-w64-ucrt-x86_64-qt6-base）")
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


def _find_qt6_dir() -> Optional[Path]:
    """搜索常见的 Qt6 安装路径，优先 MSYS2 UCRT64。"""
    candidates = [
        # MSYS2 UCRT64（最常见）
        Path("C:/msys64/ucrt64/lib/cmake/Qt6"),
        # 官方 Qt 安装
        Path("C:/Qt/6.6.3/msvc2022_64/lib/cmake/Qt6"),
        Path("C:/Qt/6.7.0/msvc2022_64/lib/cmake/Qt6"),
        Path("C:/Qt/6.8.0/msvc2022_64/lib/cmake/Qt6"),
        Path("C:/Qt/6.8.1/msvc2022_64/lib/cmake/Qt6"),
        Path("C:/Qt/6.8.2/msvc2022_64/lib/cmake/Qt6"),
        Path("C:/Qt/6.8.3/msvc2022_64/lib/cmake/Qt6"),
        Path("C:/Qt/6.9.0/msvc2022_64/lib/cmake/Qt6"),
    ]
    for p in candidates:
        if p.exists():
            return p
    return None


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

    log(f"  GUI 构建完成: {gui_dist / 'LMX2594-Calibration.exe'}")
    return True


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

    # 6. 复制 LibreVNA IPC（如果存在）
    ipc_candidates = [
        REPO_ROOT / "scripts" / "vna" / "cpp" / "build" / "Release" / "librevna-ipc.exe",
        REPO_ROOT / "scripts" / "vna" / "cpp" / "build" / "RelWithDebInfo" / "librevna-ipc.exe",
        REPO_ROOT / "scripts" / "vna" / "cpp" / "build" / "librevna-ipc.exe",
    ]
    ipc_found = False
    for ipc_path in ipc_candidates:
        if ipc_path.exists():
            target = RELEASE_DIR / "scripts" / "vna" / "cpp" / "build" / ipc_path.name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(ipc_path, target)
            log(f"  复制 librevna-ipc.exe -> {target}")
            ipc_found = True
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
        'setlocal\n'
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
        'REM PyInstaller 6: Qt plugins live under _internal\n'
        'set "QT_QPA_PLATFORM_PLUGIN_PATH=%RELEASE%_internal\\PySide6\\plugins\\platforms"\n'
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
