# LMX2594 Calibration 打包指南

本文档说明如何将 LMX2594 Calibration 打包为可在任何 Windows 电脑上直接双击运行的独立程序。

---

## 打包产物一览

| 产物 | 说明 | 是否必需 |
|------|------|---------|
| `LMX2594-Calibration.exe` | GUI 主程序（64位） | ✅ 必须 |
| `lmx2594_worker.exe` | USB2ANY 控制进程（32位） | ✅ 必须 |
| `librevna-ipc.exe` | LibreVNA 控制工具 | ⚠️ 可选 |
| `USB2ANY.dll` | USB2ANY 驱动 DLL | ✅ 必须 |

---

## 硬件说明

本软件涉及两套不同的硬件接口，需要不同位宽的运行环境：

```
┌─────────────────────────────────────────────┐
│  LMX2594-Calibration.exe  (64位 Python)      │
│                                             │
│  ├── GUI (PySide6 / PyQt5)                 │
│  ├── tinySA 控制                             │
│  └── lmx2594_worker.exe  ←── 32位进程 ──┐   │
│                                       │     │
│  LibreVNA IPC  ←── 可选 ─────────────┘     │
└─────────────────────────────────────────────┘
         │
         ▼
  USB2ANY DLL  (32位 DLL 接口)
```

- **GUI 必须是 64位**：PySide6/PyQt5 只提供 64位预编译包
- **Worker 必须是 32位**：USB2ANY 的 DLL 接口是 32位的
- **LibreVNA IPC**：Qt C++ 应用，独立运行

---

## 环境准备（一次性操作）

### 1. 安装 Python

#### 64位 Python（GUI 环境）

推荐使用 [uv](https://github.com/astral-sh/uv) 管理 Python：

```powershell
# 安装 64位 Python 3.13
uv python install 3.13.3
uv venv --python 3.13.3 .venv-x64
.venv-x64\Scripts\activate

# 安装 GUI 依赖
pip install -r requirements.txt

# 安装 PyInstaller（打包工具）
pip install pyinstaller
```

#### 32位 Python（Worker 环境）

USB2ANY 必须用 32位 Python 控制：

```powershell
# 安装 32位 Python 3.10（最后一个有官方 32位版的 Python）
uv python install cpython-3.10.11-windows-x86-none
uv venv --python cpython-3.10.11-windows-x86-none .venv-x86
.venv-x86\Scripts\activate

# Worker 只依赖 pyserial
pip install pyserial
pip install pyinstaller
```

> **注意**：`cpython-3.10.11-windows-x86-none` 中的 `none` 表示无 ABI 标签，uv会自动选择兼容版本。

### 2. 确认 MSYS2 Qt6（用于 LibreVNA IPC，可选）

LibreVNA IPC 需要 Qt6。如果你的系统没有安装 Qt6，跳过这一步，之后打包脚本会自动检测并跳过 LibreVNA IPC。

```powershell
# 在 MSYS2 UCRT64 终端中安装（如果没有的话）
pacman -S mingw-w64-ucrt-x86_64-qt6-base mingw-w64-ucrt-x86_64-libusb ninja cmake
```

确认 Qt6 可用：

```powershell
# 应该输出 Qt 版本号
C:\msys64\ucrt64\bin\windeployqt6.exe --version
```

---

## 完整打包

所有环境准备完成后，在仓库根目录运行：

```powershell
python build.py
```

这会自动完成以下全部步骤：

1. 编译 LibreVNA IPC（如 Qt6 可用）
2. 使用 PyInstaller 构建 64位 GUI → `dist/lmx2594_gui/`
3. 使用 PyInstaller 构建 32位 Worker → `dist/lmx2594_worker/`
4. 整理输出目录 → `dist/LMX2594-Release/`

---

## 打包输出目录结构

```
dist/LMX2594-Release/
│
├── LMX2594-Calibration.exe       ← GUI 主程序（双击启动）
├── lmx2594_worker.exe            ← USB2ANY 控制进程（GUI 自动发现）
├── USB2ANY.dll                   ← USB2ANY 驱动
├── run_gui.bat                   ← 启动脚本（可双击）
│
├── calibration/                  ← 校准数据
├── examples/                     ← 示例数据
│
├── scripts/
│   └── vna/cpp/build/
│       ├── librevna-ipc.exe      ← LibreVNA IPC 工具
│       ├── Qt6Core.dll
│       ├── Qt6Gui.dll
│       ├── Qt6Network.dll
│       ├── Qt6Widgets.dll
│       ├── Qt6Svg.dll
│       ├── D3Dcompiler_47.dll
│       ├── libusb-1.0.dll
│       └── ...
│
├── requirements.txt
├── requirements-x86.txt
└── README.md
```

---

## 常见构建选项

```powershell
# 仅构建 GUI（跳过 Worker）
python build.py --gui-only

# 仅构建 Worker（跳过 GUI）
python build.py --worker-only

# 跳过 LibreVNA IPC 编译
python build.py --skip-vna

# 清理所有构建产物后重新构建
python build.py --clean

# 组合使用
python build.py --clean --skip-vna
```

---

## 手动分步构建（不推荐，仅供调试）

如果 `build.py` 遇到问题，可以手动执行各步骤。

### 步骤 1：构建 LibreVNA IPC（可选）

```powershell
$env:PATH = "C:\msys64\ucrt64\bin;" + $env:PATH

cmake -S scripts\vna\cpp -B scripts\vna\cpp\build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build scripts\vna\cpp\build --target librevna-ipc

# 收集 Qt 运行时 DLL
C:\msys64\ucrt64\bin\windeployqt6.exe --no-translations scripts\vna\cpp\build\librevna-ipc.exe
```

### 步骤 2：构建 GUI

```powershell
.venv-x64\Scripts\activate
pyinstaller build\lmx2594_gui.spec --distpath dist --noconfirm
```

### 步骤 3：构建 Worker

```powershell
.venv-x86\Scripts\activate
pyinstaller build\lmx2594_worker.spec --distpath dist --noconfirm
```

### 步骤 4：整理输出

手动将 `dist/lmx2594_gui/` 和 `dist/lmx2594_worker/` 的内容复制到 `dist/LMX2594-Release/`。

---

## 分发给最终用户

1. 将整个 `dist/LMX2594-Release/` 文件夹复制到目标电脑
2. 双击 `run_gui.bat` 启动
3. 首次启动时，在 GUI 的 Hidden Settings 中确认 Worker Python 路径：
   - 默认已自动填入 `lmx2594_worker.exe`（同目录下）
   - GUI 会自动发现，无需手动配置

---

## 故障排除

### "No module named 'PyInstaller'"

```powershell
.venv-x64\Scripts\activate
pip install pyinstaller
```

### "MSYS2 moc.exe failed"（LibreVNA IPC 编译失败）

CMake 无法运行 Qt6 的 moc 工具。需要将 MSYS2 的 bin 目录加入 PATH：

```powershell
$env:PATH = "C:\msys64\ucrt64\bin;" + $env:PATH
cmake -S scripts\vna\cpp -B scripts\vna\cpp\build ...
```

或在 MSYS2 UCRT64 终端中运行 cmake。

### "No USB2ANY controllers found"

- 确认 USB2ANY 已通过 USB 连接
- 确认 `USB2ANY.dll` 在主程序同目录下
- 确认 `lmx2594_worker.exe` 在主程序同目录下

### tinySA 超时

在 tinySA 上设置 `CONFIG → Connection → USB`，然后重新插拔设备。

### GUI 启动后无响应或闪退

查看 `librevna-ipc.log` 文件（与 exe 同目录）获取详细错误信息。

---

## 技术细节

### 为什么需要两个独立的 exe？

| 模块 | 位宽 | 原因 |
|------|------|------|
| GUI | 64位 | PySide6/PyQt5 只提供 64位预编译包 |
| Worker | 32位 | USB2ANY.dll 是 32位 DLL |
| LibreVNA IPC | 64位 | Qt6 应用，无位宽限制 |

### PyInstaller 为什么不用 --onefile？

`--onedir`（文件夹模式）更适合本项目，因为：

- GUI 进程和 Worker 进程是独立的可执行文件
- 避免 PATH 问题
- 方便手动替换单个 DLL
- 启动速度更快（无需解压）

### LibreVNA IPC 为什么不用 PyInstaller？

LibreVNA IPC 是纯 C++ Qt 应用，编译为原生 Windows 可执行文件，已经可以直接双击运行，无需 Python 打包。
