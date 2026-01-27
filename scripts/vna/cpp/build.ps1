$ErrorActionPreference = "Stop"

param(
    [string]$BuildType = "Release",
    [switch]$Clean,
    [switch]$ConfigureOnly,
    [switch]$BuildOnly,
    [string]$Generator = "",
    [string]$QtDir = "",
    [string]$LibUsbInclude = "",
    [string]$LibUsbLib = ""
)

$source = $PSScriptRoot
$build = Join-Path $PSScriptRoot "build"

if ($Clean -and (Test-Path $build)) {
    Remove-Item -Recurse -Force $build
}

if (!(Test-Path $build)) {
    New-Item -ItemType Directory -Path $build | Out-Null
}

if (-not $BuildOnly) {
    $cmakeArgs = @("-S", $source, "-B", $build, "-D", "CMAKE_BUILD_TYPE=$BuildType", "-D", "LIBREVNA_HEADLESS_ENABLE_REAL_DRIVER=ON")
    if ($Generator) { $cmakeArgs += @("-G", $Generator) }
    if ($QtDir) { $cmakeArgs += @("-D", "Qt6_DIR=$QtDir") }
    if ($LibUsbInclude) { $cmakeArgs += @("-D", "LIBUSB_INCLUDE_DIR=$LibUsbInclude") }
    if ($LibUsbLib) { $cmakeArgs += @("-D", "LIBUSB_LIBRARY=$LibUsbLib") }

    cmake @cmakeArgs
}

if ($ConfigureOnly) {
    exit 0
}

cmake --build $build --config $BuildType
