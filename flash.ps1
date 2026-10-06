# P4-YOLO 烧录脚本 (ESP32-P4 v3.x / WT9932P4X-TINY)。
# Usage:
#   .\flash.ps1                 -> flash to COM17
#   .\flash.ps1 -Port COM5      -> flash to another port
#   .\flash.ps1 -EraseAll       -> erase the whole flash first (recommended on first flash)
#
# 经 FUSB 口烧录。上电后 SC2336 画面经 CSI->ISP->RGB565 显示在 ST7789 屏上，
# 并在画面里叠加 YOLO 检测框，不需要连接电脑。
param(
    [string]$Port = "COM17",
    [int]$Baud = 460800,
    [switch]$EraseAll
)

$ErrorActionPreference = "Stop"

$Esptool = "C:\Espressif\python_env\idf6.1_py3.11_env\Scripts\esptool.exe"
$Root    = Split-Path -Parent $MyInvocation.MyCommand.Path
$Build   = Join-Path $Root "build"

$Bootloader = Join-Path $Build "bootloader\bootloader.bin"
$PartTable  = Join-Path $Build "partition_table\partition-table.bin"
$App        = Join-Path $Build "P4-YOLO.bin"

foreach ($f in @($Bootloader, $PartTable, $App)) {
    if (-not (Test-Path $f)) {
        throw "Missing $f - run .\build.ps1 first."
    }
}

$espArgs = @(
    "--chip", "esp32p4",
    "-p", $Port,
    "-b", $Baud,
    "--before", "default-reset",
    "--after", "hard-reset",
    "write-flash"
)
if ($EraseAll) { $espArgs += "--erase-all" }
$espArgs += @(
    "--flash-mode", "dio",
    "--flash-freq", "80m",
    "--flash-size", "16MB",
    "0x2000",  $Bootloader,
    "0x8000",  $PartTable,
    "0x10000", $App
)

& $Esptool @espArgs
exit $LASTEXITCODE
