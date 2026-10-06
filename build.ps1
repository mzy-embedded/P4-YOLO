# P4-YOLO 构建脚本 (ESP32-P4 v3.x / WT9932P4X-TINY)。
# Usage:
#   .\build.ps1                 -> build
#   .\build.ps1 menuconfig      -> open menuconfig
#   .\build.ps1 fullclean       -> clean
#   .\build.ps1 -B build size   -> any idf.py args are forwarded
#
# This wraps the ESP-IDF v6.1 environment needed on this machine:
#  - Git Bash (MSYSTEM) is not supported by IDF v6.1, so MSYSTEM is removed
#  - tool binaries are added to PATH explicitly
#  - ESP_ROM_ELF_DIR is set (needed to generate esp_rom gdbinit)
param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$IdfArgs
)

$ErrorActionPreference = "Stop"

$IdfRoot   = "C:\Espressif\frameworks\esp-idf-v6.1"
$ToolsPath = "C:\Espressif"
$PyEnv     = "C:\Espressif\python_env\idf6.1_py3.11_env"
$Python    = "$PyEnv\Scripts\python.exe"

$env:IDF_PATH            = $IdfRoot
$env:IDF_TOOLS_PATH      = $ToolsPath
$env:IDF_PYTHON_ENV_PATH = $PyEnv
$env:ESP_IDF_VERSION     = "6.1.0"
$env:PYTHONUTF8          = "1"
$env:ESP_ROM_ELF_DIR     = "C:\Espressif\tools\esp-rom-elfs\20241011\"

Remove-Item Env:MSYSTEM -ErrorAction SilentlyContinue

$ToolBin = "C:\Espressif\tools\riscv32-esp-elf\esp-15.2.0_20251204\riscv32-esp-elf\bin;" +
           "C:\Espressif\tools\cmake\4.0.3\bin;" +
           "C:\Espressif\tools\ninja\1.12.1;" +
           "$PyEnv\Scripts;"
$env:PATH = $ToolBin + $env:PATH

Set-Location (Split-Path -Parent $MyInvocation.MyCommand.Path)

if (-not $IdfArgs -or $IdfArgs.Count -eq 0) {
    $IdfArgs = @("build")
}

& $Python -u "$IdfRoot\tools\idf.py" -B build -DIDF_SKIP_SUBMODULE_CHECK=ON @IdfArgs
exit $LASTEXITCODE
