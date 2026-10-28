# STM32F407 固件编译脚本
# 用法:
#   .\build.ps1              # 增量编译
#   .\build.ps1 -Clean       # 清理后全量重编
#   .\build.ps1 -Download    # 编译成功后用 J-Link 烧录
param(
    [switch]$Clean,
    [switch]$Download,
    [int]$Jobs = 24
)

$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

if (-not (Get-Command mingw32-make -ErrorAction SilentlyContinue)) {
    Write-Host "[error] 未找到 mingw32-make，请确认 msys2/mingw64 已加入 PATH" -ForegroundColor Red
    exit 1
}

if ($Clean -and (Test-Path "build")) {
    Remove-Item -Recurse -Force "build"
    Write-Host "[clean] 已删除 build 目录" -ForegroundColor Yellow
}

Write-Host "[build] mingw32-make -j$Jobs ..." -ForegroundColor Cyan
& mingw32-make "-j$Jobs"
if ($LASTEXITCODE -ne 0) {
    Write-Host "[build] 编译失败 (exit $LASTEXITCODE)" -ForegroundColor Red
    exit $LASTEXITCODE
}
Write-Host "[build] 编译成功" -ForegroundColor Green

if ($Download) {
    $jlink = "D:\JLink_V812\JLink.exe"
    if (-not (Test-Path $jlink)) {
        Write-Host "[error] 未找到 JLink.exe: $jlink" -ForegroundColor Red
        exit 1
    }
    Write-Host "[flash] J-Link 烧录 ..." -ForegroundColor Cyan
    & $jlink -device STM32F407IG -if SWD -speed 4000 -autoconnect 1 -NoGui 1 -ExitOnError 1 -CommandFile flash.jlink
    if ($LASTEXITCODE -ne 0) {
        Write-Host "[flash] 烧录失败 (exit $LASTEXITCODE)" -ForegroundColor Red
        exit $LASTEXITCODE
    }
    Write-Host "[flash] 烧录完成，程序已复位运行" -ForegroundColor Green
}
