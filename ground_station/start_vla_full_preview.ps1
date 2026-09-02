param(
    [ValidateSet("OpenVLA", "Pi05", "Both")]
    [string]$Policy = "OpenVLA",
    [switch]$NoBrowser
)

$ErrorActionPreference = "Stop"
$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$localConfig = Join-Path $scriptRoot "ground_station.local.ps1"

if (-not (Test-Path -LiteralPath $localConfig)) {
    throw "Missing $localConfig. Copy and fill ground_station.local.ps1.example first."
}
. $localConfig

Write-Host "Starting VLA model backend..." -ForegroundColor Cyan
& (Join-Path $scriptRoot "start_vla_backend.ps1") -Policy $Policy

Write-Host "Starting the safety-locked operator console..." -ForegroundColor Cyan
& (Join-Path $scriptRoot "start_operator_console.ps1") `
    -ReplaceExistingBackend `
    -NoBrowser:$NoBrowser
