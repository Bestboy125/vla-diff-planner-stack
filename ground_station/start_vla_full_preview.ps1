param(
    [ValidateSet("OpenVLA", "Pi05", "Both")]
    [string]$Policy = "OpenVLA",
    [switch]$NoBrowser,
    [switch]$EnableLiveControl,
    [string]$LiveControlConfirmation = "",
    [switch]$ReplaceExistingBackend
)

$ErrorActionPreference = "Stop"
$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$localConfig = Join-Path $scriptRoot "ground_station.local.ps1"

if (-not (Test-Path -LiteralPath $localConfig)) {
    throw "Missing $localConfig. Copy and fill ground_station.local.ps1.example first."
}
. $localConfig

# Historical filename retained. Live output always needs explicit arguments;
# a value in the private config must never silently enable control.
if ($EnableLiveControl) {
    $requiredConfirmation = if ($env:LIVE_CONTROL_CONFIRMATION) { $env:LIVE_CONTROL_CONFIRMATION } else { "I_ACCEPT_REAL_FLIGHT_CONTROL" }
    if ($LiveControlConfirmation -ne $requiredConfirmation) {
        throw "Live-control confirmation does not match; nothing was started."
    }
}
Write-Host "Observation mode: $($env:VLA_OBSERVATION_MODE); live requested: $EnableLiveControl"

Write-Host "Starting VLA model backend..." -ForegroundColor Cyan
& (Join-Path $scriptRoot "start_vla_backend.ps1") -Policy $Policy

Write-Host "Starting operator console (live remains guarded by NTP, tokens and confirmation)..." -ForegroundColor Cyan
& (Join-Path $scriptRoot "start_operator_console.ps1") `
    -EnableLiveControl:$EnableLiveControl `
    -LiveControlConfirmation $LiveControlConfirmation `
    -ReplaceExistingBackend:$ReplaceExistingBackend `
    -NoBrowser:$NoBrowser
