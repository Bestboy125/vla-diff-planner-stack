param(
    [switch]$EnableLiveControl,
    [string]$LiveControlConfirmation = "",
    [switch]$NoBrowser,
    [switch]$ReplaceExistingBackend
)

$ErrorActionPreference = "Stop"
$groundStationRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$backendRoot = Join-Path $groundStationRoot "backend"
$frontendRoot = Join-Path $groundStationRoot "frontend"
$projectRoot = Split-Path -Parent $groundStationRoot
$backendPython = Join-Path $backendRoot ".venv\Scripts\python.exe"
$frontendIndex = Join-Path $frontendRoot "dist\index.html"
$localConfig = Join-Path $groundStationRoot "ground_station.local.ps1"
$artifactRoot = Join-Path $projectRoot "artifacts\ground_station"

if (Test-Path -LiteralPath $localConfig) {
    . $localConfig
    Write-Host "Loaded local configuration: $localConfig"
}
if (-not $env:HOST_ONBOARD_IP) { $env:HOST_ONBOARD_IP = "127.0.0.1" }
if (-not $env:HOST_OPERATOR_IP) { $env:HOST_OPERATOR_IP = "127.0.0.1" }
if (-not $env:ONBOARD_BRIDGE_HOST) { $env:ONBOARD_BRIDGE_HOST = "127.0.0.1" }
if (-not (Test-Path -LiteralPath $backendPython)) { throw "Backend virtual environment is missing: $backendPython" }

$frontendSource = Get-ChildItem -LiteralPath (Join-Path $frontendRoot "src") -File -Recurse |
    Sort-Object LastWriteTimeUtc -Descending |
    Select-Object -First 1
$frontendNeedsBuild = -not (Test-Path -LiteralPath $frontendIndex)
if (-not $frontendNeedsBuild -and $frontendSource) {
    $frontendNeedsBuild = $frontendSource.LastWriteTimeUtc -gt (Get-Item -LiteralPath $frontendIndex).LastWriteTimeUtc
}
if ($frontendNeedsBuild) {
    Push-Location $frontendRoot
    try {
        npm run build
        if ($LASTEXITCODE -ne 0) { throw "Frontend build failed." }
    }
    finally { Pop-Location }
}

if ($EnableLiveControl) {
    $requiredConfirmation = if ($env:LIVE_CONTROL_CONFIRMATION) { $env:LIVE_CONTROL_CONFIRMATION } else { "I_ACCEPT_REAL_FLIGHT_CONTROL" }
    if ($LiveControlConfirmation -ne $requiredConfirmation) {
        throw "Live-control confirmation does not match the configured value."
    }
    if (-not $env:OPERATOR_CONTROL_TOKEN -or $env:OPERATOR_CONTROL_TOKEN -eq "REQUIRED") {
        throw "OPERATOR_CONTROL_TOKEN must be configured before live control can be enabled."
    }
    if (-not $env:ONBOARD_BRIDGE_TOKEN -or $env:ONBOARD_BRIDGE_TOKEN -eq "REQUIRED") {
        throw "ONBOARD_BRIDGE_TOKEN must be configured before live control can be enabled."
    }
    if (-not $env:ONBOARD_OBSERVATION_TOKEN -or $env:ONBOARD_OBSERVATION_TOKEN -eq "REQUIRED") {
        throw "ONBOARD_OBSERVATION_TOKEN must be configured before live control can be enabled."
    }
    if (-not $env:EXPECTED_CALIBRATION_ID -or $env:EXPECTED_CALIBRATION_ID -eq "REQUIRED") {
        throw "EXPECTED_CALIBRATION_ID must be configured before live control can be enabled."
    }
    $env:CONTROL_OUTPUT_ENABLED = "true"
}
else {
    $env:CONTROL_OUTPUT_ENABLED = "false"
}

New-Item -ItemType Directory -Force -Path $artifactRoot | Out-Null

$existingListeners = @(Get-NetTCPConnection -State Listen -LocalPort 8080 -ErrorAction SilentlyContinue)
if ($existingListeners.Count -gt 0) {
    if (-not $ReplaceExistingBackend) {
        throw "TCP port 8080 is already in use. Use -ReplaceExistingBackend only for this project's uvicorn process."
    }
    foreach ($listener in $existingListeners) {
        $existing = Get-CimInstance Win32_Process -Filter "ProcessId=$($listener.OwningProcess)"
        if (-not $existing.CommandLine -or $existing.CommandLine -notmatch "uvicorn\s+app\.main:app") {
            throw "Port 8080 belongs to an unrelated process (PID $($listener.OwningProcess)); it was not stopped."
        }
        Stop-Process -Id $listener.OwningProcess
        Wait-Process -Id $listener.OwningProcess -ErrorAction SilentlyContinue
        Write-Host "Stopped previous ground-station backend PID $($listener.OwningProcess)."
    }
}

$backendProcess = Start-Process -FilePath $backendPython `
    -ArgumentList @("-m", "uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8080") `
    -WorkingDirectory $backendRoot `
    -WindowStyle Hidden `
    -RedirectStandardOutput (Join-Path $artifactRoot "backend_stdout.log") `
    -RedirectStandardError (Join-Path $artifactRoot "backend_stderr.log") `
    -PassThru

try {
    $ready = $false
    for ($attempt = 0; $attempt -lt 60; $attempt++) {
        if ($backendProcess.HasExited) { throw "Ground-station backend exited during startup." }
        try {
            Invoke-RestMethod -Uri "http://127.0.0.1:8080/api/missions/current" -TimeoutSec 1 | Out-Null
            $ready = $true
            break
        }
        catch { Start-Sleep -Milliseconds 500 }
    }
    if (-not $ready) { throw "Ground-station backend did not become ready." }

    Write-Host "Operator console: http://127.0.0.1:8080" -ForegroundColor Green
    Write-Host "Operator laptop URL: http://$($env:HOST_OPERATOR_IP):8080"
    Write-Host "Control output enabled: $($env:CONTROL_OUTPUT_ENABLED)"
    if (-not $NoBrowser) { Start-Process "http://127.0.0.1:8080" }
    Write-Host "Close this window or press Ctrl-C to stop the web backend."
    Wait-Process -Id $backendProcess.Id
}
finally {
    if (-not $backendProcess.HasExited) { Stop-Process -Id $backendProcess.Id }
}
