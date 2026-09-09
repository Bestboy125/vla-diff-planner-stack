param(
    [switch]$NoBrowser,
    [switch]$EnableLiveControl,
    [string]$LiveControlConfirmation = ""
)
$ErrorActionPreference = "Stop"
$bundleRoot = $PSScriptRoot
$backendRoot = Join-Path $bundleRoot 'ground_station\backend'
$python = Join-Path $bundleRoot '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $python)) { throw 'Run deploy.cmd first.' }
$configPath = Join-Path $bundleRoot 'config.json'
if (-not (Test-Path -LiteralPath $configPath)) { throw 'Run deploy.cmd first to create config.json.' }
$config = Get-Content -LiteralPath $configPath -Raw | ConvertFrom-Json
$template = Get-Content -LiteralPath (Join-Path $bundleRoot 'config.example.json') -Raw | ConvertFrom-Json
$listenIp = $null
if (-not [Net.IPAddress]::TryParse([string]$config.ListenHost, [ref]$listenIp) -or
    $listenIp.AddressFamily -ne [Net.Sockets.AddressFamily]::InterNetwork) {
    throw 'ListenHost must be an IPv4 address (127.0.0.1 for local preview; 0.0.0.0 for LAN).'
}
$port = 0
if (-not [int]::TryParse([string]$config.Port, [ref]$port) -or $port -lt 1024 -or $port -gt 65535) {
    throw 'Port must be an integer between 1024 and 65535.'
}
# Apply only known data fields, never execute a transferred PowerShell config.
# Clear inherited values for these keys by applying the template defaults first.
foreach ($property in $template.Environment.PSObject.Properties) {
    [Environment]::SetEnvironmentVariable($property.Name, [string]$property.Value, 'Process')
}
foreach ($property in $config.Environment.PSObject.Properties) {
    if ($property.Name -notin @($template.Environment.PSObject.Properties.Name)) {
        throw "Unknown environment key in config.json: $($property.Name)"
    }
    [Environment]::SetEnvironmentVariable($property.Name, [string]$property.Value, 'Process')
}
$env:CONTROL_OUTPUT_ENABLED = 'false'
$env:LIVE_CONTROL_CONFIRMATION = 'I_ACCEPT_REAL_FLIGHT_CONTROL'
$env:GROUND_STATION_HOST = [string]$config.ListenHost
$env:GROUND_STATION_PORT = [string]$port
function Require-Configured([string]$Name) {
    $value = [Environment]::GetEnvironmentVariable($Name, 'Process')
    if (-not $value -or $value -eq 'REQUIRED' -or $value.StartsWith('REPLACE_WITH_')) {
        throw "Configure $Name in config.json first. Its value was not printed."
    }
}
if (-not [Net.IPAddress]::IsLoopback($listenIp)) {
    Require-Configured 'ONBOARD_OBSERVATION_TOKEN'
    Require-Configured 'EXPECTED_CALIBRATION_ID'
}
if ($EnableLiveControl) {
    if ($LiveControlConfirmation -cne 'I_ACCEPT_REAL_FLIGHT_CONTROL') {
        throw 'Live control requires -LiveControlConfirmation I_ACCEPT_REAL_FLIGHT_CONTROL.'
    }
    foreach ($name in @('ONBOARD_BRIDGE_TOKEN','ONBOARD_OBSERVATION_TOKEN','OPERATOR_CONTROL_TOKEN','EXPECTED_CALIBRATION_ID')) {
        Require-Configured $name
    }
    & $python (Join-Path $backendRoot 'tools\check_onboard_clock.py') --host $env:ONBOARD_BRIDGE_HOST --max-offset-ms $env:ONBOARD_MAX_CLOCK_OFFSET_MS
    if ($LASTEXITCODE -ne 0) { throw 'Clock preflight failed. Live backend was not started.' }
    $env:CONTROL_OUTPUT_ENABLED = 'true'
}
$listeners = @(Get-NetTCPConnection -State Listen -LocalPort $port -ErrorAction SilentlyContinue)
if ($listeners.Count -gt 0) { throw "Port $port is in use. No process was stopped. Close the other console or change Port." }
$logRoot = Join-Path $bundleRoot 'logs'
New-Item -ItemType Directory -Force -Path $logRoot | Out-Null
$stamp = Get-Date -Format 'yyyyMMdd-HHmmss-fff'
$stdout = Join-Path $logRoot "backend-$stamp.out.log"
$stderr = Join-Path $logRoot "backend-$stamp.err.log"
$process = Start-Process -FilePath $python -ArgumentList @('-m','uvicorn','app.main:app','--host',[string]$config.ListenHost,'--port',[string]$port) -WorkingDirectory $backendRoot -WindowStyle Hidden -RedirectStandardOutput $stdout -RedirectStandardError $stderr -PassThru
$probeHost = if ($config.ListenHost -eq '0.0.0.0') { '127.0.0.1' } else { [string]$config.ListenHost }
$url = "http://${probeHost}:$port"
try {
    $ready = $false
    for ($attempt = 0; $attempt -lt 40; $attempt++) {
        $process.Refresh()
        if ($process.HasExited) { throw "Backend exited. See $stderr" }
        try {
            Invoke-RestMethod "$url/api/missions/current" -TimeoutSec 1 | Out-Null
            $ready = $true
            break
        } catch { Start-Sleep -Milliseconds 500 }
    }
    if (-not $ready) { throw "Backend startup timed out. See $stderr" }
    Write-Host "Console: $url" -ForegroundColor Green
    Write-Host "Control output enabled: $($env:CONTROL_OUTPUT_ENABLED)"
    Write-Host "Logs: $logRoot"
    Write-Host 'Keep this window open. Press Ctrl+C to stop this backend. Startup does not send flight commands.'
    if (-not $NoBrowser) { Start-Process $url }
    Wait-Process -Id $process.Id
} finally {
    $process.Refresh()
    if (-not $process.HasExited) { Stop-Process -Id $process.Id }
}
