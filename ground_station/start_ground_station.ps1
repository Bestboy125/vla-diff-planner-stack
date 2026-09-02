param(
    [switch]$WithOpenVLA
)

$ErrorActionPreference = "Stop"
$groundStationRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$backendRoot = Join-Path $groundStationRoot "backend"
$frontendRoot = Join-Path $groundStationRoot "frontend"
$backendPython = Join-Path $backendRoot ".venv\Scripts\python.exe"
$frontendIndex = Join-Path $frontendRoot "dist\index.html"
$openvlaRoot = $env:OPENVLA_PROJECT_ROOT
$openvlaPython = if ($openvlaRoot) { Join-Path $openvlaRoot ".venv\Scripts\python.exe" } else { $null }
$openvlaScript = if ($openvlaRoot) { Join-Path $openvlaRoot "vla-scripts\openvla_act.py" } else { $null }
$artifactRoot = Join-Path (Split-Path -Parent $groundStationRoot) "artifacts"
$openvlaProcess = $null

if (-not (Test-Path -LiteralPath $backendPython)) {
    throw "Backend virtual environment is missing: $backendPython"
}

if (-not (Test-Path -LiteralPath $frontendIndex)) {
    Push-Location $frontendRoot
    try {
        npm run build
        if ($LASTEXITCODE -ne 0) { throw "Frontend build failed." }
    }
    finally {
        Pop-Location
    }
}

if ($WithOpenVLA) {
    if (-not $openvlaRoot -or $openvlaRoot -eq "REQUIRED") {
        throw "Set OPENVLA_PROJECT_ROOT in the private local environment before using -WithOpenVLA."
    }
    if (-not (Test-Path -LiteralPath $openvlaPython)) {
        throw "OpenVLA virtual environment is missing: $openvlaPython"
    }
    New-Item -ItemType Directory -Force -Path $artifactRoot | Out-Null
    $openvlaProcess = Start-Process `
        -FilePath $openvlaPython `
        -ArgumentList $openvlaScript `
        -WorkingDirectory $openvlaRoot `
        -WindowStyle Hidden `
        -RedirectStandardOutput (Join-Path $artifactRoot "openvla_stdout.log") `
        -RedirectStandardError (Join-Path $artifactRoot "openvla_stderr.log") `
        -PassThru
    Write-Host "OpenVLA is starting on port 5007 (PID $($openvlaProcess.Id))."
}

$env:CONTROL_OUTPUT_ENABLED = "false"
Write-Host "Ground station: http://127.0.0.1:8080"
Write-Host "Operator URL: http://$($env:HOST_OPERATOR_IP):8080"
Write-Host "Safety lock: ENABLED"

try {
    Push-Location $backendRoot
    & $backendPython -m uvicorn app.main:app --host 0.0.0.0 --port 8080
}
finally {
    Pop-Location
    if ($openvlaProcess -and -not $openvlaProcess.HasExited) {
        Stop-Process -Id $openvlaProcess.Id
        Write-Host "Stopped OpenVLA PID $($openvlaProcess.Id)."
    }
}
