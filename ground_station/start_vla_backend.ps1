param(
    [ValidateSet("OpenVLA", "Pi05", "Both")]
    [string]$Policy = "OpenVLA",
    [string]$OpenVlaRoot = $env:OPENVLA_PROJECT_ROOT,
    [string]$OpenVlaModel = $env:OPENVLA_MODEL_PATH,
    [int]$ReadyTimeoutSec = 1200
)

$ErrorActionPreference = "Stop"
$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$projectRoot = Split-Path -Parent $scriptRoot
$artifactRoot = Join-Path $projectRoot "artifacts\vla_backend"
$runtimeFile = Join-Path $artifactRoot "runtime.json"
$openVlaPython = if ($OpenVlaRoot) { Join-Path $OpenVlaRoot ".venv\Scripts\python.exe" } else { $null }
$openVlaScript = if ($OpenVlaRoot) { Join-Path $OpenVlaRoot "vla-scripts\openvla_act.py" } else { $null }
$pi05Script = Join-Path $projectRoot "pi05\start_pi05_server.ps1"

New-Item -ItemType Directory -Force -Path $artifactRoot | Out-Null

function Test-HttpReady {
    param([string]$Uri)
    try {
        Invoke-RestMethod -Uri $Uri -TimeoutSec 2 | Out-Null
        return $true
    }
    catch {
        return $false
    }
}

function Wait-HttpReady {
    param([string]$Name, [string]$Uri, [System.Diagnostics.Process]$Process)
    $deadline = [DateTime]::UtcNow.AddSeconds($ReadyTimeoutSec)
    while ([DateTime]::UtcNow -lt $deadline) {
        if (Test-HttpReady -Uri $Uri) {
            Write-Host "$Name is ready: $Uri" -ForegroundColor Green
            return
        }
        if ($Process -and $Process.HasExited) {
            throw "$Name exited before becoming ready. Exit code: $($Process.ExitCode)"
        }
        Start-Sleep -Seconds 2
    }
    throw "$Name did not become ready within $ReadyTimeoutSec seconds."
}

$started = @()
if (Test-Path -LiteralPath $runtimeFile) {
    try {
        $previous = Get-Content -LiteralPath $runtimeFile -Raw | ConvertFrom-Json
        foreach ($entry in @($previous.processes)) {
            if ($entry.started_by_script -and (Get-Process -Id ([int]$entry.pid) -ErrorAction SilentlyContinue)) {
                $started += $entry
            }
        }
    }
    catch {
        Write-Warning "Ignoring an unreadable previous runtime file: $runtimeFile"
    }
}

if ($Policy -in @("OpenVLA", "Both")) {
    if (-not $OpenVlaRoot -or $OpenVlaRoot -eq "REQUIRED") {
        throw "Set OPENVLA_PROJECT_ROOT in ground_station.local.ps1."
    }
    if (-not $OpenVlaModel -or $OpenVlaModel -eq "REQUIRED") {
        throw "Set OPENVLA_MODEL_PATH in ground_station.local.ps1."
    }
    if (-not (Test-Path -LiteralPath $openVlaPython)) { throw "OpenVLA Python is missing: $openVlaPython" }
    if (-not (Test-Path -LiteralPath $openVlaScript)) { throw "OpenVLA server is missing: $openVlaScript" }
    if (-not (Test-Path -LiteralPath $OpenVlaModel)) { throw "OpenVLA checkpoint is missing: $OpenVlaModel" }

    if (Test-HttpReady -Uri "http://127.0.0.1:5007/") {
        Write-Host "OpenVLA is already ready on port 5007."
    }
    else {
        $env:OPENVLA_MODEL_PATH = $OpenVlaModel
        $env:OPENVLA_HTTP_PORT = "5007"
        $env:OPENVLA_UNNORM_KEY = "real"
        $process = Start-Process -FilePath $openVlaPython `
            -ArgumentList $openVlaScript `
            -WorkingDirectory $OpenVlaRoot `
            -WindowStyle Hidden `
            -RedirectStandardOutput (Join-Path $artifactRoot "openvla_stdout.log") `
            -RedirectStandardError (Join-Path $artifactRoot "openvla_stderr.log") `
            -PassThru
        $started = @($started | Where-Object name -ne "openvla")
        $started += [pscustomobject]@{ name = "openvla"; pid = $process.Id; started_by_script = $true }
        Write-Host "Loading OpenVLA checkpoint (PID $($process.Id)); this can take several minutes..."
        Wait-HttpReady -Name "OpenVLA" -Uri "http://127.0.0.1:5007/" -Process $process
    }
}

if ($Policy -in @("Pi05", "Both")) {
    if (-not (Test-Path -LiteralPath $pi05Script)) { throw "Pi05 launcher is missing: $pi05Script" }
    if (Test-HttpReady -Uri "http://127.0.0.1:8000/healthz") {
        Write-Host "Pi05 is already ready on port 8000."
    }
    else {
        $process = Start-Process -FilePath "powershell.exe" `
            -ArgumentList @("-NoProfile", "-ExecutionPolicy", "Bypass", "-File", $pi05Script) `
            -WorkingDirectory $projectRoot `
            -WindowStyle Hidden `
            -RedirectStandardOutput (Join-Path $artifactRoot "pi05_stdout.log") `
            -RedirectStandardError (Join-Path $artifactRoot "pi05_stderr.log") `
            -PassThru
        $started = @($started | Where-Object name -ne "pi05")
        $started += [pscustomobject]@{ name = "pi05"; pid = $process.Id; started_by_script = $true }
        Write-Host "Loading Pi05 checkpoint in WSL (PID $($process.Id))..."
        Wait-HttpReady -Name "Pi05" -Uri "http://127.0.0.1:8000/healthz" -Process $process
    }
}

[pscustomobject]@{
    policy = $Policy
    started_at = (Get-Date).ToString("o")
    processes = $started
} | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath $runtimeFile -Encoding UTF8

Write-Host "VLA backend startup completed. Runtime state: $runtimeFile" -ForegroundColor Cyan
Write-Host "Use stop_vla_backend.ps1 to stop only processes recorded by this launcher."
