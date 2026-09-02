$ErrorActionPreference = "Stop"
$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$projectRoot = Split-Path -Parent $scriptRoot
$runtimeFile = Join-Path $projectRoot "artifacts\vla_backend\runtime.json"

if (-not (Test-Path -LiteralPath $runtimeFile)) {
    Write-Host "No VLA runtime file exists; nothing was stopped."
    exit 0
}

$runtime = Get-Content -LiteralPath $runtimeFile -Raw | ConvertFrom-Json
foreach ($entry in @($runtime.processes)) {
    if (-not $entry.started_by_script) { continue }
    $process = Get-Process -Id ([int]$entry.pid) -ErrorAction SilentlyContinue
    if ($process) {
        Stop-Process -Id $process.Id
        Write-Host "Stopped $($entry.name) PID $($process.Id)."
    }
}
Remove-Item -LiteralPath $runtimeFile
