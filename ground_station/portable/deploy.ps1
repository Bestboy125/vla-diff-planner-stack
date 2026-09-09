param(
    [string]$PythonExe = "",
    [switch]$InstallOnly,
    [switch]$NoBrowser
)
$ErrorActionPreference = "Stop"
$bundleRoot = $PSScriptRoot
$venvPython = Join-Path $bundleRoot ".venv\Scripts\python.exe"

function Test-SupportedPython([string]$Candidate) {
    if (-not $Candidate -or -not (Test-Path -LiteralPath $Candidate -PathType Leaf)) { return $false }
    & $Candidate -c "import sys,struct; sys.exit(0 if (3,11)<=sys.version_info[:2]<(3,14) and struct.calcsize('P')==8 else 1)" 2>$null
    return $LASTEXITCODE -eq 0
}
function Find-Python {
    $candidates = @()
    foreach ($version in @('313', '312', '311')) {
        $candidates += Join-Path $env:LOCALAPPDATA "Programs\Python\Python$version\python.exe"
        $candidates += Join-Path $env:ProgramFiles "Python$version\python.exe"
    }
    $launcher = Get-Command py.exe -ErrorAction SilentlyContinue
    if ($launcher) {
        foreach ($version in @('3.13', '3.12', '3.11')) {
            try { $found = & $launcher.Source "-$version" -c "import sys; print(sys.executable)" 2>$null }
            catch { continue }
            if ($LASTEXITCODE -eq 0 -and $found) { $candidates += [string]$found }
        }
    }
    $command = Get-Command python.exe -ErrorAction SilentlyContinue
    if ($command -and $command.Source -notmatch '\\WindowsApps\\python.exe$') {
        $candidates += $command.Source
    }
    foreach ($candidate in $candidates) {
        if (Test-SupportedPython $candidate) { return $candidate }
    }
    return $null
}

if (-not (Test-Path -LiteralPath (Join-Path $bundleRoot 'ground_station\frontend\dist\index.html'))) {
    throw 'Incomplete bundle. Extract the entire ZIP before running deploy.cmd.'
}
if (Test-Path -LiteralPath $venvPython) {
    if (-not (Test-SupportedPython $venvPython)) {
        throw 'Existing .venv is incompatible or was moved. Extract the ZIP into a fresh directory and deploy again.'
    }
} else {
    if ($PythonExe) {
        if (-not (Test-SupportedPython $PythonExe)) { throw 'PythonExe must be Python 3.11-3.13, Windows x64.' }
    } else {
        $PythonExe = Find-Python
        if (-not $PythonExe) {
            $winget = Get-Command winget.exe -ErrorAction SilentlyContinue
            if (-not $winget) { throw 'Install Python 3.12 x64 from python.org, then rerun deploy.cmd (or pass -PythonExe).' }
            Write-Host 'Installing Python 3.12 for the current user. Internet access is required.'
            & $winget.Source install --id Python.Python.3.12 --exact --source winget --scope user --silent --accept-package-agreements --accept-source-agreements
            if ($LASTEXITCODE -ne 0) { throw 'Python installation failed. Install Python 3.12 x64 manually, then retry.' }
            $PythonExe = Find-Python
            if (-not $PythonExe) { throw 'Python installed but not found. Reopen this window or specify -PythonExe.' }
        }
    }
    & $PythonExe -m venv (Join-Path $bundleRoot '.venv')
    if ($LASTEXITCODE -ne 0) { throw 'Cannot create Python virtual environment.' }
}
& $venvPython -m pip install --disable-pip-version-check --only-binary=:all: -r (Join-Path $bundleRoot 'requirements-runtime.txt')
if ($LASTEXITCODE -ne 0) { throw 'Dependency installation failed. Check network/proxy access, then rerun deploy.cmd.' }
& $venvPython -m pip check
if ($LASTEXITCODE -ne 0) { throw 'Dependency verification failed.' }
& $venvPython (Join-Path $bundleRoot 'smoke_test.py')
if ($LASTEXITCODE -ne 0) { throw 'Local safety-locked self-test failed. No onboard command was sent.' }
$configPath = Join-Path $bundleRoot 'config.json'
if (-not (Test-Path -LiteralPath $configPath)) {
    Copy-Item -LiteralPath (Join-Path $bundleRoot 'config.example.json') -Destination $configPath
}
Write-Host 'Deployment complete. Existing config.json was preserved.' -ForegroundColor Green
if (-not $InstallOnly) { & (Join-Path $bundleRoot 'start.ps1') -NoBrowser:$NoBrowser }
