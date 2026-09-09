param([string]$OutputDirectory = "")
$ErrorActionPreference = 'Stop'
$stationRoot = $PSScriptRoot
$projectRoot = Split-Path -Parent $stationRoot
if (-not $OutputDirectory) { $OutputDirectory = Join-Path $projectRoot 'artifacts\distributions' }
New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
$outputRoot = (Resolve-Path -LiteralPath $OutputDirectory).Path
$bundleName = 'ground-station-windows-' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff')
$bundleRoot = Join-Path $outputRoot $bundleName
if (Test-Path -LiteralPath $bundleRoot) { throw 'Output already exists; nothing was overwritten.' }
$npm = Get-Command npm.cmd -ErrorAction Stop
Push-Location (Join-Path $stationRoot 'frontend')
try {
    & $npm.Source ci
    if ($LASTEXITCODE -ne 0) { throw 'npm ci failed.' }
    & $npm.Source test
    if ($LASTEXITCODE -ne 0) { throw 'Frontend tests failed.' }
    & $npm.Source run build
    if ($LASTEXITCODE -ne 0) { throw 'Frontend build failed.' }
} finally { Pop-Location }
New-Item -ItemType Directory -Path $bundleRoot | Out-Null
# Deliberately allowlist files. Never recurse through the repository, local
# configuration, .venv, node_modules, training outputs, weights or credentials.
$backend = Join-Path $bundleRoot 'ground_station\backend'
$appDir = Join-Path $backend 'app'
$toolDir = Join-Path $backend 'tools'
$frontend = Join-Path $bundleRoot 'ground_station\frontend'
New-Item -ItemType Directory -Force -Path $appDir, $toolDir, $frontend | Out-Null
Get-ChildItem -LiteralPath (Join-Path $stationRoot 'backend\app') -File -Filter '*.py' |
    ForEach-Object { Copy-Item -LiteralPath $_.FullName -Destination $appDir }
Copy-Item -LiteralPath (Join-Path $stationRoot 'backend\tools\check_onboard_clock.py') -Destination $toolDir
Copy-Item -LiteralPath (Join-Path $stationRoot 'frontend\dist') -Destination $frontend -Recurse
$licenseRoot = Join-Path $bundleRoot 'licenses'
New-Item -ItemType Directory -Path $licenseRoot | Out-Null
foreach ($dependency in @('react','react-dom','scheduler')) {
    Copy-Item -LiteralPath (Join-Path $stationRoot "frontend\node_modules\$dependency\LICENSE") -Destination (Join-Path $licenseRoot "$dependency-LICENSE.txt")
}
$portable = Join-Path $stationRoot 'portable'
foreach ($name in @('deploy.cmd','deploy.ps1','start.cmd','start.ps1','config.example.json',
                    'requirements-runtime.txt','smoke_test.py','README.md','enable_onboard_firewall.ps1')) {
    Copy-Item -LiteralPath (Join-Path $portable $name) -Destination $bundleRoot
}
$manifest = @(Get-ChildItem -LiteralPath $bundleRoot -File -Recurse | Sort-Object FullName | ForEach-Object {
    [ordered]@{path=$_.FullName.Substring($bundleRoot.Length+1).Replace('\','/');
               bytes=$_.Length; sha256=(Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash}
})
$manifest | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $bundleRoot 'manifest.json') -Encoding UTF8
$zip = Join-Path $outputRoot ($bundleName + '.zip')
Compress-Archive -LiteralPath $bundleRoot -DestinationPath $zip
$hash = (Get-FileHash -LiteralPath $zip -Algorithm SHA256).Hash
"$hash  $bundleName.zip" | Set-Content -LiteralPath ($zip+'.sha256') -Encoding ASCII
Write-Host "Bundle: $bundleRoot" -ForegroundColor Green
Write-Host "ZIP: $zip"
Write-Host "SHA256: $hash"
