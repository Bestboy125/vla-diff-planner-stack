[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$Url,

    [Parameter(Mandatory = $true)]
    [string]$OutFile,

    [Parameter(Mandatory = $true)]
    [long]$ExpectedLength,

    [ValidateRange(1, 32)]
    [int]$Segments = 8
)

$ErrorActionPreference = "Stop"
$curl = (Get-Command curl.exe -ErrorAction Stop).Source
$outputPath = [System.IO.Path]::GetFullPath($OutFile)
$workPath = "${outputPath}.parts"

New-Item -ItemType Directory -Path ([System.IO.Path]::GetDirectoryName($outputPath)) -Force | Out-Null
New-Item -ItemType Directory -Path $workPath -Force | Out-Null

$segmentLength = [long][Math]::Ceiling($ExpectedLength / [double]$Segments)
$jobs = @()

for ($index = 0; $index -lt $Segments; $index++) {
    $start = [long]$index * $segmentLength
    $end = [Math]::Min($ExpectedLength - 1, $start + $segmentLength - 1)
    $partPath = Join-Path $workPath ("part-{0:D2}.bin" -f $index)
    $stdoutPath = Join-Path $workPath ("part-{0:D2}.stdout.log" -f $index)
    $stderrPath = Join-Path $workPath ("part-{0:D2}.stderr.log" -f $index)
    $expectedPartLength = $end - $start + 1

    if ((Test-Path -LiteralPath $partPath) -and
        ((Get-Item -LiteralPath $partPath).Length -eq $expectedPartLength)) {
        $jobs += [pscustomobject]@{
            Index = $index
            Start = $start
            End = $end
            PartPath = $partPath
            Process = $null
            StderrPath = $stderrPath
        }
        continue
    }

    Remove-Item -LiteralPath $partPath, $stdoutPath, $stderrPath -Force -ErrorAction SilentlyContinue

    $arguments = @(
        "--noproxy", "*",
        "--fail", "--location",
        "--retry", "8", "--retry-all-errors", "--retry-delay", "2",
        "--connect-timeout", "30",
        "--range", "${start}-${end}",
        "--output", $partPath,
        $Url
    )

    $process = Start-Process -FilePath $curl -ArgumentList $arguments `
        -RedirectStandardOutput $stdoutPath -RedirectStandardError $stderrPath `
        -WindowStyle Hidden -PassThru

    $jobs += [pscustomobject]@{
        Index = $index
        Start = $start
        End = $end
        PartPath = $partPath
        Process = $process
        StderrPath = $stderrPath
    }
}

foreach ($job in $jobs) {
    if ($null -ne $job.Process) {
        $job.Process.WaitForExit()
        if ($job.Process.ExitCode -ne 0) {
            $details = Get-Content -LiteralPath $job.StderrPath -Raw -ErrorAction SilentlyContinue
            throw "Range $($job.Index) failed with curl exit $($job.Process.ExitCode): $details"
        }
    }

    $expectedPartLength = $job.End - $job.Start + 1
    $actualPartLength = (Get-Item -LiteralPath $job.PartPath).Length
    if ($actualPartLength -ne $expectedPartLength) {
        throw "Range $($job.Index) length mismatch: expected $expectedPartLength, got $actualPartLength"
    }
}

$destination = [System.IO.File]::Open($outputPath, [System.IO.FileMode]::Create, [System.IO.FileAccess]::Write)
try {
    foreach ($job in ($jobs | Sort-Object Index)) {
        $source = [System.IO.File]::OpenRead($job.PartPath)
        try {
            $source.CopyTo($destination)
        }
        finally {
            $source.Dispose()
        }
    }
}
finally {
    $destination.Dispose()
}

$actualLength = (Get-Item -LiteralPath $outputPath).Length
if ($actualLength -ne $ExpectedLength) {
    throw "Combined file length mismatch: expected $ExpectedLength, got $actualLength"
}

foreach ($job in $jobs) {
    Remove-Item -LiteralPath $job.PartPath, $job.StderrPath -Force -ErrorAction SilentlyContinue
    $stdoutPath = Join-Path $workPath ("part-{0:D2}.stdout.log" -f $job.Index)
    Remove-Item -LiteralPath $stdoutPath -Force -ErrorAction SilentlyContinue
}
Remove-Item -LiteralPath $workPath -Force -ErrorAction SilentlyContinue

Write-Output "DOWNLOAD_OK path=$outputPath bytes=$actualLength segments=$Segments"
