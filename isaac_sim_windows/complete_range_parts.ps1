[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$Url,

    [Parameter(Mandatory = $true)]
    [string]$PartsDirectory,

    [Parameter(Mandatory = $true)]
    [long]$ExpectedLength,

    [ValidateRange(1, 128)]
    [int]$Segments = 32,

    [ValidateRange(1, 16)]
    [int]$TailSegments = 4
)

$ErrorActionPreference = "Stop"
$curl = (Get-Command curl.exe -ErrorAction Stop).Source
$partsPath = [System.IO.Path]::GetFullPath($PartsDirectory)
$segmentLength = [long][Math]::Ceiling($ExpectedLength / [double]$Segments)
$jobs = @()

for ($index = 0; $index -lt $Segments; $index++) {
    $partStart = [long]$index * $segmentLength
    $partEnd = [Math]::Min($ExpectedLength - 1, $partStart + $segmentLength - 1)
    $partPath = Join-Path $partsPath ("part-{0:D2}.bin" -f $index)
    $currentLength = if (Test-Path -LiteralPath $partPath) {
        (Get-Item -LiteralPath $partPath).Length
    }
    else {
        0L
    }
    $expectedPartLength = $partEnd - $partStart + 1

    if ($currentLength -gt $expectedPartLength) {
        throw "Part $index is too large: expected at most $expectedPartLength, got $currentLength"
    }
    if ($currentLength -eq $expectedPartLength) {
        continue
    }

    $remainingStart = $partStart + $currentLength
    $remainingLength = $partEnd - $remainingStart + 1
    $tailLength = [long][Math]::Ceiling($remainingLength / [double]$TailSegments)

    for ($tailIndex = 0; $tailIndex -lt $TailSegments; $tailIndex++) {
        $start = $remainingStart + ([long]$tailIndex * $tailLength)
        if ($start -gt $partEnd) {
            break
        }
        $end = [Math]::Min($partEnd, $start + $tailLength - 1)
        $tailPath = "$partPath.tail-$tailIndex"
        $stderrPath = "$tailPath.stderr.log"
        Remove-Item -LiteralPath $tailPath, $stderrPath -Force -ErrorAction SilentlyContinue

        $arguments = @(
            "--noproxy", "*",
            "--fail", "--location",
            "--retry", "8", "--retry-all-errors", "--retry-delay", "2",
            "--connect-timeout", "30",
            "--range", "${start}-${end}",
            "--output", $tailPath,
            $Url
        )
        $process = Start-Process -FilePath $curl -ArgumentList $arguments `
            -RedirectStandardError $stderrPath -WindowStyle Hidden -PassThru

        $jobs += [pscustomobject]@{
            PartIndex = $index
            TailIndex = $tailIndex
            Start = $start
            End = $end
            PartPath = $partPath
            TailPath = $tailPath
            StderrPath = $stderrPath
            Process = $process
        }
    }
}

foreach ($job in $jobs) {
    $job.Process.WaitForExit()
    if ($job.Process.ExitCode -ne 0) {
        $details = Get-Content -LiteralPath $job.StderrPath -Raw -ErrorAction SilentlyContinue
        throw "Part $($job.PartIndex) tail $($job.TailIndex) failed: $details"
    }
    $expectedTailLength = $job.End - $job.Start + 1
    $actualTailLength = (Get-Item -LiteralPath $job.TailPath).Length
    if ($actualTailLength -ne $expectedTailLength) {
        throw "Part $($job.PartIndex) tail $($job.TailIndex) length mismatch: expected $expectedTailLength, got $actualTailLength"
    }
}

foreach ($partGroup in ($jobs | Group-Object PartIndex)) {
    $partPath = $partGroup.Group[0].PartPath
    $destination = [System.IO.File]::Open($partPath, [System.IO.FileMode]::Append, [System.IO.FileAccess]::Write)
    try {
        foreach ($job in ($partGroup.Group | Sort-Object TailIndex)) {
            $source = [System.IO.File]::OpenRead($job.TailPath)
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
}

foreach ($job in $jobs) {
    Remove-Item -LiteralPath $job.TailPath, $job.StderrPath -Force -ErrorAction SilentlyContinue
}

for ($index = 0; $index -lt $Segments; $index++) {
    $partStart = [long]$index * $segmentLength
    $partEnd = [Math]::Min($ExpectedLength - 1, $partStart + $segmentLength - 1)
    $partPath = Join-Path $partsPath ("part-{0:D2}.bin" -f $index)
    $expectedPartLength = $partEnd - $partStart + 1
    $actualPartLength = (Get-Item -LiteralPath $partPath).Length
    if ($actualPartLength -ne $expectedPartLength) {
        throw "Completed part $index length mismatch: expected $expectedPartLength, got $actualPartLength"
    }
}

Write-Output "TAIL_COMPLETION_OK jobs=$($jobs.Count) segments=$Segments"
