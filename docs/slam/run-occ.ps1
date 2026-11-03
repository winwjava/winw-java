param(
    [Parameter(Mandatory=$true)][string]$Sequence,
    [double]$BaselineMm = 65,
    [string]$OutputDirectory = 'target/occ-3d'
)
$ErrorActionPreference = 'Stop'
$sequencePath = (Resolve-Path -LiteralPath $Sequence).Path
$outputPath = [System.IO.Path]::GetFullPath($OutputDirectory)
$replayPath = Join-Path $outputPath 'tracking'
& (Join-Path $PSScriptRoot 'run-recording.ps1') -Sequence $sequencePath -BaselineMm $BaselineMm -OutputDirectory $replayPath
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '../..')).Path
Push-Location -LiteralPath $projectRoot
try {
    $runtimeClasspath = (Join-Path $projectRoot 'target/classes') + [System.IO.Path]::PathSeparator + (Get-Content target/slam-classpath.txt -Raw).Trim()
    & java -cp $runtimeClasspath winw.ai.slam.OccupancyReplay (Join-Path $sequencePath 'manifest.txt') $replayPath $outputPath
    if ($LASTEXITCODE -ne 0) { throw 'OCC replay failed' }
    & python (Join-Path $PSScriptRoot 'build-occ-viewer.py') $outputPath
    if ($LASTEXITCODE -ne 0) { throw 'OCC viewer build failed' }
} finally { Pop-Location }
