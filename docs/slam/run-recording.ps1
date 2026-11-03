param(
    [Parameter(Mandatory=$true)][string]$Sequence,
    [double]$BaselineMm = 65,
    [string]$OutputDirectory = 'target/cpu-auto-slam'
)
$ErrorActionPreference = 'Stop'
if ([double]::IsNaN($BaselineMm) -or [double]::IsInfinity($BaselineMm) -or $BaselineMm -le 0) { throw 'BaselineMm must be positive' }
$sequencePath = (Resolve-Path -LiteralPath $Sequence).Path
$manifest = Join-Path $sequencePath 'manifest.txt'
if (-not (Test-Path -LiteralPath $manifest -PathType Leaf)) { throw 'Sequence must contain manifest.txt' }
$outputPath = [System.IO.Path]::GetFullPath($OutputDirectory)
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '../..')).Path
Push-Location -LiteralPath $projectRoot
try {
    & mvn -q compile dependency:build-classpath '-Dmdep.outputFile=target/slam-classpath.txt'
    if ($LASTEXITCODE -ne 0) { throw 'Build failed' }
    $runtimeClasspath = (Join-Path $projectRoot 'target/classes') + [System.IO.Path]::PathSeparator + (Get-Content target/slam-classpath.txt -Raw).Trim()
    $baselineMetres = ($BaselineMm / 1000).ToString('R', [System.Globalization.CultureInfo]::InvariantCulture)
    & java -cp $runtimeClasspath winw.ai.slam.StereoSlamDemo evaluate $manifest "auto:$baselineMetres" $outputPath orb
    if ($LASTEXITCODE -ne 0) { throw 'Replay failed' }
} finally { Pop-Location }
