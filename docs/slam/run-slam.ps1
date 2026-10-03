param(
    [Parameter(Mandatory=$true)][ValidateSet('dataset','camera','dataset-view')][string]$Mode,
    [Parameter(Mandatory=$true)][string]$Calibration,
    [Parameter(Mandatory=$true)][string]$InputSource,
    [string]$OutputDirectory = 'target/slam-output',
    [string]$LiveConfig
)
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '../..')).Path
# Resolve user paths before changing the working directory.
$calibrationFile = (Resolve-Path -LiteralPath $Calibration).Path
if ($Mode -ne 'camera') { $InputSource = (Resolve-Path -LiteralPath $InputSource).Path }
if ($LiveConfig) { $LiveConfig = (Resolve-Path -LiteralPath $LiveConfig).Path }
$outputPath = [System.IO.Path]::GetFullPath($OutputDirectory)
Push-Location -LiteralPath $projectRoot
try {
    & mvn -q compile dependency:build-classpath '-Dmdep.outputFile=target/slam-classpath.txt'
    if ($LASTEXITCODE -ne 0) { throw 'Maven build failed' }
    $dependencyClasspath = (Get-Content -LiteralPath 'target/slam-classpath.txt' -Raw).Trim()
    $runtimeClasspath = (Join-Path $projectRoot 'target/classes') + [System.IO.Path]::PathSeparator + $dependencyClasspath
    $launchArguments = @('-cp', $runtimeClasspath, 'winw.ai.slam.StereoSlamDemo', $Mode, $calibrationFile, $InputSource, $outputPath)
    if ($LiveConfig) { $launchArguments += $LiveConfig }
    & java @launchArguments
    if ($LASTEXITCODE -ne 0) { throw 'SLAM execution failed' }
} finally { Pop-Location }
