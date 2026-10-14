param(
    [Parameter(Mandatory=$true)][ValidateSet('dataset','camera','dataset-view','camera-auto')][string]$Mode,
    [string]$Calibration,
    [Parameter(Mandatory=$true)][string]$InputSource,
    [string]$OutputDirectory = 'target/slam-output',
    [string]$LiveConfig,
    [int]$EyeWidth = 640,
    [int]$EyeHeight = 480
)
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '../..')).Path
# Resolve user paths before changing the working directory.
if ($Mode -ne 'camera-auto') {
    if (-not $Calibration) { throw '-Calibration is required for metric SLAM; use camera-auto without a calibration file' }
    $calibrationFile = (Resolve-Path -LiteralPath $Calibration).Path
}
if ($Mode -eq 'dataset' -or $Mode -eq 'dataset-view') { $InputSource = (Resolve-Path -LiteralPath $InputSource).Path }
if ($LiveConfig) { $LiveConfig = (Resolve-Path -LiteralPath $LiveConfig).Path }
$outputPath = [System.IO.Path]::GetFullPath($OutputDirectory)
Push-Location -LiteralPath $projectRoot
try {
    & mvn -q compile dependency:build-classpath '-Dmdep.outputFile=target/slam-classpath.txt'
    if ($LASTEXITCODE -ne 0) { throw 'Maven build failed' }
    $dependencyClasspath = (Get-Content -LiteralPath 'target/slam-classpath.txt' -Raw).Trim()
    $runtimeClasspath = (Join-Path $projectRoot 'target/classes') + [System.IO.Path]::PathSeparator + $dependencyClasspath
    $launchArguments = @('-cp', $runtimeClasspath, 'winw.ai.slam.StereoSlamDemo', $Mode, $calibrationFile, $InputSource, $outputPath)
    if ($Mode -eq 'camera-auto') {
        $launchArguments = @('-cp', $runtimeClasspath, 'winw.ai.slam.StereoSlamDemo', $Mode, $EyeWidth, $EyeHeight, $InputSource, $outputPath)
    }
    if ($LiveConfig) { $launchArguments += $LiveConfig }
    & java @launchArguments
    if ($LASTEXITCODE -ne 0) { throw 'SLAM execution failed' }
} finally { Pop-Location }
