param(
    [Parameter(Mandatory=$true)][ValidateRange(0.001, 1.0)][double]$BaselineMetres,
    [int]$Device = 0,
    [int]$EyeWidth = 640,
    [int]$EyeHeight = 480,
    [string]$LiveConfig,
    [string]$SequenceDirectory,
    [switch]$CalibrationOnly
)
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '../..')).Path
if ($LiveConfig) { $LiveConfig = (Resolve-Path -LiteralPath $LiveConfig).Path }
if ($SequenceDirectory) { $SequenceDirectory = (Resolve-Path -LiteralPath $SequenceDirectory).Path }
if (-not $SequenceDirectory) {
    $captureRoot = Join-Path $projectRoot ('target/scene-session-' + [guid]::NewGuid().ToString('N'))
    Write-Host 'Capture: slowly translate sideways and up/down, with gentle rotations. Keep the scene static. Record at least 25 pairs, preferably 80-120. Stop, then close the preview to begin calibration.'
    $captureArgs = @{Mode='camera-auto'; InputSource="$Device"; EyeWidth=$EyeWidth; EyeHeight=$EyeHeight; OutputDirectory=$captureRoot}
    if ($LiveConfig) { $captureArgs.LiveConfig = $LiveConfig }
    & (Join-Path $PSScriptRoot 'run-slam.ps1') @captureArgs
    $recordings = @(Get-ChildItem -LiteralPath $captureRoot -Directory -Filter 'auto-*')
    if ($recordings.Count -ne 1) { throw 'Expected one captured sequence' }
    $SequenceDirectory = $recordings[0].FullName
}
$calibrationPath = Join-Path $SequenceDirectory ('scene-calibration-' + [guid]::NewGuid().ToString('N') + '.properties')
# Pass numbers using invariant culture, independent of Windows regional settings.
$baselineText = $BaselineMetres.ToString('R', [System.Globalization.CultureInfo]::InvariantCulture)
Push-Location -LiteralPath $projectRoot
try {
    & mvn -q compile dependency:build-classpath '-Dmdep.outputFile=target/slam-classpath.txt'
    if ($LASTEXITCODE -ne 0) { throw 'Maven build failed' }
    $runtimeClasspath = (Join-Path $projectRoot 'target/classes') + [IO.Path]::PathSeparator + (Get-Content -Raw 'target/slam-classpath.txt').Trim()
    & java -cp $runtimeClasspath winw.ai.slam.NaturalSceneCalibration $SequenceDirectory $baselineText $calibrationPath $EyeWidth $EyeHeight
    if ($LASTEXITCODE -ne 0) { throw 'Java/OpenCV natural-scene calibration failed; see self-calibration.log in sequence directory' }
} finally { Pop-Location }
if ($LASTEXITCODE -ne 0) { throw 'Natural-scene calibration failed quality checks; metric SLAM was not started. Original sequence is preserved.' }
Write-Host "Natural-scene calibration: $calibrationPath"
if (-not $CalibrationOnly) {
    $liveArgs = @{Mode='camera'; Calibration=$calibrationPath; InputSource="$Device"; OutputDirectory=(Join-Path $SequenceDirectory 'metric-slam')}
    if ($LiveConfig) { $liveArgs.LiveConfig = $LiveConfig }
    & (Join-Path $PSScriptRoot 'run-slam.ps1') @liveArgs
}
