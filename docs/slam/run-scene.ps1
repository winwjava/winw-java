param(
    [ValidateSet('camera','replay')][string]$Mode = 'camera',
    [string]$InputSource = '0',
    [double]$BaselineMm = 65,
    [int]$EyeWidth = 640,
    [int]$EyeHeight = 480,
    [string]$OutputDirectory = 'target/live-scene',
    [switch]$Headless
)
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '../..')).Path
if ($Mode -eq 'replay') { $InputSource = (Resolve-Path -LiteralPath $InputSource).Path }
$outputPath = [System.IO.Path]::GetFullPath($OutputDirectory)
if ([double]::IsNaN($BaselineMm) -or [double]::IsInfinity($BaselineMm) -or $BaselineMm -le 0) { throw 'BaselineMm must be positive' }
Push-Location -LiteralPath $projectRoot
try {
    & mvn -q compile dependency:build-classpath '-Dmdep.outputFile=target/slam-classpath.txt'
    if ($LASTEXITCODE -ne 0) { throw 'Build failed' }
    $runtimeClasspath = (Join-Path $projectRoot 'target/classes') + [System.IO.Path]::PathSeparator + (Get-Content target/slam-classpath.txt -Raw).Trim()
    if ($Mode -eq 'camera') {
        $baselineText = $BaselineMm.ToString('R', [System.Globalization.CultureInfo]::InvariantCulture)
        & java "-Dslam.baselineMm=$baselineText" -cp $runtimeClasspath winw.ai.slam.StereoSlamDemo camera-auto $EyeWidth $EyeHeight $InputSource $outputPath
    } else {
        $launchArguments = @('-cp',$runtimeClasspath,'winw.ai.slam.SceneReplayDemo',$InputSource)
        if ($Headless) { $launchArguments += '--headless' }
        & java @launchArguments
    }
    if ($LASTEXITCODE -ne 0) { throw 'Scene execution failed' }
} finally { Pop-Location }
