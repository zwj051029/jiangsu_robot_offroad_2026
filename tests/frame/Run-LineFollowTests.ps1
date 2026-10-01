param([string]$Compiler = 'g++')
$ErrorActionPreference = 'Stop'
$taskRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$taskOutput = Join-Path $PSScriptRoot 'out'
New-Item -ItemType Directory -Path $taskOutput -Force | Out-Null
$taskArgs = @('-std=c++11', '-Wall', '-Wextra', '-Werror',
    '-I', "$PSScriptRoot\mocks",
    '-I', "$taskRoot\FrameComponets\Bsps\Inc",
    '-I', "$taskRoot\FrameComponets\Mods\Inc",
    '-I', "$taskRoot\FrameComponets\Apps\Inc",
    '-I', "$taskRoot\FrameComponets\Sys\Inc",
    '-I', "$taskRoot\FrameComponets\Algorithm\Inc",
    "$PSScriptRoot\line_follow_tests.cpp")
foreach ($taskSource in @('Bsps/Src/bsp_dwt.c', 'Algorithm/Src/pid.cpp',
    'Algorithm/Src/std_math.cpp', 'Mods/Src/dc_motor.cpp', 'Mods/Src/gray_sensor.cpp',
    'Mods/Src/oled_factory.cpp', 'Sys/Src/Application.cpp', 'Sys/Src/System.cpp',
    'Sys/Src/StateCore.cpp', 'Sys/Src/Action.cpp', 'Sys/Src/Monitor.cpp',
    'Apps/Src/Track.cpp', 'Apps/Src/SpeedMixer.cpp', 'Apps/Src/MotorSpeedDisplay.cpp',
    'Apps/Src/LineFollowApp.cpp')) {
    $taskArgs += Join-Path "$taskRoot\FrameComponets" $taskSource
}
$taskExe = Join-Path $taskOutput 'line_follow_tests.exe'
& $Compiler @taskArgs '-o' $taskExe
if ($LASTEXITCODE -ne 0) { throw "Line follow test build failed: $LASTEXITCODE" }
& $taskExe
if ($LASTEXITCODE -ne 0) { throw "Line follow tests failed: $LASTEXITCODE" }
