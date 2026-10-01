param([string]$Compiler = 'g++')
$ErrorActionPreference = 'Stop'
$taskRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$taskOutput = Join-Path $taskRoot 'tests\frame\out'
New-Item -ItemType Directory -Path $taskOutput -Force | Out-Null
$taskArgs = @('-std=c++11','-Wall','-Wextra','-Werror','-DARM_MATH_MATRIX_CHECK',
    '-I',"$PSScriptRoot\mocks",'-I',"$PSScriptRoot\fixtures",'-I',"$taskRoot\FrameComponets\Bsps\Inc",
    '-I',"$taskRoot\FrameComponets\Mods\Inc",'-I',"$taskRoot\FrameComponets\Sys\Inc",
    '-I',"$taskRoot\FrameComponets\Apps\Inc",'-I',"$taskRoot\FrameComponets\Algorithm\Inc", "$PSScriptRoot\frame_tests.cpp", "$PSScriptRoot\motor_tests.cpp")
foreach ($taskSource in @('Bsps/Src/bsp_dwt.c','Bsps/Src/bsp_tim_pwm.c','Bsps/Src/bsp_encoder.c','Algorithm/Src/pid.cpp','Algorithm/Src/std_math.cpp',
    'Algorithm/Src/signator.cpp','Algorithm/Src/adrc.cpp','Algorithm/Src/hyperPID.cpp',
    'Sys/Src/StateCore.cpp','Sys/Src/Action.cpp','Sys/Src/Application.cpp','Sys/Src/System.cpp','Sys/Src/Monitor.cpp',
    'Mods/Src/dc_motor.cpp','Mods/Src/motor_pwm_driver.cpp','Mods/Src/gray_sensor.cpp','Mods/Src/ultrasonic.cpp','Apps/Src/OffroadApp.cpp')) {
    $taskArgs += "$taskRoot\FrameComponets\$taskSource"
}
foreach ($taskDspSource in @('arm_mat_init_f32','arm_mat_add_f32','arm_mat_sub_f32',
    'arm_mat_mult_f32','arm_mat_scale_f32','arm_mat_trans_f32','arm_mat_inverse_f32')) {
    $taskArgs += "$taskRoot\Drivers\CMSIS\DSP\Source\MatrixFunctions\$taskDspSource.c"
}
$taskExe = Join-Path $taskOutput 'frame_tests.exe'
& $Compiler @taskArgs '-o' $taskExe
if ($LASTEXITCODE -ne 0) { throw "Host test build failed: $LASTEXITCODE" }
& $taskExe
if ($LASTEXITCODE -ne 0) { throw "Host tests failed: $LASTEXITCODE" }
& (Join-Path $PSScriptRoot 'Run-GrayOledTests.ps1') -Compiler $Compiler `
    -CCompiler ($Compiler -replace 'g\+\+(\.exe)?$', 'gcc$1')
& (Join-Path $PSScriptRoot 'Run-LineFollowTests.ps1') -Compiler $Compiler
