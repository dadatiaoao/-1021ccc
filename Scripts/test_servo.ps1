# 舵机主机测试：夹爪与升降分别编译成独立可执行文件，逐个跑。
# 只测 User/servo/servo_math.c 的纯逻辑，不需要 HAL、不需要接舵机。
# 用法：powershell -ExecutionPolicy Bypass -File Scripts\test_servo.ps1
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
$vswhere = 'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe'
$vcvars = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -find 'VC\Auxiliary\Build\vcvars64.bat' | Select-Object -First 1
if (-not $vcvars) { throw 'MSVC C build tools not found' }

$testOutput = Join-Path $projectRoot 'Output\host-tests'
New-Item -ItemType Directory -Path $testOutput -Force | Out-Null
$includeUser = Join-Path $projectRoot 'User'
$mathSource = Join-Path $projectRoot 'User\servo\servo_math.c'

foreach ($name in @('jaw', 'lift')) {
    $testSource = Join-Path $projectRoot ('Tests\test_servo_{0}.c' -f $name)
    $sourceArgs = (@($testSource, $mathSource) | ForEach-Object { '"' + $_ + '"' }) -join ' '
    $command = 'call "' + $vcvars + '" >nul && cd /d "' + $testOutput + '" && cl /nologo /W4 /WX /std:c11 /D_CRT_SECURE_NO_WARNINGS /I"' + $includeUser + '" ' + $sourceArgs + ' /Fe:servo_' + $name + '_tests.exe && servo_' + $name + '_tests.exe'
    & $env:ComSpec /d /s /c $command
    if ($LASTEXITCODE -ne 0) { throw "Servo $name host tests failed" }
}
