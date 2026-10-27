$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$vswhere = 'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe'
$vcvars = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -find 'VC\Auxiliary\Build\vcvars64.bat' | Select-Object -First 1
if (-not $vcvars) { throw 'MSVC C tools not found' }
$testOutput = Join-Path $root 'Output\host-tests'
New-Item -ItemType Directory -Path $testOutput -Force | Out-Null
foreach ($suite in @('mission','mission_runtime')) {
    $sources = @(('Tests\test_' + $suite + '.c'),'User\mission\mission.c','User\mission\mission_ui.c','User\qr\qr_task_code.c')
    if ($suite -eq 'mission_runtime') { $sources += @('User\mission\mission_config.c','User\mission\mission_runtime.c','User\chassis\chassis_kinematics.c','User\vision\vision_protocol.c','User\vision\vision_service.c') }
    $sourceArgs = ($sources | ForEach-Object { '"' + (Join-Path $root $_) + '"' }) -join ' '
    $exe = $suite + '_tests.exe'
    $command = 'call "' + $vcvars + '" >nul && cd /d "' + $testOutput + '" && cl /nologo /W4 /WX /std:c11 /D_CRT_SECURE_NO_WARNINGS /I"' + (Join-Path $root 'User') + '" ' + $sourceArgs + ' /Fe:' + $exe + ' && ' + $exe
    & $env:ComSpec /d /s /c $command
    if ($LASTEXITCODE -ne 0) { throw "$suite tests failed" }
}
