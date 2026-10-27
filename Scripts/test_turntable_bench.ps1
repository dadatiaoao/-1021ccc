$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$vswhere = 'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe'
$vcvars = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -find 'VC\Auxiliary\Build\vcvars64.bat' | Select-Object -First 1
if (-not $vcvars) { throw 'MSVC C tools not found' }
$out = Join-Path $root 'Output\host-tests-table'
New-Item -ItemType Directory -Path $out -Force | Out-Null
$sources = @('Tests\test_servo_bench.c','User\servo\servo_bench.c','User\servo\servo_math.c','User\fonts\font16.c')
$args = ($sources | ForEach-Object {'"'+(Join-Path $root $_)+'"'}) -join ' '
$command = 'call "'+$vcvars+'" >nul && cd /d "'+$out+'" && cl /nologo /W4 /WX /std:c11 /D_CRT_SECURE_NO_WARNINGS /DSERVO_BENCH_ENABLE=1 /DSERVO_TURNTABLE_BENCH=1 /I"'+(Join-Path $root 'Tests\stubs')+'" /I"'+(Join-Path $root 'User')+'" '+$args+' /Fe:turntable_bench_tests.exe && turntable_bench_tests.exe'
& $env:ComSpec /d /s /c $command
if ($LASTEXITCODE -ne 0) { throw 'Turntable bench regression failed' }
