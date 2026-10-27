$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$vswhere = 'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe'
$vcvars = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -find 'VC\Auxiliary\Build\vcvars64.bat' | Select-Object -First 1
if (-not $vcvars) { throw 'MSVC C tools not found' }
$out = Join-Path $root 'Output\host-tests-servo'
New-Item -ItemType Directory -Path $out -Force | Out-Null
$sources = @('Tests\test_servo.c','Tests\servo_app_stub.c','User\servo\servo_control.c','User\servo\servo_math.c')
$sourceArgs = ($sources | ForEach-Object {'"'+(Join-Path $root $_)+'"'}) -join ' '
$command = 'call "'+$vcvars+'" >nul && cd /d "'+$out+'" && cl /nologo /W4 /WX /std:c11 /D_CRT_SECURE_NO_WARNINGS /I"'+(Join-Path $root 'Tests\stubs')+'" /I"'+(Join-Path $root 'User')+'" '+$sourceArgs+' /Fe:servo_tests.exe && servo_tests.exe'
& $env:ComSpec /d /s /c $command
if ($LASTEXITCODE -ne 0) { throw 'servo regression failed' }
