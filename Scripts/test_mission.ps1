$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$vswhere = 'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe'
$vcvars = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -find 'VC\Auxiliary\Build\vcvars64.bat' | Select-Object -First 1
if (-not $vcvars) { throw 'MSVC C tools not found' }
$out = Join-Path $root 'Output\host-tests-mission'
New-Item -ItemType Directory -Path $out -Force | Out-Null
$sources = @('Tests\test_mission.c','User\mission\mission.c','User\mission\mission_ui.c','User\qr\qr_task_code.c')
$sourceArgs = ($sources | ForEach-Object {'"'+(Join-Path $root $_)+'"'}) -join ' '
$command = 'call "'+$vcvars+'" >nul && cd /d "'+$out+'" && cl /nologo /W4 /WX /std:c11 /D_CRT_SECURE_NO_WARNINGS /I"'+(Join-Path $root 'Tests\stubs')+'" /I"'+(Join-Path $root 'User')+'" '+$sourceArgs+' /Fe:mission_tests.exe && mission_tests.exe'
& $env:ComSpec /d /s /c $command
if ($LASTEXITCODE -ne 0) { throw 'mission regression failed' }
