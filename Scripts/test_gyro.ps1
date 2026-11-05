$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
$vswhere = 'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe'
$vcvars = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -find 'VC\Auxiliary\Build\vcvars64.bat' | Select-Object -First 1
if (-not $vcvars) { throw 'MSVC C build tools not found' }
$out = Join-Path $projectRoot 'Output\host-gyro'
New-Item -ItemType Directory -Path $out -Force | Out-Null
$sources = @('Tests\test_gyro.c','User\gyro\icm42688.c','User\gyro\gyro_service.c','User\gyro\gyro_ui.c') | ForEach-Object { '"' + (Join-Path $projectRoot $_) + '"' }
# Host tests use the same button-calibration configuration as the firmware.
$base = 'call "' + $vcvars + '" >nul && cd /d "' + $out + '" && cl /nologo /W4 /WX /std:c11 /D_CRT_SECURE_NO_WARNINGS /I"' + (Join-Path $projectRoot 'User') + '" '
foreach ($mapping in @('', '/DGYRO_AXIS=0 /DGYRO_SIGN=-1', '/DGYRO_AXIS=1 /DGYRO_SIGN=1')) {
    $command = $base + $mapping + ' ' + ($sources -join ' ') + ' /Fe:gyro_tests.exe && gyro_tests.exe'
    & $env:ComSpec /d /s /c $command
    if ($LASTEXITCODE -ne 0) { throw 'Gyro host tests failed' }
}
$busSources = @('Tests\test_gyro_bus.c','User\gyro\bsp_gyro_bus.c') | ForEach-Object { '"' + (Join-Path $projectRoot $_) + '"' }
foreach ($address in @('0x68','0x69')) {
    $command = $base + '/DGYRO_ADDRESS_7BIT=' + $address + ' /I"' + (Join-Path $projectRoot 'Tests\gyro_stubs') + '" ' + ($busSources -join ' ') + ' /Fe:gyro_bus_tests.exe && gyro_bus_tests.exe'
    & $env:ComSpec /d /s /c $command
    if ($LASTEXITCODE -ne 0) { throw 'Gyro I2C1 BSP tests failed' }
}
