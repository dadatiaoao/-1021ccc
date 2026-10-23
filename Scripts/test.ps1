$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
$vswhere = 'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe'
$vcvars = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -find 'VC\Auxiliary\Build\vcvars64.bat' | Select-Object -First 1
if (-not $vcvars) { throw 'MSVC C build tools not found' }
$testOutput = Join-Path $projectRoot 'Output\host-tests'
New-Item -ItemType Directory -Path $testOutput -Force | Out-Null
$sources = @((Join-Path $projectRoot 'Tests\test_motor.c'), (Join-Path $projectRoot 'Tests\test_ui.c'), (Join-Path $projectRoot 'User\app\motor_ui.c'), (Join-Path $projectRoot 'User\fonts\font16.c')) + @(Get-ChildItem -LiteralPath (Join-Path $projectRoot 'User\motor') -Filter '*.c' | ForEach-Object { $_.FullName })
$sourceArgs = ($sources | ForEach-Object { '"' + $_ + '"' }) -join ' '
$command = 'call "' + $vcvars + '" >nul && cd /d "' + $testOutput + '" && cl /nologo /W4 /WX /std:c11 /D_CRT_SECURE_NO_WARNINGS /I"' + (Join-Path $projectRoot 'Tests\stubs') + '" /I"' + (Join-Path $projectRoot 'User') + '" ' + $sourceArgs + ' /Fe:motor_tests.exe && motor_tests.exe'
& $env:ComSpec /d /s /c $command
if ($LASTEXITCODE -ne 0) { throw 'Motor host tests failed' }
$qrSources = @('Tests\test_qr.c', 'Tests\vision_app_stub.c', 'User\vision\vision_protocol.c', 'User\vision\vision_service.c', 'User\vision\vision_ui.c', 'Tests\flow_app_stub.c', 'Tests\gyro_app_stub.c', 'User\gyro\icm42688.c', 'User\gyro\gyro_service.c', 'User\gyro\gyro_ui.c', 'User\flow\pmw3901.c', 'User\flow\flow_service.c', 'User\flow\flow_ui.c', 'User\app\qr_ui.c', 'User\app\qr_app.c', 'User\app\app_runtime.c',
    'User\usart\bsp_qr_uart.c', 'User\usart\bsp_qr_trigger.c', 'User\usart\bsp_uart_callbacks.c',
    'User\motor\emm_protocol.c', 'User\motor\motor_service.c', 'User\motor\motor_bus.c',
    'User\fonts\font16.c', 'User\fonts\font24.c') | ForEach-Object { Join-Path $projectRoot $_ }
$qrSources += @(Get-ChildItem -LiteralPath (Join-Path $projectRoot 'User\qr') -Filter '*.c' | ForEach-Object { $_.FullName })
$qrSources += @(Get-ChildItem -LiteralPath (Join-Path $projectRoot 'User\chassis') -Filter '*.c' | ForEach-Object { $_.FullName })
$qrSources += @(Get-ChildItem -LiteralPath (Join-Path $projectRoot 'User\mission') -Filter '*.c' | ForEach-Object { $_.FullName })
$qrArgs = ($qrSources | ForEach-Object { '"' + $_ + '"' }) -join ' '
# Existing gyro calibration tests use the normal calibration mode.
# A separate fixed-bias build below covers the shipped application configuration.
$qrCommand = 'call "' + $vcvars + '" >nul && cd /d "' + $testOutput + '" && cl /nologo /W4 /WX /std:c11 /D_CRT_SECURE_NO_WARNINGS /DGYRO_TEMP_FIXED_BIAS_ENABLED=0 /I"' + (Join-Path $projectRoot 'Tests\stubs') + '" /I"' + (Join-Path $projectRoot 'User') + '" ' + $qrArgs + ' /Fe:qr_tests.exe && qr_tests.exe && qr_tests.exe init && qr_tests.exe dma && qr_tests.exe receive && qr_tests.exe start && qr_tests.exe review-ui && qr_tests.exe review-gate && qr_tests.exe chassis-screen && qr_tests.exe review-parallel && qr_tests.exe chassis-active && qr_tests.exe vision-screen && qr_tests.exe mission-screen'
& $env:ComSpec /d /s /c $qrCommand
if ($LASTEXITCODE -ne 0) { throw 'QR host tests failed' }
$busSources = @('Tests\test_motor_bus.c', 'User\motor\emm_protocol.c', 'User\motor\motor_bus.c') | ForEach-Object { Join-Path $projectRoot $_ }
$busArgs = ($busSources | ForEach-Object { '"' + $_ + '"' }) -join ' '
$busCommand = 'call "' + $vcvars + '" >nul && cd /d "' + $testOutput + '" && cl /nologo /W4 /WX /std:c11 /D_CRT_SECURE_NO_WARNINGS /I"' + (Join-Path $projectRoot 'User') + '" ' + $busArgs + ' /Fe:bus_tests.exe && bus_tests.exe && bus_tests.exe review'
& $env:ComSpec /d /s /c $busCommand
if ($LASTEXITCODE -ne 0) { throw 'Multi-motor host tests failed' }
# Same bus tests compiled with the pre-relaxation gates restored, so both the
# relaxed defaults and the old strict behaviour stay covered.
$busLegacyCommand = 'call "' + $vcvars + '" >nul && cd /d "' + $testOutput + '" && cl /nologo /W4 /WX /std:c11 /D_CRT_SECURE_NO_WARNINGS /DMOTOR_DRIVER_REQUIRE_MANUAL=1 /DMOTOR_BENCH_JOG_ENABLED=0 /DMOTOR_UNKNOWN_AUTO_RECOVER_MS=0 /I"' + (Join-Path $projectRoot 'User') + '" ' + $busArgs + ' /Fe:bus_legacy_tests.exe && bus_legacy_tests.exe && bus_legacy_tests.exe review'
& $env:ComSpec /d /s /c $busLegacyCommand
if ($LASTEXITCODE -ne 0) { throw 'Legacy strict-gate bus tests failed' }
$uartSources = @('Tests\test_motor_uart.c', 'Tests\vision_app_stub.c', 'User\usart\bsp_motor_uart.c', 'User\usart\bsp_uart_callbacks.c',
    'User\motor\emm_protocol.c', 'User\motor\motor_bus.c', 'User\motor\motor_service.c') | ForEach-Object { Join-Path $projectRoot $_ }
$uartArgs = ($uartSources | ForEach-Object { '"' + $_ + '"' }) -join ' '
$uartCommand = 'call "' + $vcvars + '" >nul && cd /d "' + $testOutput + '" && cl /nologo /W4 /WX /std:c11 /D_CRT_SECURE_NO_WARNINGS /I"' + (Join-Path $projectRoot 'Tests\stubs') + '" /I"' + (Join-Path $projectRoot 'User') + '" ' + $uartArgs + ' /Fe:uart_tests.exe && uart_tests.exe && uart_tests.exe init && uart_tests.exe dma && uart_tests.exe receive && uart_tests.exe start && uart_tests.exe review'
& $env:ComSpec /d /s /c $uartCommand
if ($LASTEXITCODE -ne 0) { throw 'Motor UART5 BSP host tests failed' }
$configuredCommand = $qrCommand.Replace('/D_CRT_SECURE_NO_WARNINGS', '/D_CRT_SECURE_NO_WARNINGS /DMOTOR_TEST_CONFIGURED=1 /DMOTOR_TEST_PULSES_PER_REV=3200').Replace('/Fe:qr_tests.exe', '/Fe:qr_manual_tests.exe').Replace('&& qr_tests.exe', '&& qr_manual_tests.exe')
& $env:ComSpec /d /s /c $configuredCommand
if ($LASTEXITCODE -ne 0) { throw 'Configured manual application tests failed' }
$fixedCommand = $qrCommand.Replace('/DGYRO_TEMP_FIXED_BIAS_ENABLED=0', '/DGYRO_TEMP_FIXED_BIAS_ENABLED=1').Replace('/Fe:qr_tests.exe', '/Fe:qr_fixed_tests.exe')
$fixedCommand = $fixedCommand.Substring(0, $fixedCommand.IndexOf(' && qr_tests.exe')) + ' && qr_fixed_tests.exe chassis-screen'
& $env:ComSpec /d /s /c $fixedCommand
if ($LASTEXITCODE -ne 0) { throw 'Default fixed-bias application tests failed' }
& (Join-Path $PSScriptRoot "test_chassis.ps1")

& (Join-Path $PSScriptRoot "test_flow.ps1")

& (Join-Path $PSScriptRoot "test_gyro.ps1")

& (Join-Path $PSScriptRoot "test_vision.ps1")

& (Join-Path $PSScriptRoot "test_mission.ps1")
