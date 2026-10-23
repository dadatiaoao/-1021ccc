param([string]$PythonPath = 'python')
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$vswhere = 'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe'
$vcvars = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -find 'VC\Auxiliary\Build\vcvars64.bat' | Select-Object -First 1
if (-not $vcvars) { throw 'MSVC C tools not found' }
& $PythonPath (Join-Path $root 'Tests\generate_vision_fixtures.py')
if ($LASTEXITCODE -ne 0) { throw 'K230 fixture generation failed' }
$testOutput = Join-Path $root 'Output\host-tests'
$common = @('User\vision\vision_protocol.c','User\vision\vision_service.c','User\vision\vision_ui.c','User\qr\qr_task_code.c')
foreach ($suite in @('vision','vision_uart')) {
    $sources = @('Tests\test_' + $suite + '.c') + $common
    if ($suite -eq 'vision_uart') { $sources += @('User\usart\bsp_vision_uart.c','User\usart\bsp_uart_callbacks.c') }
    $args = ($sources | ForEach-Object { '"' + (Join-Path $root $_) + '"' }) -join ' '
    $exe = $suite + '_tests.exe'
    $command = 'call "' + $vcvars + '" >nul && cd /d "' + $testOutput + '" && cl /nologo /W4 /WX /std:c11 /D_CRT_SECURE_NO_WARNINGS /I"' + (Join-Path $root 'Tests\stubs') + '" /I"' + (Join-Path $root 'User') + '" /I"' + $testOutput + '" ' + $args + ' /Fe:' + $exe + ' && ' + $exe
    if ($suite -eq 'vision_uart') { foreach ($scenario in @('init','dma','receive','start','review')) { $command += ' && ' + $exe + ' ' + $scenario } }
    & $env:ComSpec /d /s /c $command
    if ($LASTEXITCODE -ne 0) { throw "$suite tests failed" }
}
