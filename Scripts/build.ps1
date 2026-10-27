param([switch]$Rebuild, [string]$KeilPath)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
$projectFile = Join-Path $projectRoot 'Project\YH-F429.uvprojx'
$logFile = Join-Path $projectRoot 'Output\build.log'
$buildMode = if ($Rebuild) { '-r' } else { '-b' }
if (-not $KeilPath) {
    $KeilPath = @('D:\keil5\UV4\UV4.exe', 'C:\Keil_v5\UV4\UV4.exe') | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
}
if (-not $KeilPath) { throw 'Keil not found; pass -KeilPath to UV4.exe' }
New-Item -ItemType Directory -Path (Split-Path $logFile -Parent) -Force | Out-Null
if (Test-Path -LiteralPath $logFile) { Remove-Item -LiteralPath $logFile }
$process = Start-Process -FilePath $KeilPath -WorkingDirectory $projectRoot -ArgumentList @($buildMode, ('"'+$projectFile+'"'), '-t', 'LCD', '-o', ('"'+$logFile+'"')) -WindowStyle Hidden -Wait -PassThru
if (-not (Test-Path -LiteralPath $logFile)) { throw 'Keil produced no build log' }
$log = Get-Content -LiteralPath $logFile
$log | Select-Object -Last 15
if ($process.ExitCode -gt 1 -or -not ($log -match '0 Error\(s\)')) { throw 'Keil build failed; see Output/build.log' }
$warnings = $log | Where-Object { $_ -match 'warning:' }
if ($warnings) { $warnings; throw 'Compiler warning(s) found' }
Write-Output 'BUILD PASS (0 warnings).'
