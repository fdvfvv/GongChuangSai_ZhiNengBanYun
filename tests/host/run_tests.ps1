# 主机端单元测试：只编译纯数学/轨迹状态机，不依赖 STM32 HAL
# 用法：powershell -ExecutionPolicy Bypass -File tests/host/run_tests.ps1

$ErrorActionPreference = "Stop"

$root  = Resolve-Path (Join-Path $PSScriptRoot "..\..")
$out1  = Join-Path $PSScriptRoot "test_chassis_math.exe"
$out2  = Join-Path $PSScriptRoot "test_trajectory.exe"

Write-Host "Compiling chassis_math tests..."
gcc -std=c11 -O2 -Wall -Wextra `
    -I "$root\Core\Inc" `
    "$PSScriptRoot\test_chassis_math.c" `
    "$root\Core\Src\chassis_math.c" `
    -lm -o $out1

Write-Host "Compiling trajectory tests..."
gcc -std=c11 -O2 -Wall -Wextra `
    -I "$PSScriptRoot\stubs" -I "$root\Core\Inc" `
    "$PSScriptRoot\test_trajectory.c" `
    "$root\Core\Src\trajectory.c" `
    "$root\Core\Src\chassis_math.c" `
    -lm -o $out2

Write-Host "Running chassis_math tests..."
& $out1
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Host "Running trajectory tests..."
& $out2
exit $LASTEXITCODE
