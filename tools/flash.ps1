# ============================================================================
#  飞箱课题三 —— 编译 + 烧录一条龙
#
#  用法（在 PowerShell 里）：
#      .\tools\flash.ps1              只烧录
#      .\tools\flash.ps1 -Build       先重新生成再烧
#
#  ★★★ 为什么不用「reset halt」★★★
#
#   这块板子的 nRESET（SRST）读不到，探针报 nRESET = 0。
#   于是 openocd 的 `reset halt` 会超时：
#       Error: timed out while waiting for target halted
#       ** Unable to reset target **
#   但 `halt`（靠 SWD 的 debug-request 停核）是好的。
#
#   所以流程改成：init -> halt -> program -> reset run
#   代价：烧完若不成功复位，需要手动断电重启一次。
#         （多数情况 reset run 能生效，只是不走 SRST 那条路）
#
#  ★ 如果哪天又出现 cannot read IDR / 连不上：
#     1) 检查板子供电
#     2) 检查复位键有没有卡住
#     3) 拔插一次探针
#     4) 都正常还是不行 -> 跑一次 .\tools\flash.ps1 -UnderReset
#        （connect_assert_srst，靠拉低复位让芯片停下来再连）
# ============================================================================

param(
	[switch]$Build,          # 先重新生成解决方案
	[switch]$UnderReset,     # 用 connect_assert_srst 模式（连不上时试）
	[string]$Configuration = "Debug"
)

$ErrorActionPreference = "Stop"

# ---- 路径 ----
$repoRoot = Split-Path -Parent $PSScriptRoot
$sln      = Join-Path $repoRoot "EmbeddedProject1\EmbeddedProject1.sln"
$bin      = Join-Path $repoRoot "EmbeddedProject1\VisualGDB\$Configuration\EmbeddedProject1.bin"
$msbuild  = "D:\VS2022\MSBuild\Current\Bin\MSBuild.exe"

$ocdRoot  = "C:\Users\WangHuaYu\AppData\Local\VisualGDB\EmbeddedDebugPackages\com.sysprogs.arm.openocd"
$ocdExe   = Join-Path $ocdRoot "bin\openocd.exe"
$ocdScr   = Join-Path $ocdRoot "share\openocd\scripts"

# ---- 1) 编译 ----
if ($Build) {
	if (-not (Test-Path $msbuild)) { throw "找不到 MSBuild: $msbuild" }
	Write-Host "=== 重新生成解决方案 ===" -ForegroundColor Cyan
	$out = & $msbuild $sln /t:Rebuild /p:Configuration=$Configuration /p:Platform=VisualGDB /v:minimal /nologo 2>&1
	if ($LASTEXITCODE -ne 0) {
		$out | Select-String -Pattern "error" | Select-Object -First 20 | ForEach-Object { Write-Host $_.Line -ForegroundColor Red }
		throw "编译失败 (EXIT=$LASTEXITCODE)"
	}
	$out | Select-String -Pattern "Used FLASH|Used SRAM" | ForEach-Object { Write-Host $_.Line }
}

# ---- 2) 检查产物 ----
if (-not (Test-Path $bin)) { throw "找不到固件: $bin（先加 -Build）" }
$f = Get-Item $bin
Write-Host ("=== 固件: {0}  {1} bytes  {2} ===" -f $f.Name, $f.Length, $f.LastWriteTime) -ForegroundColor Cyan

# ---- 3) 烧录 ----
if (-not (Test-Path $ocdExe)) { throw "找不到 openocd: $ocdExe" }

# ★ 路径必须用正斜杠：反斜杠会被 openocd 的 TCL 当转义吃掉，
#   表现为 "couldn't open D:FLY_BOX_Project3EmbeddedP..." 这种丢分隔符的错误。
$binFwd = $bin -replace '\\', '/'

$sdArgs = @("-s", $ocdScr, "-c", "adapter driver cmsis-dap", "-c", "transport select swd", "-c", "adapter speed 1000")

if ($UnderReset) {
	Write-Host "--- 模式: connect under reset ---" -ForegroundColor Yellow
	$sdArgs += @("-c", "reset_config srst_only srst_nogate connect_assert_srst")
	$haltCmd = "reset halt"
} else {
	$haltCmd = "halt"        # ★ 不用 reset halt，见文件头说明
}

$sdArgs += @("-f", "target/stm32f1x.cfg",
             "-c", "init",
             "-c", $haltCmd,
             "-c", "program {$binFwd} 0x08000000 verify",
             "-c", "reset run",
             "-c", "shutdown")

$out = & $ocdExe @sdArgs 2>&1
$rc  = $LASTEXITCODE

$out | Select-String -Pattern "DPIDR|Programming|Verified|Error|halted|device id|flash size|Warn" |
	ForEach-Object {
		$line = $_.Line
		if ($line -match "Error|Failed") { Write-Host $line -ForegroundColor Red }
		elseif ($line -match "Verified OK") { Write-Host $line -ForegroundColor Green }
		else { Write-Host $line }
	}

if ($rc -ne 0) {
	Write-Host ""
	Write-Host "烧录失败 (EXIT=$rc)" -ForegroundColor Red
	Write-Host "排查顺序：供电 -> 复位键 -> 拔插探针 -> 试 -UnderReset" -ForegroundColor Yellow
	exit 1
}

Write-Host ""
Write-Host "=== 烧录完成，已复位运行 ===" -ForegroundColor Green
Write-Host "上电后自动跑一轮拉箱，盯 0x7E3：0xB7(TOF健康) 0xB5(流程进度) 0xB8(距离)" -ForegroundColor Cyan
