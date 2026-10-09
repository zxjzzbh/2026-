param([string]$Preview = 'http://100.124.156.117:8080', [int]$Port = 8091)
$ErrorActionPreference = 'Stop'
$captureRoot = Split-Path -Parent $PSScriptRoot
$capturePython = Join-Path $captureRoot '.venv\Scripts\python.exe'
$captureEntry = Join-Path $PSScriptRoot 'tools\traffic_capture.py'
$captureOutput = Join-Path $captureRoot 'data\raw\traffic_lights'
$captureLogs = Join-Path $captureRoot 'run\traffic-capture-service'
if (-not (Test-Path -LiteralPath $capturePython)) { throw '项目 .venv 环境不存在，请先安装 vision 依赖。' }
$captureUrl = "http://127.0.0.1:$Port"
try { $captureExisting = Invoke-RestMethod -Uri "$captureUrl/api/status" -TimeoutSec 2 } catch { $captureExisting = $null }
if ($captureExisting -and $captureExisting.app -eq 'traffic-light-capture') {
    Write-Output "采集已运行：$captureUrl/"
    Write-Output "保存目录：$($captureExisting.output)"
    return
}
if ($captureExisting) { throw "端口 $Port 已有其他服务，请指定其他 -Port。" }
New-Item -ItemType Directory -Force -Path $captureLogs | Out-Null
$captureStamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$captureArgs = @(('"{0}"' -f $captureEntry), '--preview', ('"{0}"' -f $Preview), '--output', ('"{0}"' -f $captureOutput), '--port', "$Port")
$env:PYTHONIOENCODING = 'utf-8'
$captureProcess = Start-Process -FilePath $capturePython -ArgumentList $captureArgs -WorkingDirectory $captureRoot -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $captureLogs "$captureStamp.out.log") -RedirectStandardError (Join-Path $captureLogs "$captureStamp.err.log")
for ($captureAttempt=0; $captureAttempt -lt 20; $captureAttempt++) {
    try { $captureReady = Invoke-RestMethod -Uri "$captureUrl/api/status" -TimeoutSec 1 } catch { $captureReady = $null }
    if ($captureReady -and $captureReady.app -eq 'traffic-light-capture') {
        Write-Output "采集已启动：$captureUrl/"
        Write-Output "保存目录：$captureOutput"
        Write-Output "进程：$($captureProcess.Id)；退出请点击页面底部‘退出采集程序’。"
        return
    }
    if ($captureProcess.HasExited) { throw "启动失败，请查看 $captureLogs 中的错误日志。" }
    Start-Sleep -Milliseconds 250
}
throw "采集服务未就绪，请查看 $captureLogs。"
