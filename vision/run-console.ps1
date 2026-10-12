param([ValidateRange(1024,65535)][int]$Port = 8082)
$ErrorActionPreference = 'Stop'
$consoleRoot = Split-Path -Parent $PSScriptRoot
$consolePython = Join-Path $consoleRoot '.venv\Scripts\python.exe'
$consoleEntry = Join-Path $PSScriptRoot 'tools\console_forward.py'
$consoleLogs = Join-Path $consoleRoot 'run\console-forward'
if (-not (Test-Path -LiteralPath $consolePython)) { throw '项目 .venv 环境不存在。' }
$consoleListener = Get-NetTCPConnection -State Listen -LocalPort $Port -ErrorAction SilentlyContinue
if ($consoleListener) {
    $consoleOwner = Get-CimInstance Win32_Process -Filter "ProcessId=$($consoleListener[0].OwningProcess)"
    if (-not $consoleOwner.CommandLine -or -not $consoleOwner.CommandLine.Contains($consoleEntry) -or $consoleListener[0].LocalAddress -ne '127.0.0.1') { throw "端口 $Port 被其他程序占用。" }
    Write-Output "车辆控制台直连入口已运行：http://127.0.0.1:$Port/"
    return
}
New-Item -ItemType Directory -Force -Path $consoleLogs | Out-Null
$consoleStamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$consoleArgs = @(('"{0}"' -f $consoleEntry), '--port', "$Port")
$env:PYTHONIOENCODING = 'utf-8'
$consoleProcess = Start-Process -FilePath $consolePython -ArgumentList $consoleArgs -WorkingDirectory $consoleRoot -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $consoleLogs "$consoleStamp.out.log") -RedirectStandardError (Join-Path $consoleLogs "$consoleStamp.err.log")
for ($consoleAttempt = 0; $consoleAttempt -lt 20; $consoleAttempt++) {
    $consoleReady = Get-NetTCPConnection -State Listen -LocalAddress '127.0.0.1' -LocalPort $Port -ErrorAction SilentlyContinue
    if ($consoleReady) {
        # Windows venv python.exe may spawn the actual listener as a child.
        $consoleReadyOwner = Get-CimInstance Win32_Process -Filter "ProcessId=$($consoleReady[0].OwningProcess)"
        if ($consoleReadyOwner.CommandLine -and $consoleReadyOwner.CommandLine.Contains($consoleEntry)) { break }
        throw "端口 $Port 被其他程序占用。"
    }
    $consoleProcess.Refresh()
    if ($consoleProcess.HasExited) { throw "入口启动失败，请查看 $consoleLogs。" }
    Start-Sleep -Milliseconds 250
}
if (-not $consoleReady) { throw '入口未就绪，请检查本机端口占用。' }
Write-Output "车辆控制台直连入口：http://127.0.0.1:$Port/"
Write-Output "本机转发进程：$($consoleReady[0].OwningProcess)；车端原有控制服务保持运行。"
