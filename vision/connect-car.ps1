param(
    [switch]$NoOpen,
    [switch]$CheckOnly,
    [ValidateRange(1024,65535)][int]$Port = 8082
)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'tools/connection_helpers.ps1')
$carRoot = Split-Path -Parent $PSScriptRoot
$carAddress = '100.124.156.117'
$carDirectUrl = "http://${carAddress}:8080/api/drive/status"
$carLocalUrl = "http://127.0.0.1:$Port/"
$carForwardEntry = Join-Path $PSScriptRoot 'tools/console_forward.py'
$carLogFolder = Join-Path $carRoot 'run/connection'
[void](New-Item -ItemType Directory -Force -Path $carLogFolder)
$carLogPath = Join-Path $carLogFolder ((Get-Date -Format 'yyyyMMdd-HHmmss-fff')+'.json')
$carReport = [ordered]@{
    checked_at = (Get-Date).ToString('o'); target = $carAddress; local_url = $carLocalUrl
    check_only = [bool]$CheckOnly; connected = $false; changes = @()
    vehicle_commands_sent = $false; ssh_authenticated = $false; error = $null
}
$carExit = 1
$carMutex = New-Object Threading.Mutex($false, 'Local\SmartcarConnect8082')
$carMutexHeld = $false
try {
    try { $carMutexHeld = $carMutex.WaitOne(0) }
    catch [Threading.AbandonedMutexException] { $carMutexHeld = $true }
    if (-not $carMutexHeld) { throw '另一个连接检查正在运行，请等待它完成。' }
    Write-Host '智能车一键连接：只连接通信，不启用电机。'
    Write-Host '[1/4] 检查 Tailscale…'
    $carTailCommand = Get-Command tailscale.exe -ErrorAction SilentlyContinue
    $carTailExe = if ($carTailCommand) { $carTailCommand.Source } else { Join-Path $env:ProgramFiles 'Tailscale/tailscale.exe' }
    if (-not (Test-Path -LiteralPath $carTailExe)) { throw '未找到 Tailscale，请先安装并登录已有账号，再运行此脚本。' }
    $carService = Get-Service Tailscale -ErrorAction Stop
    if ($carService.Status -ne 'Running') {
        if ($CheckOnly) { throw 'Tailscale 服务未运行；只读检查不会启动服务。' }
        try {
            Start-Service -Name Tailscale
            $carService.WaitForStatus('Running', [TimeSpan]::FromSeconds(8))
            $carReport.changes += 'started_existing_tailscale_service'
        } catch { throw '无法启动 Tailscale 服务。请启动客户端，或右键以管理员身份运行连接脚本。' }
    }
    $carTailResult = Invoke-CarTailscale $carTailExe @('status','--json')
    if ($carTailResult.ExitCode -ne 0) { throw '无法读取 Tailscale 状态，请检查客户端是否响应。' }
    $carTailState = $carTailResult.Output | ConvertFrom-Json
    if ($carTailState.BackendState -eq 'Stopped' -and -not $CheckOnly) {
        Write-Host '正在恢复已有 Tailscale 连接…'
        # Local CLI help confirms: bare "up" changes no saved settings.
        $carUp = Invoke-CarTailscale $carTailExe @('up') 12000
        $carReport.changes += 'requested_existing_tailscale_up'
        $carTailResult = Invoke-CarTailscale $carTailExe @('status','--json')
        if ($carTailResult.ExitCode -ne 0) { throw 'Tailscale 连接未就绪，请在客户端检查后重试。' }
        $carTailState = $carTailResult.Output | ConvertFrom-Json
    }
    $carReport.tailscale_state = $carTailState.BackendState
    $carPeer = @($carTailState.Peer.PSObject.Properties.Value) | Where-Object { $_.TailscaleIPs -contains $carAddress } | Select-Object -First 1
    $carReport.target_visible = [bool]$carPeer
    $carReport.target_online = if ($carPeer) { [bool]$carPeer.Online } else { $null }
    if ($carTailState.BackendState -ne 'Running') {
        throw ('Tailscale 尚未连接（'+$carTailState.BackendState+'）。请在客户端恢复现有账号连接，再重试；脚本不会重新注册或改账号。')
    }
    if (-not $CheckOnly) {
        [void](Invoke-CarTailscale $carTailExe @('ping','--c','2','--timeout','2s',$carAddress) 6500)
    }

    Write-Host '[2/4] 检查小车控制台和 SSH 端口…'
    $carRemote = Get-CarHttpStatus $carDirectUrl
    $carReport.ssh_port_reachable = Test-CarTcp $carAddress 22
    if (-not $carRemote.Ok) {
        $carReport.remote_status_error = $carRemote.Error
        $carReport.web_port_reachable = Test-CarTcp $carAddress 8080
        if ($carPeer -and -not $carPeer.Online) {
            throw '电脑的Tailscale已连接，但小车显示离线。请让车上树莓派和5G模块上电联网，再双击本脚本；不用重新输入账号或SSH密码。'
        }
        throw '车端控制台暂未连通。请检查电脑联网、小车供电和车上5G/Wi-Fi；若SSH能通但网页不通，请检查车端预览服务。本脚本不重启车端服务。'
    }
    $carReport.web_port_reachable = $true
    $carReport.control_active = Test-CarControlActive $carRemote.Data
    Write-Host ('小车网页已连通；SSH 22端口可达：'+$carReport.ssh_port_reachable+'（未执行SSH登录）。')

    Write-Host '[3/4] 检查本机 8082 转发入口…'
    $carListeners = @(Get-NetTCPConnection -State Listen -LocalPort $Port -ErrorAction SilentlyContinue)
    $carLocal = Get-CarHttpStatus ($carLocalUrl+'api/drive/status')
    if (-not $carLocal.Ok) { $carReport.local_status_error = $carLocal.Error }
    if ($carListeners.Count -gt 0) {
        if ($carListeners.Count -ne 1) { throw "端口 $Port 有其他监听，未结束任何进程。" }
        $carListener = $carListeners[0]
        $carOwner = Get-CimInstance Win32_Process -Filter "ProcessId=$($carListener.OwningProcess)"
        if (-not (Test-OwnedCarForward $carListener $carOwner $carForwardEntry)) {
            throw "端口 $Port 由其他程序占用，未修改或结束该程序。"
        }
        if ($carLocal.Ok) {
            $carReport.forwarder = 'reused'
            Write-Host '已有转发正常，直接复用。'
        } else {
            if ($CheckOnly) { throw '本机转发响应异常；只读检查不会重建。' }
            # Refresh remote state immediately before interrupting a local relay.
            $carRecheck = Get-CarHttpStatus $carDirectUrl
            if (-not $carRecheck.Ok) { throw '车端链路再次超时，暂不重启本机转发。请检查车上联网状态后重试。' }
            if (Test-CarControlActive $carRecheck.Data) { throw '车端仍有控制会话，未重启转发。请先停止测试，再运行连接脚本。' }
            $carCurrentOwner = Get-CimInstance Win32_Process -Filter "ProcessId=$($carListener.OwningProcess)"
            if (-not (Test-OwnedCarForward $carListener $carCurrentOwner $carForwardEntry) -or $carCurrentOwner.CreationDate -ne $carOwner.CreationDate) {
                throw '端口进程已变化，未结束任何进程，请重试。'
            }
            Stop-Process -Id $carCurrentOwner.ProcessId -ErrorAction Stop
            Wait-Process -Id $carCurrentOwner.ProcessId -Timeout 5 -ErrorAction SilentlyContinue
            $carReport.changes += 'restarted_owned_inactive_local_forwarder'
            $carReport.forwarder = 'restarted'
        }
    }
    if (-not $carLocal.Ok) {
        if ($CheckOnly) { throw '本机转发尚未运行；只读检查不会启动。' }
        & (Join-Path $PSScriptRoot 'run-console.ps1') -Port $Port
        if (-not $carReport.Contains('forwarder')) { $carReport.forwarder = 'started' }
        $carLocal = Get-CarHttpStatus ($carLocalUrl+'api/drive/status') 4000
    }
    if (-not $carLocal.Ok) { throw '本机入口未通过最终检查，请查看连接记录后重试。' }
    $carReport.control_active = Test-CarControlActive $carLocal.Data
    $carReport.connected = $true
    Write-Host ('[4/4] 连接成功：'+$carLocalUrl) -ForegroundColor Green
    if ($carReport.control_active) { Write-Host '车端已有控制会话；脚本没有接管或发送行驶指令。' }
    else { Write-Host '车辆控制未启用，请按网页流程自行准备和开始。' }
    if (-not $NoOpen -and -not $CheckOnly -and -not $carReport.control_active) { Start-Process $carLocalUrl }
    $carExit = 0
} catch {
    $carReport.error = $_.Exception.Message
    Write-Host ('连接未完成：'+$carReport.error) -ForegroundColor Yellow
} finally {
    if ($carMutexHeld) { $carMutex.ReleaseMutex() }
    $carMutex.Dispose()
    [IO.File]::WriteAllText($carLogPath, ($carReport | ConvertTo-Json -Depth 5), (New-Object Text.UTF8Encoding($false)))
    Write-Host ('连接记录：'+$carLogPath)
}
exit $carExit
