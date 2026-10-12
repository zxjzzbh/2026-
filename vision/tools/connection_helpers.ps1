function Get-CarProperty($Object, [string]$Name) {
    if ($null -eq $Object) { return $null }
    $property = $Object.PSObject.Properties[$Name]
    if ($null -ne $property) { return $property.Value }
    return $null
}

function Test-CarControlActive($Status) {
    # Unknown status is not permission to interrupt a control connection.
    if ((Get-CarProperty $Status 'mode') -ne 'disabled') { return $true }
    foreach ($name in @('hardware_output', 'worker_started', 'pending_hardware_start')) {
        if (Get-CarProperty $Status $name) { return $true }
    }
    foreach ($name in @('traffic_trial', 'crosswalk_trial')) {
        if (Get-CarProperty (Get-CarProperty $Status $name) 'active') { return $true }
    }
    return $false
}

function Test-OwnedCarForward($Listener, $Process, [string]$Entry) {
    if ($null -eq $Listener -or $null -eq $Process) { return $false }
    if ($Listener.LocalAddress -ne '127.0.0.1' -or $Process.Name -notin @('python.exe', 'pythonw.exe')) { return $false }
    if ($Process.ProcessId -ne $Listener.OwningProcess -or -not $Process.CommandLine) { return $false }
    # Match one complete quoted script argument, not a similarly named file.
    return $Process.CommandLine.IndexOf(('"' + $Entry + '"'), [StringComparison]::OrdinalIgnoreCase) -ge 0
}

function Get-CarHttpStatus([string]$Url, [int]$TimeoutMs = 3000) {
    $failure = $null
    for ($httpAttempt = 0; $httpAttempt -lt 2; $httpAttempt++) {
    $response = $null; $reader = $null
    try {
        $request = [Net.HttpWebRequest]::Create($Url)
        $request.Method = 'GET'
        $request.Proxy = $null
        $request.AllowAutoRedirect = $false
        $request.KeepAlive = $false
        $request.Timeout = $TimeoutMs
        $request.ReadWriteTimeout = $TimeoutMs
        $response = $request.GetResponse()
        $reader = New-Object IO.StreamReader($response.GetResponseStream())
        $data = $reader.ReadToEnd() | ConvertFrom-Json
        if (-not (Get-CarProperty $data 'mode')) { throw '返回内容不是车辆控制状态。' }
        return [pscustomobject]@{ Ok = $true; Data = $data; Error = $null }
    } catch {
        $failure = [pscustomobject]@{ Ok = $false; Data = $null; Error = $_.Exception.Message }
    } finally {
        if ($reader) { $reader.Dispose() }
        if ($response) { $response.Close() }
    }
    if ($httpAttempt -eq 0) { Start-Sleep -Milliseconds 200 }
    }
    return $failure
}

function Test-CarTcp([string]$Address, [int]$Port, [int]$TimeoutMs = 2500) {
    $client = New-Object Net.Sockets.TcpClient
    $wait = $null
    try {
        $pending = $client.BeginConnect($Address, $Port, $null, $null)
        $wait = $pending.AsyncWaitHandle
        if (-not $wait.WaitOne($TimeoutMs)) { return $false }
        $client.EndConnect($pending)
        return $true
    } catch { return $false }
    finally { $client.Close(); if ($wait) { $wait.Close() } }
}

function Invoke-CarTailscale([string]$Exe, [string[]]$TailArguments, [int]$TimeoutMs = 8000) {
    # Arguments are fixed CLI tokens / the fixed car IP, never a shell command.
    $info = New-Object Diagnostics.ProcessStartInfo
    $info.FileName = $Exe
    $info.Arguments = $TailArguments -join ' '
    $info.UseShellExecute = $false
    $info.CreateNoWindow = $true
    $info.RedirectStandardOutput = $true
    $info.RedirectStandardError = $true
    $process = New-Object Diagnostics.Process
    $process.StartInfo = $info
    try {
        [void]$process.Start()
        $outTask = $process.StandardOutput.ReadToEndAsync()
        $errTask = $process.StandardError.ReadToEndAsync()
        if (-not $process.WaitForExit($TimeoutMs)) {
            $process.Kill() # Only this bounded CLI child; never tailscaled/service.
            [void]$process.WaitForExit(1500)
            return [pscustomobject]@{ ExitCode = -1; Output = ''; TimedOut = $true }
        }
        # Consume stderr without exposing login URLs, accounts or peer details.
        [void]$errTask.Result
        return [pscustomobject]@{ ExitCode = $process.ExitCode; Output = $outTask.Result; TimedOut = $false }
    } finally { $process.Dispose() }
}
