$ErrorActionPreference = 'Stop'
. (Join-Path (Split-Path -Parent $PSScriptRoot) 'tools/connection_helpers.ps1')
function Assert-Car($Condition, [string]$Message) { if (-not $Condition) { throw $Message } }
$idle = [pscustomobject]@{mode='disabled';hardware_output=$false;worker_started=$false;traffic_trial=[pscustomobject]@{active=$false};crosswalk_trial=[pscustomobject]@{active=$false}}
Assert-Car (-not (Test-CarControlActive $idle)) 'Idle control should be recognized.'
Assert-Car (Test-CarControlActive $null) 'Unknown control cannot authorize a restart.'
$idle.traffic_trial.active=$true
Assert-Car (Test-CarControlActive $idle) 'Independent traffic actuator must block relay restart.'
$idle.traffic_trial.active=$false;$idle.crosswalk_trial.active=$true
Assert-Car (Test-CarControlActive $idle) 'Crosswalk actuator must block relay restart.'
$idle.crosswalk_trial.active=$false;$idle.hardware_output=$true
Assert-Car (Test-CarControlActive $idle) 'Manual output must block relay restart.'
$listener=[pscustomobject]@{LocalAddress='127.0.0.1';OwningProcess=123}
$entry='D:\test project\vision\tools\console_forward.py'
$owner=[pscustomobject]@{Name='python.exe';ProcessId=123;CommandLine='"C:\Python\python.exe" "'+$entry+'" --port 8082'}
Assert-Car (Test-OwnedCarForward $listener $owner $entry) 'Exact local Python relay should be recognized.'
$owner.CommandLine='"C:\Python\python.exe" "'+$entry+'.backup" --port 8082'
Assert-Car (-not (Test-OwnedCarForward $listener $owner $entry)) 'Similarly named scripts must not be killed.'
$owner.CommandLine='"C:\Python\python.exe" "'+$entry+'" --port 8082';$owner.Name='pwsh.exe'
Assert-Car (-not (Test-OwnedCarForward $listener $owner $entry)) 'A shell mentioning the script is not its owner.'
$owner.Name='python.exe';$listener.LocalAddress='0.0.0.0'
Assert-Car (-not (Test-OwnedCarForward $listener $owner $entry)) 'Non-loopback listener must not be killed.'
$listener.LocalAddress='127.0.0.1';$owner.ProcessId=124
Assert-Car (-not (Test-OwnedCarForward $listener $owner $entry)) 'PID must match the actual listener.'
Write-Output '10 connection safety checks passed; no network or process changes.'
