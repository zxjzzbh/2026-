$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    $visionPython = Join-Path $PSScriptRoot '.venv/Scripts/python.exe'
    if (-not (Test-Path -LiteralPath $visionPython)) {
        throw 'Install the vision environment following README.md first.'
    }
    $demoStamp = Get-Date -Format 'yyyyMMdd-HHmmss-fff'
    $demoInput = "outputs/demo-$demoStamp-input"
    $demoOutput = "outputs/demo-$demoStamp-result"
    & $visionPython -m carvision demo --output $demoInput
    if ($LASTEXITCODE -ne 0) { throw 'Demo generation failed.' }
    & $visionPython -m carvision replay --source "$demoInput/synthetic-lane.avi" --output $demoOutput --save-video
    if ($LASTEXITCODE -ne 0) { throw 'Replay failed.' }
    Write-Host "Synthetic software check complete: $demoOutput/overlay.avi"
} finally {
    Pop-Location
}
