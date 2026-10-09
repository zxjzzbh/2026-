param([string]$Source, [string]$OutputDirectory, [switch]$NoWindow)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
if (-not $Source) {
    $Source = Read-Host 'Image/video path, or explicitly selected camera (e.g. camera:0)'
    $Source = $Source.Trim().Trim('"')
}
if (-not $Source) { throw 'An actual image/video/camera source is required.' }
if (-not $OutputDirectory) {
    $OutputDirectory = Join-Path $projectRoot ('run\crosswalk-' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff'))
}
$entry = Join-Path $PSScriptRoot 'tools\crosswalk_lab.py'
$projectPython = Join-Path $projectRoot '.venv\Scripts\python.exe'
$entryArgs = @($entry, 'replay', '--source', $Source, '--output', $OutputDirectory)
if (-not $NoWindow) { $entryArgs += '--show' }
if (Test-Path -LiteralPath $projectPython) {
    & $projectPython @entryArgs
} else {
    & py -3.12 @entryArgs
}
if ($LASTEXITCODE -ne 0) { throw 'Crosswalk processing failed. Read the Python error above.' }
Write-Output ('Logs and detection images: ' + $OutputDirectory)
