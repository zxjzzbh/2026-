@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0run-crosswalk.ps1" %*
pause
