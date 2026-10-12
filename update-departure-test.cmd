@echo off
setlocal
cd /d "%~dp0"
powershell.exe -NoLogo -NoProfile -ExecutionPolicy RemoteSigned -File "%~dp0vision\connect-car.ps1" -NoOpen
if errorlevel 1 goto failed
"%~dp0.venv\Scripts\python.exe" -u "%~dp0vision\tools\deploy_departure_trial.py"
if errorlevel 1 goto failed
echo Departure speech-only test installed. No movement or automatic test start.
start "" "http://127.0.0.1:8082/"
pause
exit /b 0
:failed
echo Update incomplete. Read the message above and retry after the car is connected and driving tasks are stopped.
pause
exit /b 1
