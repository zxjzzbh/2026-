@echo off
setlocal
cd /d "%~dp0"
powershell.exe -NoLogo -NoProfile -ExecutionPolicy RemoteSigned -File "%~dp0vision\connect-car.ps1" -NoOpen
if errorlevel 1 goto failed
"%~dp0.venv\Scripts\python.exe" -u "%~dp0vision\tools\deploy_parking_bay.py"
if errorlevel 1 goto failed
echo Parking observation installed. No automatic motor output or payment.
start "" "http://127.0.0.1:8082/"
pause
exit /b 0
:failed
echo Update incomplete. Reconnect the car and end existing driving/payment sessions, then retry.
pause
exit /b 1
