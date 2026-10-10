@echo off
setlocal
cd /d "%~dp0"
powershell.exe -NoLogo -NoProfile -ExecutionPolicy RemoteSigned -File "%~dp0vision\connect-car.ps1" %*
set "car_result=%errorlevel%"
if not "%car_result%"=="0" (
  echo Connection incomplete. Read the message above, then retry.
  pause
  exit /b %car_result%
)
echo Connected. This window will close shortly.
timeout /t 4 /nobreak >nul
exit /b 0
