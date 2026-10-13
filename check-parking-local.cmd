@echo off
setlocal
cd /d "%~dp0"
if not exist "%~dp0.venv\Scripts\python.exe" (
  echo Project Python environment not found.
  pause
  exit /b 1
)
"%~dp0.venv\Scripts\python.exe" -m pytest vision/tests/test_parking_bay.py -q
set "parking_result=%errorlevel%"
echo Local software checks only. No car connection, deployment, PWM or payment.
pause
exit /b %parking_result%
