@echo off
setlocal
if not exist "%~dp0.venv\Scripts\python.exe" (
  echo Project Python environment was not found. See vision\CONE_TEST.md.
  pause
  exit /b 1
)
"%~dp0.venv\Scripts\python.exe" -u "%~dp0vision\tools\cone_trial.py" live --preview http://127.0.0.1:8082
pause
