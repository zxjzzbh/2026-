@echo off
chcp 65001 >nul
cd /d "%~dp0"
set "PYTHONPATH=%~dp0vision\src"
set "PYTHONIOENCODING=utf-8"
set "RACE_FLOW_PY=python"
if exist "..\2026-current-sync\.venv\Scripts\python.exe" set "RACE_FLOW_PY=%~dp0..\2026-current-sync\.venv\Scripts\python.exe"
if exist ".venv\Scripts\python.exe" (
  ".venv\Scripts\python.exe" vision\tools\race_flow.py check
  ".venv\Scripts\python.exe" -m pytest vision\tests\test_race_flow.py -q
) else (
  "%RACE_FLOW_PY%" vision\tools\race_flow.py check
  "%RACE_FLOW_PY%" -m pytest vision\tests\test_race_flow.py -q
)
pause
