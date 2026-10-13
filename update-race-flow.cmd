@echo off
chcp 65001 >nul
cd /d "%~dp0"
set "PYTHONPATH=%~dp0vision\src"
set "PYTHONIOENCODING=utf-8"
set "RACE_FLOW_PY=python"
if exist "..\2026-current-sync\.venv\Scripts\python.exe" set "RACE_FLOW_PY=%~dp0..\2026-current-sync\.venv\Scripts\python.exe"
if exist ".venv\Scripts\python.exe" set "RACE_FLOW_PY=%~dp0.venv\Scripts\python.exe"
echo 本入口会连接小车并更新原控制服务。请先结束所有驾驶会话、关闭电调。
echo 更新成功后仍需你在切换区手动进入自动模式，不会自动发车。
"%RACE_FLOW_PY%" vision\tools\deploy_race_flow.py --apply
pause
