@echo off
setlocal
cd /d "%~dp0"
if exist "%~dp0.venv\Scripts\python.exe" (
    "%~dp0.venv\Scripts\python.exe" "%~dp0topview_controller.py" %*
) else (
    py -3 "%~dp0topview_controller.py" %*
)
if errorlevel 1 pause
