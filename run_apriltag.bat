@echo off
setlocal
if exist "%~dp0.venv\Scripts\python.exe" (
    "%~dp0.venv\Scripts\python.exe" "%~dp0read_apriltag.py" %*
) else (
    py -3 "%~dp0read_apriltag.py" %*
)
if errorlevel 1 pause
