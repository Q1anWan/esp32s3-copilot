@echo off
setlocal
cd /d "%~dp0\..\.."
echo Installing Python packages for Copilot SILAB tools...
py -3 -m pip install --upgrade pip
py -3 -m pip install paho-mqtt pyserial
echo.
echo Done. If there were no red error lines, dependencies are installed.
pause
