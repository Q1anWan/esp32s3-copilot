@echo off
setlocal
cd /d "%~dp0\..\.."
echo Starting Copilot ESP32 Bridge GUI...
echo Do not use the GUI "Start Dev Broker" button for field tests.
py -3 tools\silab_mqtt_bridge_gui.py
echo.
echo Bridge GUI exited.
pause
