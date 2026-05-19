@echo off
setlocal
set MOSQUITTO_EXE=C:\Program Files\mosquitto\mosquitto.exe
if not exist "%MOSQUITTO_EXE%" (
  echo Cannot find "%MOSQUITTO_EXE%".
  echo Install Eclipse Mosquitto first, or edit this bat file.
  pause
  exit /b 1
)
cd /d "%~dp0"
echo Starting Mosquitto in this console with mosquitto-copilot.conf ...
echo For formal tests, prefer the Windows service method in the deployment document.
"%MOSQUITTO_EXE%" -c "%~dp0mosquitto-copilot.conf" -v
pause
