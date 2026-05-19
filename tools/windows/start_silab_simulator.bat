@echo off
setlocal
cd /d "%~dp0\..\.."
set BRIDGE_IP=127.0.0.1
set /p BRIDGE_IP=Bridge PC IP [default 127.0.0.1]: 
if "%BRIDGE_IP%"=="" set BRIDGE_IP=127.0.0.1
set BRIDGE_PORT=7777
set /p BRIDGE_PORT=Bridge TCP port [default 7777]: 
if "%BRIDGE_PORT%"=="" set BRIDGE_PORT=7777
echo.
echo Sending one SILAB-like trigger pulse to %BRIDGE_IP%:%BRIDGE_PORT% ...
py -3 tools\silab_tcp_simulator.py --host %BRIDGE_IP% --port %BRIDGE_PORT% --scene 1 --seq 1 --rate-hz 10 --pre-idle 1 --hold 2 --post-idle 1 --read-ack --verbose
echo.
echo Simulator finished.
pause
