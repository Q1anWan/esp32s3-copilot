@echo off
setlocal
cd /d "%~dp0\..\.."
echo Starting logic packet probe on 0.0.0.0:7777 ...
echo Close the Bridge GUI TCP Host first, because only one program can listen on port 7777.
py -3 tools\silab_tcp_probe.py --host 0.0.0.0 --port 7777 --jsonl logic_probe_log.jsonl
echo.
echo Probe exited.
pause
