@echo off
setlocal
cd /d "%~dp0\..\.."
echo Starting SILAB packet probe on 0.0.0.0:7777 ...
echo Close the Bridge GUI TCP Host first, because only one program can listen on port 7777.
py -3 tools\silab_tcp_probe.py --host 0.0.0.0 --port 7777 --scene-aliases 1=boot --jsonl silab_probe_log.jsonl
echo.
echo Probe exited.
pause
