@echo off
rem Starts the 3D IMU viewer and opens it in your browser. Any arguments are passed
rem on, for example:  run_viewer.bat --demo   or   run_viewer.bat --port COM7
cd /d "%~dp0"
python server.py %*
pause
