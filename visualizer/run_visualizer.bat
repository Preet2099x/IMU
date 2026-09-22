@echo off
rem Starts the 3D visualizer and opens it in your browser. Any arguments are passed
rem on, for example:  run_visualizer.bat --demo   or   run_visualizer.bat --port COM7
cd /d "%~dp0"
python server.py %*
pause
