@echo off
call conda activate yolo_world
python "%~dp0split_dataset.py"
if errorlevel 1 pause & exit /b 1
python "%~dp0train.py"
pause
