@echo off
call conda activate yolo_world
labelImg "%~dp0dataset\images\all" "%~dp0dataset\classes.txt" "%~dp0dataset\labels\all"
