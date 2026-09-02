@echo off
cd /d "%~dp0.."
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0start_vla_full_preview.ps1" -Policy OpenVLA
if errorlevel 1 pause
