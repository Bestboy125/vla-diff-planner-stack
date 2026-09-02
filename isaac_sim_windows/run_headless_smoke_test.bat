@echo off
setlocal

set "ISAAC_SIM_RELEASE=D:\IsaacSim\_build\windows-x86_64\release"
set "SMOKE_TEST=E:\embodied_agent\vla_planner_project\isaac_sim_windows\smoke_test.py"
set "PYTHONUTF8=1"
set "PYTHONIOENCODING=utf-8"

if not exist "%ISAAC_SIM_RELEASE%\python.bat" (
    echo ERROR: Isaac Sim Python launcher was not found.
    exit /b 1
)

if not exist "%SMOKE_TEST%" (
    echo ERROR: Smoke test script was not found.
    exit /b 1
)

call "%ISAAC_SIM_RELEASE%\python.bat" "%SMOKE_TEST%"
exit /b %ERRORLEVEL%
