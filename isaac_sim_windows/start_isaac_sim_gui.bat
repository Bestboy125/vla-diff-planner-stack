@echo off
setlocal

set "ISAAC_SIM_RELEASE=D:\IsaacSim-5.1\_build\windows-x86_64\release"
set "ISAAC_SIM_CACHE_ROOT=D:\IsaacSimCache"
set "ISAAC_SIM_TEMP=%ISAAC_SIM_CACHE_ROOT%\temp\isaac-sim-gui"
set "WARP_CACHE_PATH=%ISAAC_SIM_CACHE_ROOT%\warp"
set "AIRSTACK_DISTRO=AirStack-22.04"
set "AIRSTACK_EXTENSION_ROOT=D:\AirStackWSL\PegasusSimulator\extensions"

if not exist "%ISAAC_SIM_RELEASE%\isaac-sim.bat" (
    echo ERROR: Isaac Sim launcher was not found at "%ISAAC_SIM_RELEASE%\isaac-sim.bat".
    exit /b 1
)

if not exist "%ISAAC_SIM_TEMP%" mkdir "%ISAAC_SIM_TEMP%"
if errorlevel 1 (
    echo ERROR: Could not create Isaac Sim temporary directory "%ISAAC_SIM_TEMP%".
    exit /b 1
)
if not exist "%WARP_CACHE_PATH%" mkdir "%WARP_CACHE_PATH%"
if errorlevel 1 (
    echo ERROR: Could not create Warp cache directory "%WARP_CACHE_PATH%".
    exit /b 1
)

rem Keep extension archive downloads and Python temporary files off the C drive.
set "TEMP=%ISAAC_SIM_TEMP%"
set "TMP=%ISAAC_SIM_TEMP%"

rem AirStack is built against Humble. Select Isaac Sim's bundled Humble bridge
rem explicitly and use a discovery server across WSL NAT.
wsl.exe -d %AIRSTACK_DISTRO% -u root -- systemctl start airstack-fastdds-discovery.service
if errorlevel 1 (
    echo ERROR: Could not start the AirStack Fast DDS discovery server.
    exit /b 1
)

for /f "tokens=1" %%I in ('wsl.exe -d %AIRSTACK_DISTRO% -- hostname -I') do set "AIRSTACK_WSL_IP=%%I"
if not defined AIRSTACK_WSL_IP (
    echo ERROR: Could not determine the AirStack WSL address.
    exit /b 1
)

set "ROS_DISTRO=humble"
set "ROS_DOMAIN_ID=42"
set "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
set "ROS_DISCOVERY_SERVER=%AIRSTACK_WSL_IP%:11811"
set "PATH=%PATH%;%ISAAC_SIM_RELEASE%\exts\isaacsim.ros2.bridge\humble\lib"

if not exist "%AIRSTACK_EXTENSION_ROOT%\pegasus.simulator\config\extension.toml" (
    echo ERROR: Pegasus extension was not found under "%AIRSTACK_EXTENSION_ROOT%".
    exit /b 1
)

echo ROS discovery server: %ROS_DISCOVERY_SERVER%
call "%ISAAC_SIM_RELEASE%\isaac-sim.bat" --ext-folder "%AIRSTACK_EXTENSION_ROOT%" --enable pegasus.simulator %*
exit /b %ERRORLEVEL%
