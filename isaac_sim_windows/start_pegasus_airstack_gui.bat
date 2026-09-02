@echo off
setlocal

set "ISAAC_SIM_RELEASE=D:\IsaacSim-5.1\_build\windows-x86_64\release"
set "ISAAC_SIM_CACHE_ROOT=D:\IsaacSimCache"
set "ISAAC_SIM_TEMP=%ISAAC_SIM_CACHE_ROOT%\temp\pegasus-airstack"
set "WARP_CACHE_PATH=%ISAAC_SIM_CACHE_ROOT%\warp"
set "PEGASUS_EXTENSIONS=D:\AirStackWSL\PegasusSimulator\extensions"
set "AIRSTACK_DISTRO=AirStack-22.04"
set "AIRSTACK_SCRIPT=E:\embodied_agent\vla_planner_project\isaac_sim_windows\run_pegasus_airstack.py"
set "AIRSTACK_SCENE_USD=D:\AirStackWSL\scenes\RetroNeighborhood\RetroNeighborhood_Export.usd"

if not exist "%ISAAC_SIM_RELEASE%\python.bat" exit /b 10
if not exist "%PEGASUS_EXTENSIONS%\pegasus.simulator\config\extension.toml" exit /b 11
if not exist "%AIRSTACK_SCRIPT%" exit /b 12
if not exist "%AIRSTACK_SCENE_USD%" exit /b 13

if not exist "%ISAAC_SIM_TEMP%" mkdir "%ISAAC_SIM_TEMP%"
if not exist "%WARP_CACHE_PATH%" mkdir "%WARP_CACHE_PATH%"
set "TEMP=%ISAAC_SIM_TEMP%"
set "TMP=%ISAAC_SIM_TEMP%"

wsl.exe -d %AIRSTACK_DISTRO% -u root -- systemctl start airstack-fastdds-discovery.service
if errorlevel 1 exit /b 20
for /f "tokens=1" %%I in ('wsl.exe -d %AIRSTACK_DISTRO% -- hostname -I') do set "AIRSTACK_WSL_IP=%%I"
if not defined AIRSTACK_WSL_IP exit /b 21

set "ROS_DISTRO=humble"
set "ROS_DOMAIN_ID=42"
set "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
set "ROS_DISCOVERY_SERVER=%AIRSTACK_WSL_IP%:11811"
set "PEGASUS_MAVLINK_BIND=0.0.0.0"
set "PEGASUS_MAVLINK_PORT=4560"
set "AIRSTACK_BENCHMARK_SCENE=0"
set "AIRSTACK_ADD_TEST_OBSTACLE=0"
set "AIRSTACK_SPAWN_X=8.38"
set "AIRSTACK_SPAWN_Y=-15.10"
set "AIRSTACK_SPAWN_Z=0.07"
set "AIRSTACK_POLE_X=12.027"
set "AIRSTACK_POLE_Y=-13.463"
set "AIRSTACK_POLE_CENTER_Z=4.0"
set "AIRSTACK_ADD_POLE_PROXY=1"
set "PATH=%PATH%;%ISAAC_SIM_RELEASE%\exts\isaacsim.ros2.bridge\humble\lib"

echo [AIRSTACK] ROS discovery server: %ROS_DISCOVERY_SERVER%
echo [AIRSTACK] Scene: %AIRSTACK_SCENE_USD%
call "%ISAAC_SIM_RELEASE%\python.bat" "%AIRSTACK_SCRIPT%" --ext-folder "%PEGASUS_EXTENSIONS%"
exit /b %ERRORLEVEL%
