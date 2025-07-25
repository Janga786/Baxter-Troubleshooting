@echo off
REM Start MoveIt planning interface for Baxter

echo 🦾 Starting Baxter MoveIt Planning Interface...

REM Check if containers are running
docker-compose ps -q >nul 2>&1
if errorlevel 1 (
    echo ❌ Containers are not running. Please run setup.bat first.
    pause
    exit /b 1
)

echo 🔧 Enabling robot first...
start /b docker exec baxter_ros1_container bash -c "source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && rosrun baxter_tools enable_robot.py -e"

echo.
echo 🚀 Starting MoveIt ROS2 container...
echo Note: This will open an interactive shell in the MoveIt container
echo You can run motion planning commands from there.
echo.
echo Available commands in the container:
echo - ros2 launch baxter_moveit_config demo.launch.py
echo - ros2 run moveit_demos simple_demo
echo.
docker exec -it moveit_ros2_container bash -c "cd /shared_ws && source /opt/ros/humble/setup.bash && bash"

pause