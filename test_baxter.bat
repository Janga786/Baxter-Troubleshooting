@echo off
REM Quick test script to verify Baxter Robot is working

echo 🤖 Testing Baxter Robot Connection...

REM Check if containers are running
docker-compose ps -q >nul 2>&1
if errorlevel 1 (
    echo ❌ Containers are not running. Please run setup.bat first.
    pause
    exit /b 1
)

echo 1. Enabling robot...
docker exec -it baxter_ros1_container bash -c "source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && rosrun baxter_tools enable_robot.py -e"

if errorlevel 1 (
    echo ❌ Failed to enable robot. Check if Baxter is connected and powered on.
    pause
    exit /b 1
)

echo 2. Running simple movement test...
docker exec -it baxter_ros1_container bash -c "source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && timeout 10 rosrun baxter_examples joint_velocity_wobbler.py || echo 'Test completed'"

echo.
echo ✅ Test complete! If you saw arm movement, Baxter is working properly.
echo.
echo 📋 Next Steps:
echo - Use start_moveit.bat to launch MoveIt planning
echo - Check documentation folder for more examples
echo - Use docker exec -it baxter_ros1_container bash for direct access
echo.
pause