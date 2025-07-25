@echo off
REM Baxter Station Setup Script for Windows
REM This script sets up the Baxter environment and starts the Docker containers

echo 🤖 Baxter Station Setup Script for Windows
echo ==========================================

REM Check if Docker is installed
docker --version >nul 2>&1
if errorlevel 1 (
    echo ❌ Docker is not installed. Please install Docker Desktop from https://docker.com
    echo    Make sure to enable Linux containers and restart your computer after installation.
    pause
    exit /b 1
)

REM Check if Docker Compose is installed
docker-compose --version >nul 2>&1
if errorlevel 1 (
    echo ❌ Docker Compose is not installed or not in PATH.
    echo    Docker Desktop usually includes Docker Compose. Try restarting your terminal.
    pause
    exit /b 1
)

REM Check if Docker is running
docker ps >nul 2>&1
if errorlevel 1 (
    echo ❌ Docker is not running. Please start Docker Desktop first.
    pause
    exit /b 1
)

REM Set Baxter IP (you may need to modify this)
set BAXTER_IP=192.168.42.2
echo 🔧 Using Baxter IP: %BAXTER_IP%
echo    (If your Baxter has a different IP, edit docker-compose.yml)

REM Build and start Docker containers
echo 🏗️  Building Docker containers (this may take 15-20 minutes on first run)...
docker-compose build
if errorlevel 1 (
    echo ❌ Failed to build containers. Check Docker Desktop is running with Linux containers enabled.
    pause
    exit /b 1
)

echo 🚀 Starting Baxter containers...
docker-compose up -d
if errorlevel 1 (
    echo ❌ Failed to start containers.
    pause
    exit /b 1
)

REM Wait for containers to be ready
echo ⏱️  Waiting for containers to start...
timeout /t 15 /nobreak >nul

REM Check container status
echo 📊 Container Status:
docker-compose ps

echo.
echo ✅ Baxter Station setup complete!
echo.
echo 🎮 Quick Start Commands:
echo ========================
echo.
echo 1. Enable robot (REQUIRED before any movement):
echo    docker exec -it baxter_ros1_container bash -c "source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && rosrun baxter_tools enable_robot.py -e"
echo.
echo 2. Test robot movement:
echo    docker exec -it baxter_ros1_container bash -c "source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && rosrun baxter_examples joint_velocity_wobbler.py"
echo.
echo 3. Access container shell:
echo    docker exec -it baxter_ros1_container bash
echo.
echo 4. Disable robot:
echo    docker exec -it baxter_ros1_container bash -c "source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && rosrun baxter_tools enable_robot.py -d"
echo.
echo 5. Start MoveIt planning:
echo    docker exec -it moveit_ros2_container bash
echo.
echo 📚 For more commands, check the documentation folder
echo.
echo 🛑 To stop everything:
echo    docker-compose down
echo.
pause