@echo off
REM Git setup script for Baxter Station

echo 📦 Initializing Git repository for Baxter Station...

git init
if errorlevel 1 (
    echo ❌ Git init failed. Make sure Git is installed.
    pause
    exit /b 1
)

git add .
if errorlevel 1 (
    echo ❌ Git add failed.
    pause
    exit /b 1
)

git commit -m "Initial commit: Windows-ready Baxter Station

- Complete Docker environment for Baxter robot
- Windows-compatible batch scripts for setup
- ROS1 Indigo + ROS2 Humble + Bridge architecture
- Cross-platform docker-compose configuration
- Comprehensive Windows documentation
- Ready-to-use example scripts

This version is optimized for Windows deployment with:
- No X11 dependencies (Windows compatible)
- Batch scripts for easy setup and testing
- Clear documentation for Windows users
- Proper Docker Desktop integration"

if errorlevel 1 (
    echo ❌ Git commit failed.
    pause
    exit /b 1
)

echo ✅ Git repository initialized successfully!
echo.
echo 📤 To push to GitHub:
echo 1. Create a new repository on GitHub: https://github.com/Janga786/Baxter_Station
echo 2. Run: git remote add origin https://github.com/Janga786/Baxter_Station.git
echo 3. Run: git push -u origin main
echo.
pause