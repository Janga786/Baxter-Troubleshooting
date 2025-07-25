@echo off
REM Stop all Baxter Station containers

echo 🛑 Stopping Baxter Station containers...

docker-compose down

if errorlevel 1 (
    echo ❌ Error stopping containers
    pause
    exit /b 1
)

echo ✅ All containers stopped successfully
echo.
echo To restart: run setup.bat
echo.
pause