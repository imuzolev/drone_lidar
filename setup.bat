@echo off
echo ============================================
echo   AADNS - Drone Project Setup
echo ============================================
echo.

:: Check Python
python --version 2>nul
if errorlevel 1 (
    echo [ERROR] Python not found! Install Python 3.11+ from python.org
    pause
    exit /b 1
)

:: Create virtual environment
echo [1/4] Creating virtual environment...
if not exist venv (
    python -m venv venv
    echo       Created!
) else (
    echo       Already exists.
)

:: Activate venv
echo [2/4] Activating virtual environment...
call venv\Scripts\activate.bat

:: Install requirements
echo [3/4] Installing Python packages...
pip install -r requirements.txt

:: Install Colosseum AirSim client
echo [4/4] Installing Colosseum AirSim client...
pip uninstall airsim -y 2>nul
pip install git+https://github.com/CodexLabsLLC/Colosseum.git#subdirectory=PythonClient

echo.
echo ============================================
echo   Setup complete!
echo ============================================
echo.
echo Next steps:
echo   1. Setup Unreal Engine 5.2 with Colosseum plugin
echo   2. Copy AirSim settings: see README.md
echo   3. Run: python src\main.py
echo.
pause
