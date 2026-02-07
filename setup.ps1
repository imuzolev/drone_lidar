# ============================================
#   AADNS - Drone Project Setup (PowerShell)
# ============================================

Write-Host "============================================" -ForegroundColor Cyan
Write-Host "  AADNS - Drone Project Setup" -ForegroundColor Cyan
Write-Host "============================================" -ForegroundColor Cyan
Write-Host ""

# Check Python
try {
    $pyVersion = python --version 2>&1
    Write-Host "[OK] $pyVersion" -ForegroundColor Green
} catch {
    Write-Host "[ERROR] Python not found! Install Python 3.11+ from python.org" -ForegroundColor Red
    exit 1
}

# Create virtual environment
Write-Host "[1/4] Creating virtual environment..." -ForegroundColor Yellow
if (-not (Test-Path "venv")) {
    python -m venv venv
    Write-Host "       Created!" -ForegroundColor Green
} else {
    Write-Host "       Already exists." -ForegroundColor Green
}

# Activate venv
Write-Host "[2/4] Activating virtual environment..." -ForegroundColor Yellow
& .\venv\Scripts\Activate.ps1

# Install requirements
Write-Host "[3/4] Installing Python packages..." -ForegroundColor Yellow
pip install -r requirements.txt

# Install Colosseum AirSim client
Write-Host "[4/4] Installing Colosseum AirSim client..." -ForegroundColor Yellow
pip uninstall airsim -y 2>$null
pip install git+https://github.com/CodexLabsLLC/Colosseum.git#subdirectory=PythonClient

Write-Host ""
Write-Host "============================================" -ForegroundColor Green
Write-Host "  Setup complete!" -ForegroundColor Green
Write-Host "============================================" -ForegroundColor Green
Write-Host ""
Write-Host "Next steps:" -ForegroundColor Cyan
Write-Host "  1. Setup Unreal Engine 5.2 with Colosseum plugin"
Write-Host "  2. Copy AirSim settings: see README.md"
Write-Host "  3. Run: python src\main.py"
