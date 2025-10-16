# Restart Windows Camera for zkML Demo
# This script stops any running camera process and starts camera ID 1

Write-Host "======================================" -ForegroundColor Cyan
Write-Host "Restarting Windows Camera (ID 1)" -ForegroundColor Cyan
Write-Host "======================================" -ForegroundColor Cyan
Write-Host ""

# Step 1: Kill any existing camera processes
Write-Host "[1/3] Stopping existing camera processes..." -ForegroundColor Yellow
$pythonProcesses = Get-Process python* -ErrorAction SilentlyContinue | Where-Object {
    $_.CommandLine -like "*simple_windows_camera*"
}

if ($pythonProcesses) {
    $pythonProcesses | ForEach-Object {
        Write-Host "  Stopping process $($_.Id)..." -ForegroundColor Gray
        Stop-Process -Id $_.Id -Force -ErrorAction SilentlyContinue
    }
    Write-Host "  ✓ Stopped existing processes" -ForegroundColor Green
} else {
    Write-Host "  No existing camera processes found" -ForegroundColor Gray
}

Start-Sleep -Seconds 1

# Step 2: Navigate to robotics directory
Write-Host ""
Write-Host "[2/3] Navigating to robotics directory..." -ForegroundColor Yellow

# Try to find robotics directory (common locations in WSL)
$possiblePaths = @(
    "\\wsl$\Ubuntu\home\hshadab\robotics",
    "\\wsl.localhost\Ubuntu\home\hshadab\robotics",
    "$env:USERPROFILE\robotics"
)

$roboticsPath = $null
foreach ($path in $possiblePaths) {
    if (Test-Path $path) {
        $roboticsPath = $path
        break
    }
}

if (-not $roboticsPath) {
    Write-Host "  ERROR: Could not find robotics directory" -ForegroundColor Red
    Write-Host "  Please navigate to the robotics directory manually and run:" -ForegroundColor Yellow
    Write-Host "    python simple_windows_camera.py 1" -ForegroundColor White
    Write-Host ""
    pause
    exit 1
}

Write-Host "  ✓ Found: $roboticsPath" -ForegroundColor Green
Set-Location $roboticsPath

# Step 3: Start camera with ID 1
Write-Host ""
Write-Host "[3/3] Starting camera ID 1..." -ForegroundColor Yellow
Write-Host ""
Write-Host "="*50 -ForegroundColor Green
Write-Host "Starting Windows Camera Server (Camera ID 1)" -ForegroundColor Green
Write-Host "="*50 -ForegroundColor Green
Write-Host ""
Write-Host "Your webcam light should turn ON in a moment." -ForegroundColor Cyan
Write-Host "Keep this window open." -ForegroundColor Cyan
Write-Host "Press Ctrl+C to stop the camera." -ForegroundColor Cyan
Write-Host ""

# Start the camera
python simple_windows_camera.py 1
