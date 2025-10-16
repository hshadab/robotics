# Start Windows Camera ID 1 for zkML Demo
# PowerShell script that works with UNC paths

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "Windows Camera Server (ID 1)" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

# Get the current directory (works with UNC paths in PowerShell)
$scriptDir = $PSScriptRoot
if (-not $scriptDir) {
    $scriptDir = Get-Location
}

Write-Host "Working directory: $scriptDir" -ForegroundColor Gray
Write-Host ""

# Stop existing camera processes
Write-Host "Stopping existing camera processes..." -ForegroundColor Yellow
Get-Process python* -ErrorAction SilentlyContinue | Where-Object {
    $_.Path -and (Get-Process -Id $_.Id | Select-Object -ExpandProperty CommandLine -ErrorAction SilentlyContinue) -match "simple_windows_camera"
} | Stop-Process -Force -ErrorAction SilentlyContinue

Start-Sleep -Seconds 1

# Start camera
Write-Host "Starting camera ID 1..." -ForegroundColor Green
Write-Host "Your webcam light should turn ON now." -ForegroundColor Cyan
Write-Host "Keep this window open." -ForegroundColor Cyan
Write-Host "Press Ctrl+C to stop." -ForegroundColor Cyan
Write-Host ""

# Check if Python script exists
$pythonScript = Join-Path $scriptDir "simple_windows_camera.py"
if (-not (Test-Path $pythonScript)) {
    Write-Host "ERROR: Cannot find simple_windows_camera.py" -ForegroundColor Red
    Write-Host "Expected location: $pythonScript" -ForegroundColor Red
    Write-Host ""
    Write-Host "Please make sure you're running this from the robotics directory" -ForegroundColor Yellow
    pause
    exit 1
}

# Run the Python script
try {
    python $pythonScript 1
} catch {
    Write-Host ""
    Write-Host "ERROR: Failed to start camera" -ForegroundColor Red
    Write-Host $_.Exception.Message -ForegroundColor Red
    pause
}
