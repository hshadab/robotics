#!/bin/bash
# Quick Start Script for JOLT Atlas zkML Robotics Demo
# Launches all services: verifier, UI server, camera, guard, and teleop

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}JOLT Atlas zkML Robotics Demo Launcher${NC}"
echo -e "${BLUE}========================================${NC}\n"

# Check if ROS workspace is built
if [ ! -d "install" ]; then
    echo -e "${YELLOW}⚠ ROS workspace not built. Building now...${NC}"
    colcon build
fi

# Source ROS environment
if [ -f "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" ]; then
    source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
    echo -e "${GREEN}✓${NC} Sourced ROS ${ROS_DISTRO:-jazzy}"
else
    echo -e "${RED}✗ ROS installation not found${NC}"
    exit 1
fi

if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo -e "${GREEN}✓${NC} Sourced workspace"
fi

# Check Python dependencies
if ! python3 -c "import onnxruntime, numpy, PIL, requests" 2>/dev/null; then
    echo -e "${YELLOW}⚠ Python dependencies not installed${NC}"
    echo -e "${YELLOW}  Please install with: pip install -r src/zkml_guard/requirements.txt${NC}"
    echo -e "${YELLOW}  Or use --break-system-packages if needed${NC}"
    echo -e "${YELLOW}  Continuing anyway...${NC}"
fi

# Check Node.js dependencies for verifier
if [ ! -d "tools/onnx-verifier/node_modules" ]; then
    echo -e "${YELLOW}⚠ Installing ONNX verifier dependencies...${NC}"
    cd tools/onnx-verifier
    npm install
    cd "$SCRIPT_DIR"
fi

# Check Node.js dependencies for UI
if [ ! -d "tools/robotics-ui/node_modules" ]; then
    echo -e "${YELLOW}⚠ Installing UI server dependencies...${NC}"
    cd tools/robotics-ui
    npm install
    cd "$SCRIPT_DIR"
fi

echo -e "\n${GREEN}✓${NC} All dependencies ready\n"

# Parse command line arguments
MODE="http"
BURGER="false"
RECORD="false"
OPEN_BROWSER="true"

while [[ $# -gt 0 ]]; do
    case $1 in
        --cli)
            MODE="cli"
            shift
            ;;
        --burger)
            BURGER="true"
            shift
            ;;
        --record)
            RECORD="true"
            shift
            ;;
        --no-browser)
            OPEN_BROWSER="false"
            shift
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --cli         Use CLI verifier mode (default: HTTP)"
            echo "  --burger      Use test pattern instead of real camera"
            echo "  --record      Record session to MCAP file"
            echo "  --no-browser  Don't auto-open browser"
            echo "  --help        Show this help message"
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

echo -e "${BLUE}Configuration:${NC}"
echo -e "  Mode: ${YELLOW}$MODE${NC}"
echo -e "  Camera: ${YELLOW}$([ "$BURGER" = "true" ] && echo "Test pattern" || echo "Real device")${NC}"
echo -e "  Recording: ${YELLOW}$([ "$RECORD" = "true" ] && echo "Enabled" || echo "Disabled")${NC}"
echo -e ""

# Function to wait for service to be ready
wait_for_service() {
    local url=$1
    local name=$2
    local max_wait=30
    local count=0

    echo -n "Waiting for $name to start"
    while ! curl -s "$url" > /dev/null 2>&1; do
        echo -n "."
        sleep 1
        count=$((count + 1))
        if [ $count -ge $max_wait ]; then
            echo -e "\n${RED}✗ $name failed to start${NC}"
            return 1
        fi
    done
    echo -e " ${GREEN}✓${NC}"
}

# Start UI server (this auto-starts verifier and proxies)
echo -e "${BLUE}Starting UI server...${NC}"
cd tools/robotics-ui
npm start > /tmp/robotics-ui.log 2>&1 &
UI_PID=$!
cd "$SCRIPT_DIR"

# Wait for UI server
wait_for_service "http://localhost:9200/health" "UI server" || exit 1

# Wait a bit for verifier to start (UI auto-starts it)
sleep 3
wait_for_service "http://localhost:9100/health" "ONNX verifier" || exit 1

# Start all services via the UI API
echo -e "\n${BLUE}Starting demo components...${NC}"

# Build query parameters
QUERY="mode=$MODE"
[ "$BURGER" = "true" ] && QUERY="$QUERY&burger=1" || QUERY="$QUERY&burger=0"
[ "$RECORD" = "true" ] && QUERY="$QUERY&record=1" || QUERY="$QUERY&record=0"

# Start full demo
RESPONSE=$(curl -s "http://localhost:9200/start/full?$QUERY")

if echo "$RESPONSE" | grep -q '"ok":true'; then
    echo -e "${GREEN}✓${NC} Camera started"
    echo -e "${GREEN}✓${NC} zkML Guard started"
    echo -e "${GREEN}✓${NC} Teleop publisher started"
    [ "$RECORD" = "true" ] && echo -e "${GREEN}✓${NC} MCAP recording started"
else
    echo -e "${RED}✗ Failed to start services${NC}"
    echo "$RESPONSE"
    exit 1
fi

# Open browser
if [ "$OPEN_BROWSER" = "true" ]; then
    echo -e "\n${BLUE}Opening browser...${NC}"
    sleep 2
    if command -v xdg-open > /dev/null; then
        xdg-open "http://localhost:9200/demo.html" 2>/dev/null &
    elif command -v open > /dev/null; then
        open "http://localhost:9200/demo.html" 2>/dev/null &
    else
        echo -e "${YELLOW}⚠ Please open http://localhost:9200/demo.html manually${NC}"
    fi
fi

echo -e "\n${GREEN}========================================${NC}"
echo -e "${GREEN}Demo started successfully!${NC}"
echo -e "${GREEN}========================================${NC}\n"

echo -e "${BLUE}Access Points:${NC}"
echo -e "  Demo UI:    ${YELLOW}http://localhost:9200/demo.html${NC}"
echo -e "  Status:     ${YELLOW}http://localhost:9200/status${NC}"
echo -e "  Verifier:   ${YELLOW}http://localhost:9100/health${NC}"
echo -e ""

echo -e "${BLUE}Logs:${NC}"
echo -e "  UI Server:     tail -f /tmp/robotics-ui.log"
echo -e "  Event Proxy:   tail -f /tmp/event_proxy.py.log"
echo -e "  Frame Proxy:   tail -f /tmp/frame_proxy.py.log"
echo -e ""

echo -e "${BLUE}To stop the demo:${NC}"
echo -e "  Run: ${YELLOW}./stop_demo.sh${NC}"
echo -e "  Or:  ${YELLOW}curl http://localhost:9200/stop/all${NC}"
echo -e ""

# Keep script running and show status
echo -e "${BLUE}Press Ctrl+C to stop all services${NC}\n"

# Trap Ctrl+C
trap 'echo -e "\n${YELLOW}Stopping demo...${NC}"; curl -s http://localhost:9200/stop/all > /dev/null; kill $UI_PID 2>/dev/null; echo -e "${GREEN}Demo stopped${NC}"; exit 0' INT

# Monitor services
while true; do
    sleep 5
    # Check if UI is still running
    if ! kill -0 $UI_PID 2>/dev/null; then
        echo -e "${RED}UI server stopped unexpectedly${NC}"
        exit 1
    fi
done
