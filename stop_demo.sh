#!/bin/bash
# Stop all JOLT Atlas zkML Robotics Demo services

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}Stopping JOLT Atlas zkML Robotics Demo...${NC}\n"

# Stop via UI API if available
if curl -s http://localhost:9200/health > /dev/null 2>&1; then
    echo -e "${YELLOW}Stopping services via UI API...${NC}"
    curl -s http://localhost:9200/stop/all > /dev/null
    sleep 2
fi

# Kill any remaining processes
echo -e "${YELLOW}Cleaning up processes...${NC}"

# Stop ROS nodes
pkill -f "ros2 run image_tools" 2>/dev/null && echo -e "${GREEN}✓${NC} Stopped camera" || true
pkill -f "ros2 launch zkml_guard" 2>/dev/null && echo -e "${GREEN}✓${NC} Stopped zkML guard" || true
pkill -f "ros2 topic pub" 2>/dev/null && echo -e "${GREEN}✓${NC} Stopped teleop" || true
pkill -f "ros2 bag record" 2>/dev/null && echo -e "${GREEN}✓${NC} Stopped recording" || true
pkill -f "twist_mux" 2>/dev/null && echo -e "${GREEN}✓${NC} Stopped twist_mux" || true

# Stop proxies
pkill -f "event_proxy.py" 2>/dev/null && echo -e "${GREEN}✓${NC} Stopped event proxy" || true
pkill -f "frame_proxy.py" 2>/dev/null && echo -e "${GREEN}✓${NC} Stopped frame proxy" || true

# Stop servers
pkill -f "node server.js" 2>/dev/null && echo -e "${GREEN}✓${NC} Stopped verifier" || true
pkill -f "npm start" 2>/dev/null && echo -e "${GREEN}✓${NC} Stopped UI server" || true

# Clean up PID files
rm -f /tmp/rdemo-*.pid 2>/dev/null

echo -e "\n${GREEN}All services stopped${NC}"
