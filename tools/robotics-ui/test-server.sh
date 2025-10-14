#!/bin/bash
set -e

echo "=== Testing Robotics UI Server ==="
echo ""

cd "$(dirname "$0")"

echo "1. Checking Node.js..."
node --version || { echo "ERROR: Node.js not found"; exit 1; }
npm --version || { echo "ERROR: npm not found"; exit 1; }

echo ""
echo "2. Installing dependencies..."
if [ ! -d "node_modules" ]; then
    npm install
else
    echo "   node_modules exists, skipping"
fi

echo ""
echo "3. Starting server in background..."
node server.js &
SERVER_PID=$!
echo "   Server PID: $SERVER_PID"

echo ""
echo "4. Waiting for server to start..."
sleep 3

echo ""
echo "5. Testing endpoints..."

echo "   Testing /health..."
curl -s http://localhost:9200/health | jq -r '.ok // "FAIL"' || echo "FAIL"

echo "   Testing /status..."
curl -s http://localhost:9200/status | jq -r 'keys | join(", ")' || echo "FAIL"

echo "   Testing /api/proxy_status..."
curl -s http://localhost:9200/api/proxy_status | jq -r '.event_proxy.running, .frame_proxy.running' || echo "FAIL"

echo "   Testing /debug/files..."
curl -s http://localhost:9200/debug/files | jq -r 'keys | join(", ")' || echo "FAIL"

echo ""
echo "6. Checking proxy processes..."
ps aux | grep -E "(event_proxy|frame_proxy)" | grep -v grep || echo "   No proxies running (expected if ROS not running)"

echo ""
echo "7. Server logs (last 20 lines):"
sleep 1
kill -0 $SERVER_PID 2>/dev/null && echo "   Server still running" || echo "   Server died!"

echo ""
echo "=== Test Complete ==="
echo "Server running at: http://localhost:9200"
echo "Stop with: kill $SERVER_PID"
echo "Or visit http://localhost:9200 in your browser"
