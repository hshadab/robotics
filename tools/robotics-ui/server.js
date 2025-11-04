const express = require('express');
const cors = require('cors');
const path = require('path');
const { spawn } = require('child_process');
const fs = require('fs');

const app = express();
const PORT = process.env.PORT || 9200;
app.use(cors());
app.use(express.json());
app.use(express.urlencoded({ extended: true }));

const WS_ROOT = path.resolve(__dirname, '../../');
const VERIFIER_DIR = path.resolve(__dirname, '../onnx-verifier');
const PROXY_EVENT_PID = '/tmp/rdemo-event-proxy.pid';
const PROXY_FRAME_PID = '/tmp/rdemo-frame-proxy.pid';
const LAST_EVENT_FILE = '/tmp/rdemo-last-event.json';
const VERIFIED_PROOFS_FILE = '/tmp/rdemo-verified-proofs.json';
const STOP_FILE = '/tmp/rdemo-stop.txt';
const FRAME_FILE = '/tmp/rdemo-frame.png';
const SNAPSHOTS_DIR = '/tmp/rdemo-snapshots';
const PROXY_EVENT_LOG = '/tmp/event_proxy.py.log';
const PROXY_FRAME_LOG = '/tmp/frame_proxy.py.log';

// In-memory state to replace brittle temp-file IPC
const state = {
  lastEvent: null,
  stop: null,
  lastFrame: null, // Buffer
  verifiedProofs: [],
};

// Simple SSE client registry for push updates
const sseClients = new Set();
function sseBroadcast(eventName, dataObj) {
  const payload = `event: ${eventName}\n` +
                  `data: ${JSON.stringify(dataObj)}\n\n`;
  for (const res of sseClients) {
    try { res.write(payload); } catch (_) {}
  }
}

function rosEnvCmd(userCmd) {
  // Build a bash command that sources ROS + workspace and runs user command
  const setupRos = `/opt/ros/${process.env.ROS_DISTRO || 'jazzy'}/setup.bash`;
  return `set -e; source '${setupRos}' >/dev/null 2>&1 || true; if [ -f '${WS_ROOT}/install/setup.bash' ]; then . '${WS_ROOT}/install/setup.bash'; fi; ${userCmd}`;
}

function startProc(cmd, opts = {}) {
  // Use bash -c to run command with proper environment
  const p = spawn('/bin/bash', ['-c', cmd], {
    cwd: opts.cwd || WS_ROOT,
    detached: true,
    stdio: ['ignore', 'pipe', 'pipe']  // Keep stdout/stderr so processes don't die
  });

  // Log output to files for debugging
  const logFile = `/tmp/rdemo-${opts.name || 'process'}.log`;
  if (p.stdout && p.stderr) {
    const logStream = fs.createWriteStream(logFile, { flags: 'a' });
    p.stdout.pipe(logStream);
    p.stderr.pipe(logStream);
  }

  p.unref();
  return p.pid;
}

function writePid(name, pid) {
  fs.writeFileSync(`/tmp/rdemo-${name}.pid`, String(pid));
}

function readPid(name) {
  try { return parseInt(fs.readFileSync(`/tmp/rdemo-${name}.pid`, 'utf8')); } catch { return null; }
}

function stopProc(name) {
  const pid = readPid(name);
  if (pid && Number.isInteger(pid)) {
    try {
      // Kill the entire process group (negative PID)
      process.kill(-pid, 'SIGTERM');
    } catch (e) {
      // Fallback: kill just the main process
      try { process.kill(pid, 'SIGTERM'); } catch {}
    }
  }
}

function isRunning(name) {
  const pid = readPid(name);
  if (!pid) return false;
  try { process.kill(pid, 0); return true; } catch { return false; }
}

// API routes defined above; mount static last so it doesn't shadow them

app.get('/health', (req, res) => res.json({ ok: true }));

app.get('/status', (req, res) => {
  res.json({
    verifier: isRunning('verifier'),
    camera: isRunning('camera'),
    guard: isRunning('guard'),
    teleop: isRunning('teleop'),
    bag: isRunning('bag'),
    proxy_event: isRunning('event-proxy'),
    proxy_frame: isRunning('frame-proxy'),
  });
});

// Live /zkml/event stream via Server-Sent Events (SSE)
function sseEvents(req, res){
  res.writeHead(200, {
    'Content-Type': 'text/event-stream',
    'Cache-Control': 'no-cache',
    Connection: 'keep-alive',
  });
  // Send current snapshot immediately
  if (state.lastEvent) {
    res.write('event: zkml\n');
    res.write(`data: ${JSON.stringify(state.lastEvent)}\n\n`);
  }
  sseClients.add(res);
  const cleanup = () => { try { sseClients.delete(res); } catch {} };
  req.on('close', cleanup);
  req.on('end', cleanup);
}
app.get('/events', sseEvents);
app.get('/api/events', sseEvents);

// Polling fallback: return one latest /zkml/event sample
app.get('/api/last_event', (req, res) => {
  if (state.lastEvent) return res.json(state.lastEvent);
  try {
    const txt = fs.readFileSync(LAST_EVENT_FILE, 'utf8');
    const obj = JSON.parse(txt);
    state.lastEvent = obj; // hydrate cache
    return res.json(obj);
  } catch {
    return res.status(204).end();
  }
});

// Verified proofs history
app.get('/api/verified_proofs', (req, res) => {
  if (state.verifiedProofs.length) return res.json(state.verifiedProofs);
  try {
    const txt = fs.readFileSync(VERIFIED_PROOFS_FILE, 'utf8');
    const arr = JSON.parse(txt);
    state.verifiedProofs = arr;
    return res.json(arr);
  } catch {
    return res.json([]);
  }
});

// Clear verified proofs history
app.post('/api/clear_verified_proofs', (req, res) => {
  try {
    fs.writeFileSync(VERIFIED_PROOFS_FILE, JSON.stringify([]));
    return res.json({ ok: true });
  } catch (e) {
    return res.status(500).json({ ok: false, error: e.message });
  }
});

// STOP lock indicator
app.get('/api/stop_state', (req, res) => {
  if (typeof state.stop === 'boolean') return res.json({ stop: state.stop });
  try {
    const val = fs.readFileSync(STOP_FILE, 'utf8').trim();
    if (val === 'true' || val === 'false') {
      state.stop = (val === 'true');
      return res.json({ stop: state.stop });
    }
  } catch {}
  return res.json({ stop: null });
});

// Latest camera frame as PNG
app.get('/api/frame.png', (req, res) => {
  try {
    if (state.lastFrame) {
      res.setHeader('Content-Type', 'image/png');
      return res.end(state.lastFrame);
    }
  } catch {}
  try {
    const png = fs.readFileSync(FRAME_FILE);
    state.lastFrame = png; // hydrate cache
    res.setHeader('Content-Type', 'image/png');
    res.end(png);
  } catch {
    res.status(204).end();
  }
});

// Snapshot image by proof ID
app.get('/api/snapshot/:proofId.png', (req, res) => {
  try {
    const proofId = req.params.proofId;
    const snapshotPath = path.join(SNAPSHOTS_DIR, `${proofId}.png`);
    const png = fs.readFileSync(snapshotPath);
    res.setHeader('Content-Type', 'image/png');
    res.setHeader('Cache-Control', 'public, max-age=31536000'); // Cache for 1 year since snapshots don't change
    res.end(png);
  } catch {
    res.status(404).end();
  }
});

// Static frontend (mounted last) - disable caching for HTML files
app.use('/', express.static(path.join(__dirname, 'public'), {
  setHeaders: (res, filePath) => {
    if (filePath.endsWith('.html')) {
      // Disable caching for HTML files to always serve latest version
      res.setHeader('Cache-Control', 'no-cache, no-store, must-revalidate');
      res.setHeader('Pragma', 'no-cache');
      res.setHeader('Expires', '0');
    }
  }
}));

// Verifier
app.post('/start/verifier', (req, res) => {
  if (isRunning('verifier')) return res.json({ started: false, reason: 'already running' });
  const pid = startProc(`cd '${VERIFIER_DIR}'; node server.js`, { cwd: VERIFIER_DIR });
  writePid('verifier', pid);
  return res.json({ started: true, pid });
});
app.post('/stop/verifier', (req, res) => { stopProc('verifier'); return res.json({ stopped: true }); });

// Camera
app.post('/start/camera', (req, res) => {
  if (isRunning('camera')) return res.json({ started: false, reason: 'already running' });
  const burger = !!(req.body && req.body.burger);
  const cmd = rosEnvCmd(`ros2 run image_tools cam2image ${burger ? "--ros-args -p burger_mode:=true" : ''}`);
  const pid = startProc(cmd, { name: 'camera' });
  writePid('camera', pid);
  return res.json({ started: true, pid });
});
app.post('/stop/camera', (req, res) => { stopProc('camera'); return res.json({ stopped: true }); });

// Guard (includes twist_mux via launch)
app.post('/start/guard', (req, res) => {
  if (isRunning('guard')) return res.json({ started: false, reason: 'already running' });
  const mode = (req.body && req.body.mode) || 'http';
  // ROS args must come AFTER the launch file, and parameters use verifier_mode:=cli syntax
  const launchFile = 'zkml_guard zkml_guard_proof.launch.py';
  const args = mode === 'cli' ? 'verifier_mode:=cli' : '';
  const cmd = rosEnvCmd(`ros2 launch ${launchFile} ${args}`);
  const pid = startProc(cmd, { name: 'guard' });
  writePid('guard', pid);
  return res.json({ started: true, pid });
});
app.post('/stop/guard', (req, res) => { stopProc('guard'); return res.json({ stopped: true }); });

// Teleop (demo publisher)
app.post('/start/teleop', (req, res) => {
  if (isRunning('teleop')) return res.json({ started: false, reason: 'already running' });
  const cmd = rosEnvCmd("ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.15}}' -r 5");
  const pid = startProc(cmd, { name: 'teleop' });
  writePid('teleop', pid);
  return res.json({ started: true, pid });
});
app.post('/stop/teleop', (req, res) => { stopProc('teleop'); return res.json({ stopped: true }); });

// MCAP Record
app.post('/start/bag', (req, res) => {
  if (isRunning('bag')) return res.json({ started: false, reason: 'already running' });
  const cmd = rosEnvCmd('ros2 bag record -a -s mcap');
  const pid = startProc(cmd, { name: 'bag' });
  writePid('bag', pid);
  return res.json({ started: true, pid });
});
app.post('/stop/bag', (req, res) => { stopProc('bag'); return res.json({ stopped: true }); });

// One-shot full demo start/stop (GET for easy browser clicks)
app.get('/start/full', async (req, res) => {
  const mode = (req.query.mode || 'http').toString();
  const burger = !!(req.query.burger && req.query.burger !== '0' && req.query.burger !== 'false');
  const record = !!(req.query.record && req.query.record !== '0' && req.query.record !== 'false');

  console.log(`[start/full] Starting demo: mode=${mode}, burger=${burger}, record=${record}`);

  try {
    if (mode === 'http' && !isRunning('verifier')) {
      console.log('[start/full] Starting verifier...');
      const pid = startProc(`cd '${VERIFIER_DIR}'; node server.js`, { cwd: VERIFIER_DIR, name: 'verifier' });
      writePid('verifier', pid);
    }
    if (!isRunning('camera')) {
      console.log('[start/full] Starting camera (burger=' + burger + ')...');
      const cmdCam = rosEnvCmd(`ros2 run image_tools cam2image ${burger ? "--ros-args -p burger_mode:=true" : ''}`);
      writePid('camera', startProc(cmdCam, { name: 'camera' }));
    }
    if (!isRunning('guard')) {
      console.log('[start/full] Starting guard (mode=' + mode + ')...');
      const launchFile = 'zkml_guard zkml_guard_proof.launch.py';
      const args = mode === 'cli' ? 'verifier_mode:=cli' : '';
      const cmdGuard = rosEnvCmd(`ros2 launch ${launchFile} ${args}`);
      writePid('guard', startProc(cmdGuard, { name: 'guard' }));
    }
    if (!isRunning('teleop')) {
      console.log('[start/full] Starting teleop...');
      const cmdTele = rosEnvCmd("ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.15}}' -r 5");
      writePid('teleop', startProc(cmdTele, { name: 'teleop' }));
    }
    if (record && !isRunning('bag')) {
      console.log('[start/full] Starting bag recording...');
      const cmdBag = rosEnvCmd('ros2 bag record -a -s mcap');
      writePid('bag', startProc(cmdBag, { name: 'bag' }));
    }

    // Wait a moment for processes to start
    await new Promise(resolve => setTimeout(resolve, 500));

    const status = {
      verifier: isRunning('verifier'),
      camera: isRunning('camera'),
      guard: isRunning('guard'),
      teleop: isRunning('teleop'),
      bag: isRunning('bag')
    };

    console.log('[start/full] Started. Status:', status);

    res.json({ ok: true, mode, burger, record, status });
  } catch (e) {
    console.error('[start/full] Error:', e);
    res.status(500).json({ ok: false, error: e.message });
  }
});

app.get('/stop/all', (req, res) => {
  ['bag','teleop','guard','camera','verifier'].forEach(stopProc);

  // Clear last event file to prevent stale data when restarting
  try {
    if (fs.existsSync(LAST_EVENT_FILE)) {
      fs.unlinkSync(LAST_EVENT_FILE);
      console.log('[stop/all] Cleared last event file');
    }
  } catch (e) {
    console.error('[stop/all] Failed to clear last event file:', e);
  }

  res.json({ ok: true });
});

app.listen(PORT, () => {
  console.log(`[robotics-ui] listening on http://localhost:${PORT}`);
});

// Ensure ROS background proxies are running (event + frame) without venv
function ensureProxy(pyName, pidFile, logFile) {
  const proxyName = pyName.replace('.py', '');
  try {
    const pid = parseInt(fs.readFileSync(pidFile, 'utf8'));
    if (pid && !isNaN(pid)) {
      try {
        process.kill(pid, 0);
        console.log(`[robotics-ui] ${proxyName} already running (PID ${pid})`);
        return { started: false, pid, reason: 'already running' };
      } catch {
        console.log(`[robotics-ui] Stale PID for ${proxyName}, restarting...`);
      }
    }
  } catch {}

  const py = path.join(__dirname, pyName);
  if (!fs.existsSync(py)) {
    console.error(`[robotics-ui] ERROR: Proxy script not found: ${py}`);
    return { started: false, error: 'script not found' };
  }

  const cmd = rosEnvCmd(`python3 '${py}' 2>&1`);
  console.log(`[robotics-ui] Starting ${proxyName}...`);

  const p = spawn('/bin/bash', ['-lc', cmd], {
    cwd: __dirname,
    detached: true,
    stdio: ['ignore', 'pipe', 'pipe']
  });

  // Capture logs
  const logStream = fs.createWriteStream(logFile, { flags: 'a' });
  p.stdout.pipe(logStream);
  p.stderr.pipe(logStream);

  p.on('error', (err) => {
    console.error(`[robotics-ui] Failed to start ${proxyName}: ${err.message}`);
  });

  p.on('exit', (code) => {
    console.log(`[robotics-ui] ${proxyName} exited with code ${code}`);
  });

  p.unref();

  if (p.pid) {
    fs.writeFileSync(pidFile, String(p.pid));
    console.log(`[robotics-ui] Started ${proxyName} (PID ${p.pid})`);
    return { started: true, pid: p.pid };
  } else {
    console.error(`[robotics-ui] Failed to start ${proxyName}: no PID`);
    return { started: false, error: 'no pid' };
  }
}

// Start proxies on server launch with retry
let proxyRetries = 0;
const MAX_PROXY_RETRIES = 3;

function startProxiesWithRetry() {
  const eventResult = ensureProxy('event_proxy.py', PROXY_EVENT_PID, PROXY_EVENT_LOG);
  const frameResult = ensureProxy('frame_proxy.py', PROXY_FRAME_PID, PROXY_FRAME_LOG);

  const bothRunning = (eventResult.started || eventResult.reason === 'already running') &&
                      (frameResult.started || frameResult.reason === 'already running');

  if (!bothRunning && proxyRetries < MAX_PROXY_RETRIES) {
    proxyRetries++;
    console.log(`[robotics-ui] Retrying proxy startup (${proxyRetries}/${MAX_PROXY_RETRIES}) in 2s...`);
    setTimeout(startProxiesWithRetry, 2000);
  } else if (bothRunning) {
    console.log('[robotics-ui] All proxies running successfully');
  } else {
    console.error('[robotics-ui] WARNING: Failed to start proxies after retries. Check logs at /tmp/*_proxy.py.log');
  }
}

startProxiesWithRetry();

// Internal push endpoints for proxies (replace temp-file IPC)
app.post('/internal/event', (req, res) => {
  try {
    const data = req.body && Object.keys(req.body).length ? req.body : null;
    if (!data) return res.status(400).json({ ok: false, error: 'missing body' });
    state.lastEvent = data;

    // Mirror to disk for compatibility
    try { fs.writeFileSync(LAST_EVENT_FILE, JSON.stringify(data)); } catch {}

    // If verified proof, add to history and snapshot current frame
    if (data.proof_verified === true && data.proof_id) {
      const exists = state.verifiedProofs.find(p => p.proof_id === data.proof_id);
      if (!exists) {
        // Save snapshot if we have a frame
        if (state.lastFrame) {
          try {
            const id = (data.proof_id || '').replace(/^0x/, '');
            const snapshotPath = path.join(SNAPSHOTS_DIR, `${id}.png`);
            fs.writeFileSync(snapshotPath, state.lastFrame);
            data.snapshot = id;
          } catch {}
        }
        state.verifiedProofs.unshift(data);
        state.verifiedProofs = state.verifiedProofs.slice(0, 50);
        try { fs.writeFileSync(VERIFIED_PROOFS_FILE, JSON.stringify(state.verifiedProofs)); } catch {}
      }
    }

    sseBroadcast('zkml', data);
    return res.json({ ok: true });
  } catch (e) {
    return res.status(500).json({ ok: false, error: e.message });
  }
});

app.post('/internal/stop', (req, res) => {
  try {
    const stop = !!(req.body && (req.body.stop === true || req.body.stop === 'true'));
    state.stop = stop;
    try { fs.writeFileSync(STOP_FILE, stop ? 'true' : 'false'); } catch {}
    sseBroadcast('stop', { stop });
    return res.json({ ok: true });
  } catch (e) {
    return res.status(500).json({ ok: false, error: e.message });
  }
});

app.post('/internal/frame', express.raw({ type: 'image/png', limit: '5mb' }), (req, res) => {
  try {
    if (!req.body || !req.body.length) return res.status(400).json({ ok: false, error: 'missing PNG body' });
    state.lastFrame = Buffer.from(req.body);
    // Mirror to disk for compatibility
    try { fs.writeFileSync(FRAME_FILE, state.lastFrame); } catch {}
    // No SSE broadcast to avoid flooding; clients pull /api/frame.png on demand
    return res.json({ ok: true });
  } catch (e) {
    return res.status(500).json({ ok: false, error: e.message });
  }
});

// Proxy status and control
app.get('/api/proxy_status', (req, res) => {
  const eventRunning = isRunning('event-proxy');
  const frameRunning = isRunning('frame-proxy');
  const eventLog = eventRunning ? null : (() => {
    try { return fs.readFileSync(PROXY_EVENT_LOG, 'utf8').split('\n').slice(-10).join('\n'); } catch { return null; }
  })();
  const frameLog = frameRunning ? null : (() => {
    try { return fs.readFileSync(PROXY_FRAME_LOG, 'utf8').split('\n').slice(-10).join('\n'); } catch { return null; }
  })();

  res.json({
    event_proxy: {
      running: eventRunning,
      pid: readPid('event-proxy'),
      log_tail: eventLog
    },
    frame_proxy: {
      running: frameRunning,
      pid: readPid('frame-proxy'),
      log_tail: frameLog
    },
    files: {
      last_event: fs.existsSync(LAST_EVENT_FILE),
      stop_file: fs.existsSync(STOP_FILE),
      frame_file: fs.existsSync(FRAME_FILE)
    }
  });
});

app.get('/start/proxies', (req, res) => {
  const eventResult = ensureProxy('event_proxy.py', PROXY_EVENT_PID, PROXY_EVENT_LOG);
  const frameResult = ensureProxy('frame_proxy.py', PROXY_FRAME_PID, PROXY_FRAME_LOG);
  res.json({ ok: true, event: eventResult, frame: frameResult });
});

app.get('/debug/files', (req, res) => {
  const info = {};
  try { const s = fs.statSync(LAST_EVENT_FILE); info.last_event = { size: s.size, mtime: s.mtime }; } catch { info.last_event = null; }
  try { const s = fs.statSync(STOP_FILE); info.stop = { size: s.size, mtime: s.mtime, value: fs.readFileSync(STOP_FILE, 'utf8').trim() }; } catch { info.stop = null; }
  try { const s = fs.statSync(FRAME_FILE); info.frame = { size: s.size, mtime: s.mtime }; } catch { info.frame = null; }
  res.json(info);
});

app.get('/debug/logs', (req, res) => {
  const logs = {};
  try { logs.event = fs.readFileSync('/tmp/event_proxy.py.log', 'utf8').split('\n').slice(-50).join('\n'); } catch { logs.event = null; }
  try { logs.frame = fs.readFileSync('/tmp/frame_proxy.py.log', 'utf8').split('\n').slice(-50).join('\n'); } catch { logs.frame = null; }
  res.json(logs);
});

// Check Windows camera HTTP server status
app.get('/api/windows_camera_status', async (req, res) => {
  const http = require('http');

  const checkUrl = (url) => {
    return new Promise((resolve) => {
      const request = http.get(url, { timeout: 1000 }, (response) => {
        resolve({ running: response.statusCode === 200, url });
      });
      request.on('error', () => resolve({ running: false, url }));
      request.on('timeout', () => {
        request.destroy();
        resolve({ running: false, url });
      });
    });
  };

  // Try localhost first, then Windows host IP
  const localhostResult = await checkUrl('http://localhost:8080/status');

  if (localhostResult.running) {
    return res.json({
      running: true,
      url: 'http://localhost:8080',
      message: 'Windows camera server is running'
    });
  }

  // Try to detect Windows host IP
  try {
    const { exec } = require('child_process');
    exec('cat /etc/resolv.conf | grep nameserver | head -1 | awk \'{print $2}\'', async (error, stdout) => {
      if (!error && stdout.trim()) {
        const windowsIp = stdout.trim().split('\n')[0]; // Take first line only
        if (windowsIp && !windowsIp.includes(' ')) { // Validate it's a single IP
          const windowsResult = await checkUrl(`http://${windowsIp}:8080/status`);
          if (windowsResult.running) {
            return res.json({
              running: true,
              url: `http://${windowsIp}:8080`,
              message: 'Windows camera server is running'
            });
          }
        }
      }

      res.json({
        running: false,
        message: 'Windows camera server not detected. Please start it manually.',
        instructions: 'Run in PowerShell: python simple_windows_camera.py 1'
      });
    });
  } catch {
    res.json({
      running: false,
      message: 'Windows camera server not detected',
      instructions: 'Run in PowerShell: python simple_windows_camera.py 1'
    });
  }
});

// Helper: Open Windows camera instructions
app.get('/api/windows_camera_instructions', (req, res) => {
  const instructions = {
    title: 'Start Windows Camera',
    steps: [
      'Open PowerShell on Windows',
      'Navigate to: cd \\\\wsl$\\Ubuntu\\home\\hshadab\\robotics',
      'Run: python simple_windows_camera.py 1',
      'Keep the PowerShell window open',
      'Your webcam light should turn ON'
    ],
    quickCommand: 'python simple_windows_camera.py 1',
    scriptPath: '\\\\wsl$\\Ubuntu\\home\\hshadab\\robotics\\Start-Camera.ps1'
  };
  res.json(instructions);
});

// Tampered Model Demo API
// Try multiple possible model locations
function findModelPath() {
  const possiblePaths = [
    path.join(process.env.HOME || '', '.cache/zkml_guard/models/mobilenetv2-7.onnx'),
    path.join(__dirname, '../onnx-verifier/models/mobilenetv2-12.onnx'),
    path.join(__dirname, '../../tools/onnx-verifier/models/mobilenetv2-12.onnx'),
  ];

  for (const p of possiblePaths) {
    if (fs.existsSync(p)) {
      console.log(`[model] Found model at: ${p}`);
      return p;
    }
  }

  console.log('[model] No model found in any location');
  return null;
}

// These are no longer used at the module level since we call findModelPath() at runtime
// Kept for backward compatibility in case any code references them
const MODEL_DIR = path.join(process.env.HOME || '', '.cache/zkml_guard/models');
const ORIGINAL_MODEL = path.join(MODEL_DIR, 'mobilenetv2-7.onnx');
const TAMPERED_MODEL = ORIGINAL_MODEL + '.tampered';
const BACKUP_MODEL = ORIGINAL_MODEL + '.original';

app.post('/api/flip_model', (req, res) => {
  try {
    // Re-check model path at runtime (in case it was downloaded after server started)
    const modelPath = findModelPath();
    if (!modelPath) {
      return res.json({ ok: false, error: 'Original model not found. Start the guard first to download it.' });
    }

    const backupPath = modelPath + '.original';
    const tamperedPath = modelPath + '.tampered';

    // Check if we already have a backup (meaning we've already flipped)
    if (fs.existsSync(backupPath)) {
      return res.json({ ok: false, error: 'Model already tampered. Restore first.' });
    }

    // Create tampered model if it doesn't exist
    if (!fs.existsSync(tamperedPath)) {
      console.log('[flip_model] Creating tampered model...');
      // Copy original and modify a few bytes to create hash mismatch
      const originalData = fs.readFileSync(modelPath);
      const tamperedData = Buffer.from(originalData);

      // Flip some bytes in the middle to create a different hash but keep structure valid
      // This simulates a model substitution attack
      const flipPos = Math.floor(tamperedData.length / 2);
      tamperedData[flipPos] ^= 0xFF;
      tamperedData[flipPos + 1] ^= 0xFF;
      tamperedData[flipPos + 2] ^= 0xFF;

      fs.writeFileSync(tamperedPath, tamperedData);
      console.log('[flip_model] Tampered model created');
    }

    // Backup original and replace with tampered
    fs.copyFileSync(modelPath, backupPath);
    fs.copyFileSync(tamperedPath, modelPath);

    console.log('[flip_model] Model swapped to tampered version');

    res.json({
      ok: true,
      message: 'Model swapped to tampered version. Next proof will fail due to hash mismatch.',
      originalHash: require('crypto').createHash('sha256').update(fs.readFileSync(backupPath)).digest('hex').substring(0, 8),
      tamperedHash: require('crypto').createHash('sha256').update(fs.readFileSync(modelPath)).digest('hex').substring(0, 8)
    });
  } catch (e) {
    console.error('[flip_model] Error:', e);
    res.status(500).json({ ok: false, error: e.message });
  }
});

app.post('/api/restore_model', (req, res) => {
  try {
    // Re-check model path at runtime
    const modelPath = findModelPath();
    if (!modelPath) {
      return res.json({ ok: false, error: 'Model not found.' });
    }

    const backupPath = modelPath + '.original';

    // Check if backup exists
    if (!fs.existsSync(backupPath)) {
      return res.json({ ok: false, error: 'No backup found. Model is already in original state.' });
    }

    // Restore original from backup
    fs.copyFileSync(backupPath, modelPath);
    fs.unlinkSync(backupPath);

    console.log('[restore_model] Model restored to original version');

    res.json({
      ok: true,
      message: 'Model restored to original version. Next proof should succeed.',
      restoredHash: require('crypto').createHash('sha256').update(fs.readFileSync(modelPath)).digest('hex').substring(0, 8)
    });
  } catch (e) {
    console.error('[restore_model] Error:', e);
    res.status(500).json({ ok: false, error: e.message });
  }
});

// No background proxy required; CLI-based polling is used
