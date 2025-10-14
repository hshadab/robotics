const express = require('express');
const cors = require('cors');
const path = require('path');
const { spawn, execSync } = require('child_process');
const fs = require('fs');

const app = express();
const PORT = process.env.PORT || 9200;
app.use(cors());
app.use(express.json());

const ROOT = path.resolve(__dirname, '../../');
const WS_ROOT = path.resolve(__dirname, '../../');
const VERIFIER_DIR = path.resolve(__dirname, '../onnx-verifier');
const VENV_SITE = path.join(process.env.HOME || '', '.venvs/zkml_guard_ui/lib/python3.12/site-packages');
const PROXY_EVENT_PID = '/tmp/rdemo-event-proxy.pid';
const PROXY_FRAME_PID = '/tmp/rdemo-frame-proxy.pid';
const LAST_EVENT_FILE = '/tmp/rdemo-last-event.json';
const STOP_FILE = '/tmp/rdemo-stop.txt';
const FRAME_FILE = '/tmp/rdemo-frame.png';
const PROXY_EVENT_LOG = '/tmp/event_proxy.py.log';
const PROXY_FRAME_LOG = '/tmp/frame_proxy.py.log';

function rosEnvCmd(userCmd) {
  // Build a bash -lc string that sources ROS + workspace and runs user command
  const setupRos = `/opt/ros/${process.env.ROS_DISTRO || 'jazzy'}/setup.bash`;
  return `set -e; source '${setupRos}' >/dev/null 2>&1 || true; if [ -f '${WS_ROOT}/install/setup.bash' ]; then . '${WS_ROOT}/install/setup.bash'; fi; export PYTHONPATH='${VENV_SITE}:$PYTHONPATH'; ${userCmd}`;
}

function rosEnvCmdBare(userCmd) {
  // Same as rosEnvCmd, but without forcing the UI venv into PYTHONPATH
  const setupRos = `/opt/ros/${process.env.ROS_DISTRO || 'jazzy'}/setup.bash`;
  return `set -e; source '${setupRos}' >/dev/null 2>&1 || true; if [ -f '${WS_ROOT}/install/setup.bash' ]; then . '${WS_ROOT}/install/setup.bash'; fi; ${userCmd}`;
}

function startProc(cmd, opts = {}) {
  const p = spawn('/bin/bash', ['-lc', cmd], { cwd: opts.cwd || WS_ROOT, detached: true, stdio: 'ignore' });
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
    try { process.kill(pid); } catch {}
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
  const tick = () => {
    try {
      const txt = fs.readFileSync(LAST_EVENT_FILE, 'utf8');
      const obj = JSON.parse(txt);
      res.write('event: zkml\n');
      res.write(`data: ${JSON.stringify(obj)}\n\n`);
    } catch {}
  };
  tick();
  const timer = setInterval(tick, 1000);
  const stop = () => clearInterval(timer);
  req.on('close', stop); req.on('end', stop);
}
app.get('/events', sseEvents);
app.get('/api/events', sseEvents);

// Polling fallback: return one latest /zkml/event sample
app.get('/api/last_event', (req, res) => {
  try {
    const txt = fs.readFileSync(LAST_EVENT_FILE, 'utf8');
    const obj = JSON.parse(txt);
    return res.json(obj);
  } catch {
    return res.status(204).end();
  }
});

// STOP lock indicator
app.get('/api/stop_state', (req, res) => {
  try {
    const val = fs.readFileSync(STOP_FILE, 'utf8').trim();
    if (val === 'true' || val === 'false') return res.json({ stop: val === 'true' });
  } catch {}
  return res.json({ stop: null });
});

// Latest camera frame as PNG
app.get('/api/frame.png', (req, res) => {
  try {
    const png = fs.readFileSync(FRAME_FILE);
    res.setHeader('Content-Type', 'image/png');
    res.end(png);
  } catch {
    res.status(204).end();
  }
});

// Static frontend (mounted last)
app.use('/', express.static(path.join(__dirname, 'public')));

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
  const pid = startProc(cmd);
  writePid('camera', pid);
  return res.json({ started: true, pid });
});
app.post('/stop/camera', (req, res) => { stopProc('camera'); return res.json({ stopped: true }); });

// Guard (includes twist_mux via launch)
app.post('/start/guard', (req, res) => {
  if (isRunning('guard')) return res.json({ started: false, reason: 'already running' });
  const mode = (req.body && req.body.mode) || 'http';
  const args = mode === 'cli' ? "--ros-args -p verifier_mode:=cli" : '';
  const cmd = rosEnvCmd(`ros2 launch zkml_guard zkml_guard_proof.launch.py ${args}`);
  const pid = startProc(cmd);
  writePid('guard', pid);
  return res.json({ started: true, pid });
});
app.post('/stop/guard', (req, res) => { stopProc('guard'); return res.json({ stopped: true }); });

// Teleop (demo publisher)
app.post('/start/teleop', (req, res) => {
  if (isRunning('teleop')) return res.json({ started: false, reason: 'already running' });
  const cmd = rosEnvCmd("ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.15}}' -r 5");
  const pid = startProc(cmd);
  writePid('teleop', pid);
  return res.json({ started: true, pid });
});
app.post('/stop/teleop', (req, res) => { stopProc('teleop'); return res.json({ stopped: true }); });

// MCAP Record
app.post('/start/bag', (req, res) => {
  if (isRunning('bag')) return res.json({ started: false, reason: 'already running' });
  const cmd = rosEnvCmd('ros2 bag record -a -s mcap');
  const pid = startProc(cmd);
  writePid('bag', pid);
  return res.json({ started: true, pid });
});
app.post('/stop/bag', (req, res) => { stopProc('bag'); return res.json({ stopped: true }); });

// One-shot full demo start/stop (GET for easy browser clicks)
app.get('/start/full', async (req, res) => {
  const mode = (req.query.mode || 'http').toString();
  const burger = !!(req.query.burger && req.query.burger !== '0' && req.query.burger !== 'false');
  const record = !!(req.query.record && req.query.record !== '0' && req.query.record !== 'false');
  try {
    if (mode === 'http' && !isRunning('verifier')) {
      const pid = startProc(`cd '${VERIFIER_DIR}'; node server.js`, { cwd: VERIFIER_DIR });
      writePid('verifier', pid);
    }
    if (!isRunning('camera')) {
      const cmdCam = rosEnvCmd(`ros2 run image_tools cam2image ${burger ? "--ros-args -p burger_mode:=true" : ''}`);
      writePid('camera', startProc(cmdCam));
    }
    if (!isRunning('guard')) {
      const args = mode === 'cli' ? "--ros-args -p verifier_mode:=cli" : '';
      const cmdGuard = rosEnvCmd(`ros2 launch zkml_guard zkml_guard_proof.launch.py ${args}`);
      writePid('guard', startProc(cmdGuard));
    }
    if (!isRunning('teleop')) {
      const cmdTele = rosEnvCmd("ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.15}}' -r 5");
      writePid('teleop', startProc(cmdTele));
    }
    if (record && !isRunning('bag')) {
      const cmdBag = rosEnvCmd('ros2 bag record -a -s mcap');
      writePid('bag', startProc(cmdBag));
    }
    res.json({ ok: true, mode, burger, record, status: {
      verifier: isRunning('verifier'), camera: isRunning('camera'), guard: isRunning('guard'), teleop: isRunning('teleop'), bag: isRunning('bag')
    }});
  } catch (e) {
    res.status(500).json({ ok: false, error: e.message });
  }
});

app.get('/stop/all', (req, res) => {
  ['bag','teleop','guard','camera','verifier'].forEach(stopProc);
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

  const cmd = rosEnvCmdBare(`python3 '${py}' 2>&1`);
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

// No background proxy required; CLI-based polling is used
