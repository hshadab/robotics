# Robotics UI Changelog

## [2025-10-16] - UI State Management & Snapshot Reliability

### Fixed

#### Detection Display State Management
- **Issue**: Detection display showed stale inference results after stopping demo or during proof generation
  - Old detections (e.g., "parachute 3.7%") persisted when pressing Stop then Start
  - Detection data not cleared between demo sessions
- **Solution**: Implemented comprehensive state clearing in both frontend and backend
  - **Backend** (`server.js:304-318`): `/stop/all` endpoint now deletes `/tmp/rdemo-last-event.json`
  - **Frontend** (`demo.html:1100-1113`): Check guard status first, clear `lastEvent` and UI immediately when stopped
  - Detection clears immediately when guard stops (lines 1107-1112)
  - Detection clears during proof generation (when `predicate_met && !proof_ms`)
  - Detection shows continuously when guard running (regardless of threshold)
  - Detection resumes after inference pause completes (lines 896-922)
- **Result**: Clean slate on every Start, no stale data from previous sessions

#### Snapshot Capture Reliability
- **Issue**: ~33% of verified proofs missing snapshot thumbnails due to race condition
  - `frame_proxy.py` throttles frame writes (every 5th frame)
  - `event_proxy.py` tried to copy frame immediately when proof verified
  - Frame file sometimes didn't exist yet
- **Solution**: Added retry mechanism in `event_proxy.py`
  - Wait up to 500ms for frame file (5 attempts × 100ms) (lines 62-68)
  - Added `import time` for sleep functionality (line 6)
  - Log warning if frame unavailable after retries (line 84)
- **Result**: All verified proofs now consistently include snapshots

### Changed

#### Configuration Updates
- Threshold increased from 25% to 40% (`zkml_guard.params.yaml:12`)
- Inference rate changed to 2000ms / 2 seconds (`zkml_guard.params.yaml:13`)

**Note**: Config changes require guard service restart to take effect

### Technical Details

**Files Modified:**
- `/home/hshadab/robotics/tools/robotics-ui/public/demo.html`
  - Lines 896-922: `updateDetection()` function with state machine logic
  - Lines 1131-1136: Clear `lastEvent` when guard stops
  - Line 1148: Update detection on guard status change

- `/home/hshadab/robotics/tools/robotics-ui/event_proxy.py`
  - Line 6: Added `import time`
  - Lines 62-68: Frame file retry loop (5 attempts × 100ms)
  - Lines 70-84: Snapshot saving with existence check
  - Line 84: Warning log for missing frame file

**State Machine Logic:**

```
Guard Stopped    → Clear display, set lastEvent = null
Guard Running    → Show all detections continuously
  ├─ No threshold met     → Show detection
  ├─ Threshold met        → Clear display (proof generating)
  └─ Proof complete       → Resume detections after pause
```

**Retry Mechanism:**

```python
frame_wait_attempts = 5
for attempt in range(frame_wait_attempts):
    if os.path.exists(FRAME_FILE):
        break
    if attempt < frame_wait_attempts - 1:
        time.sleep(0.1)  # 100ms between attempts
```

This handles the timing mismatch between:
- `frame_proxy.py`: Writes every 5th frame (~150-200ms gaps)
- `event_proxy.py`: Needs frame immediately when proof verified

## Previous Changes

See main project README for earlier history.
