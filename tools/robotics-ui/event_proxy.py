#!/usr/bin/env python3
import json
import os
import sys
import shutil
import time
import rclpy
try:
    import requests  # type: ignore
except Exception:
    requests = None
from rclpy.node import Node
from std_msgs.msg import String as StringMsg
from std_msgs.msg import Bool as BoolMsg

LAST_EVENT = '/tmp/rdemo-last-event.json'
STOP_FILE = '/tmp/rdemo-stop.txt'
VERIFIED_PROOFS_HISTORY = '/tmp/rdemo-verified-proofs.json'
FRAME_FILE = '/tmp/rdemo-frame.png'
SNAPSHOTS_DIR = '/tmp/rdemo-snapshots'


class EventProxy(Node):
    def __init__(self):
        super().__init__('rdemo_event_proxy')
        self.create_subscription(StringMsg, '/zkml/event', self.on_event, 10)
        self.create_subscription(BoolMsg, '/zkml/stop', self.on_stop, 10)
        self.server_url = os.environ.get('RD_UI_SERVER', 'http://localhost:9200')
        os.makedirs('/tmp', exist_ok=True)
        os.makedirs(SNAPSHOTS_DIR, exist_ok=True)

        # Initialize verified proofs history file
        if not os.path.exists(VERIFIED_PROOFS_HISTORY):
            with open(VERIFIED_PROOFS_HISTORY, 'w') as f:
                json.dump([], f)

        self.get_logger().info(f'EventProxy started: subscribed to /zkml/event and /zkml/stop')
        self.get_logger().info(f'Writing event data to {LAST_EVENT}')
        self.get_logger().info(f'Writing stop state to {STOP_FILE}')
        self.get_logger().info(f'Writing verified proofs history to {VERIFIED_PROOFS_HISTORY}')
        self.get_logger().info(f'Saving snapshots to {SNAPSHOTS_DIR}')

    def on_event(self, msg: StringMsg):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'Failed to parse event JSON: {e}')
            data = {'raw': msg.data}
        try:
            # Push directly to UI server (preferred)
            try:
                if requests is None:
                    raise RuntimeError('requests not available')
                requests.post(f'{self.server_url}/internal/event', json=data, timeout=0.5)
            except Exception as e:
                # Mirror to disk as fallback
                with open(LAST_EVENT, 'w') as f:
                    json.dump(data, f)

            # If this event has a verified proof, add it to the history
            if data.get('proof_verified') is True and data.get('proof_id'):
                try:
                    # Read existing history
                    with open(VERIFIED_PROOFS_HISTORY, 'r') as f:
                        history = json.load(f)

                    # Check if this proof_id is already in history (avoid duplicates)
                    proof_ids = {item.get('proof_id') for item in history}
                    if data.get('proof_id') not in proof_ids:
                        # Save a snapshot of the current frame
                        snapshot_saved = False

                        # Wait briefly for frame file if it doesn't exist yet (up to 500ms)
                        frame_wait_attempts = 5
                        for attempt in range(frame_wait_attempts):
                            if os.path.exists(FRAME_FILE):
                                break
                            if attempt < frame_wait_attempts - 1:
                                time.sleep(0.1)  # Wait 100ms between attempts

                        if os.path.exists(FRAME_FILE):
                            try:
                                # Use proof_id as filename (strip 0x prefix if present)
                                proof_id = data.get('proof_id')
                                if proof_id.startswith('0x'):
                                    proof_id = proof_id[2:]
                                snapshot_path = os.path.join(SNAPSHOTS_DIR, f'{proof_id}.png')
                                shutil.copy2(FRAME_FILE, snapshot_path)
                                data['snapshot'] = proof_id  # Store just the ID, not full path
                                snapshot_saved = True
                                self.get_logger().info(f'Snapshot saved: {snapshot_path}')
                            except Exception as e:
                                self.get_logger().warn(f'Failed to save snapshot: {e}')
                        else:
                            self.get_logger().warn(f'Frame file not found after {frame_wait_attempts} attempts: {FRAME_FILE}')

                        # Add the new verified proof to the beginning of the list
                        history.insert(0, data)

                        # Keep only the last 50 verified proofs
                        history = history[:50]

                        # Write updated history
                        try:
                            if requests is None:
                                raise RuntimeError('requests not available')
                            # Attempt to refresh server history, else write locally
                            requests.post(f'{self.server_url}/internal/event', json=data, timeout=0.5)
                        except Exception:
                            with open(VERIFIED_PROOFS_HISTORY, 'w') as f:
                                json.dump(history, f)

                        snap_msg = ' (with snapshot)' if snapshot_saved else ''
                        self.get_logger().info(f'Verified proof added to history: {data.get("top1_label")} (ID: {data.get("proof_id")[:12]}...){snap_msg}')
                except Exception as e:
                    self.get_logger().error(f'Failed to update verified proofs history: {e}')

            self.get_logger().debug(f'Event written: top1={data.get("top1_label", "?")} proof={data.get("proof_verified", "?")}')
        except Exception as e:
            self.get_logger().error(f'Failed to write event file: {e}')

    def on_stop(self, msg: BoolMsg):
        try:
            # Push to UI server; fallback to file
            try:
                if requests is None:
                    raise RuntimeError('requests not available')
                requests.post(f'{self.server_url}/internal/stop', json={'stop': bool(msg.data)}, timeout=0.5)
            except Exception:
                with open(STOP_FILE, 'w') as f:
                    f.write('true' if msg.data else 'false')
            self.get_logger().debug(f'Stop state written: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Failed to write stop file: {e}')


def main():
    try:
        rclpy.init()
        node = EventProxy()
        print('[event_proxy] Started successfully', file=sys.stderr, flush=True)
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('[event_proxy] Interrupted', file=sys.stderr, flush=True)
    except Exception as e:
        print(f'[event_proxy] Fatal error: {e}', file=sys.stderr, flush=True)
        raise
    finally:
        try:
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
