#!/usr/bin/env python3
import json
import os
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String as StringMsg
from std_msgs.msg import Bool as BoolMsg

LAST_EVENT = '/tmp/rdemo-last-event.json'
STOP_FILE = '/tmp/rdemo-stop.txt'


class EventProxy(Node):
    def __init__(self):
        super().__init__('rdemo_event_proxy')
        self.create_subscription(StringMsg, '/zkml/event', self.on_event, 10)
        self.create_subscription(BoolMsg, '/zkml/stop', self.on_stop, 10)
        os.makedirs('/tmp', exist_ok=True)
        self.get_logger().info(f'EventProxy started: subscribed to /zkml/event and /zkml/stop')
        self.get_logger().info(f'Writing event data to {LAST_EVENT}')
        self.get_logger().info(f'Writing stop state to {STOP_FILE}')

    def on_event(self, msg: StringMsg):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'Failed to parse event JSON: {e}')
            data = {'raw': msg.data}
        try:
            with open(LAST_EVENT, 'w') as f:
                json.dump(data, f)
            self.get_logger().debug(f'Event written: top1={data.get("top1_label", "?")} proof={data.get("proof_verified", "?")}')
        except Exception as e:
            self.get_logger().error(f'Failed to write event file: {e}')

    def on_stop(self, msg: BoolMsg):
        try:
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

