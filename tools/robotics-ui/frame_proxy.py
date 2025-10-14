#!/usr/bin/env python3
import io
import os
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from PIL import Image as PILImage

FRAME_FILE = '/tmp/rdemo-frame.png'


class FrameProxy(Node):
    def __init__(self):
        super().__init__('rdemo_frame_proxy')
        topic = self.declare_parameter('topic', '/image').get_parameter_value().string_value or '/image'
        self.sub = self.create_subscription(Image, topic, self.cb, 10)
        os.makedirs('/tmp', exist_ok=True)
        self._throttle = 0
        self._frame_count = 0
        self.get_logger().info(f'FrameProxy started: subscribed to {topic}')
        self.get_logger().info(f'Writing frames to {FRAME_FILE} (throttled to every 5th frame)')

    def cb(self, msg: Image):
        self._frame_count += 1
        try:
            # simple throttle: write every ~5th frame
            self._throttle = (self._throttle + 1) % 5
            if self._throttle != 0:
                return
            h, w, step = msg.height, msg.width, msg.step
            data = np.frombuffer(msg.data, dtype=np.uint8)
            if msg.encoding in ('rgb8', 'rgb_8'):
                arr = data.reshape((h, step))[:, : w * 3].reshape((h, w, 3))
            elif msg.encoding in ('bgr8', 'bgr_8'):
                arr = data.reshape((h, step))[:, : w * 3].reshape((h, w, 3))[:, :, ::-1]
            elif msg.encoding in ('mono8', '8UC1'):
                gray = data.reshape((h, step))[:, : w].reshape((h, w))
                arr = np.stack([gray, gray, gray], axis=-1)
            else:
                self.get_logger().warn(f'Unsupported encoding: {msg.encoding}, trying fallback')
                arr = data.reshape((h, w, -1))
                if arr.shape[2] >= 3:
                    arr = arr[:, :, :3]
                else:
                    self.get_logger().error(f'Cannot handle encoding {msg.encoding} with shape {arr.shape}')
                    return
            img = PILImage.fromarray(arr)
            buf = io.BytesIO()
            img.save(buf, format='PNG')
            with open(FRAME_FILE, 'wb') as f:
                f.write(buf.getvalue())
            if self._frame_count % 50 == 0:
                self.get_logger().info(f'Processed {self._frame_count} frames ({h}x{w} {msg.encoding})')
        except Exception as e:
            self.get_logger().error(f'Failed to process frame: {e}')


def main():
    try:
        rclpy.init()
        node = FrameProxy()
        print('[frame_proxy] Started successfully', file=sys.stderr, flush=True)
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('[frame_proxy] Interrupted', file=sys.stderr, flush=True)
    except Exception as e:
        print(f'[frame_proxy] Fatal error: {e}', file=sys.stderr, flush=True)
        raise
    finally:
        try:
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()

