#!/usr/bin/env python3
import io
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from PIL import Image as PILImage


class OneShotSnapshot(Node):
    def __init__(self):
        super().__init__('snapshot_once')
        topic = self.declare_parameter('topic', '/image').get_parameter_value().string_value or '/image'
        self._done = False
        self.sub = self.create_subscription(Image, topic, self.cb, 10)

    def cb(self, msg: Image):
        if self._done:
            return
        try:
            h, w = msg.height, msg.width
            step = msg.step
            data = np.frombuffer(msg.data, dtype=np.uint8)
            if msg.encoding in ('rgb8', 'rgb_8'):
                arr = data.reshape((h, step))[:, : w * 3].reshape((h, w, 3))
            elif msg.encoding in ('bgr8', 'bgr_8'):
                arr = data.reshape((h, step))[:, : w * 3].reshape((h, w, 3))[:, :, ::-1]
            elif msg.encoding in ('mono8', '8UC1'):
                arr = data.reshape((h, step))[:, : w].reshape((h, w))
                arr = np.stack([arr, arr, arr], axis=-1)
            else:
                arr = data.reshape((h, w, -1))
                if arr.shape[2] >= 3:
                    arr = arr[:, :, :3]
                else:
                    raise ValueError('unsupported image encoding')
            img = PILImage.fromarray(arr)
            buf = io.BytesIO()
            img.save(buf, format='PNG')
            sys.stdout.buffer.write(buf.getvalue())
            sys.stdout.flush()
            self._done = True
            rclpy.shutdown()
        except Exception:
            # swallow errors; will time out on server side
            pass


def main():
    rclpy.init()
    node = OneShotSnapshot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()

