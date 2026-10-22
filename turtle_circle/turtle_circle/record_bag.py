#!/usr/bin/env python3
"""Small wrapper node that records a rosbag2 while the launch is alive.

`ros2 bag record` must receive SIGINT (not SIGKILL) so that it can finalize
``metadata.yaml``. Running it as a child of a node lets us forward the signal
on shutdown, which keeps the bag valid.
"""

import signal
import subprocess
import sys

import rclpy
from rclpy.node import Node

TOPICS = ['/turtle1/cmd_vel', '/turtle1/pose']


class RecordBag(Node):
    def __init__(self) -> None:
        super().__init__('record_bag')
        self.declare_parameter('bag_dir', 'turtle_bag')
        bag_dir = self.get_parameter('bag_dir').value

        self.get_logger().info('recording %s -> %s' % (', '.join(TOPICS), bag_dir))
        self.proc = subprocess.Popen(
            ['ros2', 'bag', 'record', '-o', bag_dir] + TOPICS,
            stdout=sys.stdout, stderr=sys.stderr,
        )

    def stop(self) -> None:
        if self.proc is not None and self.proc.poll() is None:
            self.get_logger().info('stopping bag record (SIGINT) ...')
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.proc.kill()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RecordBag()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
