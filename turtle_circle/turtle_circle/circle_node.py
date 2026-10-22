#!/usr/bin/env python3
"""Drive a turtlesim turtle along a circle.

The motion is fully described by two ROS 2 parameters:

* ``period``    -- seconds needed for one full revolution (controls angular speed)
* ``amplitude`` -- linear speed of the turtle in m/s (controls how "big" the circle is)

From those, the node derives the angular velocity and the resulting circle radius:

    omega  = 2 * pi / period          [rad/s]
    radius = amplitude / omega = amplitude * period / (2 * pi)

The parameters can be changed at runtime (e.g. ``ros2 param set`` or rqt_reconfigure);
the node picks the new value up on the next timer tick.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from turtlesim.msg import Pose


class TurtleCircle(Node):
    def __init__(self) -> None:
        super().__init__('turtle_circle')

        # ---- ROS 2 parameters -------------------------------------------------
        self.declare_parameter('period', 8.0)        # s  -> one full circle
        self.declare_parameter('amplitude', 2.0)     # m/s -> linear speed
        self.declare_parameter('publish_rate', 20.0)  # Hz
        self.declare_parameter('cmd_topic', '/turtle1/cmd_vel')
        self.declare_parameter('pose_topic', '/turtle1/pose')

        self.period = float(self.get_parameter('period').value)
        self.amplitude = float(self.get_parameter('amplitude').value)
        rate = float(self.get_parameter('publish_rate').value)
        cmd_topic = self.get_parameter('cmd_topic').value
        pose_topic = self.get_parameter('pose_topic').value

        # ---- publishers / subscribers ----------------------------------------
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(Pose, pose_topic, self._on_pose, 10)
        self.create_timer(1.0 / rate, self._on_timer)

        # allow live re-configuration of period / amplitude
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.latest_pose = None
        self._log_config()

    # ------------------------------------------------------------------ helpers
    @property
    def omega(self) -> float:
        return 2.0 * math.pi / self.period

    @property
    def radius(self) -> float:
        return self.amplitude / self.omega

    def _log_config(self) -> None:
        self.get_logger().info(
            'circle config: period=%.3f s, amplitude=%.3f m/s '
            '-> omega=%.3f rad/s, radius=%.3f m'
            % (self.period, self.amplitude, self.omega, self.radius)
        )

    # ------------------------------------------------------------------ callbacks
    def _on_pose(self, msg: Pose) -> None:
        self.latest_pose = msg

    def _on_set_parameters(self, params):
        for p in params:
            if p.name == 'period' and p.value is not None:
                self.period = float(p.value)
            elif p.name == 'amplitude' and p.value is not None:
                self.amplitude = float(p.value)
        self._log_config()
        return SetParametersResult(successful=True)

    def _on_timer(self) -> None:
        msg = Twist()
        msg.linear.x = self.amplitude      # 幅值 -> forward speed
        msg.angular.z = self.omega         # 周期 -> turning rate
        self.cmd_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TurtleCircle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
