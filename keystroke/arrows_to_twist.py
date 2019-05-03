import sys

# pynput throws an error if we import it before $DISPLAY is set on LINUX
if sys.platform not in ('darwin', 'win32'):
    import os

    os.environ.setdefault('DISPLAY', ':0')

from geometry_msgs.msg import Twist, Vector3
from pynput.keyboard import Key
import rclpy
from rclpy.constants import S_TO_NS
from std_msgs.msg import UInt32


class ArrowsToTwist:
    def __init__(self, name=None):
        self.node = rclpy.create_node(name or type(self).__name__)
        self.sub_code = self.node.create_subscription(UInt32, 'key_pressed', self.on_code)
        self.pub_twist = self.node.create_publisher(Twist, 'cmd_vel')
        publish_period_sec = self.get_param('publish_period', float, 0.2)
        self.tmr_twist = self.node.create_timer(publish_period_sec, self.on_tmr)
        self.linear_scale = self.get_param('linear_scale', float, 0.1)
        self.angular_scale = self.get_param('angular_scale', float, 0.2)
        self.current_linear = [0, 0, 0]
        self.current_angular = [0, 0, 0]
        self.logger.info('Arrows to twist node ready - [F1] for usage')

    def get_param(self, name, expected_type, default):
        param = self.node.get_parameter(name)
        value = param.value
        if isinstance(value, expected_type):
            return value
        else:
            self.logger.warn(
                'Parameter {}={} is not a {}. Assuming {}.'.format(
                    param.name,
                    param.value,
                    expected_type,
                    default),
            )
            return default

    def on_tmr(self):
        twist = Twist(
            linear=Vector3(
                x=float(self.current_linear[0]),
                y=float(self.current_linear[1]),
                z=float(self.current_linear[2]),
            ),
            angular=Vector3(
                x=float(self.current_angular[0]),
                y=float(self.current_angular[1]),
                z=float(self.current_angular[2]),
            )
        )
        self.pub_twist.publish(twist)

    @property
    def logger(self):
        return self.node.get_logger()

    def set_publish_period(self, seconds):
        self.tmr_twist.timer_period_ns = int(seconds * S_TO_NS)
        self.tmr_twist.reset()

    def on_code(self, msg):
        if msg.data == Key.f1.value.vk:
            self.logger.info("\n".join([
                'Use the arrow keys to change speed.',
                '[F1] = Show this help',
                '[Up]/[Down] = Forward and backward',
                '[Left]/[Right] = Clockwise and counterclockwise'
                '[Space] = Stop'
            ]))
        elif msg.data == Key.up.value.vk:
            self.logger.info('Increasing forward speed')
            self.current_linear[0] += self.linear_scale
        elif msg.data == Key.down.value.vk:
            self.logger.info('Decreasing forward speed')
            self.current_linear[0] -= self.linear_scale
        elif msg.data == Key.left.value.vk:
            self.logger.info('Yawing left')
            self.current_angular[2] -= self.angular_scale
        elif msg.data == Key.right.value.vk:
            self.logger.info('Yawing right')
            self.current_angular[2] += self.angular_scale
        elif msg.data == Key.space.value.vk:
            self.logger.info('Resetting speed')
            self.current_angular = [0, 0, 0]
            self.current_linear = [0, 0, 0]
        else:
            self.logger.debug('Key ignored: {}'.format(msg.data))

    def spin(self):
        while rclpy.ok():
            rclpy.spin(self.node)


def main(args=None):
    rclpy.init(args=args)
    node = ArrowsToTwist()
    node.spin()


if __name__ == '__main__':
    main()
