import sys

# pynput throws an error if we import it before $DISPLAY is set on LINUX

if sys.platform not in ('darwin', 'win32'):
    import os

    os.environ.setdefault('DISPLAY', ':0')

from pynput import keyboard

import rclpy
import std_msgs.msg


class KeystrokeListen:
    def __init__(self, name=None):
        self.node = rclpy.create_node(name or type(self).__name__)
        self.pub_glyph = self.node.create_publisher(std_msgs.msg.String, 'glyphkey_pressed')
        # todo: when ROS2 supports Enums, use them: https://github.com/ros2/rosidl/issues/260
        self.pub_code = self.node.create_publisher(std_msgs.msg.UInt32, 'key_pressed')
        self.exit_on_esc = self.node.declare_parameter('exit on esc', False).value
        if self.exit_on_esc:
            self.logger.info('To end this node, press the escape key')

    def spin(self):
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            while rclpy.ok() and listener.running:
                rclpy.spin_once(self.node, timeout_sec=0.1)

    @property
    def logger(self):
        return self.node.get_logger()

    def on_release(self, key):
        # todo: implement this
        pass

    def on_press(self, key):
        try:
            char = getattr(key, 'char', None)
            if isinstance(char, str):
                self.logger.info('pressed ' + char)
                self.pub_glyph.publish(self.pub_glyph.msg_type(data=char))
            else:
                try:
                    # known keys like spacebar, ctrl
                    name = key.name
                    vk = key.value.vk
                except AttributeError:
                    # unknown keys like headphones skip song button
                    name = 'UNKNOWN'
                    vk = key.vk
                self.logger.info('pressed {} ({})'.format(name, vk))
                # todo: These values are not cross-platform. When ROS2 supports Enums, use them instead
                self.pub_code.publish(self.pub_code.msg_type(data=vk))
        except Exception as e:
            self.logger.error(str(e))
            raise

        if key == keyboard.Key.esc and self.exit_on_esc:
            self.logger.info('stopping listener')
            raise keyboard.Listener.StopException


def main(args=None):
    rclpy.init(args=args)
    KeystrokeListen().spin()


if __name__ == '__main__':
    main()
