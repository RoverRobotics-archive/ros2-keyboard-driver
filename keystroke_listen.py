#!/usr/bin/env python3
from pynput import keyboard

import rclpy
from std_msgs.msg import Char, UInt32

pub_char = None
node = None
pub_code = None
exit_on_esc = True
listener = None


def on_release(_):
    pass


def on_press(key):
    global node, pub_char
    try:
        if isinstance(key, keyboard.KeyCode):
            node.get_logger().info('pressed ' + str(key))
            pub_char.publish(pub_char.msg_type(data=key.char))
        else:
            node.get_logger().info('pressed {} ({})'.format(key, key.value.vk))
            pub_code.publish(pub_code.msg_type(data=key.value.vk))
    except Exception as e:
        node.get_logger().error(str(e))
        raise

    if key == keyboard.Key.esc and exit_on_esc:
        node.get_logger().info('stopping listener')
        raise keyboard.Listener.StopException


def main(args=None):
    rclpy.init(args=args)
    global node, pub_char, pub_code, listener
    node = rclpy.create_node('keystroke_listen')
    pub_char = node.create_publisher(Char, 'charkey_pressed')
    pub_code = node.create_publisher(UInt32, 'key_pressed')
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        while rclpy.ok() and listener.running:
            rclpy.spin_once(node, timeout_sec=0.1)


if __name__ == '__main__':
    main()
