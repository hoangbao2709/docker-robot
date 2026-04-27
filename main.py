#!/usr/bin/env python3
# coding=utf-8

import threading

import rclpy

from ros_node import LiveMapWeb
from web_server import start_web_server


def main():
    threading.Thread(target=start_web_server, daemon=True).start()

    rclpy.init()
    node = LiveMapWeb()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
