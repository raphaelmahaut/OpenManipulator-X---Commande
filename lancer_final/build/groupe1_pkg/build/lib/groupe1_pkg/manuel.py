# +---------+
# | IMPORTS |
# +---------+

import pyzed.sl as sl
import cv2 # pip install opencv-python
import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, acos, asin
import torchvision.transforms as T
import torchvision

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from rclpy.qos import QoSProfile
import sys


class Objectif(Node):

    def __init__(self):
        super().__init__('objectif')
        self.publisher_ = self.create_publisher(Vector3, 'gob', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.x = 0.5
        self.y = 0.
        self.z = 0.8

    def timer_callback(self):
        msg = Vector3()
        msg.x = self.x
        msg.y = self.y
        msg.z = self.z
        self.publisher_.publish(msg)
        self.get_logger().info("demande de lancement: objectif x = {}, y = {}, z = {}".format(self.x, self.y, self.z))


def main():
    rclpy.init(args=None)

    objectif = Objectif()

    objectif.x, objectif.y, objectif.z = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
    rclpy.spin_once(objectif)

    objectif.destroy_node()
    rclpy.shutdown()






if __name__ == "__main__" :
    main()