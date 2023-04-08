import rclpy
import numpy as np
from rclpy.node import Node
from .modules.image_processing import cameraVision, cameraVision_calibrate
from geometry_msgs.msg import Pose, PoseArray


class Campub(Node):

    def __init__(self):
        super().__init__('envoi_cam')
        self.publisher_ = self.create_publisher(PoseArray, 'camera', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        global parameters
        min_area, max_area, threshold1, threshold2, crop, thickness, color_divider, robot_line, robot_column, distance_ratio_x, distance_ratio_y = parameters
        mtx = np.array([[426, 0, 332], [0, 426, 198], [0, 0, 1]])
        distorsion_coefficients = np.array(
            [[-0.266, 0.07, -0.006, 0.0008, 0.0058]])

        msg = PoseArray()

        Planches = cameraVision(2, mtx, distorsion_coefficients, min_area, max_area, threshold1, threshold2,
                                crop, thickness, color_divider, robot_line, robot_column, distance_ratio_x, distance_ratio_y)

        for pl in Planches:
            plank = Pose()
            plank.position.x, plank.position.y, plank.position.z = pl[0][0], pl[0][1], 0.
            if pl[5]:
                plank.orientation.w = 1.
            else:
                plank.orientation.w = 0.
            msg.poses.append(plank)

        for pl in msg.poses:
            print(pl.position.x, pl.position.y, pl.orientation.w)

        self.publisher_.publish(msg)
        self.get_logger().info('envoi des données')
        self.i += 1


def main(args=None):
    global parameters
    rclpy.init(args=args)

    campub = Campub()

    mtx = np.array([[426, 0, 332], [0, 426, 198], [0, 0, 1]])
    distorsion_coefficients = np.array(
        [[-0.266, 0.07, -0.006, 0.0008, 0.0058]])
    parameters = cameraVision_calibrate(2, mtx, distorsion_coefficients)

    rclpy.spin(campub)

    campub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
