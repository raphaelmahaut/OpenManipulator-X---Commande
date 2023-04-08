import sys
import numpy as np

from open_manipulator_msgs.srv import SetKinematicsPose, SetJointPosition
import rclpy
from rclpy.node import Node


class TaskSpaceCmdClient(Node):

    def __init__(self):
        super().__init__('task_space_cmd_client')  # node name
        self.cli = self.create_client(SetKinematicsPose,
                                      '/goal_task_space_path')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetKinematicsPose.Request()

    def send_request(self, x, y, z, w=np.pi/2):
        self.req.end_effector_name = "gripper"
        self.req.kinematics_pose.pose.position.x = 0.15  # meters
        self.req.kinematics_pose.pose.position.y = 0.
        self.req.kinematics_pose.pose.position.z = 0.1
        self.req.kinematics_pose.pose.orientation.x = 0.  # meters
        self.req.kinematics_pose.pose.orientation.y = 1.
        self.req.kinematics_pose.pose.orientation.z = 0.
        self.req.kinematics_pose.pose.orientation.w = w
        self.req.path_time = 2.0
        self.future = self.cli.call_async(self.req)


def main(args=None):
    

    rclpy.init(args=args)

    task_space_cmd_client = TaskSpaceCmdClient()
    task_space_cmd_client.send_request(1., 1., 1.)

    while rclpy.ok():
        rclpy.spin(task_space_cmd_client)
        if task_space_cmd_client.future.done():
            try:
                response = task_space_cmd_client.future.result()
            except Exception as e:
                task_space_cmd_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                task_space_cmd_client.get_logger().info(
                    'The request has been accepted')
            break

    task_space_cmd_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
