import sys
import numpy as np
import math as m
import time

from open_manipulator_msgs.srv import SetKinematicsPose, SetJointPosition
import rclpy
from rclpy.node import Node


class TaskSpaceCmdClient(Node):

    def __init__(self):
        super().__init__('task_space_cmd_client')  # node name
        self.cli = self.create_client(SetKinematicsPose,
                                      '/goal_task_space_path')
        self.cli2 = self.create_client(SetJointPosition,
                                       '/goal_tool_control')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetKinematicsPose.Request()

        while not self.cli2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req2 = SetJointPosition.Request()

    def send_request(self, x, y, z, w=np.pi/2, tps=2.0):
        self.req.end_effector_name = "gripper"
        self.req.kinematics_pose.pose.position.x = x  # meters
        self.req.kinematics_pose.pose.position.y = y
        self.req.kinematics_pose.pose.position.z = z
        self.req.kinematics_pose.pose.orientation.x = 0.  # meters
        self.req.kinematics_pose.pose.orientation.y = 1.
        self.req.kinematics_pose.pose.orientation.z = 0.
        self.req.kinematics_pose.pose.orientation.w = w
        self.req.path_time = tps
        self.future = self.cli.call_async(self.req)

    def grip(self, o, tps=2.0):
        self.req2.joint_position.joint_name = {"gripper"}
        self.req2.joint_position.position = {o}
        self.req2.path_time = tps
        self.future2 = self.cli2.call_async(self.req2)


def déplacement(task_space_cmd_client, x, y, z, w, tps=2.0):
    

    task_space_cmd_client = TaskSpaceCmdClient()
    task_space_cmd_client.send_request(x, y, z, w, tps)

    while rclpy.ok():
        rclpy.spin_until_future_complete(
            task_space_cmd_client, task_space_cmd_client.future)
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

    
    


def prise(grip_client, o=-0.01, tps=2.0):

    grip_client.grip(o, tps)

    while rclpy.ok():
        rclpy.spin_until_future_complete(
            grip_client, grip_client.future2)
        if grip_client.future2.done():

            try:
                response = grip_client.future2.result()
            except Exception as e:
                grip_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                grip_client.get_logger().info(
                    'The request has been accepted')
            break


def main(args=None):
    ouverture = 0.01
    rclpy.init(args=args)

    task_space_cmd_client = TaskSpaceCmdClient()
    positions = [(0., 0.09, 0.07, 3.14, 0.01), (0., 0.09, 0.028, 3.14, -0.01), (0., 0.09, 0.07, 3.14, -0.01),
    (0.2, 0., 0.07, 3.14, -0.01), (0.2, 0., 0.03, 3.14, 0.01), (0.2, 0., 0.07, 3.14, 0.01),
    (0., -0.09, 0.07, 3.14, 0.01), (0., -0.09, 0.028, 3.14, -0.01), (0., -0.09, 0.07, 3.14, -0.01),
    (0.2, 0., 0.07, 3.14, -0.01), (0.2, 0., 0.037, 3.14, 0.01), (0.2, 0., 0.07, 3.14, 0.01),
    ]
    
    prise(task_space_cmd_client, ouverture)
    déplacement(task_space_cmd_client, 0.2, 0., 0.2, 0.)
    
    ### pince fermée pour -0.01 ouverte pour 0.01
    ### pince verticale pour 3.14

    

    for x, y, z, w, o in positions:

        task_space_cmd_client.get_logger().info(
                    '{}'.format(x))
        task_space_cmd_client.get_logger().info(
                    '{}'.format(y))                    


        déplacement(task_space_cmd_client, x, y, z, w)
        time.sleep(2)
        if o != ouverture:
            prise(task_space_cmd_client, o)
            time.sleep(2)
            ouverture = o

    task_space_cmd_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
