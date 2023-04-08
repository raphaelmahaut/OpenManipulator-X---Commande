import sys
from turtle import position
import numpy as np
import time

from open_manipulator_msgs.srv import SetKinematicsPose, SetJointPosition
from geometry_msgs.msg import Pose2D
import rclpy
from rclpy.node import Node

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscriber = self.create_subscription(Pose2D, 'camera', self.appel_cam, 10)
        self.subscriber

        self.objet = Pose2D()
        self.objet.x, self.objet.y = (0., 0.09)

    def appel_cam(self, msg):
        self.objet = msg
        self.get_logger().info('message récupéré')


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


def structure(name):
    S = {'tas': [(0., 0.09, 0.07, 3.14, 0.01), (0., 0.09, 0.028, 3.14, -0.01), (0., 0.09, 0.07, 3.14, -0.01),
                 (0.2, 0., 0.07, 3.14, -0.01), (0.2, 0., 0.03,
                                                3.14, 0.01), (0.2, 0., 0.07, 3.14, 0.01),
                 (0., -0.09, 0.07, 3.14, 0.01), (0., -0.09, 0.028,
                                                 3.14, -0.01), (0., -0.09, 0.07, 3.14, -0.01),
                 (0.2, 0., 0.07, 3.14, -0.01), (0.2, 0., 0.037,
                                                3.14, 0.01), (0.2, 0., 0.07, 3.14, 0.01),
                 ],'dolmen': [(0., 0.09, 0.07, 3.14, 0.01), (0., 0.09, 0.031, 3.14, -0.01), (0., 0.09, 0.07, 3.14, -0.01),
                               (0.25, 0., 0.08, -0.1, -0.01), (0.25, 0.,
                                                             0.062, -0.1, 0.01), (0.18, 0., 0.08, 3.14, 0.01),
                               (0., -0.09, 0.07, 3.14, 0.01), (0., -0.09, 0.027,
                                                               3.14, -0.01), (0., -0.09, 0.07, 3.14, -0.01),
                               (0.21, 0., 0.08, 0., -0.01), (0.204, 0.,
                                                             0.058, 0., 0.01), (0.1, 0., 0.08, 3.14, 0.01),
                               (0., 0.09, 0.07, 3.14, 0.01), (0., 0.09, 0.027,
                                                              3.14, -0.01), (0., 0.09, 0.15, 3.14, -0.01),
                               (0.1, 0.05, 0.07, 3.14, -0.01), (0.1, 0.05, 0.027, 3.14,
                                                                0.01), (0.1, 0.05, 0.018, 1., -0.01), (0.1, 0.05, 0.16, 1., -0.01),
                               (0.24, 0., 0.16, 1., -0.01), (0.24, 0., 0.152, 1., 0.01), (0.1, 0., 0.15,
                                                                                        1., 0.01), (0., 0.09, 0.15, 1., 0.01),  (0., 0.09, 0.03, 1., 0.01),
                               ], 'test' : [(0.2, 0., 0.2, 0., 0.01), (0.076, 0.060, 0.028, 3.14, 0.01), (0.2, 0., 0.2, 0., 0.01), (0.197, -0.124, 0.028, 3.14, 0.01)]
                               , 'demande' : [(0., 0.09, 0.07, 3.14, 0.01)]}
    return S[name]


def prise(grip_client, o=-0.01, tps=4.0):

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


def correc(x, y):
    alpha = 2*np.arctan(y/(x + np.sqrt(x**2 + y**2)))
    delt = np.sqrt(x**2 + y**2)*np.tan((0.16/3.14)*(alpha - 3.14/2))
    return x + delt*np.sin(alpha), y - delt*np.cos(alpha)


def main(args=None):
    ouverture = 0.01
    tinit = time.time()
    rclpy.init(args=args)

    task_space_cmd_client = TaskSpaceCmdClient()
    minimal_subscriber = MinimalSubscriber()

    positions = structure('demande')

    prise(task_space_cmd_client, ouverture)
    déplacement(task_space_cmd_client, 0.2, 0., 0.2, 0.)

    # pince fermée pour -0.01 ouverte pour 0.01
    # pince verticale pour 3.14

    while time.time() - tinit < 45:
        z = 0.03
        w = 3.14
        o = 0.01
        rclpy.spin_once(minimal_subscriber)
        x, y = minimal_subscriber.objet.x, minimal_subscriber.objet.y
        print(x, y)
        x, _ = correc(x, y)
        
        task_space_cmd_client.get_logger().info(
            '{}'.format(x))
        task_space_cmd_client.get_logger().info(
            '{}'.format(y))

        déplacement(task_space_cmd_client, x, y, z, w)
        time.sleep(2.1)
        if o != ouverture:
            prise(task_space_cmd_client, o)
            time.sleep(4)
            ouverture = o

    task_space_cmd_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
