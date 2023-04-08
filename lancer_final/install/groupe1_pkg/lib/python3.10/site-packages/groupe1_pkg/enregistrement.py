import rclpy
import os
import sys
import select
import numpy as np
from rclpy.node import Node
import time
import matplotlib.pyplot as plt
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty


##########################################
### Noeud d'observation d'arrière plan ###
##########################################

Angles = [[], [], [], [], []]
Vitesses = [[], [], [], [], []]
Efforts = [[], [], [], [], []]
Temps = []


class Signal(Node):
    def __init__(self):
        super().__init__('signal')
        self.subscriber = self.create_subscription(
            Float32, 'etats', self.appel, 10)
        self.subscriber
        self.flag = False

    def appel(self, msg):
        self.objet = msg.data
        self.get_logger().info('enregistrement demandé pendant {}'.format(msg.data))
        self.flag = True


class Observation(Node):
    global Angles, Vitesses, Efforts, Temps
    qos = QoSProfile(depth=10)

    def __init__(self):
        super().__init__('observation')

        # Create joint_states subscriber
        self.joint_state_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            self.qos)
        self.joint_state_subscription

    def joint_state_callback(self, msg):
        """
        Angles[0].append(msg.position[0])
        Angles[1].append(msg.position[1])
        Angles[2].append(msg.position[2])
        Angles[3].append(msg.position[3])
        Angles[4].append(msg.position[4])
        """
        Vitesses[0].append(msg.velocity[0])
        Vitesses[1].append(msg.velocity[1])
        Vitesses[2].append(msg.velocity[2])
        Vitesses[3].append(msg.velocity[3])
        Vitesses[4].append(msg.position[4])

        """Efforts[0].append(msg.effort[0])
        Efforts[1].append(msg.effort[1])
        Efforts[2].append(msg.effort[2])
        Efforts[3].append(msg.effort[3])
        Efforts[4].append(msg.effort[4])"""


def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    global Angles, Vitesses, Efforts, Temps

    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    signal = Signal()
    observateur = Observation()
    while True:

        rclpy.spin_once(signal)
        if signal.flag:
            origine = time.time()
            Angles = [[], [], [], [], []]
            Vitesses = [[], [], [], [], []]
            Efforts = [[], [], [], [], []]
            Temps = []
            delta = 0

            while delta <= signal.objet:
                rclpy.spin_once(observateur)
                Temps.append(delta)
                delta = time.time() - origine

            plt.subplot(5, 1, 1)
            plt.plot(Temps, Vitesses[0])
            plt.subplot(5, 1, 2)
            plt.plot(Temps, Vitesses[1])
            plt.subplot(5, 1, 3)
            plt.plot(Temps, Vitesses[2])
            plt.subplot(5, 1, 4)
            plt.plot(Temps, Vitesses[3])
            plt.subplot(5, 1, 5)
            plt.plot(Temps, Vitesses[4])

            plt.show()

            signal.flag = False

        key_value = get_key(settings)
        if key_value in {"c"}:
            plt.show()
            break

    observateur.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
