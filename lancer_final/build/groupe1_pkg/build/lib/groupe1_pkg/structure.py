import sys
import os
from turtle import position
import numpy as np
import math as m
import select
import time
import rclpy

from open_manipulator_msgs.msg import KinematicsPose, OpenManipulatorState
from open_manipulator_msgs.srv import SetJointPosition
from geometry_msgs.msg import Pose2D, Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState


class Information(Node):
    global Pla, pos_act, angle_act
    qos = QoSProfile(depth=10)

    def __init__(self):
        super().__init__('information')

        # Create joint_states subscriber
        self.angle = self.create_subscription(
            JointState,
            'joint_states',
            self.appel_angle,
            self.qos)
        self.angle

        # Create kinematics_pose subscriber
        self.position = self.create_subscription(
            KinematicsPose,
            'kinematics_pose',
            self.appel_position,
            self.qos)
        self.position

        # Create manipulator state subscriber
        self.etat = self.create_subscription(
            OpenManipulatorState,
            'states',
            self.appel_etat,
            self.qos)
        self.etat

    def appel_position(self, msg):
        pos_act[0] = msg.pose.position.x
        pos_act[1] = msg.pose.position.y
        pos_act[2] = msg.pose.position.z
        pos_act[3] = msg.pose.orientation.w
        pos_act[4] = msg.pose.orientation.x
        pos_act[5] = msg.pose.orientation.y
        pos_act[6] = msg.pose.orientation.z

    def appel_angle(self, msg):
        angle_act[0] = msg.position[0]
        angle_act[1] = msg.position[1]
        angle_act[2] = msg.position[2]
        angle_act[3] = msg.position[3]
        angle_act[4] = msg.position[4]

    def appel_etat(self, msg):
        self.action = (msg.open_manipulator_moving_state == 'STOPPED')


class Camtopic(Node):
    def __init__(self):
        super().__init__('appel_camera')
        self.subscriber = self.create_subscription(
            Pose2D, 'camera', self.appel, 10)
        self.subscriber

        self.objet = PoseArray()

    def appel(self, msg):
        self.objet = msg
        self.get_logger().info('message récupéré')


class Mouvement(Node):
    def __init__(self):
        super().__init__('mouvement')  # node name
        self.depl_cli = self.create_client(SetJointPosition,
                                           'goal_joint_space_path')
        self.pince_cli = self.create_client(SetJointPosition,
                                            '/goal_tool_control')

        while not self.pince_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        while not self.depl_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.pince_req = SetJointPosition.Request()
        self.depl_req = SetJointPosition.Request()

    def depl_requete(self, th1, th2, th3, th4, tps=2.0):
        self.depl_req.joint_position.joint_name = [
            "joint1", "joint2", "joint3", "joint4"]
        self.depl_req.joint_position.position = [th1, th2, th3, th4]
        self.depl_req.path_time = tps
        try:
            self.depl_future = self.depl_cli.call_async(self.depl_req)
        except Exception as e:
            self.get_logger().info('problème de requête')

    def prise(self, o, tps=2.0):
        self.pince_req.joint_position.joint_name = {"gripper"}
        self.pince_req.joint_position.position = {o}
        self.pince_req.path_time = tps
        self.pince_future = self.pince_cli.call_async(self.pince_req)


"""def vérif_pl(Pla):
    # vérifie si les planches déjà placée sont bien en place
    return True


def choix_mouv(obj, cible, Pla):
    # cherche le mouvement à faire pour éviter les obstacles et respecter les contraintes du bras
    Poss = ['ligne', 'parabole', 'cadre']

    for méthode in Poss:
        pass
    return méthode"""


def structure(name):
    # positionnement final des différentes planches dans le repère de la structure en unité kappla
    # hor : horizontal, centre d'inertie en bas du kappla, 1
    # vert: vertical, point central en bas, 15
    # lat: latéral, centre d'inertie en bas, 3

    Str = {'tas': [('hor', 0, 0, 0), ('hor', 0, 0, 1), ],
           'dolmen': [('vert', 0, 0, 0), ('vert', 0, 7, 0), ('hor', 0, 3.5, 15), ]}
    return Str[name]


def acquisition(cam, Pla, origine):
    # demande la position des objets à utiliser sur le support
    rclpy.spin_once(cam)
    Obj = cam.objet

    def distance(A, B):
        return np.sqrt((A[0]-B[0])**2 + (A[1]-B[1])**2)

    # tri des objets par rapport à leur distance au bras et à la possibilité de les prendres, suppressions des planches déjà placées
    obj = None
    d = np.inf
    autre = None
    d_autre = np.inf
    for planche in Obj.poses:
        for pla in Pla:
            # retire les planches déjà placées
            if distance((planche.position.x, planche.position.y), (pla[0], pla[1])) < 0.02:
                continue
        if planche.prenable:
            if distance((planche.position.x, planche.position.y), origine) < d:
                obj = planche
                d = distance((planche.position.x, planche.position.y), origine)
        else:
            if distance((planche.position.x, planche.position.y), origine) < d_autre:
                autre = planche
                d_autre = distance(
                    (planche.position.x, planche.position.y), origine)

    if not obj:
        return (autre.position.x, autre.position.y, autre.position.z)

    """def distance(A, B):
        return np.sqrt((A[0]-B[0])**2 + (A[1]-B[1])**2)

    # tri des objets par rapport à leur distance au bras et à la possibilité de les prendres, suppressions des planches déjà placées
    obj = None
    d = np.inf
    autre = None
    d_autre = np.inf
    for planche in Obj:
        for pla in Pla:
            # retire les planches déjà placées
            if distance((planche.centre.x, planche.centre.y), (pla[0], pla[1])) < 0.02:
                continue
        if planche.prenable:
            if distance((planche.centre.x, planche.centre.y), origine) < d:
                obj = planche
                d = distance((planche.centre.x, planche.centre.y), origine)
        else:
            if distance((planche.centre.x, planche.centre.y), origine) < d_autre:
                autre = planche
                d_autre = distance(
                    (planche.centre.x, planche.centre.y), origine)

    if not obj:
        return autre"""
    return (obj.position.x, obj.position.y, obj.position.z)


def intersection(pla, plb):
    # regarde si deux planches s'intersectent
    pass


def reduction(x, y, z, th):

    def angle_triangle(a, b, c):
        return np.arccos((a**2 + b**2 - c**2)/(2*a*b))

    th1 = 2*np.arctan(y/(x + np.sqrt(x**2 + y**2)))
    zb = z + 0.144*np.sin(np.pi*th/(2*90)) - 0.077
    r = np.sqrt(x**2 + y**2) - 0.144*np.cos(np.pi*th/(2*90))
    hp = np.sqrt(zb**2 + r**2)

    if hp > 0.130 + 0.124:
        return "impossible"

    if 0.024 > hp:
        return "collision"

    th2 = np.pi/2 - (0.186 + np.arctan(zb/r) +
                     angle_triangle(0.130, hp, 0.124))
    th3 = (np.pi/2 + 0.186) - angle_triangle(0.130, 0.124, hp)
    th4 = (np.pi * th)/180 - (th2 + th3)

    return th1, th2, th3, th4


def reduction_pol(r, z, th):

    def angle_triangle(a, b, c):
        return np.arccos((a**2 + b**2 - c**2)/(2*a*b))

    zb = z + 0.144*np.sin(np.pi*th/(2*90)) - 0.077
    rb = r - 0.144*np.cos(np.pi*th/(2*90))
    hp = np.sqrt(zb**2 + rb**2)

    if hp < 0.130 + 0.124 - 0.005:
        return "impossible"

    if 0.024 < hp:
        return "collision"

    th2 = np.pi/2 - (0.186 + np.arctan(zb/rb) +
                     angle_triangle(0.130, hp, 0.124))
    th3 = (np.pi/2 + 0.186) - angle_triangle(0.130, 0.124, hp)
    th4 = (np.pi * th)/180 - (th2 + th3)

    return th2, th3, th4


def trajectoire(A, B, hauteur, fonction='ligne', n=100):
    # renvoie un ensemble de points définissant une trajectoire entre deux points
    def ligne(x):
        return 0

    def parabole(x):
        return x*(1-x)

    Courbe = {'ligne': ligne, 'parabole': parabole}

    if fonction == 'cadre':
        P = [A, (A[0], A[1], hauteur), (B[0], B[1], hauteur), B]
        fonction = 'ligne'
        n = n//3 + 1
    else:
        P = [A, B]
    traj = []
    for i in range(len(P) - 1):
        traj += [(P[i][0]*k/n + P[i+1][0]*(1-k/n), P[i][1]*k/n + P[i+1][1]*(1-k/n),
                  P[i][2]*k/n + P[i+1][2]*(1-k/n) + Courbe[fonction](k/n)) for k in range(n+1)]

    return traj


def securite(mouvement, information):
    list_point = trajectoire(pos_act, (0.2, 0., 0.), 0.05, "cadre", 40)
    tps = 1.0/len(list_point)
    while list_point:
        rclpy.spin_once(information)
        if information.action:
            x, y, z = list_point.pop(0)
            mouv(mouvement, x, y, z, 90, tps)
    pass


def mgd(pos_act):
    alpha = 0.186
    beta = 0.105
    z1 = 0.077 + 0.13*np.cos(pos_act[1] + alpha) + 0.124*np.cos(pos_act[1] + pos_act[2] +
                                                                np.pi/2) + 0.144*np.cos(pos_act[1] + pos_act[2] + np.pi/2 + pos_act[3] + beta)
    z2 = 0.077 + 0.13*np.cos(pos_act[1] + alpha) + 0.124*np.cos(pos_act[1] + pos_act[2] +
                                                                np.pi/2) + 0.144*np.cos(pos_act[1] + pos_act[2] + np.pi/2 + pos_act[3] - beta)
    r = 0.13*np.sin(pos_act[1] + alpha) + 0.124*np.sin(pos_act[1] + pos_act[2] +
                                                       np.pi/2) + 0.144*np.sin(pos_act[1] + pos_act[2] + np.pi/2 + pos_act[3])
    x, y = r*np.cos(pos_act[0]), r*np.sin(pos_act[0])
    return round(x, 3), round(y, 3), round((z1+z2)/2, 3), round(180*(pos_act[1] + pos_act[2] + pos_act[3])/3.14116, 2)

# Actionnement du bras


def prise(mouvement, ouverture="fermer", tps=2.0):
    # controlle l'ouverture de la pince

    if ouverture == "fermer":
        o = -0.01
    elif ouverture == "ouvrir":
        o = 0.01

    mouvement.prise(o, tps)

    while rclpy.ok():
        rclpy.spin_until_future_complete(
            mouvement, mouvement.pince_future)
        if mouvement.pince_future.done():

            try:
                réponse = mouvement.pince_future.result()
            except Exception as e:
                mouvement.get_logger().info(
                    'Erreur de prise%r' % (e,))
            else:
                mouvement.get_logger().info(
                    'Requête acceptée')
            break


def mouv(mouvement, x, y, z, th, tps=2.0):
    # controlle le déplacement de la pince en cartésien
    # convertit les coordonnées en position angulaire
    print(x, y, z, th, tps)
    th1, th2, th3, th4 = reduction(x, y, z, th)

    mouvement.get_logger().info("objectif:" + str((x, y, z, th)) +
                                "\n point d'arrivée:" + str(mgd([th1, th2, th3, th4])))

    mouvement.depl_requete(th1, th2, th3, th4, tps)

    while rclpy.ok():
        rclpy.spin_until_future_complete(
            mouvement, mouvement.depl_future)
        if mouvement.depl_future.done():

            try:
                réponse = mouvement.depl_future.result()
            except Exception as e:
                mouvement.get_logger().info(
                    'Erreur de déplacement %r' % (e,))
            else:
                mouvement.get_logger().info(
                    'Requête acceptée')
            break


def mouv_pol(mouvement, r, th1, z, th, tps=2.0):
    # controlle le déplacement de la pince en cartésien
    # convertit les coordonnées en position angulaire
    th2, th3, th4 = reduction_pol(r, z, th)

    mouvement.get_logger().info("objectif:" + str((round(r*np.cos(th1), 3), round(r*np.sin(th1), 3), z, th)) +
                                "\n point d'arrivée:" + str(mgd([th1, th2, th3, th4])))

    mouvement.depl_requete(th1, th2, th3, th4, tps)

    while rclpy.ok():
        rclpy.spin_until_future_complete(
            mouvement, mouvement.depl_future)
        if mouvement.depl_future.done():

            try:
                réponse = mouvement.depl_future.result()
            except Exception as e:
                mouvement.get_logger().info(
                    'Erreur de déplacement %r' % (e,))
            else:
                mouvement.get_logger().info(
                    'Requête acceptée')
            break


def manoeuvre(mouvement, information, n):
    placement = [0., 0.072]

    list_point = trajectoire(pos_act[:4], (0., 0.18, 0.005), 0.05, "cadre", 40)
    tps = 1.0
    while list_point:
        rclpy.spin_once(information)
        if information.action:
            x, y, z = list_point.pop(0)
            mouv(mouvement, x, y, z, 90, tps)

    prise(mouvement, "fermer", 1.0)

    list_point = trajectoire(pos_act[:4], (0., 0.13, 0.005), 0.05, "ligne", 40)
    tps = 1.0
    while list_point:
        rclpy.spin_once(information)
        if information.action:
            x, y, z = list_point.pop(0)
            mouv(mouvement, x, y, z, 90, tps)

    prise(mouvement, "ouvrir", 1.0)

    list_point = trajectoire(pos_act[:4], (0., 0.07, 0.), 0.05, "cadre", 40)
    tps = 1.0
    while list_point:
        rclpy.spin_once(information)
        if information.action:
            x, y, z = list_point.pop(0)
            mouv(mouvement, x, y, z, 90, tps)

    prise(mouvement, "fermer", 1.0)

    list_point = trajectoire(pos_act[:4], (0., 0.09, 0.02), 0.05, "ligne", 40)
    tps = 1.0
    while list_point:
        rclpy.spin_once(information)
        if information.action:
            x, y, z = list_point.pop(0)
            mouv(mouvement, x, y, z, 90, tps)

    mouv(mouvement, 0., placement[n//5 - 1], 0.02, 90, 2.0)
    mouv(mouvement, 0., placement[n//5 - 1], 0., 90, 1.0)
    prise(mouvement, "ouvrir", 1.0)


def bougeage(mouvement, information, obj):

    th1 = 2*np.arctan(y/(x + np.sqrt(x**2 + y**2)))
    r = np.sqrt(x**2 + y**2)

    list_point = trajectoire(pos_act[:4], (0., 0.09, 0.02), 0.05, "cadre", 40)
    tps = 1.0/len(list_point)
    while list_point:
        rclpy.spin_once(information)
        if information.action:
            x, y, z = list_point.pop(0)
            mouv(mouvement, x, y, z, 90, tps)


# Fonction principale


def main(args=None, hmax=5):
    global Pla, pos_act, angle_act
    rclpy.init(args=args)

    mouvement = Mouvement()
    cam = Camtopic()
    information = Information()

    Pla = []
    angle_act = [0 for i in range(7)]
    list_point = []

    pos_act = [0., 0.2, 0.2, 0, 0, 0, 0]
    mouv(mouvement, pos_act[0], pos_act[1], pos_act[2], pos_act[3], 1.0)

    Str = structure('tas')
    base = (0.2, 0., 0.)
    n = 0
    vide = True

    while(rclpy.ok()):
        rclpy.spin_once(information)

        if not list_point:
            if vide:
                prise(mouvement, "ouvrir", 1.0)
                Pla += [mgd(pos_act[:4])]
                obj = acquisition(cam, Pla, (0., 0.09))
                obj = (obj.x, obj.y, 0.)
                if obj:
                    if not n//5 and n:
                        manoeuvre(mouvement, information, n)
                    """if not obj.prenable:
                        bougeage(mouvement, obj)"""
                    hauteur = 0.05
                    list_point = trajectoire(
                        pos_act[:4], obj, hauteur, "cadre", 40)
                    tps = 1.0
                else:
                    print("le plateau est nettoyé")
                    break
                vide = False
            else:
                prise(mouvement, "fermer", 1.0)
                hauteur = 0.05
                obj = (0., 0.09, (n % 5)*0.8)
                list_point = trajectoire(pos_act[:4], obj, hauteur, "cadre")
                tps = 1.0
                vide = True
                n += 1
        elif information.action:
            x, y, z = list_point.pop(0)
            mouv(mouvement, x, y, z, 90, tps)
            time.sleep(0.5)

    securite(mouvement, information)
    cam.destroy_node()
    mouvement.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
