import sys
import os
from turtle import position
import numpy as np
import math as m
import select
import time
import rclpy

from open_manipulator_msgs.srv import SetJointPosition
from geometry_msgs.msg import PoseArray
from rclpy.node import Node


class Camtopic(Node):
    def __init__(self):
        super().__init__('appel_camera')
        self.subscriber = self.create_subscription(
            PoseArray, 'camera', self.appel, 10)
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

    def prise_requete(self, o, tps=2.0):
        self.pince_req.joint_position.joint_name = {"gripper"}
        self.pince_req.joint_position.position = {o}
        self.pince_req.path_time = tps
        self.pince_future = self.pince_cli.call_async(self.pince_req)


def acquisition(cam, Pla, origine):
    # demande la position des objets à utiliser sur le support
    entree = time.time()
    while time.time() - entree < 5:
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
        print(5)
        for pla in Pla:
            # retire les planches déjà placées
            if distance((planche.position.x, planche.position.y), (pla[0], pla[1])) < 0.02:
                print(0)
                continue
        if not planche.orientation.w:
            print(1)
            if distance((planche.position.x, planche.position.y), origine) < d:
                obj = planche
                d = distance((planche.position.x, planche.position.y), origine)
        else:
            print(2)
            if distance((planche.position.x, planche.position.y), origine) < d_autre:
                autre = planche
                d_autre = distance(
                    (planche.position.x, planche.position.y), origine)

    if not obj:
        if autre:
            return (autre.position.x, autre.position.y, autre.position.z), False
        return None, False

    return (obj.position.x, obj.position.y, obj.position.z), True


def mgd(pos_rob):
    # calcule la position en bout de pince à partir des positions fournies

    alpha = 0.186
    beta = 0.105
    z1 = 0.077 + 0.13*np.cos(pos_rob[1] + alpha) + 0.124*np.cos(pos_rob[1] + pos_rob[2] +
                                                                np.pi/2) + 0.144*np.cos(pos_rob[1] + pos_rob[2] + np.pi/2 + pos_rob[3] + beta)
    z2 = 0.077 + 0.13*np.cos(pos_rob[1] + alpha) + 0.124*np.cos(pos_rob[1] + pos_rob[2] +
                                                                np.pi/2) + 0.144*np.cos(pos_rob[1] + pos_rob[2] + np.pi/2 + pos_rob[3] - beta)
    r = 0.13*np.sin(pos_rob[1] + alpha) + 0.124*np.sin(pos_rob[1] + pos_rob[2] +
                                                       np.pi/2) + 0.144*np.sin(pos_rob[1] + pos_rob[2] + np.pi/2 + pos_rob[3])
    x, y = r*np.cos(pos_rob[0]), r*np.sin(pos_rob[0])
    return round(x, 3), round(y, 3), round((z1+z2)/2, 3), round(180*(pos_rob[1] + pos_rob[2] + pos_rob[3])/3.14116, 2)


def mgi(x, y, z, th):
    # calculle la position angulaire à partir de la position cartésienne voulue

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


def trajectoire(A, B, hauteur, fonction='ligne', n=1):
    # renvoie un ensemble de points définissant une trajectoire entre deux points

    if fonction == 'cadre':
        P = [A, (A[0], A[1], hauteur), (B[0], B[1], hauteur), B]
        n = n//3 + 1
    elif fonction == 'ligne':
        P = [A, B]

    traj = [(A[0], A[1], A[2])]
    for i in range(len(P) - 1):
        traj += [(P[i+1][0]*k/n + P[i][0]*(1-k/n), P[i+1][1]*k/n + P[i][1]*(1-k/n),
                  P[i+1][2]*k/n + P[i][2]*(1-k/n)) for k in range(1, n+1)]

    return traj


def securite(mouvement):
    global pos_act, prec_tps
    # envoie le robot dans une positio de sécurité si on l'étteind ensuite

    list_point = trajectoire(pos_act, (0.2, 0., 0.), 0.05, "cadre", 1)
    tps = 2.0
    while list_point:
        if time.time() - prec_tps > tps:
            x, y, z = list_point.pop(0)
            mouv(mouvement, x, y, z, 90, tps)
            pos_act = [x, y, z, 90]

    print("la tâche est terminée")


# Actionnement du bras


def prise(mouvement, ouverture="fermer", tps=2.0):
    # controlle l'ouverture de la pince

    # convertion des demandes en angle de l'actionneur (peut permettre plus tard de choisir une ouverture plus précise si des objets sont proches les uns des autres)
    if ouverture == "fermer":
        o = -0.01
    elif ouverture == "ouvrir":
        o = 0.01

    mouvement.prise_requete(o, tps)

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

    time.sleep(tps)


def mouv(mouvement, x, y, z, th, tps=2.0):
    # controlle le déplacement de la pince en cartésien

    # convertit les coordonnées en position angulaire
    print(x, y, z, th, tps)
    th1, th2, th3, th4 = mgi(x, y, z, th)

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


def manoeuvre(mouvement, n):
    global prec_tps, pos_act, Pla
    placement = [0., 0.072]

    list_point = trajectoire(pos_act, (0., 0.18, 0.005), 0.05, "cadre", 1)
    tps = 1.0
    while list_point:

        if time.time() - prec_tps > tps:
            prec_tps = time.time()
            x, y, z = list_point.pop(0)
            mouv(mouvement, x, y, z, 90, tps)
            pos_act = [x, y, z, 90]

    prise(mouvement, "fermer", 1.0)

    list_point = trajectoire(pos_act, (0., 0.13, 0.005), 0.05, "ligne", 1)
    tps = 1.0
    while list_point:

        if time.time() - prec_tps > tps:
            prec_tps = time.time()
            x, y, z = list_point.pop(0)
            mouv(mouvement, x, y, z, 90, tps)
            pos_act = [x, y, z, 90]

    prise(mouvement, "ouvrir", 1.0)

    list_point = trajectoire(pos_act, (0., 0.07, 0.), 0.05, "cadre", 1)
    tps = 1.0
    while list_point:

        if time.time() - prec_tps > tps:
            prec_tps = time.time()
            x, y, z = list_point.pop(0)
            mouv(mouvement, x, y, z, 90, tps)
            pos_act = [x, y, z, 90]

    prise(mouvement, "fermer", 1.0)

    list_point = trajectoire(pos_act, (0., 0.09, 0.02), 0.05, "ligne", 1)
    tps = 1.0
    while list_point:
        if time.time() - prec_tps > tps:
            prec_tps = time.time()
            x, y, z = list_point.pop(0)
            mouv(mouvement, x, y, z, 90, tps)
            pos_act = [x, y, z, 90]

    mouv(mouvement, placement[n//5 - 1], -0.09, 0.02, 90, 2.0)
    time.sleep(2.0)
    mouv(mouvement, placement[n//5 - 1], -0.09, 0., 90, 1.0)
    time.sleep(1.0)
    pos_act = [placement[n//5 - 1], -0.09, 0., 90]
    Pla[-5:] = [(placement[n//5 - 1], -0.09, (i % 5)*0.008, 90)
                for i in range(5)]
    prise(mouvement, "ouvrir", 1.0)


def bougeage(mouvement, obj):
    global prec_tps, pos_act

    th1 = 2*np.arctan(obj[1]/(obj[0] + np.sqrt(obj[0]**2 + obj[1]**2)))
    r = np.sqrt(obj[0]**2 + obj[1]**2)

    if obj[1] >= 0:
        list_point = trajectoire(pos_act, ((r - 0.05)*np.sin(th1 + 3*np.pi/180),
                                 (r - 0.05)*np.cos(th1 + 3*np.pi/180), 0.002), 0.05, "cadre", 1)
    else:
        list_point = trajectoire(pos_act, ((r - 0.05)*np.sin(th1 - 3*np.pi/180),
                                 (r - 0.05)*np.cos(th1 - 3*np.pi/180), 0.002), 0.05, "cadre", 1)
    tps = 1.0
    while list_point:
        if time.time() - prec_tps > tps:
            prec_tps = time.time()
            x, y, z = list_point.pop(0)
            mouv(mouvement, x, y, z, 90, tps)
            pos_act = [x, y, z, 90]

    prise(mouvement, "fermer", 1.0)

    if obj[1] >= 0:
        list_point = trajectoire(pos_act, ((r + 0.05)*np.sin(th1 + 3*np.pi/180),
                                 (r + 0.05)*np.cos(th1 + 3*np.pi/180), 0.002), 0.05, "ligne", 1)
    else:
        list_point = trajectoire(pos_act, ((r + 0.05)*np.sin(th1 - 3*np.pi/180),
                                 (r + 0.05)*np.cos(th1 - 3*np.pi/180), 0.002), 0.05, "ligne", 1)
    tps = 1.0
    while list_point:

        if time.time() - prec_tps > tps:
            prec_tps = time.time()
            x, y, z = list_point.pop(0)
            mouv(mouvement, x, y, z, 90, tps)
            pos_act = [x, y, z, 90]

    list_point = trajectoire(pos_act, (0., 0.2, 0.), 0.05, "cadre", 1)
    tps = 1.0
    while list_point:

        if time.time() - prec_tps > tps:
            x, y, z = list_point.pop(0)
            mouv(mouvement, x, y, z, 90, tps)
            pos_act = [x, y, z, 90]

    prise(mouvement, "ouvrir", 1.0)


# Fonction principale

def main(args=None, hmax=5):
    global Pla, pos_act,  prec_tps
    rclpy.init(args=args)

    mouvement = Mouvement()
    cam = Camtopic()

    Pla = []
    list_point = []
    prec_tps = time.time()

    pos_act = [0., 0.2, 0.05, 90]
    mouv(mouvement, pos_act[0], pos_act[1], pos_act[2], pos_act[3], 1.0)

    n = 0
    vide = True

    while(rclpy.ok()):
        if not list_point:
            if vide:
                prise(mouvement, "ouvrir", 1.0)
                Pla += [mgd(pos_act)]
                obj, prenable = acquisition(cam, Pla, (0., 0.09))

                if obj:
                    obj = (obj[0], obj[1], 0.)
                    if not n % 5 and n:
                        manoeuvre(mouvement, n)
                    if not prenable:
                        bougeage(mouvement, obj)
                        continue
                    hauteur = 0.05
                    list_point = trajectoire(
                        pos_act, obj, hauteur, "cadre", 1)
                    tps = 1.0
                    vide = False
                else:
                    print("le plateau est nettoyé")
                    time.sleep(2.0)
                    break

            else:
                prise(mouvement, "fermer", 1.0)
                time.sleep(1.0)
                hauteur = 0.05
                obj = (0., 0.09, (n % 5)*0.008)
                list_point = trajectoire(pos_act, obj, hauteur, "cadre", 1)
                tps = 1.0
                vide = True
                n += 1
        elif time.time() - prec_tps > tps:
            prec_tps = time.time()
            x, y, z = list_point.pop(0)
            mouv(mouvement, x, y, z, 90, tps)
            pos_act = [x, y, z, 90]

    securite(mouvement)
    cam.destroy_node()
    mouvement.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
