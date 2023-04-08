import sys
import os
from turtle import position
import numpy as np
import math as m
import select
import time
import rclpy

from open_manipulator_msgs.srv import SetJointPosition, SetDrawingTrajectory 
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from std_msgs.msg import Bool

#######################################################################################################################

#                                           Définition des noeuds ROS2

#######################################################################################################################

class Enregistrement(Node):

    def __init__(self):
        super().__init__('enregistrement')
        self.publisher_ = self.create_publisher(Bool, 'etats', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Bool()
        msg.data = self.i % 2 == 0
        self.publisher_.publish(msg)
        self.get_logger().info('message envoyé')
        self.i += 1


class Camtopic(Node):
    # Définition d'un noeud subscriber permettant de récupérer les informations de la caméra

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
    # Définition d'un noeud permettant de déplacer le bras et d'ouvrir la pince

    def __init__(self):
        super().__init__('mouvement')  # nom du noeud dans le graph
        self.depl_cli = self.create_client(SetJointPosition,
                                           'goal_joint_space_path')  # client pour le déplacement
        self.pince_cli = self.create_client(SetJointPosition,
                                            '/goal_tool_control')  # client pour l'ouverture de la pince
        self.traj_cli = self.create_client(SetDrawingTrajectory,
                                           'goal_drawing_trajectory')  # client pour la trajectoire

        while not self.pince_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service non disponible, nouvelle tentative...')

        while not self.depl_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service non disponible, nouvelle tentative...')

        while not self.traj_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service non disponible, nouvelle tentative...')

        self.pince_req = SetJointPosition.Request()
        self.depl_req = SetJointPosition.Request()
        self.traj_req = SetDrawingTrajectory.Request()

    # requête correspondant au déplacement

    def depl_requete(self, th1, th2, th3, th4, tps=2.0):
        self.depl_req.joint_position.joint_name = [
            "joint1", "joint2", "joint3", "joint4"]
        self.depl_req.joint_position.position = [th1, th2, th3, th4]
        self.depl_req.path_time = tps
        try:
            self.depl_future = self.depl_cli.call_async(self.depl_req)
        except Exception as e:
            self.get_logger().info('problème de requête')

    def prise_requete(self, o, tps=2.0):  # requête correspondant à l'ouverture de la pince
        self.pince_req.joint_position.joint_name = {"gripper"}
        self.pince_req.joint_position.position = {o}
        self.pince_req.path_time = tps
        self.pince_future = self.pince_cli.call_async(self.pince_req)

    # requête pour utiliser une trajectoire
    def traj_requete(self, A, B, tps=2.0):
        self.traj_req.end_effector_name = "joint4"
        self.traj_req.drawing_trajectory_name = "circle"
        self.traj_req.path_time = tps
        self.traj_req.param = [0., 0.1, 0.]

        try:
            self.traj_future = self.traj_cli.call_async(self.traj_req)
        except Exception as e:
            self.get_logger().info('problème de requête')


#######################################################################################################################

#                                          Informations de la caméra

#######################################################################################################################


def acquisition(cam, origine):
    global tps
    # demande la position des objets à utiliser sur le support
    entree = time.time()
    while time.time() - entree < tps + 1.5:
        rclpy.spin_once(cam)
    Obj = cam.objet  # message provenant de la caméra

    def distance(A, B):  # fonction auxiliaire
        return np.sqrt((A[0]-B[0])**2 + (A[1]-B[1])**2)

    # tri des objets par rapport à leur distance au bras et à la possibilité de les prendres, suppressions des planches déjà placées
    obj = None
    d = np.inf
    autre = None
    d_autre = np.inf

    for planche in Obj.poses:
        if not planche.orientation.w:  # vérifie la possibilité d'attraper la planche
            if distance((planche.position.x, planche.position.y), origine) < d:
                obj = planche
                d = distance((planche.position.x, planche.position.y), origine)
        else:
            if distance((planche.position.x, planche.position.y), origine) < d_autre:
                autre = planche
                d_autre = distance(
                    (planche.position.x, planche.position.y), origine)

    # renvoie la meilleure planche à attraper
    if not obj:
        if autre:
            return (autre.position.x, autre.position.y, autre.orientation.x, autre.orientation.y), False
        return None, False

    return (obj.position.x, obj.position.y, obj.position.z), True

#######################################################################################################################

#                                             Calculs géométriques


#######################################################################################################################
Dim = [0.0765, 0.13, 0.124, 0.1466]


def mgd(pos_rob):
    # calcule la position en bout de pince à partir des positions fournies
    global Dim

    alpha = np.arcsin(0.024/Dim[1])
    beta = np.arctan(0.012/Dim[3])

    alpha = 0.186
    beta = 0.105

    z1 = Dim[0] + Dim[1]*np.cos(pos_rob[1] + alpha) + Dim[2]*np.cos(pos_rob[1] + pos_rob[2] +
                                                                    np.pi/2) + Dim[3]*np.cos(pos_rob[1] + pos_rob[2] + np.pi/2 + pos_rob[3] + beta)
    z2 = Dim[0] + Dim[1]*np.cos(pos_rob[1] + alpha) + Dim[2]*np.cos(pos_rob[1] + pos_rob[2] +
                                                                    np.pi/2) + Dim[3]*np.cos(pos_rob[1] + pos_rob[2] + np.pi/2 + pos_rob[3] - beta)
    r = Dim[1]*np.sin(pos_rob[1] + alpha) + Dim[2]*np.sin(pos_rob[1] + pos_rob[2] +
                                                          np.pi/2) + Dim[3]*np.sin(pos_rob[1] + pos_rob[2] + np.pi/2 + pos_rob[3])
    x, y = r*np.cos(pos_rob[0]), r*np.sin(pos_rob[0])
    return round(x, 3), round(y, 3), round((z1+z2)/2, 3), round(180*(pos_rob[1] + pos_rob[2] + pos_rob[3])/np.pi, 2)


def mgi(x, y, z, th):
    # calcule la position angulaire à partir de la position cartésienne voulue
    global Dim

    alpha = np.arcsin(0.024/Dim[1])

    def angle_triangle(a, b, c):
        return np.arccos((a**2 + b**2 - c**2)/(2*a*b))

    th1 = 2*np.arctan(y/(x + np.sqrt(x**2 + y**2)))
    zb = z + Dim[3]*np.sin(np.pi*th/(2*90)) - Dim[0]
    r = np.sqrt(x**2 + y**2) - Dim[3]*np.cos(np.pi*th/(2*90))
    hp = np.sqrt(zb**2 + r**2)

    if hp > Dim[1] + Dim[2]:  # la position demandée est trop éloignée de la base
        return "position trop éloignée"

    if 0.024 > hp:  # la position est trop proche de la base
        return "collision"

    th2 = np.pi/2 - (alpha + np.arctan(zb/r) +
                     angle_triangle(Dim[1], hp, Dim[2]))
    th3 = (np.pi/2 + alpha) - angle_triangle(Dim[1], Dim[2], hp)
    th4 = (np.pi * th)/180 - (th2 + th3)

    return th1, th2, th3, th4


#######################################################################################################################

#                                        Fonctions d'actionnement du bras

#######################################################################################################################


def prise(mouvement, ouverture, tps):
    # controlle l'ouverture de la pince
    mouvement.prise_requete(ouverture, tps)

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


def trajection(trajectoire, A, B, tps):
    # controlle l'ouverture de la pince
    global enregistre

    rclpy.spin_once(enregistre)
    trajectoire.traj_requete(A, B, tps)

    while rclpy.ok():
        rclpy.spin_until_future_complete(
            trajectoire, trajectoire.traj_future)
        if trajectoire.traj_future.done():

            try:
                réponse = trajectoire.traj_future.result()
            except Exception as e:
                trajectoire.get_logger().info(
                    'Erreur de prise%r' % (e,))
            else:
                trajectoire.get_logger().info(
                    'Requête acceptée')
            break


def mouv(mouvement, x, y, z, th, tps):
    # contrôle le déplacement de la pince en cartésien

    # convertit les coordonnées en position angulaire
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
    
def mouv2(mouvement, x, y, z, th, tps):
    global enregistre
    # contrôle le déplacement de la pince en cartésien

    # convertit les coordonnées en position angulaire
    th1, th2, th3, th4 = mgi(x, y, z, th)

    mouvement.get_logger().info("objectif:" + str((x, y, z, th)) +
                                "\n point d'arrivée:" + str(mgd([th1, th2, th3, th4])))

    mouvement.depl_requete(th1, th2, th3, th4 - np.pi, tps)

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
    
    time.sleep(tps+1.0)
    rclpy.spin_once(enregistre)

    time.sleep(0.5)
    mouvement.depl_requete(th1, th2, th3, th4 -np.pi/2, 2.0)
    

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
    """
    time.sleep(0.3)
    mouvement.depl_requete(th1, th2, th3, th4, 0.1)
    

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
    """
    """time.sleep(0.3)
    mouvement.prise_requete(0.01, 0.001)

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
"""

def action(mouvement, opération, tps):
    global pos_act
    # décode l'action demandée et appel les clients nécessaires

    if opération == "fermer":
        ouverture = 0.0
        prise(mouvement, ouverture, tps)
    elif opération == "ouvrir":
        ouverture = 0.01
        prise(mouvement, ouverture, 0.001)
    elif opération == "trajet":
        trajection(mouvement, pos_act, (0.2, 0., 0.05, 90), 2.0)

    elif opération == "lancer":
        mouv(mouvement, 0.1, 0., 0.05, 90, 1.0)
        time.sleep(1)
        prise(mouvement, 0.01, 0.01)
        time.sleep(0.02)
        mouv(mouvement, 0.25, 0., 0.2, 20, 0.01)
        time.sleep(0.01)
        mouv(mouvement, 0.27, 0., 0.25, 0, 0.02)
    elif opération == "test":
        mouv2(mouvement, 0.1, 0., 0.05, 90, 1.0)
    else:
        x, y, z, th = opération
        mouv(mouvement, x, y, z, th, tps)
        pos_act = opération


#######################################################################################################################

#                                             Algorithme de commande

#######################################################################################################################

def trajectoire(A, B, hauteur=0.05, fonction='ligne', n=1):
    global Actions
    # renvoie un ensemble de points définissant une trajectoire et le temps pour atteindre cette position

    def f(x):
        return x

    if fonction == 'cadre':
        P = [A, (A[0], A[1], hauteur), (B[0], B[1], hauteur), B]
        n = n//3 + 1
    elif fonction == 'ligne':
        P = [A, B]

    Actions.append((A[0], A[1], A[2], 90))
    for i in range(len(P) - 1):
        Actions.extend([(P[i+1][0]*f(k/n) + P[i][0]*f(1-k/n), P[i+1][1]*f(k/n) + P[i][1]*f(1-k/n),
                         P[i+1][2]*f(k/n) + P[i][2]*f(1-k/n), 90) for k in range(1, n+1)])


def main(args=None):
    global pos_act, Actions, tps, enregistre
    # Séquençage des actions à effectuer pour ranger les planches présentent sur la zone de travail

    def distance(A, B):  # fonction auxiliaire
        return np.sqrt((A[0]-B[0])**2 + (A[1]-B[1])**2 + (A[2]-B[2])**2)

    # initialisation de la structure ROS2
    rclpy.init(args=args)

    mouvement = Mouvement()
    cam = Camtopic()
    enregistre = Enregistrement()

    # initialisation des variables utiles
    Actions = []
    prec_tps = time.time()
    Pla = []
    vitesse = 0.1
    tps = 2.0

    # mise en position initiale
    pos_act = [0.2, 0., 0.05, 90]
    mouv(mouvement, pos_act[0], pos_act[1], pos_act[2], pos_act[3], tps)

    Actions.append("fermer")
    #Actions.append("lancer")
    Actions.append("trajet")



    # boucle de décision
    while(rclpy.ok()):
        if not Actions:
            """obj, prenable = acquisition(cam, (0., 0.09))

            if obj:
                trajectoire(pos_act, obj, "cadre", 1)
                Actions.append("fermer")
                trajectoire(obj, pos_act, "cadre", 1)"""

            """if pos_act == [0., 0.2, 0.05, 90]:
                trajectoire(pos_act, (0.2, 0., 0.05, 90), n=1)
            else:
                trajectoire(pos_act, (0., 0.2, 0.05, 90), n=1)"""
            break
            Actions.append("trajet")

        elif time.time() - prec_tps > tps:
            prec_tps = time.time()
            opération = Actions.pop(0)
            if not type(opération) == str:
                tps = distance(pos_act, opération)/vitesse
                tps = 0.1
            else:
                tps = 1.0
            print(opération, tps)
            #tps = max(tps, 0.2)
            #rclpy.spin_once(enregistre)
            action(mouvement, opération, tps)

    # retour à une position de sécurité
    while Actions:
        if time.time() - prec_tps > tps:
            prec_tps = time.time()
            opération = Actions.pop(0)
            if not type(opération) == str:
                tps = distance(pos_act, opération)/vitesse
                tps = 2.0
            else:
                tps = 1.0
            print(opération, tps)
            #tps = max(tps, 0.2)
            #rclpy.spin_once(enregistre)
            action(mouvement, opération, tps)

    # fermeture de la structure ROS2
    cam.destroy_node()
    mouvement.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
