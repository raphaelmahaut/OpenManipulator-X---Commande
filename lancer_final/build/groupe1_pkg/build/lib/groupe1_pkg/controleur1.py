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

#######################################################################################################################

#                                           Définition des noeds ROS2

#######################################################################################################################


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

        while not self.pince_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service non disponible, nouvelle tentative...')

        while not self.depl_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service non disponible, nouvelle tentative...')

        self.pince_req = SetJointPosition.Request()
        self.depl_req = SetJointPosition.Request()

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

#######################################################################################################################

#                                          Informations de la caméra

#######################################################################################################################


def acquisition(cam, Pla, origine):
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
        for pla in Pla:
            # retire les planches déjà placées
            if distance((planche.position.x, planche.position.y), (pla[0], pla[1])) < 0.02:
                print(0)
                continue

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

    ajustement = 180
    mini = (np.inf, th)
    maxi = (0, th)
    while True:

        th = (th + 180) % 360 - 180
        alpha = np.arcsin(0.024/Dim[1])

        def angle_triangle(a, b, c):
            return np.arccos((a**2 + b**2 - c**2)/(2*a*b))

        th1 = 2*np.arctan(y/(x + np.sqrt(x**2 + y**2)))
        zb = z + Dim[3]*np.sin(np.pi*th/(2*90)) - Dim[0]
        r = np.sqrt(x**2 + y**2) - Dim[3]*np.cos(np.pi*th/(2*90))
        hp = np.sqrt(zb**2 + r**2)

        if hp > Dim[1] + Dim[2]:  # la position demandée est trop éloignée de la base
            if hp < mini[0]:
                mini = (hp, th)
            th -= 1
            ajustement -= 1

            if not ajustement:
                p = (Dim[1] + Dim[2])/mini[0] - 0.005
                r = p*r
                zb = p*zb
                th = mini[1]
                hp = np.sqrt(zb**2 + r**2)

                print("ajustement de la position voulue:")
                print("le robot est au maximum de son extension")
                break

        elif 0.024 > hp:  # la position est trop proche de la base
            if hp > maxi[0]:
                maxi = (hp, th)
            th += 1
            ajustement -= 1

            if not ajustement:
                p = 0.024/maxi[0] + 0.005
                r = p*r
                zb = p*zb
                th = maxi[1]
                hp = np.sqrt(zb**2 + r**2)

                print("ajustement de la position voulue:")
                print("éloignement de la base du robot dû à une colision")
                break

        else:
            break

    th2 = np.pi/2 - (alpha + np.arctan(zb/r) +
                     angle_triangle(Dim[1], hp, Dim[2]))
    th3 = (np.pi/2 + alpha) - angle_triangle(Dim[1], Dim[2], hp)
    th4 = (np.pi * th)/180 - (th2 + th3)

    return th1, th2, th3, th4


def trajectoire(A, B, hauteur, fonction='ligne', n=1):
    global Actions
    # renvoie un ensemble de points définissant une trajectoire et le temps pour atteindre cette position

    if fonction == 'cadre':
        P = [A, (A[0], A[1], hauteur), (B[0], B[1], hauteur), B]
        n = n//3 + 1
    elif fonction == 'ligne':
        P = [A, B]

    Actions.append(((A[0], A[1], A[2], 90), 0.8))
    for i in range(len(P) - 1):
        Actions.extend([((P[i+1][0]*k/n + P[i][0]*(1-k/n), P[i+1][1]*k/n + P[i][1]*(1-k/n),
                         P[i+1][2]*k/n + P[i][2]*(1-k/n), 90), 0.8) for k in range(1, n+1)])


#######################################################################################################################

#                                        Fonctions d'actionnement du bras

#######################################################################################################################

def securite():
    global pos_act, hauteur
    # envoie le robot dans une position de sécurité si on l'étteind ensuite

    trajectoire(pos_act, (0.2, 0., 0.), hauteur, "cadre", 1)


# Actionnement du bras


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


def action(mouvement, opération):
    global pos_act, tps, Pla, n
    # décode l'action demandée et appel les clients nécessaires

    if opération == "fermer":
        ouverture = -0.01
        prise(mouvement, ouverture, tps)
    elif opération == "ouvrir":
        ouverture = 0.01
        prise(mouvement, ouverture, tps)
    elif opération == "poser":
        ouverture = 0.01
        prise(mouvement, ouverture, tps)
        Pla += [pos_act]
        n += 1
    else:
        x, y, z, th = opération
        mouv(mouvement, x, y, z, th, tps)
        pos_act = opération


def mouv(mouvement, x, y, z, th, tps):
    # contrôle le déplacement de la pince en cartésien

    # convertit les coordonnées en position angulaire
    print(x, y, z, th)
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


def manoeuvre(n):
    global pos_act, Pla, hmax, hauteur
    # déplace la pile crée pour pouvoir en recommencer une autre à la place

    placement = [(0., -0.09), (0.072, -0.09), (0.15, -0.09), (0.23, -0.09),
                 (0.23, 0.09), (0.15, 0.09), (0.072, 0.09), (0.23, 0.09)]

    # définition d'une suite d'actions à effectuer pour réaliser la manoeuvre
    trajectoire(pos_act, (0., 0.09, 0.002), hauteur, "cadre", 1)
    Actions.append(("fermer", 1.0))
    Actions.append(("ouvrir", 1.0))
    trajectoire((0., 0.09, 0.002), (0., 0.18, 0.007), hauteur, "cadre", 1)
    Actions.append(("fermer", 1.0))
    trajectoire((0., 0.18, 0.004), (0., 0.14, 0.007), hauteur, "ligne", 5)
    trajectoire((0., 0.14, 0.007), (0., 0.18, 0.007), hauteur, "ligne", 5)
    Actions.append(("ouvrir", 1.0))
    trajectoire((0., 0.13, 0.002), (0., 0.08, 0.0), hauteur, "cadre", 1)
    Actions.append(("fermer", 1.0))
    trajectoire((0., 0.08, 0.0), (0., 0.09, 0.02), hauteur, "ligne", 1)

    Actions.append(((placement[n//hmax - 1][0],
                   placement[n//hmax - 1][1], 0.02, 90), 2.0))
    Actions.append(((placement[n//hmax - 1][0],
                   placement[n//hmax - 1][1], 0., 90), 1.0))

    Actions.append(("ouvrir", 1.0))

    # mise à jour de la position des planches
    Pla[-hmax:] = [(placement[n//hmax - 1][0], placement[n//hmax - 1][1], (i % hmax)*0.008, 90)
                   for i in range(hmax)]


def bougeage(A, B):
    global Actions, pos_act
    # bouge une planche qui ne serait pas prenable par la pince en la percutant latéralement

    trajectoire(pos_act, A, hauteur, "cadre", 1)
    Actions.append(("fermer", 1.0))
    trajectoire(A, B, hauteur, "ligne", 20)
    Actions.append(("ouvrir", 1.0))
    trajectoire(B, (0., 0.17, 0.05), hauteur, "cadre", 1)


#######################################################################################################################

#                                             Algorithme de commande

#######################################################################################################################


def main(args=None):
    global Pla, pos_act, Actions, tps, hmax, hauteur, n
    # Séquençage des actions à effectuer pour ranger les planches présentent sur la zone de travail

    # initialisation de la structure ROS2
    rclpy.init(args=args)

    mouvement = Mouvement()
    cam = Camtopic()

    # initialisation des variables utiles
    Pla = []
    Actions = []
    prec_tps = time.time()
    tps = 1.0
    n = 0
    hmax = 5
    hauteur = 0.05
    cas_boucl = True

    # mise en position initiale
    pos_act = [0., 0.2, 0.05, 90]
    mouv(mouvement, pos_act[0], pos_act[1], pos_act[2], pos_act[3], tps)
    Actions.append(("ouvrir", 1.0))

    # boucle de décision
    while(rclpy.ok()):
        if not Actions:
            obj, prenable = acquisition(cam, Pla, (0., 0.09))

            if obj:

                if not n % hmax and n and cas_boucl:  # on doit bouger la pile en construction pour faire de la place
                    manoeuvre(n)
                    cas_boucl = False
                    continue

                if not prenable:  # on doit changer l'orientation de la planche
                    bougeage((obj[2], obj[3], 0.006), (obj[0], obj[1], 0.006))
                    continue

                obj = (obj[0], obj[1], 0.)
                trajectoire(pos_act, obj, hauteur, "cadre", 1)
                Actions.append(("fermer", 1.0))
                trajectoire(obj, (0., 0.09, (n % hmax)*0.01),
                            hauteur, "cadre", 1)
                Actions.append(("poser", 1.0))
                cas_boucl = True

            else:  # il n'y a plus de planches sur la zone de travail
                print("le plateau est nettoyé")
                time.sleep(2.0)
                break

        elif time.time() - prec_tps > tps:
            prec_tps = time.time()
            opération, tps = Actions.pop(0)
            action(mouvement, opération)

    # retour à une position de sécurité
    securite()
    while Actions:
        if time.time() - prec_tps > tps:
            prec_tps = time.time()
            opération, tps = Actions.pop(0)
            print(opération)
            action(mouvement, opération)

    print("la tâche est terminée \n Planches:", Pla)

    # fermeture de la structure ROS2
    cam.destroy_node()
    mouvement.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
