import sys
import os
from turtle import position
import numpy as np
import math as m
import select
import time
import rclpy

from open_manipulator_msgs.srv import SetJointPosition, SetDrawingTrajectory
from geometry_msgs.msg import PoseArray, Vector3
from rclpy.node import Node
from std_msgs.msg import Float32

#######################################################################################################################

#                                           Définition des noeuds ROS2

#######################################################################################################################
class Signal(Node):
    def __init__(self):
        super().__init__('signal')
        self.subscriber = self.create_subscription(
            Vector3, 'gob', self.appel, 10)
        self.subscriber
        self.flag = False

    def appel(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.get_logger().info('Début du lancement en {}, {}, {}'.format(msg.x, msg.y, msg.z))
        self.flag = True

class Enregistrement(Node):

    def __init__(self):
        super().__init__('enregistrement')
        self.publisher_ = self.create_publisher(Float32, 'etats', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.tps = 2.0

    def timer_callback(self):
        msg = Float32()
        msg.data = self.tps
        self.publisher_.publish(msg)
        self.get_logger().info("demande d'enregistrement pendant {} secondes".format(self.tps))


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
            self.get_logger().info('problème avec la requête')

    def prise_requete(self, o, tps=2.0):  # requête correspondant à l'ouverture de la pince
        self.pince_req.joint_position.joint_name = {"gripper"}
        self.pince_req.joint_position.position = {o}
        self.pince_req.path_time = tps
        self.pince_future = self.pince_cli.call_async(self.pince_req)

    # requête pour utiliser une trajectoire
    def traj_requete1(self, acc_time, dec_time, vitesse_r, vitesse_z, rAcc, zAcc, rDec, zDec):
        self.traj_req.end_effector_name = "gripper"
        self.traj_req.drawing_trajectory_name = "courbe"
        self.traj_req.path_time = acc_time + dec_time
        self.traj_req.param = [acc_time, vitesse_r,
                               vitesse_z, rAcc, zAcc, rDec, zDec]

        try:
            self.traj_future = self.traj_cli.call_async(self.traj_req)
        except Exception as e:
            self.get_logger().info('problème avec la requête')

    def traj_requete2(self, move_time, angle, vitesse):
        self.traj_req.drawing_trajectory_name = "depliage"
        self.traj_req.path_time = move_time
        self.traj_req.param = [angle, vitesse]

        try:
            self.traj_future = self.traj_cli.call_async(self.traj_req)
        except Exception as e:
            self.get_logger().info('problème avec la requête')


#######################################################################################################################

#                                             Calculs géométriques

#######################################################################################################################
Dim = [0.0765, 0.13, 0.124, 0.1466, 0.]


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


def mgi_lanc(x, y, z, th):
    # calcule la position angulaire à partir de la position cartésienne voulue
    global Dim

    if z < 0:
        print("trop bas")
        return None
    # petit angle du au décalge de l'axe du deuxième segment (rad)
    alpha = np.arcsin(0.024/Dim[1])
    th_rad = np.pi*th/180  # angle de la pince dans un repère cartésien

    def angle_triangle(a, b, c):
        return np.arccos((a**2 + b**2 - c**2)/(2*a*b))

    th1 = np.arctan(y/x)

    # calcul des grandeurs utiles
    zb = z + Dim[3]*np.sin(th_rad) - Dim[0]
    r = np.sign(x)*np.sqrt(x**2 + y**2) - Dim[3]*np.cos(th_rad)
    hp = np.sqrt(zb**2 + r**2)

    # angle entre l'axe de la pince et l'oigine du bras
    beta = 2*np.arctan(r/(zb + hp))

    if hp > Dim[1] + Dim[2]:  # la position demandée est trop éloignée de la base
        print("position trop éloignée")
        return None

    if 0.02 > hp:  # la position est trop proche de la base
        print("collision")
        return None

    th2 = beta - (alpha + angle_triangle(Dim[1], hp, Dim[2]))
    th3 = (np.pi/2 + alpha) - angle_triangle(Dim[1], Dim[2], hp)
    th4 = th_rad - (th2 + th3)

    return th1, th2, th3, th4


def mgi(x, y, z, th):
    # calcule la position angulaire à partir de la position cartésienne voulue
    global Dim

    if z < 0:
        print("trop bas")
        return None

    # petit angle du au décalge de l'axe du deuxième segment (rad)
    alpha = np.arcsin(0.024/Dim[1])
    th_rad = np.pi*th/180  # angle de la pince dans un repère cartésien

    def angle_triangle(a, b, c):
        return np.arccos((a**2 + b**2 - c**2)/(2*a*b))

    th1 = 2*np.arctan(y/(x + np.sqrt(x**2 + y**2)))

    # calcul des grandeurs utiles
    zb = z + Dim[3]*np.sin(th_rad) - Dim[0]
    r = np.sqrt(x**2 + y**2) - Dim[3]*np.cos(th_rad)
    hp = np.sqrt(zb**2 + r**2)

    # angle entre l'axe de la pince et l'oigine du bras
    beta = 2*np.arctan(r/(zb + hp))

    if hp > Dim[1] + Dim[2]:  # la position demandée est trop éloignée de la base
        print("position trop éloignée")
        return None

    if 0.02 > hp:  # la position est trop proche de la base
        print("collision")
        return None

    th2 = beta - (alpha + angle_triangle(Dim[1], hp, Dim[2]))
    th3 = (np.pi/2 + alpha) - angle_triangle(Dim[1], Dim[2], hp)
    th4 = th_rad - (th2 + th3)

    return th1, th2, th3, th4

#######################################################################################################################

#                   Modèle géomètrique inverse utilisant descente de gradient et jacobienne

#######################################################################################################################


#######################################
# Défintion des paramètres de DH du bras

alpha = np.arcsin(0.024/Dim[1])


def passage(qi, alpha, d, a):
    T = np.eye(4)
    T[0, 0] = np.cos(qi)
    T[0, 1] = -np.cos(alpha)*np.sin(qi)
    T[0, 2] = np.sin(alpha)*np.sin(qi)
    T[0, 3] = a*np.cos(qi)
    T[1, 0] = np.sin(qi)
    T[1, 1] = np.cos(alpha)*np.cos(qi)
    T[1, 2] = -np.cos(qi)*np.sin(alpha)
    T[1, 3] = a*np.sin(qi)
    T[2, 1] = np.sin(alpha)
    T[2, 2] = np.cos(alpha)
    T[2, 3] = d
    return (T)

# Définition du modèle géométrique direct


def MGD(q):
    q1, q2, q3, q4 = q[0, 0], q[1, 0], q[2, 0], q[3, 0]

    T01 = passage(q1, -np.pi/2, Dim[0], Dim[4])
    T12 = passage(q2 - np.pi/2 + alpha, 0, 0, Dim[1])
    T23 = passage(q3 + np.pi/2 - alpha, 0, 0, Dim[2])
    T34 = passage(q4, 0, 0, Dim[3])

    TCP = np.dot(T01, np.dot(T12, np.dot(T23, T34)))

    return np.array([[TCP[0, 3]], [TCP[1, 3]], [TCP[2, 3]]])

# Définition de la matrice Jacobienne : reprise des lignes 1,2,3 & 5 de la matrice Jacobienne calculée dans le programme "Calcul matrices de transformations et jacobiens"


def Jacob(qk):
    J = np.array([[-(Dim[1]*np.sin(alpha + qk[1, 0]) + Dim[2]*np.cos(qk[1, 0] + qk[2, 0]) + Dim[3]*np.cos(qk[1, 0] + qk[2, 0] + qk[3, 0]) + Dim[4])*np.sin(qk[0, 0]), (Dim[1]*np.cos(alpha + qk[1, 0]) - Dim[2]*np.sin(qk[1, 0] + qk[2, 0]) - Dim[3]*np.sin(qk[1, 0] + qk[2, 0] + qk[3, 0]))*np.cos(qk[0, 0]), -(Dim[2]*np.sin(qk[1, 0] + qk[2, 0]) + Dim[3]*np.sin(qk[1, 0] + qk[2, 0] + qk[3, 0]))*np.cos(qk[0, 0]), -Dim[3]*np.sin(qk[1, 0] + qk[2, 0] + qk[3, 0])*np.cos(qk[0, 0])],
                  [(Dim[1]*np.sin(alpha + qk[1, 0]) + Dim[2]*np.cos(qk[1, 0] + qk[2, 0]) + Dim[3]*np.cos(qk[1, 0] + qk[2, 0] + qk[3, 0]) + Dim[4])*np.cos(qk[0, 0]), (Dim[1]*np.cos(alpha + qk[1, 0]) - Dim[2]*np.sin(qk[1, 0] + qk[2, 0]) - Dim[3] *
                                                                                                                                                                      np.sin(qk[1, 0] + qk[2, 0] + qk[3, 0]))*np.sin(qk[0, 0]), -(Dim[2]*np.sin(qk[1, 0] + qk[2, 0]) + Dim[3]*np.sin(qk[1, 0] + qk[2, 0] + qk[3, 0]))*np.sin(qk[0, 0]), -Dim[3]*np.sin(qk[1, 0] + qk[2, 0] + qk[3, 0])*np.sin(qk[0, 0])],
                  [0, -Dim[1]*np.sin(alpha + qk[1, 0]) - Dim[2]*np.cos(qk[1, 0] + qk[2, 0]) - Dim[3]*np.cos(qk[1, 0] + qk[2, 0] + qk[3, 0]), -Dim[2]*np.cos(qk[1, 0] + qk[2, 0]) - Dim[3]*np.cos(qk[1, 0] + qk[2, 0] + qk[3, 0]), -Dim[3]*np.cos(qk[1, 0] + qk[2, 0] + qk[3, 0])]])
    return(J)

# Définition du modèle géométrique inverse


def MGI(x, y, z, q0=np.array([[0.1], [-0.4], [0.4], [1.3]]), alpha_k=1.0, alpha_k2=0.1, tol=0.001):

    final_pose = np.array([[x], [y], [z]])
    q = q0
    p0 = MGD(q0)
    pose_error = abs(final_pose - p0)
    max_pose_error = pose_error.max()
    max_steps = 500
    i = 0

    while(max_pose_error > tol and i < max_steps):

        J = Jacob(q)
        J_JT = np.dot(J, np.transpose(J))
        J_pseudo_inv = np.dot(np.transpose(J), np.linalg.inv(J_JT))

        qmax = np.array([[np.pi/2], [1.67], [1.53], [2]])
        qmin = np.array([[-np.pi/2], [-2.05], [-1.67], [-1.8]])
        qmean = 0.5*(qmax + qmin)
        qdiff = qmax - qmin
        NJ = np.eye(4) - np.dot(J_pseudo_inv, J)
        dH = -2*(q - qmean) / (qdiff*qdiff)

        pose = MGD(q)
        pose_error = final_pose - pose
        max_pose_error = (abs(pose_error)).max()

        q += alpha_k2*np.dot(J_pseudo_inv, pose_error) + alpha_k*np.dot(NJ, dH)

        i += 1

    if (i == max_steps):
        print("Error : the algorithm did not converged within ", max_steps, " steps")
        return False

    else:
        th1, th2, th3, th4 = ((q[0, 0] + np.pi) % (2*np.pi)) - np.pi, ((q[1, 0] + np.pi) % (2*np.pi)) - \
            np.pi, ((q[2, 0] + np.pi) % (2*np.pi)) - \
            np.pi, ((q[3, 0] + np.pi) % (2*np.pi)) - np.pi
        if -np.pi/2 > th1 or np.pi/2 < th1:
            print("limite dépassée pour th1: " + str(th1))
            return False
        if -2.05 > th2 or 1.67 < th2:
            print("limite dépassée pour th2: " + str(th2))
            return False
        if -1.67 > th3 or 1.53 < th3:
            print("limite dépassée pour th3: " + str(th3))
            return False
        if -1.8 > th4 or 2 < th4:
            print("limite dépassée pour th4: " + str(th4))
            return False
        return True


#######################################################################################################################

#                                  Simulation des trajectoires pour la sécurité

#######################################################################################################################


def calCoeffs(tps, etat_init, etat_fin):
    A = np.array([[tps**3, tps**4, tps**5],
                  [3*(tps**2), 4*(tps**3), 5*(tps**4)],
                  [6*tps, 12*(tps**2), 20*(tps**3)]])
    b = np.array([etat_fin[0] - etat_init[0] - (etat_init[1]*tps + (0.5*etat_init[2]*(tps**2))),
                  etat_fin[1] - etat_init[1] - etat_init[2]*tps,
                  etat_fin[2] - etat_init[2]])

    x = np.linalg.solve(A, b)
    coefs = [x[2], x[1], x[0], 0.5*etat_init[2], etat_init[1], etat_init[0]]
    return coefs


def calc_etats(t, coefs):
    P = 0
    V = 0
    A = 0
    n = 5
    for i in coefs:
        P *= t
        if n:
            V *= t
            if n-1:
                A *= t
        P += i
        V += n*i
        A += (n-1)*n*i
        n -= 1
    return P, V, A


class point():
    def __init__(self, position_r, vitesse_r, accélération_r, position_z, vitesse_z, accélération_z, angle):
        self.angle = angle
        self.position = [position_r, position_z]
        self.vitesse = [vitesse_r, vitesse_z]
        self.accélération = [accélération_r, accélération_z]


def simu(acc_time, dec_time, start, vitesse_r, vitesse_z, rAcc, zAcc, rDec, zDec):

    coefs_acc_r = calCoeffs(acc_time, [
                            start.position[0], start.vitesse[0], start.accélération[0]], [rAcc, vitesse_r, 0.])
    coefs_dec_r = calCoeffs(dec_time, [rAcc, vitesse_r, 0.], [rDec, 0., 0.])
    coefs_acc_z = calCoeffs(acc_time, [
                            start.position[1], start.vitesse[1], start.accélération[1]], [zAcc, vitesse_z, 0.])
    coefs_dec_z = calCoeffs(dec_time, [zAcc, vitesse_z, 0.], [zDec, 0., 0.])

    angle = start.angle

    for t in np.linspace(0., acc_time + dec_time, 1000):
        if t <= acc_time:
            r_pos, r_v, r_a = calc_etats(t, coefs_acc_r)
            z_pos, z_v, z_a = calc_etats(t, coefs_acc_z)
            orientation = 45. * (1 - t/acc_time) + 45.
        elif t <= acc_time + dec_time:
            t_ = t - acc_time
            r_pos, r_v, r_a = calc_etats(t_, coefs_dec_r)
            z_pos, z_v, z_a = calc_etats(t_, coefs_dec_z)
            orientation = 45. * (1 - t_/dec_time)

        if (not (z_pos <= 0)) or MGI(r_pos*np.sin(start.angle), r_pos*np.cos(start.angle), z_pos, orientation):
            return False

    return True


def opti_temps(angle, vitesse):
    move_time = 1.0

    for temps in np.linspace(0.1, 2.0, 40):
        coefs = calCoeffs(temps/2, [-angle, 0., 0.], [0., vitesse, 0.])
        A = []
        for t in np.linspace(0., temps/2, 100):
            _, v, _ = calc_etats(t, coefs)
            A.append(v)
        if (-0.0001 < min(A) < 0.0001) and (-0.0001 < max(A) - vitesse < 0.0001):
            move_time = temps

    print("le temps de mouvement choisi est: " + str(move_time))
    return move_time


#######################################################################################################################

#                                        Calcul de la vitesse à atteindre

#######################################################################################################################

def sansFrottements(x, y, z, xlancer=0., ylancer=0., zlancer=0.477, alpha0=np.pi/2):
    '''modèle sans frottements qui renvoit la vitesse pour lâcher la balle

    paramètres:
        obligatoires:
            x, y, z: coordonnées cartésiennes du point à atteindre dans le repère du robot

        optionnels:
            xlancer, ylancer, zlancer: coordonnées cartésiennes du point de lancement
            alpha0: angle de la vitesse au moment du lâcher par rapport à l'horizontale

    sortie:
        v0: norme de la vitesse à donner à la balle au moment du lâcher   
    '''
    g = 9.81
    D = np.sqrt((xlancer - x)**2 + (ylancer - y)**2)
    v0 = np.sqrt(((g/2)*(D/np.sin(alpha0))**2 )/ (np.cos(alpha0)/np.sin(alpha0)*D + zlancer - z))
    print("la vitesse de lacher sans frottements: " + str(v0))
    return v0


#######################################################################################################################

#                                        Fonctions d'actionnement du bras

#######################################################################################################################


def prise(mouvement, ouverture, tps):
    '''méthode qui envoie une requête à la pince pour l'ouvrir ou la fermer

    paramètres:
        obligatoires:
            mouvement: noeud comportant le client /goal_tool_control
            ouverture: float compris entre -0.01 (fermeture maximale) et 0.01 (ouverture maximale)
            tps: temps pour exécuter l'action (0. donnera la fermeture la plus rapide possible)
    '''
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


def mouv(mouvement, x, y, z, th, tps, mgi=mgi):
    '''méthode qui permet de déplacer le bras à la position cartésienne demandée

    paramètres:
        obligatoires:
            mouvement: noeud comportant le client /goal_joint_space_path
            x, y, z: coordonnées cartésienne du point que l'on veut atteindre
            th: angle de la pince par rapport à l'horizontale (degrés), dans le repère cartésien. (90 correspond à  la pince vers le bas)
            tps: temps pour exécuter l'action (0. exécutera l'action le plus rapidement possible pour les moteurs, ce qui peut être assez brutal)

        optionels:
            mgi: modèle géométrique inverse à utiliiser, par défault le modéle utilise un calcul géométrique
    '''
    # appel du modèle géométrique inverse pour trouver les positions angulaires correspondantes
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


def trajectoire(mouvement, acc_time, dec_time, start, vitesse_r, vitesse_z, rAcc, zAcc, rDec, zDec, attente_ouv=0.04):
    '''méthode qui permet d'effectuer la trajectoire nommée courbe, celle-ci comprend une phase d'accélération
    jusqu'au lâcher de la balle, puis une phase de déccélération toutes deux calculées à l'aide d'une interpolation polynomiale.
    Le lâcher de la balle est coordonné au mouvement.

    paramètres:
        obligatoires:
            mouvement: noeud comportant le client /goal_drawing_trajectory
            acc_time: durée de la phase d'accélération
            dec_time: durée de la phase de décélération
            start: position du bras au début de la trajectoire
            vitesse_r: composante horizontale de la vitesse que l'on veut atteindre au moment du lâcher
            vitesse_z: composante verticale de la vitesse que l'on veut atteindre au moment du lâcher
            rAcc: écartement horizontal du bras avec l'origine du repére dans le plan du mouvement, au moment du lâcher (peut être négatif)
            zAcc: hauteur du bras au moment du lâcher
            rDec: écartement horizontal du bras avec l'origine du repére dans le plan du mouvement, au moment de l'arrêt du bras
            zDec: hauteur du bras au moment de l'arrêt du bras

        optionels:
            attente_ouv: anticipation dans l'envoie de la requête d'ouverture pour synchroniser le mouvement et le lâcher
    '''
    global enregistre

    # simulation du mouvement pour vérifier sa faisabilité
    if not simu(acc_time, dec_time, start, vitesse_r, vitesse_z, rAcc, zAcc, rDec, zDec):
        print("La simulation montre un problème")
        return

    # calcul des temps pou la coordination ouverture mouvement
    if acc_time < attente_ouv:
        tps_pince = 0.
        tps_traj = attente_ouv - acc_time
    else:
        tps_pince = acc_time - attente_ouv
        tps_traj = 0.

    # demande d'enregistrement des vitesses pendant le mouvement du bras
    enregistre.tps = acc_time + dec_time + 1.0
    rclpy.spin_once(enregistre)

    # mise en place de l'origine des temps et des flags pour coordonner le mouvement et l'ouverture
    origine = time.time()
    Flag = [False, False]

    # envoi des requêtes de manière coordonnée
    while not Flag[0] or not Flag[1]:
        if (time.time() - origine) > tps_pince and not Flag[0]:
            Flag[0] = True
            mouvement.prise_requete(0.01, tps)

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
                            'Ouverture de la pince en cours')
                    break

        if (time.time() - origine) > tps_traj and not Flag[1]:
            Flag[1] = True

            mouvement.traj_requete1(acc_time, dec_time,
                                    vitesse_r, vitesse_z, rAcc, zAcc, rDec, zDec)

            while rclpy.ok():
                rclpy.spin_until_future_complete(
                    mouvement, mouvement.traj_future)
                if mouvement.traj_future.done():

                    try:
                        réponse = mouvement.traj_future.result()
                    except Exception as e:
                        mouvement.get_logger().info(
                            'Erreur de prise%r' % (e,))
                    else:
                        mouvement.get_logger().info(
                            'Trajectoire demandée')
                    break


def depliage(mouvement, x, y, z, angle=25.):
    # controlle l'ouverture de la pince
    global enregistre

    angle = (np.pi * angle) / 180
    vitesse = sansFrottements(x, y, z)
    print("vitesse à atteindre: " + str(vitesse))
    move_time = opti_temps(angle, vitesse)


    attente_ouv = 0.
    acc_time = move_time/2

    if acc_time < attente_ouv:
        tps_pince = 0.
        tps_traj = attente_ouv - acc_time
    else:
        tps_pince = acc_time - attente_ouv
        tps_traj = 0.


    

    ######################################################################################
    # mise en posittion initiale

    mouvement.depl_requete(2*np.arctan(y/(x + np.sqrt(x**2 + y**2))), - (angle + 0.1857), -(angle + np.pi/2 - 0.1857), -angle, 6.0)
    time.sleep(7.0)

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

    ######################################################################################

    # demande d'enregistrement des vitesses pendant le mouvement du bras
    enregistre.tps = move_time + 1.0
    rclpy.spin_once(enregistre)

    # mise en place de l'origine des temps et des flags pour coordonner le mouvement et l'ouverture
    origine = time.time()
    Flag = [False, False]

    # envoi de la demande de trajectoire

    """mouvement.traj_requete2(move_time, angle, vitesse)

    while rclpy.ok():
        rclpy.spin_until_future_complete(
            mouvement, mouvement.traj_future)
        if mouvement.traj_future.done():

            try:
                réponse = mouvement.traj_future.result()
            except Exception as e:
                mouvement.get_logger().info(
                    'Erreur de prise%r' % (e,))
            else:
                mouvement.get_logger().info(
                    'Trajectoire demandée')
            break"""

    while not Flag[0] or not Flag[1]:
        if (time.time() - origine) > tps_pince and not Flag[0]:
            Flag[0] = True
            mouvement.prise_requete(0.01, tps)

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
                        mouvement.get_logger().info('Ouverture de la pince en cours')
                    break

        if (time.time() - origine) > tps_traj and not Flag[1]:
            Flag[1] = True

            mouvement.traj_requete2(move_time, angle, vitesse)

            while rclpy.ok():
                rclpy.spin_until_future_complete(
                    mouvement, mouvement.traj_future)
                if mouvement.traj_future.done():

                    try:
                        réponse = mouvement.traj_future.result()
                    except Exception as e:
                        mouvement.get_logger().info(
                            'Erreur de prise%r' % (e,))
                    else:
                        mouvement.get_logger().info(
                            'Trajectoire demandée')
                    break


def action(mouvement, opération, tps):
    global pos_act
    # décode l'action demandée et appel les clients nécessaires

    if opération == "fermer":
        ouverture = 0.0
        prise(mouvement, ouverture, tps)
    elif opération == "ouvrir":
        ouverture = 0.01
        prise(mouvement, ouverture, 0.001)

    elif opération == "lancer":
        start = point(
            np.sqrt(pos_act[0]**2 + pos_act[1]**2), 0., 0., pos_act[2], 0., 0., 0.)

        acc_time = 0.5
        dec_time = 0.3
        vitesse_r = 1.3
        vitesse_z = 0.2
        rAcc = 0.05
        zAcc = 0.4
        rDec = 0.15
        zDec = 0.35

        trajectoire(mouvement, acc_time, dec_time, start,
                    vitesse_r, vitesse_z, rAcc, zAcc, rDec, zDec)
        time.sleep(acc_time + dec_time + 1.0)
        mouv(mouvement, pos_act[0], pos_act[1],
             pos_act[2], pos_act[3], 2.0, mgi_lanc)

    elif opération == "depli":
        
        signal = Signal()
        nb = 0

        while nb < 3:
            
            rclpy.spin_once(signal)
            if signal.flag:

                mouv(mouvement, 0., 0.123, 0.065, 90, 1.0)
                time.sleep(1.0)
                prise(mouvement, 0., 1.0)
                time.sleep(1.0)
                mouv(mouvement, 0., 0.123, 0.1, 90, 1.0)
                time.sleep(1.0)

                depliage(mouvement, signal.x, signal.y, signal.z)
                time.sleep(3.0)
                mouv(mouvement, pos_act[0], pos_act[1], pos_act[2], pos_act[3], 3.0)

                signal.flag = False
                nb += 1
        
        signal.destroy_node()

    elif opération[0] == "ang":
        th1, th2, th3, th4 = opération[1:]

        mouvement.get_logger().info("point d'arrivée:" +
                                    str(mgd([th1, th2, th3, th4])))
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
        pos_act = mgd([th1, th2, th3, th4])
    else:
        x, y, z, th = opération
        mouv(mouvement, x, y, z, th, tps)
        pos_act = opération


#######################################################################################################################

#                                             Algorithme de commande

#######################################################################################################################


def main(args=None):
    global pos_act, Actions, tps, enregistre

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
    vit = 0.05
    tps = 2.0

    # mise en position initiale
    pos_act = [0.1, 0., 0.1, 90]
    mouv(mouvement, pos_act[0], pos_act[1], pos_act[2], pos_act[3], tps)

    # actions initiales
    Actions.append("ouvrir")
    Actions.append((0., 0.123, 0.1, 90))
    Actions.append("depli")

    # boucle de décision
    while(rclpy.ok()):
        if not Actions:
            break

        elif time.time() - prec_tps > tps:
            prec_tps = time.time()
            opération = Actions.pop(0)
            if not type(opération) == str:
                if opération[0] == "ang":
                    tps = distance(pos_act, mgd(opération[1:]))/vit
                else:
                    tps = distance(pos_act, opération)/vit
            else:
                tps = 1.0
            print(opération, tps)
            action(mouvement, opération, tps)

    # fermeture de la structure ROS2
    cam.destroy_node()
    mouvement.destroy_node()
    enregistre.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
