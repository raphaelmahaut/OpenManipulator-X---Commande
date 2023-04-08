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

###########################################################################################
#                                                                                         #
#                                       MGI                                               #
#                                                                                         #
###########################################################################################

from numpy import*
import matplotlib.pyplot as plt

# Défintion des paramètres de DH du bras

l1 = 0.077
l2 = 0.130
l3 = 0.124
l4 = 0.147
# l4=0.147 : VRAIE LONGUEUR BOUT DE PINCE
alpha = arcsin(0.024/0.130)

# Définition des matrices de passage d'un repère au suivant


def passage(qi, alpha, d, a):
    T = eye(4)
    T[0, 0] = cos(qi)
    T[0, 1] = -cos(alpha)*sin(qi)
    T[0, 2] = sin(alpha)*sin(qi)
    T[0, 3] = a*cos(qi)
    T[1, 0] = sin(qi)
    T[1, 1] = cos(alpha)*cos(qi)
    T[1, 2] = -cos(qi)*sin(alpha)
    T[1, 3] = a*sin(qi)
    T[2, 1] = sin(alpha)
    T[2, 2] = cos(alpha)
    T[2, 3] = d
    return (T)

# Définition du modèle géométrique direct


def MGD(q):
    q1, q2, q3, q4 = q[0, 0], q[1, 0], q[2, 0], q[3, 0]
    T01 = passage(q1, -pi/2, l1, 0)
    T12 = passage(q2-pi/2+alpha, 0, 0, l2)
    T23 = passage(q3+pi/2-alpha, 0, 0, l3)
    T34 = passage(q4, 0, 0, l4)
    TCP = dot(T01, dot(T12, dot(T23, T34)))
    return array([[TCP[0, 3]], [TCP[1, 3]], [TCP[2, 3]]])

# Définition du modèle géométrique inverse


def Jacob(qk):
    J = array([[-(1.0*l2*sin(alpha + qk[1, 0]) + 1.0*l3*cos(qk[1, 0] + qk[2, 0]) + 1.0*l4*cos(qk[1, 0] + qk[2, 0] + qk[3, 0]) + 0.0119)*sin(qk[0, 0]), 1.0*(l2*cos(alpha + qk[1, 0]) - l3*sin(qk[1, 0] + qk[2, 0]) - l4*sin(qk[1, 0] + qk[2, 0] + qk[3, 0]))*cos(qk[0, 0]), -1.0*(l3*sin(qk[1, 0] + qk[2, 0]) + l4*sin(qk[1, 0] + qk[2, 0] + qk[3, 0]))*cos(qk[0, 0]), -1.0*l4*sin(qk[1, 0] + qk[2, 0] + qk[3, 0])*cos(qk[0, 0])], [(1.0*l2*sin(alpha + qk[1, 0]) + 1.0*l3*cos(qk[1, 0] + qk[2, 0]) + 1.0*l4*cos(qk[1, 0] + qk[2, 0] +
              qk[3, 0]) + 0.0119)*cos(qk[0, 0]), 1.0*(l2*cos(alpha + qk[1, 0]) - l3*sin(qk[1, 0] + qk[2, 0]) - l4*sin(qk[1, 0] + qk[2, 0] + qk[3, 0]))*sin(qk[0, 0]), -1.0*(l3*sin(qk[1, 0] + qk[2, 0]) + l4*sin(qk[1, 0] + qk[2, 0] + qk[3, 0]))*sin(qk[0, 0]), -1.0*l4*sin(qk[1, 0] + qk[2, 0] + qk[3, 0])*sin(qk[0, 0])], [0, -l2*sin(alpha + qk[1, 0]) - l3*cos(qk[1, 0] + qk[2, 0]) - l4*cos(qk[1, 0] + qk[2, 0] + qk[3, 0]), -l3*cos(qk[1, 0] + qk[2, 0]) - l4*cos(qk[1, 0] + qk[2, 0] + qk[3, 0]), -l4*cos(qk[1, 0] + qk[2, 0] + qk[3, 0])]])
    return(J)


def MGI(final_pose, q0=array([[0], [-0.6], [0.6], [1.3]]), alpha_k=1, alpha_k2=0.1, tol=0.001, Dthe4=0.4):
    q = q0
    p0 = MGD(q0)
    pose_error = abs(final_pose-p0)
    max_pose_error = pose_error.max()
    max_steps = 500
    i = 0

    while(max_pose_error > tol and i < max_steps):

        J = Jacob(q)
        J_JT = dot(J, transpose(J))
        J_pseudo_inv = dot(transpose(J), linalg.inv(J_JT))

        qmax = array([[pi/2], [pi/2], [pi/2], [pi/2-q[1, 0]-q[2, 0]+Dthe4]])
        qmin = array([[-pi/2], [-pi/2], [-pi/2], [pi/2-q[1, 0]-q[2, 0]-Dthe4]])
        qmean = 0.5*(qmax+qmin)
        qdiff = qmax-qmin
        NJ = eye(4)-dot(J_pseudo_inv, J)
        dH = -2*alpha_k*(q-qmean)/(qdiff*qdiff)

        pose = MGD(q)
        pose_error = final_pose - pose
        max_pose_error = (abs(pose_error)).max()
        q = q+alpha_k2*dot(J_pseudo_inv, pose_error)+dot(NJ, dH)
        print(max_pose_error)

        i += 1

    if (i == max_steps):
        print("Error : the algorithm did not converged within ", max_steps, " steps")

    else:
        print("Convergence en "+str(i)+" étapes"+"\n")
        return (q)

# Tracé de la position du robot


def trace(q):
    Pos = MGD(q)
    xf = sqrt((Pos[0, 0])*(Pos[0, 0])+Pos[1, 0]*Pos[1, 0])
    x0 = 0.0119
    z0 = 0
    x1 = x0
    z1 = l1
    x2 = x1+l2*sin(q[1, 0]+alpha)
    z2 = z1+l2*cos(q[1, 0]+alpha)
    x2bis = x1+0.128*sin(q[1, 0])
    z2bis = z1+0.128*cos(q[1, 0])
    x3 = x2+l3*cos(-(q[1, 0]+q[2, 0]))
    z3 = z2+l3*sin(-(q[1, 0]+q[2, 0]))
    x4 = x3+l4*cos(-(q[1, 0]+q[2, 0]+q[3, 0]))
    z4 = z3+l4*sin(-(q[1, 0]+q[2, 0]+q[3, 0]))

    plt.scatter(xf, Pos[2, 0], c='red', marker='s')
    plt.scatter(x0, z0, c='blue', s=20)
    plt.scatter(x1, z1, c='blue', s=20)
    plt.scatter(x2, z2, c='blue', s=20)
    plt.scatter(x3, z3, c='blue', s=20)

    plt.plot([x0, x1], [z0, z1], "b")
    plt.plot([x1, x2bis], [z1, z2bis], "b")
    plt.plot([x2bis, x2], [z2bis, z2], "b")
    plt.plot([x2, x3], [z2, z3], "b")
    plt.plot([x3, x4], [z3, z4], "b")

    plt.axis('equal')
    plt.title(label="Position du robot")
    plt.show()


# Objectif de position à atteindre en cm
final_pose = array([[0], [0.09], [0.07]])
qf = MGI(final_pose, q0=array([[0.1], [-0.4], [0.4], [1.3]]))

# Communication des résultats

print("Position cartésienne (cm)")
print("x = "+str(final_pose[0, 0])+" ----> "+str(round(MGD(qf)[0, 0], 5)))
print("y = "+str(final_pose[1, 0])+" ----> "+str(round(MGD(qf)[1, 0], 5)))
print("z = "+str(final_pose[2, 0])+" ----> "+str(round(MGD(qf)[2, 0], 5)))
print("\n"+"Avec les paramètres angulaires suivants (rad)")
print("q1 = "+str(round(qf[0, 0], 5)))
print("q2 = "+str(round(qf[1, 0], 5)))
print("q3 = "+str(round(qf[2, 0], 5)))
print("q4 = "+str(round(qf[3, 0], 5)))
trace(qf)

###########################################################################################
#                                                                                         #
#                                       CONTROLE                                          #
#                                                                                         #
###########################################################################################


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


def trajectoire(A, B, hauteur, fonction='ligne', n=1):
    global Actions
    # renvoie un ensemble de points définissant une trajectoire et le temps pour atteindre cette position

    if fonction == 'cadre':
        P = [A, (A[0], A[1], hauteur), (B[0], B[1], hauteur), B]
        n = n//3 + 1
    elif fonction == 'ligne':
        P = [A, B]

    Actions.append(((A[0], A[1], A[2]), 0.8))
    for i in range(len(P) - 1):
        Actions.extend([((P[i+1][0]*k/n + P[i][0]*(1-k/n), P[i+1][1]*k/n + P[i][1]*(1-k/n),
                         P[i+1][2]*k/n + P[i][2]*(1-k/n)), 0.8) for k in range(1, n+1)])


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
        x, y, z = opération
        mouv(mouvement, x, y, z, tps)
        pos_act = opération


def mouv(mouvement, x, y, z, tps):
    # contrôle le déplacement de la pince en cartésien

    # convertit les coordonnées en position angulaire
    print(x, y, z)
    res = MGI(np.array([[x], [y], [z]]))
    th1, th2, th3, th4 = res[0, 0], res[0, 1], res[0, 2], res[0, 3]

    mouvement.get_logger().info("objectif:" + str((x, y, z)) +
                                "\n point d'arrivée:" + str(MGD([th1, th2, th3, th4])))

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
                   placement[n//hmax - 1][1], 0.02), 2.0))
    Actions.append(((placement[n//hmax - 1][0],
                   placement[n//hmax - 1][1], 0.), 1.0))

    Actions.append(("ouvrir", 1.0))

    # mise à jour de la position des planches
    Pla[-hmax:] = [(placement[n//hmax - 1][0], placement[n//hmax - 1][1], (i % hmax)*0.008)
                   for i in range(hmax)]


def bougeage(A, B):
    global Actions, pos_act
    # bouge une planche qui ne serait pas prenable par la pince en la percutant latéralement

    trajectoire(pos_act, A, hauteur, "cadre", 1)
    Actions.append(("fermer", 1.0))
    trajectoire(A, B, hauteur, "ligne", 20)
    Actions.append(("ouvrir", 1.0))
    trajectoire(B, (0., 0.17, 0.05), hauteur, "cadre", 1)


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
    pos_act = [0., 0.2, 0.05]
    mouv(mouvement, pos_act[0], pos_act[1], pos_act[2], tps)
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
