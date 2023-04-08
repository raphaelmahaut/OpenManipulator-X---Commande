# Importations des modules

import numpy as np
import matplotlib.pyplot as plt

# Défintion des paramètres de DH du bras

l1 = 0.0765
l2 = 0.130
l3 = 0.124

l4 = 0.1466  # vraie longueur correspondant au bout de la pince
# l4=0.126 # longueur vers le point au milieu de la pince utilisée dans le logiciel
alpha = np.arcsin(0.024/0.130)

# Définition des matrices de passage d'un repère au suivant


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

    T01 = passage(q1, -np.pi/2, l1, 0.012)
    T12 = passage(q2 - np.pi/2 + alpha, 0, 0, l2)
    T23 = passage(q3 + np.pi/2 - alpha, 0, 0, l3)
    T34 = passage(q4, 0, 0, l4)

    TCP = np.dot(T01, np.dot(T12, np.dot(T23, T34)))

    return np.array([[TCP[0, 3]], [TCP[1, 3]], [TCP[2, 3]]])

# Définition de la matrice Jacobienne : reprise des lignes 1,2,3 & 5 de la matrice Jacobienne calculée dans le programme "Calcul matrices de transformations et jacobiens"


def Jacob(qk):
    J = np.array([[-(1.0*l2*np.sin(alpha + qk[1, 0]) + 1.0*l3*np.cos(qk[1, 0] + qk[2, 0]) + 1.0*l4*np.cos(qk[1, 0] + qk[2, 0] + qk[3, 0]) + 0.012)*np.sin(qk[0, 0]), 1.0*(l2*np.cos(alpha + qk[1, 0]) - l3*np.sin(qk[1, 0] + qk[2, 0]) - l4*np.sin(qk[1, 0] + qk[2, 0] + qk[3, 0]))*np.cos(qk[0, 0]), -1.0*(l3*np.sin(qk[1, 0] + qk[2, 0]) + l4*np.sin(qk[1, 0] + qk[2, 0] + qk[3, 0]))*np.cos(qk[0, 0]), -1.0*l4*np.sin(qk[1, 0] + qk[2, 0] + qk[3, 0])*np.cos(qk[0, 0])], [(1.0*l2*np.sin(alpha + qk[1, 0]) + 1.0*l3*np.cos(qk[1, 0] + qk[2, 0]) + 1.0*l4*np.cos(qk[1, 0] + qk[2, 0] +
                 qk[3, 0]) + 0.012)*np.cos(qk[0, 0]), 1.0*(l2*np.cos(alpha + qk[1, 0]) - l3*np.sin(qk[1, 0] + qk[2, 0]) - l4*np.sin(qk[1, 0] + qk[2, 0] + qk[3, 0]))*np.sin(qk[0, 0]), -1.0*(l3*np.sin(qk[1, 0] + qk[2, 0]) + l4*np.sin(qk[1, 0] + qk[2, 0] + qk[3, 0]))*np.sin(qk[0, 0]), -1.0*l4*np.sin(qk[1, 0] + qk[2, 0] + qk[3, 0])*np.sin(qk[0, 0])], [0, -l2*np.sin(alpha + qk[1, 0]) - l3*np.cos(qk[1, 0] + qk[2, 0]) - l4*np.cos(qk[1, 0] + qk[2, 0] + qk[3, 0]), -l3*np.cos(qk[1, 0] + qk[2, 0]) - l4*np.cos(qk[1, 0] + qk[2, 0] + qk[3, 0]), -l4*np.cos(qk[1, 0] + qk[2, 0] + qk[3, 0])]])
    return(J)

# Définition du modèle géométrique inverse


def MGI(final_pose, q0=np.array([[0.1], [-0.4], [0.4], [1.3]]), alpha_k=1, alpha_k2=0.1, tol=0.001, Dthe4=0.4):
    q = q0
    p0 = MGD(q0)
    pose_error = abs(final_pose-p0)
    max_pose_error = pose_error.max()
    max_steps = 500
    i = 0

    while(max_pose_error > tol and i < max_steps):

        J = Jacob(q)
        J_JT = np.dot(J, np.transpose(J))
        J_pseudo_inv = np.dot(np.transpose(J), np.linalg.inv(J_JT))

        qmax = np.array([[np.pi/2], [np.pi/2], [np.pi/2],
                        [np.pi/2 - q[1, 0] - q[2, 0] + Dthe4]])
        qmin = np.array([[-np.pi/2], [-np.pi/2], [-np.pi/2],
                        [np.pi/2 - q[1, 0] - q[2, 0] - Dthe4]])
        qmean = 0.5*(qmax + qmin)
        qdiff = qmax - qmin
        NJ = np.eye(4) - np.dot(J_pseudo_inv, J)
        dH = -2*alpha_k*(q - qmean) / (qdiff*qdiff)

        pose = MGD(q)
        pose_error = final_pose - pose
        max_pose_error = (abs(pose_error)).max()

        q += alpha_k2*np.dot(J_pseudo_inv, pose_error) + np.dot(NJ, dH)

        i += 1

    if (i == max_steps):
        print("Error : the algorithm did not converged within ", max_steps, " steps")

    else:
        print("Convergence en "+str(i)+" étapes"+"\n")
        return (q)

# Tracé de la position du robot


def trace(q):
    Pos = MGD(q)
    xf = np.sqrt((Pos[0, 0])*(Pos[0, 0]) + Pos[1, 0]*Pos[1, 0])
    x0 = 0.012
    z0 = 0
    x1 = x0
    z1 = l1
    x2 = x1+l2*np.sin(q[1, 0] + alpha)
    z2 = z1+l2*np.cos(q[1, 0] + alpha)
    x2bis = x1+0.128*np.sin(q[1, 0])
    z2bis = z1+0.128*np.cos(q[1, 0])
    x3 = x2+l3*np.cos(-(q[1, 0] + q[2, 0]))
    z3 = z2+l3*np.sin(-(q[1, 0] + q[2, 0]))
    x4 = x3+l4*np.cos(-(q[1, 0] + q[2, 0] + q[3, 0]))
    z4 = z3+l4*np.sin(-(q[1, 0] + q[2, 0] + q[3, 0]))

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

# Objectif de position (x,y,z) à atteindre en cm


final_pose = np.array([[0.15], [0.], [0.25]])
# conditions initiales de l'algorithme de descente du gradient
qf = MGI(final_pose, q0=np.array([[0.1], [-0.4], [0.4], [1.3]]))

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
