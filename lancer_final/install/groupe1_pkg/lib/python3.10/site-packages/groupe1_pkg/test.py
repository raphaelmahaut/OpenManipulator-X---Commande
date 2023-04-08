import numpy as np
import matplotlib.pyplot as plt

# mettre 0.012 si on prend en compte l'écart à l'origines
Dim = [0.0765, 0.13, 0.124, 0.1466, 0.]

#######################################
# Défintion des paramètres de DH du bras
# Dim[3]=0.126 # longueur vers le point au milieu de la pince utilisée dans le logiciel

alpha = np.arcsin(0.024/Dim[1])

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


def MGI(final_pose, q0=np.array([[0.1], [-0.4], [0.4], [1.3]]), alpha_k=2.0, alpha_k2=0.1, tol=0.001, Dthe4=0.4):
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
        dH = -((q - qmean)/qdiff)

        pose = MGD(q)
        pose_error = abs(final_pose - pose)
        max_pose_error = pose_error.max()

        q += alpha_k2*np.dot(J_pseudo_inv, pose_error) + alpha_k*np.dot(NJ, dH)

        i += 1

    if (i == max_steps):
        print("Error : the algorithm did not converged within ", max_steps, " steps")

    else:
        th1, th2, th3, th4 = ((q[0, 0] + np.pi) % (2*np.pi)) - np.pi, ((q[1, 0] + np.pi) % (2*np.pi)) - \
            np.pi, ((q[2, 0] + np.pi) % (2*np.pi)) - \
            np.pi, ((q[3, 0] + np.pi) % (2*np.pi)) - np.pi
        if -np.pi/2 > th1 or np.pi/2 < th1:
            print("limite dépassée pour th1: " + str(th1))
        if -2.05 > th2 or 1.67 < th2:
            print("limite dépassée pour th2: " + str(th2))
        if -1.67 > th3 or 1.53 < th3:
            print("limite dépassée pour th3: " + str(th3))
        if -1.8 > th4 or 2 < th4:
            print("limite dépassée pour th4: " + str(th4))
        return th1, th2, th3, th4


position = np.array([[0.1], [-0.4], [0.4], [1.3]])


def mgi_jac(x, y, z, x_d=0, y_d=0, z_d=0):
    global position
    qf = MGI(np.array([[x], [y], [z]]), position)
    position = qf
    J = Jacob(qf)
    J_JT = np.dot(J, np.transpose(J))
    J_pseudo_inv = np.dot(np.transpose(J), np.linalg.inv(J_JT))
    result = np.dot(J_pseudo_inv, np.array([[x_d], [y_d], [z_d]]))
    return qf[0, 0], qf[1, 0], qf[2, 0],  qf[3, 0], result[0, 0], result[1, 0], result[2, 0],  result[3, 0]


def trace(q):
    x0 = Dim[4]
    z0 = 0
    x1 = x0
    z1 = Dim[0]
    x2 = x1 + Dim[1]*np.sin(q[1] + alpha)
    z2 = z1 + Dim[1]*np.cos(q[1] + alpha)
    x2bis = x1 + 0.128*np.sin(q[1])
    z2bis = z1 + 0.128*np.cos(q[1])
    x3 = x2 + Dim[2]*np.cos(-(q[1] + q[2]))
    z3 = z2 + Dim[2]*np.sin(-(q[1] + q[2]))
    x4 = x3 + Dim[3]*np.cos(-(q[1] + q[2] + q[3]))
    z4 = z3 + Dim[3]*np.sin(-(q[1] + q[2] + q[3]))

    plt.scatter([x0, x1, x2, x3], [z0, z1, z2, z3], c='blue', s=20)

    plt.plot([x0, x1, x2bis, x2, x3, x4], [z0, z1, z2bis, z2, z3, z4], "b")

    plt.axis('equal')
    plt.title("Position du robot")
    plt.show()


def mgd(pos_rob):
    # calcule la position en bout de pince à partir des positions fournies
    global Dim

    alpha = np.arcsin(0.024/Dim[1])
    beta = np.arctan(Dim[4]/Dim[3])

    z1 = Dim[0] + Dim[1]*np.cos(pos_rob[1] + alpha) + Dim[2]*np.cos(pos_rob[1] + pos_rob[2] +
                                                                    np.pi/2) + Dim[3]*np.cos(pos_rob[1] + pos_rob[2] + np.pi/2 + pos_rob[3] + beta)
    z2 = Dim[0] + Dim[1]*np.cos(pos_rob[1] + alpha) + Dim[2]*np.cos(pos_rob[1] + pos_rob[2] +
                                                                    np.pi/2) + Dim[3]*np.cos(pos_rob[1] + pos_rob[2] + np.pi/2 + pos_rob[3] - beta)
    r = Dim[1]*np.sin(pos_rob[1] + alpha) + Dim[2]*np.sin(pos_rob[1] + pos_rob[2] +
                                                          np.pi/2) + Dim[3]*np.sin(pos_rob[1] + pos_rob[2] + np.pi/2 + pos_rob[3])
    x, y = r*np.cos(pos_rob[0]), r*np.sin(pos_rob[0])
    return round(x, 3), round(y, 3), round((z1+z2)/2, 3), round(180*(pos_rob[1] + pos_rob[2] + pos_rob[3])/np.pi, 2)


def mgi_grip(x, y, z, th, x_d=0, y_d=0, z_d=0, th_d=0):
    # calcule la position angulaire à partir de la position cartésienne voulue
    global Dim

    if z < 0:
        return "trop bas"
    # petit angle du au décalge de l'axe du deuxième segment (rad)
    alpha = np.arcsin(0.024/Dim[1])
    th_rad = np.pi*th/180  # angle de la pince dans un repère cartésien
    th_d_rad = np.pi*th_d/180

    def angle_triangle(a, b, c):
        return np.arccos((a**2 + b**2 - c**2)/(2*a*b))

    def angle_triangle_d(a, b, c, a_d, b_d, c_d):
        arccos_d = 1/(np.sqrt(1 + (a**2 + b**2 - c**2)/(2*a*b)))
        duv = (a_d*a + b_d*b - c*c_d)/(a*b)
        dvu = -2*(a**2 + b**2 - c**2)*(a_d*b + b_d*a)/((2*a*b)**2)
        return arccos_d*(duv + dvu)

    R = np.sqrt(x**2 + y**2)
    R_d = (y_d*y + x_d*x)/(np.sqrt(x**2 + y**2))
    th1 = 2*np.arctan(y/(x + R))
    th1_d = 2*((y_d/(x + R)) + (y*(x_d + R_d) /
                                ((x + R)**2))) / (1 + (y/(x + R))**2)

    # calcul des grandeurs utiles
    zb = z + Dim[3]*np.sin(th_rad) - Dim[0]
    r = np.sqrt(x**2 + y**2) - Dim[3]*np.cos(th_rad)
    hp = np.sqrt(zb**2 + r**2)

    zb_d = z_d + Dim[3]*np.cos(th_rad)*th_d_rad
    r_d = (y_d*y + x_d*x)/(np.sqrt(x**2 + y**2)) + \
        Dim[3]*np.sin(th_rad)*th_d_rad
    hp_d = (y_d*y + x_d*x)/(np.sqrt(x**2 + y**2))

    # angle entre l'axe de la pince et l'oigine du bras
    beta = 2*np.arctan(r/(zb + hp))
    beta_d = 2*((r_d/(zb + hp)) + (r*(zb_d + hp_d) /
                ((zb + hp)**2))) / (1 + (r/(zb + hp))**2)

    if hp > Dim[1] + Dim[2]:  # la position demandée est trop éloignée de la base
        return "position trop éloignée"

    if 0.02 > hp:  # la position est trop proche de la base
        return "collision"

    th2 = beta - (alpha + angle_triangle(Dim[1], hp, Dim[2]))
    th2_d = beta_d - angle_triangle_d(Dim[1], hp, Dim[2], 0, hp_d, 0)
    th3 = (np.pi/2 + alpha) - angle_triangle(Dim[1], Dim[2], hp)
    th3_d = -angle_triangle_d(Dim[1], Dim[2], hp, 0, 0, hp_d)
    th4 = th_rad - (th2 + th3)
    th4_d = th_d_rad - (th2_d + th3_d)

    angle_test = 2*np.arctan(np.sqrt(x**2 + y**2) /
                             (z - Dim[0] + np.sqrt((z - Dim[0])**2 + x**2 + y**2)))
    if (np.pi/2 - angle_test < th1):
        return "croisement des bras"

    return th1, th2, th3, th4, th1_d, th2_d, th3_d, th4_d


def mgi_1(x, y, z, th):
    # calcule la position angulaire à partir de la position cartésienne voulue
    global Dim

    # petit angle du au décalge de l'axe du deuxième segment (rad)
    alpha = np.arcsin(0.024/Dim[1])
    th_rad = np.pi*th/180  # angle du premier segment dans un repère cartésien

    def angle_triangle(a, b, c):
        return np.arccos((a**2 + b**2 - c**2)/(2*a*b))

    th1 = 2*np.arctan(y/(x + np.sqrt(x**2 + y**2)))
    th2 = th_rad

    zb = z - (Dim[1]*np.cos(th_rad + alpha) + Dim[0])
    r = np.sqrt(x**2 + y**2) - Dim[1]*np.sin(th_rad + alpha)
    hp = np.sqrt(zb**2 + r**2)

    # angle entre l'axe de la pince et l'oigine du bras
    beta = 2*np.arctan(r/(zb + np.sqrt(zb**2 + r**2))) - (th_rad + alpha)

    if hp > Dim[2] + Dim[3]:  # la position demandée est trop éloignée de la base
        return "position trop éloignée"

    if 0.01 > hp:  # la position est trop proche de la base
        return "collision"

    th3 = beta - (np.pi/2 - alpha + angle_triangle(hp, Dim[2], Dim[3]))
    th4 = np.pi - angle_triangle(Dim[2], Dim[3], hp)

    return th1, th2, th3, th4


def mgi_grip_inv(x, y, z, th, x_d=0, y_d=0, z_d=0, th_d=0):
    # calcule la position angulaire à partir de la position cartésienne voulue
    global Dim

    if z < 0:
        print("trop bas")
        return None
    # petit angle du au décalge de l'axe du deuxième segment (rad)
    alpha = np.arcsin(0.024/Dim[1])
    th_rad = np.pi*th/180  # angle de la pince dans un repère cartésien
    th_d_rad = np.pi*th_d/180

    def angle_triangle(a, b, c):
        return np.arccos((a**2 + b**2 - c**2)/(2*a*b))

    def angle_triangle_d(a, b, c, a_d, b_d, c_d):
        arccos_d = 1/(np.sqrt(1 + (a**2 + b**2 - c**2)/(2*a*b)))
        duv = (a_d*a + b_d*b - c*c_d)/(a*b)
        dvu = -2*(a**2 + b**2 - c**2)*(a_d*b + b_d*a)/((2*a*b)**2)
        return arccos_d*(duv + dvu)

    R = np.sqrt(x**2 + y**2)
    R_d = (y_d*y + x_d*x)/(np.sqrt(x**2 + y**2))
    th1 = np.arctan(y/x)
    th1_d = ((y_d/x) + (y*x_d)/(x**2)) / (1 + (y/x)**2)

    # calcul des grandeurs utiles
    zb = z + Dim[3]*np.sin(th_rad) - Dim[0]
    r = np.sign(x)*np.sqrt(x**2 + y**2) - Dim[3]*np.cos(th_rad)
    hp = np.sqrt(zb**2 + r**2)

    zb_d = z_d + Dim[3]*np.cos(th_rad)*th_d_rad
    r_d = np.sign(x)*(y_d*y + x_d*x)/(np.sqrt(x**2 + y**2)) + \
        Dim[3]*np.sin(th_rad)*th_d_rad
    hp_d = (y_d*y + x_d*x)/(np.sqrt(x**2 + y**2))

    # angle entre l'axe de la pince et l'oigine du bras
    beta = 2*np.arctan(r/(zb + hp))
    beta_d = 2*((r_d/(zb + hp)) + (r*(zb_d + hp_d) /
                ((zb + hp)**2))) / (1 + (r/(zb + hp))**2)

    if hp > Dim[1] + Dim[2]:  # la position demandée est trop éloignée de la base
        print("position trop éloignée")
        return None

    if 0.02 > hp:  # la position est trop proche de la base
        print("collision")
        return None

    th2 = beta - (alpha + angle_triangle(Dim[1], hp, Dim[2]))
    th2_d = beta_d - angle_triangle_d(Dim[1], hp, Dim[2], 0, hp_d, 0)
    th3 = (np.pi/2 + alpha) - angle_triangle(Dim[1], Dim[2], hp)
    th3_d = -angle_triangle_d(Dim[1], Dim[2], hp, 0, 0, hp_d)
    th4 = th_rad - (th2 + th3)
    th4_d = th_d_rad - (th2_d + th3_d)

    angle_test = 2*np.arctan(np.sqrt(x**2 + y**2) /
                             (z - Dim[0] + np.sqrt((z - Dim[0])**2 + x**2 + y**2)))
    """if (angle_test < th2):
        print(angle_test, th2)
        _, _, Z, R = mgd([th1, th2, th3, th4])
        plt.plot(R, Z)
        plt.axis("equal")
        plt.show()
        print("croisement des bras")
        return None"""

    return th1, th2, th3, th4, th1_d, th2_d, th3_d, th4_d


"""qf = MGI(np.array([[0.15], [0.], [0.25]]))
trace(qf)"""
print(mgd([0., -(np.arcsin(0.024/Dim[1]) - 0.),
      np.arcsin(0.024/Dim[1])-(np.pi/2 - 0.), 0.]))
print(np.arcsin(0.024/Dim[1]))
