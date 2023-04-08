import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection

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


def MGI(x, y, z, x_d=0., y_d=0., z_d=0., q0=np.array([[0.1], [-0.4], [0.4], [1.3]]), alpha_k=1.0, alpha_k2=0.1, tol=0.001, Dthe4=0.4):

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

        J = Jacob(q)
        J_JT = np.dot(J, np.transpose(J))
        J_pseudo_inv = np.dot(np.transpose(J), np.linalg.inv(J_JT))
        result = np.dot(J_pseudo_inv, np.array([[x_d], [y_d], [z_d]]))

        print(i)

        return th1, th2, th3, th4, result[0, 0], result[1, 0], result[2, 0],  result[3, 0]


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

#######################################


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


def mgd(pos_rob):
    # calcule la position en bout de pince à partir des positions fournies
    global Dim

    alpha = np.arcsin(0.024/Dim[1])

    Z = [0, Dim[0]]
    R = [0, 0]

    Z.append(Z[-1] + Dim[1]*np.cos(pos_rob[1] + alpha))
    R.append(R[-1] + Dim[1]*np.sin(pos_rob[1] + alpha))

    Z.append(Z[-1] + Dim[2]*np.cos(pos_rob[1] + pos_rob[2] + np.pi/2))
    R.append(R[-1] + Dim[2]*np.sin(pos_rob[1] + pos_rob[2] + np.pi/2))

    Z.append(Z[-1] + Dim[3]*np.cos(pos_rob[1] +
             pos_rob[2] + np.pi/2 + pos_rob[3]))
    R.append(R[-1] + Dim[3]*np.sin(pos_rob[1] +
             pos_rob[2] + np.pi/2 + pos_rob[3]))
    X = []
    Y = []
    for r in R:
        X.append(np.sin(pos_rob[0])*r)
        Y.append(np.cos(pos_rob[0])*r)

    return X, Y, Z, R


def mgi_grip_inv_rad(x, y, z, x_d=0, y_d=0, z_d=0):
    # calcule la position angulaire à partir de la position cartésienne voulue
    global Dim

    if z < 0:
        print("trop bas")
        return None
    # petit angle du au décalge de l'axe du deuxième segment (rad)
    alpha = np.arcsin(0.024/Dim[1])
    R = np.sqrt(x**2 + y**2)
    R_d = (x_d*x + y_d*y)/R
    Z = z - Dim[0]
    H = np.sqrt(Z**2 + R**2)
    H_d = (z_d*Z + R_d*R)/H

    # angle de la pince dans un repère cartésien
    th_rad = 2*np.arctan(Z/(R + H))
    th_d_rad = 2*((z_d/(R + H)) + (Z*(R_d + H_d) /
                                   ((R + H)**2))) / (1 + (Z/(R + H))**2)

    def angle_triangle(a, b, c):
        return np.arccos((a**2 + b**2 - c**2)/(2*a*b))

    def angle_triangle_d(a, b, c, a_d, b_d, c_d):
        arccos_d = 1/(np.sqrt(1 + (a**2 + b**2 - c**2)/(2*a*b)))
        duv = (a_d*a + b_d*b - c*c_d)/(a*b)
        dvu = -2*(a**2 + b**2 - c**2)*(a_d*b + b_d*a)/((2*a*b)**2)
        return arccos_d*(duv + dvu)

    th1 = np.arctan(y/x)
    th1_d = ((y_d/x) - (y*x_d)/(x**2)) / (1 + (y/x)**2)

    # calcul des grandeurs utiles
    zb = z + Dim[3]*np.sin(th_rad) - Dim[0]
    r = np.sign(x)*R - Dim[3]*np.cos(th_rad)
    hp = np.sqrt(zb**2 + r**2)

    zb_d = z_d + Dim[3]*np.cos(th_rad)*th_d_rad
    r_d = np.sign(x)*(y_d*y + x_d*x)/(np.sqrt(x**2 + y**2)) + \
        Dim[3]*np.sin(th_rad)*th_d_rad
    hp_d = (zb_d*zb + r_d*r)/(np.sqrt(zb**2 + r**2))

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

    return th1, th2, th3, th4, th1_d, th2_d, th3_d, th4_d


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

    th1 = np.arctan(y/x)
    th1_d = ((y_d/x) - (y*x_d)/(x**2)) / (1 + (y/x)**2)

    # calcul des grandeurs utiles
    zb = z + Dim[3]*np.sin(th_rad) - Dim[0]
    r = np.sign(x)*np.sqrt(x**2 + y**2) - Dim[3]*np.cos(th_rad)
    hp = np.sqrt(zb**2 + r**2)

    zb_d = z_d + Dim[3]*np.cos(th_rad)*th_d_rad
    r_d = np.sign(x)*(y_d*y + x_d*x)/(np.sqrt(x**2 + y**2)) + \
        Dim[3]*np.sin(th_rad)*th_d_rad
    hp_d = (zb_d*zb + r_d*r)/(np.sqrt(zb**2 + r**2))

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

    return th1, th2, th3, th4, th1_d, th2_d, th3_d, th4_d


def sphere(ax, zi, r, couleur):
    u = np.linspace(0, np.pi, 30)
    v = np.linspace(0, 2 * np.pi, 30)

    x = r*np.outer(np.sin(u), np.sin(v))
    y = r*np.outer(np.sin(u), np.cos(v))
    z = r*np.outer(np.cos(u), np.ones_like(v))
    Zi = np.full_like(z, zi)
    z = z + Zi
    ax.plot_wireframe(x, y, z, color=couleur, alpha=0.3)


def plot_cube(cube_definition, ax):
    cube_definition_array = [
        np.array(list(item))
        for item in cube_definition
    ]
    points = []
    points += cube_definition_array
    vectors = [
        cube_definition_array[1] - cube_definition_array[0],
        cube_definition_array[2] - cube_definition_array[0],
        cube_definition_array[3] - cube_definition_array[0]
    ]
    points += [cube_definition_array[0] + vectors[0] + vectors[1]]
    points += [cube_definition_array[0] + vectors[0] + vectors[2]]
    points += [cube_definition_array[0] + vectors[1] + vectors[2]]
    points += [cube_definition_array[0] + vectors[0] + vectors[1] + vectors[2]]
    points = np.array(points)
    edges = [
        [points[0], points[3], points[5], points[1]],
        [points[1], points[5], points[7], points[4]],
        [points[4], points[2], points[6], points[7]],
        [points[2], points[6], points[3], points[0]],
        [points[0], points[2], points[4], points[1]],
        [points[3], points[6], points[7], points[5]]
    ]

    faces = Poly3DCollection(edges, linewidths=0, edgecolors='none')
    faces.set_facecolor((0.7, 0.7, 0.7, 0.9))
    ax.add_collection3d(faces)


class point():
    def __init__(self, position_r, vitesse_r, accélération_r, position_z, vitesse_z, accélération_z, angle):
        self.angle = angle
        self.position = [position_r, position_z]
        self.vitesse = [vitesse_r, vitesse_z]
        self.accélération = [accélération_r, accélération_z]


def simu(acc_time, dec_time, start, vitesse_r, vitesse_z, rAcc, zAcc, rDec, zDec, angle_init, angle_final):

    coefs_acc_r = calCoeffs(acc_time, [
                            start.position[0], start.vitesse[0], start.accélération[0]], [rAcc, vitesse_r, 0.])
    coefs_dec_r = calCoeffs(dec_time, [rAcc, vitesse_r, 0.], [rDec, 0., 0.])
    coefs_acc_z = calCoeffs(acc_time, [
                            start.position[1], start.vitesse[1], start.accélération[1]], [zAcc, vitesse_z, 0.])
    coefs_dec_z = calCoeffs(dec_time, [zAcc, vitesse_z, 0.], [zDec, 0., 0.])

    X = []
    Y = []
    Z = []
    V = []
    V1 = []
    V1_jac = []
    V2 = []
    V2_jac = []
    V3 = []
    V3_jac = []
    V4 = []
    V4_jac = []
    T = []

    v_max = 2*(angle_final - angle_init) / (dec_time + acc_time)

    fig = plt.figure(1, figsize=plt.figaspect(2.))
    ax = fig.add_subplot(1, 1, 1, projection='3d')

    for t in np.linspace(0., acc_time + dec_time, 100):
        if t <= acc_time:
            r_pos, r_v, r_a = calc_etats(t, coefs_acc_r)
            z_pos, z_v, z_a = calc_etats(t, coefs_acc_z)
            orientation = v_max*(t**2)/(2*acc_time) + angle_init
            orientation_v = v_max*t/acc_time

        elif t <= acc_time + dec_time:
            t_ = t - acc_time
            r_pos, r_v, r_a = calc_etats(t_, coefs_dec_r)
            z_pos, z_v, z_a = calc_etats(t_, coefs_dec_z)
            orientation = -v_max*(t_**2)/(2*dec_time) + \
                v_max*t_ + (v_max*acc_time)/2 + angle_init
            orientation_v = v_max - v_max*t_/dec_time

        th1, th2, th3, th4, th1_d, th2_d, th3_d, th4_d = mgi_grip_inv(
            r_pos*np.cos(start.angle), r_pos*np.sin(start.angle), z_pos, orientation, r_v*np.cos(start.angle), r_v*np.sin(start.angle), z_v, orientation_v)
        th1_jac, th2_jac, th3_jac, th4_jac, th1_d_jac, th2_d_jac, th3_d_jac, th4_d_jac = MGI(r_pos*np.cos(start.angle), r_pos*np.sin(
            start.angle), z_pos, r_v*np.cos(start.angle), r_v*np.sin(start.angle), z_v)

        Xr, Yr, Zr, _ = mgd([th1, th2, th3, th4])

        V1.append(th1_d)
        V2.append(th2_d)
        V3.append(th3_d)
        V4.append(th4_d)

        V1_jac.append(th1_d_jac)
        V2_jac.append(th2_d_jac)
        V3_jac.append(th3_d_jac)
        V4_jac.append(th4_d_jac)

        X.append(r_pos*np.sin(start.angle))
        Y.append(r_pos*np.cos(start.angle))
        Z.append(z_pos)
        V.append(np.sqrt(r_v**2 + z_v**2))
        T.append(t)

        ax.clear()
        cube_definition = [(-0.15, -0.04, 0), (-0.15, 0.26, 0),
                           (0.15, -0.04, 0), (-0.15, -0.04, -0.01)]
        plot_cube(cube_definition, ax)
        ax.plot3D(X, Y, Z, 'red', zorder=5)
        ax.plot3D(Xr, Yr, Zr, 'black', zorder=5)
        sphere(ax, Dim[0], Dim[1]+Dim[2], 'green')
        sphere(ax, Dim[0], Dim[1]+Dim[2] + Dim[3], 'blue')

        ax.set_xlim(xmin=-0.4, xmax=0.4)
        ax.set_ylim(ymin=-0.4, ymax=0.4)
        ax.set_zlim(zmin=-0.4, zmax=0.4)

        plt.pause(0.01)

    Xballe = [rAcc*np.sin(start.angle)]
    Yballe = [rAcc*np.cos(start.angle)]
    Zballe = [zAcc]
    tballe = 0

    while Zballe[-1] > 0:
        tballe += 0.001
        Xballe += [Xballe[0] + vitesse_r*np.sin(start.angle)*tballe]
        Yballe += [Yballe[0] + vitesse_r*np.cos(start.angle)*tballe]
        Zballe += [zAcc + vitesse_z*tballe - (9.81/2)*(tballe**2)]

    ax.plot3D(Xballe, Yballe, Zballe, 'brown', zorder=5)

    fig2 = plt.figure(2)

    ax = fig2.add_subplot(5, 1, 1)
    ax.plot(T, V, label="vitesse en bout de pince")
    ax.set_ylabel("m/s")
    ax.legend()
    ax = fig2.add_subplot(5, 1, 2)
    ax.plot(T, V1, label="th1")
    ax.plot(T, V1_jac, label="th1_jac")
    ax.set_ylabel("vitesse (rad/s)")
    ax.plot([0, t], [4.8, 4.8], 'r')
    ax.plot([0, t], [-4.8, -4.8], 'r')
    ax.legend()
    ax = fig2.add_subplot(5, 1, 3)
    ax.plot(T, V2, label="th2")
    ax.plot(T, V2_jac, label="th2_jac")
    ax.set_ylabel("vitesse (rad/s)")
    ax.plot([0, t], [4.8, 4.8], 'r')
    ax.plot([0, t], [-4.8, -4.8], 'r')
    ax.legend()
    ax = fig2.add_subplot(5, 1, 4)
    ax.plot(T, V3, label="th3")
    ax.plot(T, V3_jac, label="th3_jac")
    ax.set_ylabel("vitesse (rad/s)")
    ax.plot([0, t], [4.8, 4.8], 'r')
    ax.plot([0, t], [-4.8, -4.8], 'r')
    ax.legend()
    ax = fig2.add_subplot(5, 1, 5)
    ax.plot(T, V4, label="th4")
    ax.plot(T, V4_jac, label="th4_jac")
    ax.set_ylabel("vitesse (rad/s)")
    ax.set_xlabel("temps (s)")
    ax.plot([0, t], [4.8, 4.8], 'r')
    ax.plot([0, t], [-4.8, -4.8], 'r')
    ax.legend()

    plt.show()


"""start = point(0.05, 0., 0., 0.04, 0., 0., 0.)
simu(0.6, 0.15, start, 0.4, 0.8, 0.22, 0.3, 0.25, 0.35, 60, -20)"""

start = point(-0.3, 0., 0., 0.25, 0., 0., 0.)
simu(0.5, 0.2, start, 1.3, 0.2, 0.05, 0.4, 0.18, 0.35, -150, -70)
"""
start = point(0.05, 0., 0., 0.03, 0., 0., 0.)
simu(2., 2., start, 0.05, 0.05, 0.1, 0.07, 0.12, 0.09, 90, 0)
"""
