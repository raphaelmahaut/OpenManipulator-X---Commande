import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection

Dim = [0.0765, 0.13, 0.124, 0.1466, 0.]
alpha = np.arcsin(0.024/Dim[1])


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


def opti_temps(angle, vitesse):
    '''recherche du temps à donner pour effectuer la trajectoire le plus approprié possible. Si le temps est trop long,
     la trajectoire peut partir en arrière avant de commencer, si au contraire le temps est trop court le bras doit atteindre un vitesse
     plus importante que celle demandée puis ralentir avant de lâcher.
     L'algorithme garde le temps le plus long pour lequel, la vitesse est comprise entre 0. et la viitesse à atteindre sur l'ensemble de la trajectoire 

    paramètres:
        obligatoires:
            angle: angle relatif entre es sections du bras au début de la trajectoire
            vitesse: vitesse que la balle doit atteindre au moment du lâcher

    sortie:
        move_time: temps à donner en paramètre à la trajectoire  
    '''

    move_time = 1.0

    v_ang = vitesse / (Dim[1] + 2*Dim[2] + 3*Dim[3])

    for temps in np.linspace(0.1, 2.0, 41):
        coefs = calCoeffs(temps/2, [-angle, 0., 0.], [0., v_ang, 0.])
        A = []
        for t in np.linspace(0., temps/2, 100):
            _, v, _ = calc_etats(t, coefs)
            A.append(v)
        if (-0.0001 < min(A) < 0.0001) and (-0.0001 < max(A) - v_ang < 0.0001):
            move_time = temps

    print("le temps de mouvement choisi est: " + str(move_time))
    return move_time


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
    v0 = np.sqrt(((g/2)*(D/np.sin(alpha0))**2) /
                 (np.cos(alpha0)/np.sin(alpha0)*D + zlancer - z))
    print("la vitesse de lacher sans frottements: " + str(v0))
    return v0


def simu(x, y, z, dec=0, angle=25.):

    angle = (np.pi * angle) / 180

    orientation = 2*np.arctan(y/(x + np.sqrt(x**2 + y**2)))

    dec = (np.pi * dec) / 180
    vitesse = sansFrottements(x, y, z, xlancer=-(0.477 - Dim[0])*np.sin(orientation)*np.sin(dec), ylancer=-(
        0.477 - Dim[0])*np.cos(orientation)*np.sin(dec), zlancer=(0.477 - Dim[0])*np.cos(dec) + Dim[0])
    print("vitesse à atteindre: " + str(vitesse) + " dec: " + str(dec))
    move_time = opti_temps(angle, vitesse)

    v_ang = vitesse / (Dim[0] + 2*Dim[1] + 3*Dim[2])
    coefs = calCoeffs(move_time/2, [-angle, 0., 0.], [0., v_ang, 0.])

    X = []
    Y = []
    Z = []
    V = []
    V1 = []
    V2 = []
    V3 = []
    V4 = []
    T = []

    fig = plt.figure(1, figsize=plt.figaspect(2.))
    ax = fig.add_subplot(1, 1, 1, projection='3d')

    for t in np.linspace(0., move_time, 100):
        if t <= move_time/2:
            th, th_d, _ = calc_etats(t, coefs)

        elif t <= move_time:
            t_ = move_time - t
            th, th_d, _ = calc_etats(t_, coefs)
            th = -th
            th_d = th_d

        th1, th2, th3, th4 = (orientation, th - alpha - dec,
                              th - np.pi/2 + alpha, th)

        Xr, Yr, Zr, _ = mgd([th1, th2, th3, th4])

        V1.append(0.)
        V2.append(th_d)
        V3.append(th_d)
        V4.append(th_d)

        X.append(Xr[-1])
        Y.append(Yr[-1])
        Z.append(Zr[-1])
        r_d = th_d * (Dim[1]*np.sin(th) + 2*Dim[2] *
                      np.sin(2*th) + 3*Dim[3]*np.sin(3*th))
        z_d = -th_d * (Dim[1]*np.cos(th) + 2*Dim[2] *
                       np.cos(2*th) + 3*Dim[3]*np.cos(3*th))
        V.append(np.sqrt(r_d**2 + z_d**2))
        T.append(t)

        ax.clear()
        cube_definition = [(-0.15, -0.04, 0), (-0.15, 0.26, 0),
                           (0.15, -0.04, 0), (-0.15, -0.04, -0.01)]
        plot_cube(cube_definition, ax)
        ax.plot3D(X, Y, Z, 'red', zorder=5)
        ax.plot3D(Xr, Yr, Zr, 'black', zorder=5)
        sphere(ax, Dim[0], Dim[1]+Dim[2], 'green')
        sphere(ax, Dim[0], Dim[1]+Dim[2] + Dim[3], 'blue')

        ax.set_xlim(xmin=-0.6, xmax=0.6)
        ax.set_ylim(ymin=-0.6, ymax=0.6)
        ax.set_zlim(zmin=-0.6, zmax=0.6)

        plt.pause(0.01)

    Xballe = [-(0.477 - Dim[0])*np.sin(orientation)*np.sin(dec)]
    Yballe = [-(0.477 - Dim[0])*np.cos(orientation)*np.sin(dec)]
    Zballe = [(0.477 - Dim[0])*np.cos(dec) + Dim[0]]
    tballe = 0

    while Zballe[-1] > 0:
        tballe += 0.001
        Xballe += [Xballe[0] + vitesse*np.cos(dec)*np.sin(orientation)*tballe]
        Yballe += [Yballe[0] + vitesse*np.cos(dec)*np.cos(orientation)*tballe]
        Zballe += [(0.477 - Dim[0])*np.cos(dec) + Dim[0] +
                   vitesse*np.sin(dec)*tballe - (9.81/2)*(tballe**2)]

    ax.plot3D(Xballe, Yballe, Zballe, 'brown', zorder=5)

    fig2 = plt.figure(2)

    ax = fig2.add_subplot(5, 1, 1)
    ax.plot(T, V, label="vitesse en bout de pince")
    ax.set_ylabel("m/s")
    ax.legend()
    ax = fig2.add_subplot(5, 1, 2)
    ax.plot(T, V1, label="th1")
    ax.set_ylabel("vitesse (rad/s)")
    ax.plot([0, t], [4.8, 4.8], 'r', label="limite")
    ax.plot([0, t], [-4.8, -4.8], 'r')
    ax.legend()
    ax = fig2.add_subplot(5, 1, 3)
    ax.plot(T, V2, label="th2")
    ax.set_ylabel("vitesse (rad/s)")
    ax.plot([0, t], [4.8, 4.8], 'r', label="limite")
    ax.plot([0, t], [-4.8, -4.8], 'r')
    ax.legend()
    ax = fig2.add_subplot(5, 1, 4)
    ax.plot(T, V3, label="th3")
    ax.set_ylabel("vitesse (rad/s)")
    ax.plot([0, t], [4.8, 4.8], 'r', label="limite")
    ax.plot([0, t], [-4.8, -4.8], 'r')
    ax.legend()
    ax = fig2.add_subplot(5, 1, 5)
    ax.plot(T, V4, label="th4")
    ax.set_ylabel("vitesse (rad/s)")
    ax.set_xlabel("temps (s)")
    ax.plot([0, t], [4.8, 4.8], 'r', label="limite")
    ax.plot([0, t], [-4.8, -4.8], 'r')
    ax.legend()

    plt.show()


simu(1.0, 0., 0.08, 0)
