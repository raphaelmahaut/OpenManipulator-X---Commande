######################################################
### DETAIL DES FONCTIONS DU FICHIER call_camera.py ###
######################################################

- openCapture(camera_number_on_pc) : prend en paramètre le numéro identifiant la caméra connectée à l'ordinateur (typiquement 0, 1 ou 2)
et retourne une capture ouverte pour cette caméra, i.e. un objet permettant de prendre des photos. Pour plus de détails, se référer à la section II.1.b du rapport.

- closeCapture(capture) : ferme la capture passée en paramètre

- get_frame_from_capture(capture) : prend une photo avec la capture passée en paramètre et retourne cette photo (il s'agit d'une matrice de pixels)

- manually_get_frame_from_capture(capture) : ouvre sur l'ordinateur une fenêtre affichant ce que la caméra perçoit. Lorsque la touche ECHAP est pressée, 
cette fenêtre se ferme et la fonction renvoie une photo prise avec la capture passée en paramètre.

- cameraMatricRecover(checkerboard_images, checkerboard_internal_corners) : prend en paramètres une liste de photos d'un échiquier prises avec une caméra à distorsion
ainsi que le nombre de coins interne de cet échquier, et retourne la matrice de caméra et les coefficients de distorsion (variables mtx et dist) de la caméra. 
Pour plus de détails, se référer à la section II.1.c du rapport. La variable newcameramtx sert pour des réglages très fins avec OpenCV et peut sans problème être
considérée identique à mtx pour nos applications.

- undistortImage(img, mtx, newcameramtx, dist) : prend en paramètre une image distordue, la matrice de caméra et les coefficients de distorsion de la caméra ayant capturé
l'image, et renvoie l'image sans distorsion. Deux méthodes aux résultats parfaitement semblables sont présentées dans cette fonction.

- manually_get_undistorted_frame_from_capture(capture, mtx, newcameramtx, dist, window_name="source", flip=True) : fonction de test permettant de visualiser la correction
de distorsion appliquée à une caméra au travers d'une fenêtre qui se ferme lorsque la touche ECHAP est pressée.

- liveCameraCalibration(camera_number_on_pc, checkerboard_internal_corners) : permet de réaliser "en live" une mesure de la matrice de caméra et des coefficients de
distorsion d'une caméra connectée à l'ordinateur, et ce à l'aide d'un échiquier imprimé. 
Cette fonction prend en paramètre le numéro identifiant la caméra connectée à l'ordinateur et le nombre de coins internes de l'échiquier utilisé pour la mesure, puis
ouvre sur l'ordinateur une fenêtre affichant ce que la caméra perçoit. L'utilisateur doit alors appuyer 15 fois sur la touche ECHAP pour prendre 15 photos de
l'échiquier. La fonction affiche alors la détection par OpenCV des coins internes de l'échiquier sur chaque photo puis renvoie la matrice de caméra et les coefficients
de distorsion.

RECOMMANDATIONS POUR LA FONCTION liveCameraCalibration :
- l'échiquier utilisé doit impérativement être le même pour les 15 photos
- l'échiquier doit être normal à l'axe optique de la caméra sur les 15 photos
- l'échiquier doit impérativement être entièrement visible sur chaque photo
- il est préférable que l'échiquier se trouve toujours à altitude presque constante sur l'axe optique de la caméra
- il est fortement recommandé de varier les positions et inclinaisons de l'échiquier au cours des 15 photos, et en particulier il est conseillé de prendre quelques 
photos où celui-ci est positionné dans les coins ou aux bords.



###############################################################
### DETAIL DES FONCTIONS DU FICHIER image_processing_cv2.py ###
###############################################################

*** Fonctions de la section "CAMERA FUNCTIONS" : voir ci-dessus.


*** Fonctions de la section "IMAGE PROCESSING FUNCTIONS" :

- chopChop(img, additional_crop) : prend en argument une image au format paysage (typique pour une caméra) et en supprime les bords gauche et droit pour retourner
une image carrée. Le second argument permet de réduire encore la taille de l'image retournée.

- colorQuantization(img, k) : prend en paramètre une image et un entier, et renvoie la même image avec un nombre de couleurs réduit de 256^3 à (256/k)^3.
NOTE : cette opération porte le nom de "quantification" des couleurs ("color quantization" en anglais) et peut être effectuée de façon optimale par l'alogrithme
d'intelligence artificielle des K-Means. Cependant, l'exécution de ce dernier est très lente, tandis que la méthode proposée ici, bien qu'approximative, est 
presque instantanée. Pour plus de détails, se référer à la section II.2.a du rapport.

- cannyEdgeDetector(img, threshold1, threshold2, color_divider = 8, thickness = 5) : retourne une image en noir et blanc représentant les contours de l'image 
passée en premier paramètre. Ces contours sont détectés par la méthode de Canny sur une version aux couleurs quantifiées de l'image passée en premier paramètre,
puis sont épaissis. Pour plus de détails, se référer à la section II.2.a du rapport.

- cleanContours(edges_img, edges_img_canvas, contours, min_area, max_area) : 
	- edges_img est une image de contours (comme celle renvoyée par cannyEdgeDetector par exemple). La fonction va y effacer certains contours.
	- edges_img_canvas est une copie de edges_img. Les mêmes opérations d'effacement y seront appliquées, mais en appliquant du gris au lieu du noir, de sorte
à pouvoir visualiser les contours effacés par la fonction.
	- contours est une liste des frontières (ou contours fermés) isolées de edges_img. Pour plus de détails, se référer à la section II.2.b du rapport.
Cette fonction passe en revue chacune des frontières isolées détectées sur edges_img et élimine celles qui ne peuvent pas correspondre à une planche de bois à trier
par le bras robotique (que ce soit par leur aire trop grande ou trop petite, ou par leur forme qui ne convient pas). Ces éliminations effacent directement sur 
edges_img les frontières concernées. La fonction renvoie enfin un booléen indiquant si elle a dû réaliser une élimination.


- findCentersGravity(img, edges_img, edges_img_canvas, min_area, max_area) : pour une explication des arguments de cette fonction, voir cleanContours ci-dessus.
Cette fonction détecte les frontières fermées présentes sur edges_img, puis fait appel à cleanContours tant que des éliminations de frontières ne correspondant pas
à des planches de bois à trier sont nécessaires. Cette fonction calcule ensuite les coordonnées du centre de gravité de chaque planche de bois (i.e. chaque frontière
non éliminée) détectée, et renvoie une liste contenant une liste de 5 points par planche - le centre de gravité et les 4 coins. Pour plus de détails, se référer à 
la section II.2.b du rapport.


*** Fonctions de la section "PLANK PROCESSING FUNCTIONS"

- isPlankMisaligned(robot_line, robot_column, plank) : prend en argument la position du centre de gravité du robot sur une image et une liste des 5 points d'une planche
(voir findCentersGravity ci-dessus) en renvoie un booléen indiquant si l'inclinaison de ladite planche l'empêche d'être saisie par le bras robotique. Pour plus de détails,
se référer à la section II.3 du rapport.

- euclidianNorm2D(u) : calcule et renvoie la norme euclidienne du vecteur u à 2 dimensions

- is_among_planks(plank_0, planks_list, epsilon) : compare la planche (liste de 5 points) plank_0 à toutes les planches de planks_list, et renvoie un booléen indiquant si
plank_0 est déjà parmi les planches de planks_list à epsilon pixels près.

- convert_point_coordinates(column, line, column_center, line_center, distance_ratio) : convertit les coordonnées d'un point dans une image prise par la caméra en coordonnées
dans le repère du bras robotique. Pour plus de détails, se référer à la section II.1.d du rapport.

- is_in_square(x, y, square) : vérifie si le point de coordonnées (x,y) est dans un carré dont les coordonnées des coins sont données dans la liste square (retourne un booléen). 
En pratique, cela permet de vérifier si une frontière non-éliminée par cleanContours (voir plus haut) est bien située sur le plan de travail du robot.

- convertPlankCoordinates(planks_list, nb_lines_frame, nb_columns_frame, robot_line, robot_column, distance_ratio) : prend un argument une liste de planches (listes de 5 points)
comme celle renvoyée par findCentersGravity par exemple (voir plus haut) et convertit chaque planche en une liste de 4 éléments :
	- le couple (x, y) des coordonnées du centre de gravité de la planche dans le repère du bras robotique
	- un booléen indiquant valant True si la planche est mal inclinée
	- deux points (couples de coordonnées) définissant la trajectoire du robot pour la correction d'inclinaison de la planche si nécessaire
Une liste de ces planches est converties est alors renvoyée. Elle est prête à être exploitée par le robot.
Pour plus de détails, se référer aux sections II.1.d, II.3 et IV.2.d du rapport.


*** Fonctions de la section "CAMERA VISION CALIBRATION" :

- red_filter(img) : applique un filtre rouge à une image passée en paramètre

- blue_filter(img) : applique un filtre bleu à une image passée en paramètre

- find_red_stickers(img) : détecte et renvoie la position du centre de gravité de 3 stickers rouges sur une image passée en paramètre.

- stickers2robot(img) : à partir d'une image du plan de travail du robot avec 3 stickers rouges au niveau de 3 coins passée en paramètre, renvoie la position de l'origine du
repère du bras robotique sur cette image, le ratio de distances utilisé dans les fonctions de la section précédente (voir section II.4 du rapport), et une liste des coordonnées
des coins du plan de travail.


*** Fonction de la section "CAMERA VISION" :

- cameraVision(camera_number_on_pc, robot_line, robot_column, distance_ratio, square) : cette fonction met en relation tous les éléments mentionnés précédemment : à partir de
l'identifiant de la caméra sur l'ordinateur et des éléments renvoyés par la calibration de la section précédente, elle détecte les planches de bois sur une photo du plan de 
travail du robot et les retourne sous le même format que décrit dans convertPlankCoordinates plus haut.














