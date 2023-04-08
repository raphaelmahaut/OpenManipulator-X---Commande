Les deux programmes ci-contre permettent d'établir le modèle géométrique inverse du robot OpenManipulator :

- Le programme "Calcul matrices de transformations et jacobiens" est repris d'un programme fourni par 
notre encadrant Miguel Da Silva. Nous l'avons adapté dans le cadre précis de notre robot. Il fournit 
l'expression de la matrice Jacobienne (6x4) de notre robot, 4 de ces lignes sont réutilisées dans 
l'autre programme (Cf. rapport pour explications supplémentaires).

- Le programme "Modèle géométrique inverse" contient les fonctions suivantes :
	- PASSAGE : établit la matrice de passage d'un repère à un autre
	- MGD : établit le modèle géométrique direct : (q1,q2,q3,q4) --> (x,y,z)
	- Jacob : reprend la Jacobienne réduite du programme précédent pour des (q1,q2,q3,q4) donnés
	- MGI : établit le modèle géométrique inverse, avec les paramètres suivants :
		- final_pose : tableau numpy contenant les coordonnées x,y & z (en m)
		- q0 : tableau numpy contenant les conditions initiales de la descente du gradient (en rad)
		- alpha_k : nombre flottant donnant l'amplitude du terme de convergence dans la formule
		  de récurrence de la descente du gradient
		- alpha_k2 : nombre flottant donnant l'amplitude du terme de respect des intervalles
		  dans la formule de récurrence de la descente du gradient
		- tol : nombre flottant donnant l'erreur de consistance autorisée (en m)
		- Dthe4 : nombre flottant définissant l'intervalle de tolérance de verticalité de la pince (rad)
	- TRACE : trace la position du robot pour les angles (q1,q2,q3,q4) donnés
  On peut tout en bas (ligne 127) directement modifier la position cartésienne souhaitée, il ne reste plus
  qu'à lancer l'entièreté du programme. La position du robot est dessinée et les angles solutions sont
  renvoyés dans le shell, avec la position réellement atteinte