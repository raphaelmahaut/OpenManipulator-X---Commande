Ce package ROS2 contient 4 fichiers développés au cours de ce projet:


	- "lancement.py" qui permet de commander le mouvement du robot pour lancer une balle dans un gobelet
	- "enregistrement.py" qui permet de visualisé la position et/ou la vitesse du bras au cours d'un mouvement et se déclenche en
	envoyant un message dans le topic 'etats'.
	- "manuel.py" qui permet à un utilisateur de spécifier les coordonnées à atteindre pour le lancer d'une balle.
	- "camera.py" qui permet d'envoyer automatiquement les coordonnées à atteindre pour le lancer d'une balle à l'aide de la camera Zed2