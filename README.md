Ce répertoire contient tous les documents réalisé lors d'un projet avec le bras robotique OpenManipulator-X


Le dossier rapport-1A correspond au rapport du projet consistant à effectuer automatiquement une tâche de dévracage au bras robotique.
Il contient une vidéo de présentation, un rapport écris et les codes ayant été développé au cours de ce projet.


Les trois autres dossiers corrspondent à la suite du projet consistant à lacer automatiquement une balle de ping-pong dans un verre à l'aide du bras.
Le dossier "lancer_final" contient un package ROS2 écrit à l'aide de python permettant la communication avec la caméra et la commande du bras robotique.
Le dossier "robotis_workspace" contient la définition en C++ du noeud controller utilisé pour commander le mouvement du bras.
Le dossier "siulations" contient les autres fichiers utilisés pour simuler au cours de ce projet.


Pour utiliser les outils développés au cours de ce projet:

	- Copier les dossiers "lancer_final" et "robotis_workspace" dans le répertoire racine de l'ordinateur.
	- Ouvrir un terminal dans chacun des dossiers et compiler les packages ROS2 avec l'instruction "colcon build"
	- Ouvrir trois terminal dans lequel il faut définir l'environement ROS2 à utiliser
		source ~/lancer_final/install/setup.bash
		source ~/robotis_workspace/install/setup.bash
	- Dans le premier terminal lancer
		ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py
	- Dans le second terminal lancer
		ros2 run groupe1_pkg lancement
	- Dans le troisième terminal lancer
		ros2 run groupe1_pkg manuel [coordonnées à atteindre (exemple: 0.5 0. 0.08)]  (pour utiliser sans la camera)
		ros2 run groupe1_pkg camera (pour utiliser la camera détection de gobelet à l'aide de la caméra)

Le fichier lancement est fait pour effectuer trois lancés de balles successifs sans être relancé.