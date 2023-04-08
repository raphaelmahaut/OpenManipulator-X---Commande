########################
### MISE EN PRATIQUE ###
########################

Cette section détaille les étapes à suivre pour utiliser les différents fichiers du projet. 
Il est nécessaire d'effectuer les actions détaillées plus bas sur un ordinateur muni d'une distribution ROS2 et auquel le bras robotique alimenté est connecté.

Tout d'abord, copier le dossier Package_ROS2_S1_1_A dans un répertoire de travail. Ce dossier correspond au package ROS2 développé par le groupe.
Ensuite, lancer les commandes suivantes dans un terminal ouvert sur ce répertoire où l’on source ROS2 :
$   colcon build
$   ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py
La première des commandes ci-dessus permet de compiler le package, et la seconde initialise les services du constructeur du robot. Pour plus de détails à ce sujet, se référer au rapport.

Ensuite, presser ctrl + shift + t pour ouvrir un nouveau terminal, puis positionner les marqueurs rouges en 3 coins du support du robot avant de lancer la commande suivante :
$   ros2 run groupe1_pkg vision_cal 
Cette commande permet la calibration automatique de la caméra à l'aide des marqueurs rouges. 
Suite à l'exécution de la commande, le résultat de la calibration apparaît à l'écran : des cercles colorés permettent de vérifier que les marqueurs ont été correctement repérés. Une fois cette vérification effectuée, presser la touche Echap.

Positionner ensuite des planches de bois à dévraquer sur l'espace de travail du robot, en évitant qu'elles se touchent ou se superposent, puis lancer la commande suivante :
$   ros2 run groupe1_pkg controleur 

Le robot va alors effectuer la tâche de dévracage, et s'arrêtera dans une position de sécurité une fois la tâche effectuée.