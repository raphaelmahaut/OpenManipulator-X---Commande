####################################
### DETAILS DES FICHIERS FOURNIS ###
####################################

- S1_1_A Rapport de développement d'un bras manipulateur intelligent.pdf : rapport écrit du déroulé de ce projet. Il contient toutes les informations nécessaires pour comprendre les autres fichiers fournis.

- Annexe 1 au rapport S1_1_A appel camera.pdf : première annexe au rapport présenté ci-dessus, détaille le code python permettant de corriger la distorsion d'une caméra et de faire appel à cette dernière. 

- Annexe 2 au rapport S1_1_A traitement images.pdf : seconde annexe au rapport, détaille le code python utilisé pour traiter les images récupérées par la caméra, y identifier les planches de bois, déterminer leur inclinaison et convertir leurs coordonnées vers le repère du robot. Ce fichier contient également les fonctions de calibration automatique de la caméra.

- S1_1_A Diagramme de Gantt.pdf : diagramme de Gantt présentant le déroulé des actions effectuées par les membres du groupe au fil des séances.

- teleop_keyboard.py : programme python de contrôle du bras robotique à l'aide du clavier. Ce fichier sert de répertoire des cas d'utilisation des services fournis par le constructeur du robot.

- Dossier Code camera :
    - call_camera.py : version python exécutable de l'annexe 1. La dernière section permet d'effectuer la correction de distorsion d'une caméra "en live" à l'aide d'un échiquer imprimé. 
    - image_processing_cv2_vf.py : version python exécutable de l'annexe 2. 

- Dossier Modèle géométrique inverse : ensemble des programmmes permettant de convertir une position cartésienne de la pince dans le repère lié au robot en un quadruplet correspondant aux angles que doivent prendre les moteurs du robot pour atteindre cette position.

- Dossier Package_ROS2_S1_1_A : package ROS2 développé par le groupe et permettant de faire réaliser un dévracage au bras robotique. Un exemple d'utilisation est fourni dans le fichier README de ce dossier. Les éléments curiciaux sont les suivants :
    - Dossier groupe1_pkg : ensemble des fichiers à compiler pour faire fonctionner le robot.
        - controleur.py : Programme qui permet de définir les actions à réaliser par le robot, en connaissant la position des planches à trier.
        - vision_cal.py : Programme de calibration automatique de la caméra pour connaitre la position des objets dans le repère du robot à partir de leur position sur l'image issue de la caméra.