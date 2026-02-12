# Bocbot Hunter (ROS 2)

Robot autonome dans Gazebo qui explore une arene, detecte une balle blanche et la pousse vers le but adverse. Le projet propose un mode "pipeline" classique (vision + evitement + controle) et un mode rapide base sur LIDAR 360 degres avec une machine d'etats.

## Demarrage rapide (match)

```bash
./launch.sh
```

Le script lance Gazebo, deux robots (bleu et orange), le gestionnaire de score, puis la fenetre de jeu.

### Controles (fenetre de jeu)

- Bleu : W / A / S / D
- Orange : fleches directionnelles
- Espace : stop les deux robots

## Autres lancements

```bash
# Mode rapide LIDAR (1 robot)
ros2 launch bocbot_hunter bocbot_hunter_gazebo.launch.py

# Pipeline complet (vision + evitement + controle)
ros2 launch bocbot_hunter bocbot_hunter.launch.py
```

## Architecture (resume)

### Packages
- `bocbot_hunter` : nodes de controle, vision, evitement, gestion de match.
- `bocbot_interfaces` : messages custom (`BallDetection`, `ColorRing`).

### Nodes principaux
- `ball_detector` : detection de balle blanche par OpenCV.
- `obstacle_avoidance` : analyse LIDAR pour zones libres.
- `hunter_controller` : fusion des infos et commandes `cmd_vel`.
- `fast_hunter` : mode rapide LIDAR 360 degres + machine d'etats.
- `game_manager` : score + reset automatique.
- `game_viewer` : fenetre de jeu et teleoperation.

## Structure du depot

- `bocbot_ws/` : workspace ROS 2 (sources, build, install)
- `launch.sh` : match complet (bleu vs orange)
- `assets/` : images et ressources
- `logo_chicken_drive.png` : logo du projet

## Prerequis

- ROS 2 Humble
- Gazebo (gazebo_ros)
- Xacro
- cv_bridge / OpenCV

Installation minimale :

```bash
sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-cv-bridge
```

## Build (si besoin)

```bash
cd bocbot_ws
colcon build
source install/setup.bash
```

## Notes

- Le mode match lance automatiquement le reset et garde les robots au sol.
- Les vitesses et seuils se reglent directement dans les nodes (ex: `fast_hunter.py`).
