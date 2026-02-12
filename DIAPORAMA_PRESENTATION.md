# Diaporama - Projet Bocbot (ROS2 + Gazebo)

---

## Diapo Titre
### Ce que j'ecris
- Simulation de match robotique 1v1 avec ROS2 et Gazebo
- Equipe BLEU vs ORANGE
- Controle manuel + score automatique

### Ce qu'on dit
- Le projet simule un match en temps reel entre deux robots.
- On combine une simulation physique Gazebo et des noeuds ROS2.
- L'objectif est d'avoir un jeu jouable, stable et facile a expliquer.

---

## Diapo 1 - Objectif du projet
### Ce que j'ecris
- Match 1v1
- Deplacement, buts, score, reset
- Interface joueur en direct

### Ce qu'on dit
- Le systeme couvre toute la boucle: commande clavier, mouvement, detection de but, score et relance.
- On ne fait pas une simple demo robot: on fait une mini architecture complete orientee jeu.

---

## Diapo 2 - Architecture globale
### Ce que j'ecris
- Simulation: Gazebo + `arena.world`
- Robots: URDF/Xacro + `diff_drive`
- Logique: `game_manager`
- IHM: `game_viewer`
- Orchestration: `launch.sh`

### Ce qu'on dit
- On a separe clairement les responsabilites pour simplifier debug et evolutions.
- Chaque bloc peut etre teste independamment.

---

## Diapo 3 - Sequence de lancement
### Ce que j'ecris
1. Source ROS2 + workspace
2. Lancement Gazebo et spawn robots
3. Demarrage gestion de match
4. Demarrage viewer

### Ce qu'on dit
- L'entree standard est `./launch.sh`.
- Le script initialise l'environnement, lance les composants, puis ouvre le viewer de match.

---

## Diapo 4 - Monde Gazebo
### Ce que j'ecris
- Terrain octogonal
- Cages en renfoncement
- Top-down camera
- Dome transparent

### Ce qu'on dit
- Le decor n'est pas juste visuel: il impacte les collisions et la lisibilite du jeu.
- La camera top-down sert a suivre le match globalement.

---

## Diapo 5 - Robot et dynamique
### Ce que j'ecris
- `bocbot.urdf.xacro` + `bocbot.gazebo`
- Deux variantes couleur
- Roues + plugin `diff_drive`
- POV 3e personne

### Ce qu'on dit
- La meme base robot est parametree pour bleu et orange.
- Les reglages de drive/physique ont ete ajustes pour la reactivite et la stabilite.

---

## Diapo 6 - Noeuds ROS2
### Ce que j'ecris
- `robot_state_publisher` x2
- `spawn_entity.py` x2
- `game_manager`
- `game_viewer`
- `robot_uprighter`

### Ce qu'on dit
- `game_manager` gere score, manches et reset.
- `game_viewer` gere affichage et commandes.
- `robot_uprighter` evite qu'un robot reste bloque retourne.

---

## Diapo 7 - RQT Graph (a inserer ici)
### Ce que j'ecris
- Capturer le graphe ROS2 du match
- Fichier image: `assets/rqt_graph_projet.png`
- Fichier source: `assets/rqt_graph_projet.dot`

### Ce qu'on dit
- Cette diapo doit afficher l'image du graphe en grand.
- Place recommandee dans la slide:
  - image `assets/rqt_graph_projet.png` au centre, 80-90% de la largeur
  - titre en haut: `RQT Graph - Flux ROS2 du match`
  - legende courte en bas
- Comment l'expliquer:
  - les rectangles sont des noeuds
  - les ovales sont des topics
  - on lit le flux principal:
    - `game_viewer` publie `cmd_vel`
    - plugins drive de Gazebo appliquent le mouvement
    - `game_manager` lit `/gazebo/model_states`
    - `game_manager` publie `/game/score` et `/game/status`
    - `game_viewer` lit camera + score + status

---

## Diapo 8 - Topics et services cles
### Ce que j'ecris
- Commande: `/bocbot_blue/cmd_vel`, `/bocbot_orange/cmd_vel`
- Vision: `/.../camera/image_raw`, `/game/top_down/image_raw`
- Jeu: `/game/score`, `/game/status`, `/game/restart`
- Simu: `/gazebo/model_states`, `/gazebo/set_entity_state`

### Ce qu'on dit
- Les topics transportent l'etat en continu.
- Le service `set_entity_state` sert aux resets propres des robots et de la balle.

---

## Diapo 9 - Controle et logique match
### Ce que j'ecris
- BLEU: WASD
- ORANGE: Fleches
- Combos possibles (avancer + tourner)
- BO3: premier a 2 gagne
- Relance a `Espace`

### Ce qu'on dit
- Les deux joueurs sont independants.
- Les appuis sont traites en key down/up pour eviter les comportements parasites.
- Le statut de victoire est affiche et la relance remet le match proprement.

---

## Diapo 10 - Robustesse et limites
### Ce que j'ecris
- Auto-redressement robot
- Reset automatique apres but
- Structure de projet nettoyee
- Base evolutive (IA, stats, replay)

### Ce qu'on dit
- Le systeme est jouable et stable pour une demo.
- Les prochaines evolutions naturelles sont IA adversaire, enregistrement et telemetry avancee.

---

## Diapo Fin
### Ce que j'ecris
- Merci
- Questions
- Demo live: `./launch.sh`

### Ce qu'on dit
- On peut lancer une demo immediate et montrer le graphe ROS en direct.
- Si besoin, on detaille un noeud ou un topic precis.

---

## Annexe - Regenerer le graphe
### Ce que j'ecris
- Commande standard:
```bash
cd /home/seatech/Bureau/ROS2_SEATECH/projet
./launch.sh
source /opt/ros/humble/setup.bash
source bocbot_ws/install/setup.bash
rqt_graph
```

### Ce qu'on dit
- Pour une capture screenshot "officielle", ouvrir `rqt_graph` pendant le match puis exporter/faire capture.
- Le graphe deja prepare pour les slides est dans:
  - `assets/rqt_graph_projet.png`
  - `assets/rqt_graph_projet.dot`
