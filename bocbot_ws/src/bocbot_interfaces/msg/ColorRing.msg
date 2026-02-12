# Anneau de couleurs 360 degres - fusion cameras 360
# Chaque entree = couleur dominante a ce degre (0-359)
# 0=UNKNOWN, 1=RED (balle), 2=BLUE (cage), 3=ORANGE (cage), 4=WHITE (mur), 5=GREEN (pelouse)

uint8[] color_ring       # 360 entrees, index = degre (0=avant robot, 90=droite, etc.)

# Angles detectes (radians, 0=avant, positif=droite)
float32 ball_angle       # angle vers la balle (RED), NaN si non detectee
float32 blue_goal_angle  # angle vers la cage bleue, NaN si non detectee
float32 orange_goal_angle # angle vers la cage orange, NaN si non detectee

# Confiance globale (0.0 a 1.0)
float32 confidence

# Nombre de degres couverts par chaque couleur
uint16 red_count
uint16 blue_count
uint16 orange_count
uint16 white_count
uint16 green_count
