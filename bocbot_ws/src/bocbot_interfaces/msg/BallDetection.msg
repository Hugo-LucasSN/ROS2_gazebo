# Message pour les informations de détection de balle

# Angle horizontal de la balle par rapport au centre de l'image (en radians)
# Positif = à droite, Négatif = à gauche
float32 angle

# Estimation de la distance basée sur la taille de la balle dans l'image (en mètres)
# Plus la valeur est petite, plus la balle est proche
float32 distance

# Rayon de la balle détectée en pixels (pour estimer la distance)
float32 radius_pixels

# Confiance de la détection (0.0 à 1.0)
# 1.0 = très confiant, 0.0 = pas confiant
float32 confidence

# Position X de la balle dans l'image (pixels)
int32 center_x

# Position Y de la balle dans l'image (pixels)
int32 center_y