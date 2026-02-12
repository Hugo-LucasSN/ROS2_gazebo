#!/bin/bash
# Script de lancement robuste du POV Viewer avec auto-restart et gestion Ctrl+C

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# Indicateur d'arrêt
SHUTDOWN=false

# Fonction de nettoyage
cleanup() {
    if [ "$SHUTDOWN" = true ]; then
        return
    fi
    SHUTDOWN=true
    echo -e "${YELLOW}→ Arrêt du POV Viewer...${NC}"
    # Tuer tous les processus python du viewer
    pkill -9 -f "dual_pov_viewer" 2>/dev/null || true
    exit 0
}

# Capturer les signaux
trap cleanup SIGINT SIGTERM SIGHUP SIGQUIT EXIT
trap '' SIGPIPE

echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}   📷 POV VIEWER - Auto-restart activé${NC}"
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

# Tuer toute instance précédente du viewer (évite les doublons entre sessions)
pkill -9 -f "dual_pov_viewer" 2>/dev/null || true
sleep 0.5

while [ "$SHUTDOWN" = false ]; do
    echo -e "${YELLOW}→ Démarrage du POV Viewer...${NC}"

    # Source l'environnement ROS2
    source /opt/ros/humble/setup.bash 2>/dev/null
    source /home/seatech/Bureau/ROS2_SEATECH/projet/bocbot_ws/install/setup.bash 2>/dev/null

    # Lancer le viewer (sans background pour pouvoir recevoir les signaux)
    ros2 run bocbot_hunter dual_pov_viewer

    # Si on arrive ici et SHUTDOWN n'est pas vrai, c'est un crash
    EXIT_CODE=$?
    if [ "$SHUTDOWN" = true ]; then
        break
    fi

    echo -e "${RED}✗ POV Viewer a crashé (exit code: $EXIT_CODE)${NC}"
    echo -e "${YELLOW}→ Redémarrage dans 1 seconde...${NC}"
    echo ""

    # Attendre un peu avant de redémarrer
    sleep 1
done

echo -e "${GREEN}✓ POV Viewer arrêté${NC}"
