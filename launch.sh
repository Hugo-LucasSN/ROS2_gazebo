#!/bin/bash
# BOCBOT - Match BLEU vs ORANGE
# Contrôles dans la fenêtre Python (cliquez pour activer)

set +e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
ORANGE='\033[0;33m'
NC='\033[0m'

ROS_PID=""
GAME_PID=""
SHUTDOWN_REQUESTED=false

cleanup() {
    if [ "$SHUTDOWN_REQUESTED" = true ]; then return; fi
    SHUTDOWN_REQUESTED=true
    echo ""
    echo -e "${YELLOW}Arret...${NC}"
    kill -SIGINT $GAME_PID 2>/dev/null || true
    kill -SIGINT $ROS_PID 2>/dev/null || true
    sleep 1
    pkill -9 -f "gzserver" 2>/dev/null || true
    pkill -9 -f "gzclient" 2>/dev/null || true
    pkill -9 -f "gazebo" 2>/dev/null || true
    pkill -9 -f "game_viewer" 2>/dev/null || true
    pkill -9 -f "game_manager" 2>/dev/null || true
    pkill -9 -f "robot_state_publisher" 2>/dev/null || true
    pkill -9 -f "python3.*bocbot" 2>/dev/null || true
    if command -v xset >/dev/null 2>&1; then
        xset r on 2>/dev/null || true
    fi
    echo -e "${GREEN}Termine.${NC}"
}

trap cleanup SIGINT SIGTERM EXIT

cd "$(dirname "$0")"

# Nettoyage
killall -9 gzserver gzclient gazebo 2>/dev/null || true
pkill -9 -f "python3.*bocbot" 2>/dev/null || true
sleep 1

# Environnement
unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
export LD_LIBRARY_PATH=""
export PYTHONPATH=""
source /opt/ros/humble/setup.bash
source bocbot_ws/install/setup.bash
export SVGA_VGPU10=0

# Evite les micro-coupures dues a l'auto-repeat clavier X11 pendant le jeu
if command -v xset >/dev/null 2>&1; then
    xset r off 2>/dev/null || true
fi

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  MATCH: BLEU vs ORANGE${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "  ${BLUE}BLEU  ${NC}= W/A/S/D"
echo -e "  ${ORANGE}ORANGE${NC} = Fleches directionnelles"
echo -e "  ESPACE = Stop les deux"
echo ""
echo -e "${YELLOW}Gazebo demarre...${NC}"

# 1. Lancer Gazebo + robots + game_manager en arriere-plan
ros2 launch bocbot_hunter manual_vs_auto.launch.py &
ROS_PID=$!

# 2. Attendre que tout soit pret
echo -e "${YELLOW}Attente spawn robots (8s)...${NC}"
sleep 8

# 3. Lancer game_viewer (fenêtre Python avec contrôles intégrés)
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  PRET !${NC}"
echo -e "${GREEN}  Cliquez sur la fenêtre pour contrôler${NC}"
echo -e "${GREEN}  Ctrl+C ici pour quitter${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

ros2 run bocbot_hunter game_viewer &
GAME_PID=$!

# Attendre que game_viewer se termine
wait $GAME_PID
