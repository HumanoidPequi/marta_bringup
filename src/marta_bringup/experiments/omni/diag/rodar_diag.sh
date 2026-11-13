#!/bin/bash
# Diagnostico gol/bola da marcha omnidirecional (docs/marcha_omnidirecional.md, "Obstaculos no mundo").
#   bash rodar_diag.sh <dir>   (<dir> = este diretorio: campo_livre.world, log_modelos.py, desenho_diag.csv)
# Mesmas linhas de roslaunch do rodar_panorama.sh; <dir> e montado como /scratch.
# W = world.world (campo SPL com gols e bola), L = campo_livre.world (mesma fisica, sem gol nem bola).
set -u
S="$(cd "$1" && pwd)"; RAIZ=/home/lucasolives/Documents/ic/marta_bringup; cd "$RAIZ"
EXP=src/marta_bringup/experiments
MX="joint_effort:=2.5 joint_velocity:=100 implicit_damping:=true joint_damping:=0.43 gains:=/ws/src/marta_description/config/controller_mx28.yaml"
tail -n +2 "$S/desenho_diag.csv" | while IFS=, read -r TAG MUNDO TMS ZC HX G EXTRA; do
  if [ "$MUNDO" = L ]; then WARG="world_name:=/scratch/campo_livre.world"; else WARG=""; fi
  grep -q "^$TAG," "$S/campanha_panorama.csv" 2>/dev/null && continue
  echo "$(date +%H:%M:%S)  $TAG"
  timeout 300 docker run --rm -v "$RAIZ":/ws -v "$S":/scratch -w /ws marta_bringup:1.0 bash -lc "
    source /opt/ros/noetic/setup.bash && source devel/setup.bash
    PYTHONUNBUFFERED=1 roslaunch marta_bringup stabilizer_test.launch gui:=false \
      stabilizer_enabled:=true heading_enabled:=true foot_collision:=box $MX \
      torso_mass_scale:=$TMS z_com:=$ZC hip_offset_x:=$HX \
      gravity_comp_enabled:=true gravity_comp_joints:=hip_roll,knee gravity_comp_gain:=$G \
      $WARG $EXTRA > /tmp/l.log 2>&1 &
    sleep 24
    python3 /ws/$EXP/registrar_pe.py _out:=/scratch/dados/${TAG}_pe.csv > /dev/null 2>&1 &
    python3 /scratch/log_modelos.py /scratch/dados/${TAG}_modelos.csv > /dev/null 2>&1 &
    rosrun marta_bringup walk_test.py _duration:=30 _velocity_x:=0.1 \
      _tag:=$TAG _tau:=2.5 \
      _bag_out:=/tmp/$TAG.bag _campanha_csv:=/scratch/campanha_panorama.csv 2>&1 \
      | grep -E 'RESULTADO|tempo real'
    kill %2 %3 2>/dev/null; sleep 1
    python3 /ws/$EXP/bag_para_csv.py /tmp/$TAG.bag /scratch/dados > /dev/null
    cp /tmp/l.log /scratch/dados/${TAG}_launch.log 2>/dev/null
  " 2>&1 | sed 's/^/      /'
done
echo "$(date +%H:%M:%S)  fim."
