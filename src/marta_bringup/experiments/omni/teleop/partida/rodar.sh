#!/bin/bash
# Diagnostico da queda para tras na partida do teleop (2026-11-13).
# Uso: bash rodar.sh <imediato|espera|func> <id>   (rode 3 em paralelo para carregar a maquina)
# bash rodar.sh <modo> : sobe o launch M34 + roll + cmd_vel (sem a janela) e roda repro.py junto, como o teleop
S=$(dirname "$(readlink -f "$0")"); cd /home/lucasolives/Documents/ic/marta_bringup
docker run --rm --name "rep_$2" -v "$PWD":/ws -v "$S":/s -w /ws marta_bringup:1.0 bash -lc "
source /opt/ros/noetic/setup.bash && source devel/setup.bash
roslaunch marta_bringup stabilizer_test.launch gui:=false cmd_vel_enabled:=true \
  foot_collision:=box stabilizer_enabled:=true heading_enabled:=true \
  joint_effort:=2.5 joint_velocity:=100 implicit_damping:=true joint_damping:=0.43 \
  gains:=/ws/src/marta_description/config/controller_mx28.yaml \
  gravity_comp_enabled:=true gravity_comp_joints:=hip_roll,knee gravity_comp_gain:=0.3 \
  torso_mass_scale:=0.534 z_com:=0.239 hip_offset_x:=0.0036 hip_pitch_offset_deg:=-5 \
  roll_stabilizer_enabled:=true kp_roll_hip:=0.5 kd_roll_hip:=0.075 kp_roll_ankle:=0.5 kd_roll_ankle:=0.075 > /tmp/l.log 2>&1 &
sleep 1
python3 /s/repro.py $1 2>&1 | grep -v WARN
grep -E 'Loaded controllers|Started controllers' /tmp/l.log | cut -c1-90
"
