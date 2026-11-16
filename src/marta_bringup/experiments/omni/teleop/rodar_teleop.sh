#!/bin/bash
# Validacao do teleop (docs/marcha_omnidirecional.md, secao Teleop).
#   bash rodar_teleop.sh <desenho.csv>
# desenho: tag,torso_mass_scale,z_com,hip_offset_x,omega,extra
# Campo livre (campanha automatica); M34 + roll; cmd_vel_enabled; sequencia_cmd.py
# dirige a velocidade; walk_test.py mede 92 s (a sequencia dura 90 s).
set -u
RAIZ=/home/lucasolives/Documents/ic/marta_bringup; cd "$RAIZ"
EXP=src/marta_bringup/experiments; OUT=$EXP/omni/teleop; CAMP=$OUT/campanha_panorama.csv
mkdir -p "$OUT/dados"
MX="joint_effort:=2.5 joint_velocity:=100 implicit_damping:=true joint_damping:=0.43 gains:=/ws/src/marta_description/config/controller_mx28.yaml"
ROLL="roll_stabilizer_enabled:=true kp_roll_hip:=0.5 kd_roll_hip:=0.075 kp_roll_ankle:=0.5 kd_roll_ankle:=0.075"
W="world_name:=/ws/$EXP/omni/diag/campo_livre.world"
tail -n +2 "$1" | while IFS=, read -r TAG TMS ZC HX OM EXTRA; do
  grep -q "^$TAG," "$CAMP" 2>/dev/null && continue
  echo "$(date +%H:%M:%S)  $TAG"
  timeout 400 docker run --rm -v "$RAIZ":/ws -w /ws marta_bringup:1.0 bash -lc "
    source /opt/ros/noetic/setup.bash && source devel/setup.bash
    PYTHONUNBUFFERED=1 roslaunch marta_bringup stabilizer_test.launch gui:=false arm_shoulder_roll_deg:=50 arm_shoulder_pitch_deg:=0 arm_elbow_deg:=0 \
      stabilizer_enabled:=true heading_enabled:=true foot_collision:=box $MX \
      torso_mass_scale:=$TMS z_com:=$ZC hip_offset_x:=$HX hip_pitch_offset_deg:=-5 \
      gravity_comp_enabled:=true gravity_comp_joints:=hip_roll,knee gravity_comp_gain:=0.3 \
      $ROLL $W cmd_vel_enabled:=true $EXTRA > /tmp/l.log 2>&1 &
    sleep 24
    python3 /ws/$EXP/registrar_pe.py _out:=/ws/$OUT/dados/${TAG}_pe.csv > /dev/null 2>&1 &
    python3 /ws/$OUT/sequencia_cmd.py $OM > /ws/$OUT/dados/${TAG}_seq.txt 2>&1 &
    rosrun marta_bringup walk_test.py _duration:=92 _tag:=$TAG _tau:=2.5 \
      _bag_out:=/tmp/$TAG.bag _campanha_csv:=/ws/$CAMP 2>&1 | grep -E 'RESULTADO|tempo real'
    kill %2 %3 2>/dev/null; sleep 1
    python3 /ws/$EXP/bag_para_csv.py /tmp/$TAG.bag /ws/$OUT/dados > /dev/null
    cp /tmp/l.log /ws/$OUT/dados/${TAG}_launch.log 2>/dev/null
  " 2>&1 | sed 's/^/      /'
done
echo "$(date +%H:%M:%S)  fim."
