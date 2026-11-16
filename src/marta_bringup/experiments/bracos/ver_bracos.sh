#!/bin/bash
# Ver no Gazebo (GUI, campo completo world.world) a marcha com uma postura de
# braco do estudo docs/bracos.md. Sobe a simulacao, anda sozinho e fecha.
#
#   bash src/marta_bringup/experiments/bracos/ver_bracos.sh [postura] [opcoes]
#   bash src/marta_bringup/experiments/bracos/ver_bracos.sh todas     [opcoes]
#
# [postura] (sem ela: P2a, o default do launch desde 2026-11-14):
#            P0 (historica, aberto 50 graus, esticado) | P1 (junto, esticado)
#            P3 (junto, cotovelo 45) | P2 (junto, cotovelo 90, "contraida")
#            P2c (P2 com ombro -15,2: CoM devolvido)
#            P0f (P0 com ombro +18,7: CoM a frente)
#            P2a (P2c com o ombro aberto 15 graus: cotovelo a ~25 mm do tronco,
#                 contra 4,5 mm na P2c; ombro -15,8 mantem o CoM da P0)
#            todas (roda as 7 em sequencia)
# [opcoes], em qualquer ordem:
#   reta (default) | giro | curva | lateral | lugar   movimento
#   s0 | s1 (default) | s2                            balanco dos bracos
#   semroll                                           desliga o estabilizador de roll (B2/B2x)
#   m33                                               3,30 kg (default: m38, 3,80 kg)
#   abre=<graus>                                      troca a abertura do ombro (roll) da postura
#   vy=<m/s>                                          velocidade do lateral (default 0,05; negativo = direita)
#   passo05                                           step_period 0,5 e y_separation 0,05 (proposta da
#                                                     A3/A3x: lateral ate +-0,09; docs/marcha_lateral.md)
#   ff                                                compensa o avanco espurio: vx -0,015 (m38) / -0,010 (m33)
#                                                     no lateral e no lugar (proposta da A1x)
# DURACAO=<s> muda o tempo de marcha (default 15 s: a reta a 0,1 m/s chega a
# 1,5 m, antes da bola em x = 2 m e do gol em x = 3 m).
#
# Exemplos:
#   bash .../ver_bracos.sh lateral            # P2a andando de lado a 0,05
#   bash .../ver_bracos.sh lateral vy=0.09 passo05 ff   # lateral rapido, com as propostas
#   bash .../ver_bracos.sh P2                 # contraida, reta, com roll
#   bash .../ver_bracos.sh P0 giro semroll    # atual, giro, sem roll (deve cair)
#   bash .../ver_bracos.sh P2c abre=20        # P2c com o ombro aberto 20 graus
#   bash .../ver_bracos.sh todas reta s2      # as 7 posturas, uma depois da outra
set -u
ESTE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
RAIZ="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
cd "$RAIZ"

braco() {
  case "$1" in
    P0)  echo "arm_shoulder_roll_deg:=50 arm_elbow_deg:=0 arm_shoulder_pitch_deg:=0" ;;
    P1)  echo "arm_shoulder_roll_deg:=0 arm_elbow_deg:=0 arm_shoulder_pitch_deg:=0" ;;
    P3)  echo "arm_shoulder_roll_deg:=0 arm_elbow_deg:=45 arm_shoulder_pitch_deg:=0" ;;
    P2)  echo "arm_shoulder_roll_deg:=0 arm_elbow_deg:=90 arm_shoulder_pitch_deg:=0" ;;
    P2c) echo "arm_shoulder_roll_deg:=0 arm_elbow_deg:=90 arm_shoulder_pitch_deg:=-15.2" ;;
    P0f) echo "arm_shoulder_roll_deg:=50 arm_elbow_deg:=0 arm_shoulder_pitch_deg:=18.7" ;;
    P2a) echo "arm_shoulder_roll_deg:=15 arm_elbow_deg:=90 arm_shoulder_pitch_deg:=-15.8" ;;
    *)   return 1 ;;
  esac
}

case "${1:-}" in
  -h|--help|ajuda) sed -n '2,/^set -u/{/^set -u/!p}' "$ESTE" | sed 's/^# \{0,1\}//'; exit 0 ;;
esac
POSTURA=P2a
if [ -n "${1:-}" ] && { [ "$1" = todas ] || braco "$1" > /dev/null; }; then
  POSTURA=$1; shift
fi

MOV=reta; S=1; ROLL=true; MASSA=m38; ABRE=; VY=0.05; PASSO=; FF=0
for A in "$@"; do
  case "$A" in
    reta|giro|curva|lateral|lugar) MOV=$A ;;
    s0|s1|s2) S=${A#s} ;;
    semroll) ROLL=false ;;
    m33|m38) MASSA=$A ;;
    abre=*) ABRE=${A#abre=} ;;
    vy=*) VY=${A#vy=}; MOV=lateral ;;
    passo05) PASSO="step_period:=0.5 y_separation:=0.05" ;;
    ff) FF=1 ;;
    *) echo "opcao desconhecida: $A (ajuda: bash $ESTE -h)"; exit 1 ;;
  esac
done

case "$MOV" in
  reta)    VEL="walk_velocity_x:=0.1 walk_velocity_y:=0 walk_velocity_yaw:=0" ;;
  giro)    VEL="walk_velocity_x:=0 walk_velocity_y:=0 walk_velocity_yaw:=0.175" ;;
  curva)   VEL="walk_velocity_x:=0.1 walk_velocity_y:=0 walk_velocity_yaw:=0.175" ;;
  lateral) VEL="walk_velocity_x:=0 walk_velocity_y:=$VY walk_velocity_yaw:=0" ;;
  lugar)   VEL="walk_velocity_x:=0 walk_velocity_y:=0 walk_velocity_yaw:=0" ;;
esac
if [ "$MASSA" = m33 ]; then
  CORPO="torso_mass_scale:=0.5 z_com:=0.250 hip_offset_x:=0.0036"
else
  CORPO="torso_mass_scale:=0.67 z_com:=0.239 hip_offset_x:=0.009"
fi
if [ "$FF" = 1 ] && { [ "$MOV" = lateral ] || [ "$MOV" = lugar ]; }; then
  if [ "$MASSA" = m33 ]; then VX=-0.010; else VX=-0.015; fi
  VEL="${VEL/walk_velocity_x:=0 /walk_velocity_x:=$VX }"
fi

xhost +local:root > /dev/null

rodar() {
  local P=$1
  echo
  echo "=== $P${ABRE:+ (ombro aberto $ABRE graus)} | $MOV$([ "$MOV" = lateral ] && echo " vy $VY")${PASSO:+ | passo 0,5/0,05}$([ "$FF" = 1 ] && echo " | ff vx") | s = $S | roll $ROLL | $MASSA | ${DURACAO:-15} s de marcha (Ctrl+C interrompe) ==="
  docker run --rm -it --init --name marta_ver --network host --gpus all --device /dev/dri \
    -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 \
    -e __GLX_VENDOR_LIBRARY_NAME=nvidia -e __NV_PRIME_RENDER_OFFLOAD=1 \
    -v "$RAIZ":/ws -v /tmp/.X11-unix:/tmp/.X11-unix -w /ws marta_bringup:1.0 \
    bash -c "
      source /opt/ros/noetic/setup.bash && source devel/setup.bash
      trap 'kill 0' INT TERM
      roslaunch marta_bringup stabilizer_test.launch gui:=true \
        foot_collision:=box stabilizer_enabled:=true heading_enabled:=true \
        joint_effort:=2.5 joint_velocity:=100 implicit_damping:=true joint_damping:=0.43 \
        gains:=/ws/src/marta_description/config/controller_mx28.yaml \
        gravity_comp_enabled:=true gravity_comp_joints:=hip_roll,knee gravity_comp_gain:=0.3 \
        hip_pitch_offset_deg:=-5 $CORPO \
        roll_stabilizer_enabled:=$ROLL kp_roll_hip:=0.5 kd_roll_hip:=0.075 \
        kp_roll_ankle:=0.5 kd_roll_ankle:=0.075 kd_pitch:=0.06 \
        $VEL $PASSO $(braco "$P") ${ABRE:+arm_shoulder_roll_deg:=$ABRE} arms_compensation_factor:=$S > /tmp/launch.log 2>&1 &
      sleep 24
      rosrun marta_bringup walk_test.py _duration:=${DURACAO:-15} 2>&1 | grep -E 'RESULTADO|tempo real'
      sleep 3
      kill %1 2>/dev/null; wait
    "
}

if [ "$POSTURA" = todas ]; then
  for P in P0 P1 P3 P2 P2c P2a P0f; do rodar "$P"; done
else
  rodar "$POSTURA"
fi
