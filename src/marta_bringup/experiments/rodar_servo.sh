#!/bin/bash
# Experimento 2 da fila (ajustes_marcha.md): modelo de servo na Marta.
#
#   PER=.. DS=.. TMS=.. bash src/marta_bringup/experiments/rodar_servo.sh vel
#   PER=.. DS=.. TMS=.. bash src/marta_bringup/experiments/rodar_servo.sh piloto
#   PER=.. DS=.. TMS=.. bash src/marta_bringup/experiments/rodar_servo.sh e3
#
# vel:    servo atual (p = 50, sem amortecimento), tau = 6: limite de velocidade
#         das juntas 0,01 rad/s (original do URDF) x 100 (sem efeito). O Gazebo
#         zera o esforco que empurra a junta acima do limite.
# e3:     servo atual (S) x modelo MX-28 (M: p = 12,74, amortecimento implicito
#         0,43 N.m.s/rad, sem limite de velocidade artificial) em tau = 2,5; 4;
#         6 (e 1000 no M). plano_comparacao_darwin.md §3.
# Marcha: z_com 0,252, defaults do launch, step_period PER e
# double_support_ratio DS (melhor do experimento 1); tronco TMS.
# n = 4, intercaladas, uma simulacao por vez; registrar_pe.py grava pe + juntas.
set -u
RAIZ="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$RAIZ"
EXP=src/marta_bringup/experiments
MODO=${1:-e3}
OUT="$EXP/servo_$MODO${SUF:-}"
CAMP="$OUT/campanha_servo.csv"
mkdir -p "$OUT/dados"
S="joint_velocity:=0.01"
M="joint_velocity:=100 implicit_damping:=true joint_damping:=0.43 gains:=/ws/src/marta_description/config/controller_mx28.yaml"
V="joint_velocity:=100"
K="joint_velocity:=100 gains:=/ws/src/marta_description/config/controller_mx28.yaml"
D="joint_velocity:=100 implicit_damping:=true joint_damping:=0.43"
CORRIDAS=()   # "rotulo tau repeticao"
case $MODO in
  vel)    for R in $(seq 1 ${NREP:-4}); do CORRIDAS+=("vel0.01 6 $R" "vel100 6 $R"); done ;;
  piloto) CORRIDAS+=("mx28 1000 0" "mx28 2.5 0") ;;
  massa)  for R in $(seq 1 ${NREP:-4}); do CORRIDAS+=("mx28 2.5 $R"); done ;;
  decomp) for R in $(seq 1 ${NREP:-2}); do CORRIDAS+=("so_kp 6 $R" "so_amort 6 $R" "mx28 6 $R"); done ;;
  e3)     for R in $(seq 1 ${NREP:-4}); do
            for T in 2.5 4 6; do CORRIDAS+=("atual $T $R" "mx28 $T $R"); done
            CORRIDAS+=("mx28 1000 $R")
          done ;;
esac
echo "$(date +%H:%M:%S)  servo/$MODO: ${#CORRIDAS[@]} corridas (per $PER ds $DS tronco $TMS)"
N=0
for C in "${CORRIDAS[@]}"; do
  read -r ROT TAU REP <<< "$C"
  case $ROT in vel0.01|atual) ARGS=$S ;; vel100) ARGS=$V ;; mx28) ARGS=$M ;; so_kp) ARGS=$K ;; so_amort) ARGS=$D ;; esac
  N=$((N + 1))
  TAG="${ROT}_tau${TAU}_r${REP}"
  echo "$(date +%H:%M:%S)  [$N/${#CORRIDAS[@]}] $TAG"
  timeout 300 docker run --rm -v "$RAIZ":/ws -w /ws marta_bringup:1.0 bash -lc "
    source /opt/ros/noetic/setup.bash && source devel/setup.bash
    roslaunch marta_bringup stabilizer_test.launch gui:=false \
      stabilizer_enabled:=true heading_enabled:=true joint_effort:=$TAU \\
      foot_collision:=box z_com:=0.252 torso_mass_scale:=$TMS \\
      step_period:=$PER double_support_ratio:=$DS $ARGS > /tmp/l.log 2>&1 &
    sleep 24
    python3 /ws/$EXP/registrar_pe.py _out:=/ws/$OUT/dados/${TAG}_pe.csv > /dev/null 2>&1 &
    rosrun marta_bringup walk_test.py _duration:=30 _velocity_x:=0.1 \
      _tag:=$TAG _tau:=$TAU _rep:=$REP \
      _bag_out:=/tmp/$TAG.bag _campanha_csv:=/ws/$CAMP 2>&1 \
      | grep -E 'RESULTADO|tempo real|INVALIDA'
    kill %2 2>/dev/null; sleep 1
    python3 /ws/$EXP/bag_para_csv.py /tmp/$TAG.bag /ws/$OUT/dados > /dev/null
    rosparam get /marta/l_knee_position/pid/p 2>/dev/null | sed 's/^/kp joelho: /'
  " 2>&1 | sed 's/^/      /' || echo "      FALHOU (segue)"
done
echo "$(date +%H:%M:%S)  fim."
