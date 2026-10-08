#!/bin/bash
# Campanha completa do estabilizador de pitch.
#
#   bash src/marta_bringup/experiments/rodar_campanha.sh
#
# Uma corrida por vez, de proposito: rodar em paralelo disputaria CPU e
# contaminaria o fator de tempo real, que e uma das variaveis controladas.
# Cada corrida acrescenta uma linha em campanha.csv e grava sua serie temporal.
set -u
RAIZ="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$RAIZ"
EXP=src/marta_bringup/experiments
CAMP="$EXP/campanha.csv"
mkdir -p "$EXP/series"

# Niveis mais repeticoes onde a variancia importa; menos onde o desfecho e obvio.
declare -A REPS=( [1000]=5 [12]=5 [8]=8 [6]=8 [5]=8 [4]=5 )
ORDEM=(1000 12 8 6 5 4)

TOTAL=0
for t in "${ORDEM[@]}"; do TOTAL=$((TOTAL + REPS[$t] * 2)); done
echo "$(date +%H:%M:%S)  campanha: $TOTAL corridas"
N=0
for TAU in "${ORDEM[@]}"; do
  for STAB in false true; do
    for REP in $(seq 1 "${REPS[$TAU]}"); do
      N=$((N + 1))
      TAG="tau${TAU}_$([ "$STAB" = true ] && echo on || echo off)_r${REP}"
      echo "$(date +%H:%M:%S)  [$N/$TOTAL] $TAG"
      timeout 300 docker run --rm -v "$RAIZ":/ws -w /ws marta_bringup:1.0 bash -lc "
        source /opt/ros/noetic/setup.bash && source devel/setup.bash
        roslaunch marta_bringup stabilizer_test.launch gui:=false \
          stabilizer_enabled:=$STAB joint_effort:=$TAU > /tmp/l.log 2>&1 &
        sleep 24
        rosrun marta_bringup walk_test.py _duration:=20 _velocity_x:=0.1 \
          _tag:=$TAG _tau:=$TAU _rep:=$REP \
          _csv_out:=/ws/$EXP/series/$TAG.csv \
          _campanha_csv:=/ws/$CAMP 2>&1 | tail -3
      " 2>&1 | sed 's/^/      /' || echo "      FALHOU (segue)"
    done
  done
done
echo "$(date +%H:%M:%S)  fim. $(( $(wc -l < "$CAMP") - 1 )) linhas em $CAMP"
