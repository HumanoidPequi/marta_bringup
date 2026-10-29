#!/bin/bash
# Investigacao do pe de balanco que raspa/pousa de ponta (ajustes_marcha.md).
#
#   bash src/marta_bringup/experiments/rodar_pe.sh            # campanha
#   bash src/marta_bringup/experiments/rodar_pe.sh piloto      # 1 corrida
#
# Configuracao atual da marcha (z_com 0,252 e os defaults novos do launch),
# tronco com massa normal. Fatores:
#   tau     1000 x 6   -> o pe nao sobe porque a junta SATURA, ou porque o
#                         servo simulado e lento mesmo sem limite?
#   foot_mu 1e10 x 1.0 -> o pe "gruda" porque o atrito e praticamente infinito?
# n = 4 por condicao, intercaladas, uma simulacao por vez.
set -u
RAIZ="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$RAIZ"
EXP=src/marta_bringup/experiments
OUT="$EXP/${PE_OUT:-pe}"
CAMP="$OUT/campanha_pe.csv"
mkdir -p "$OUT/dados"
CORRIDAS=()   # "tau foot_mu repeticao"
if [ "${1:-}" = piloto ]; then
  CORRIDAS+=("6 1e10 0")
elif [ "${1:-}" = juntas ]; then
  # Pitch do pe x angulos reais das juntas no mesmo instante (pouso de ponta)
  for REP in $(seq 1 ${NREP:-4}); do CORRIDAS+=("1000 1e10 $REP"); done
elif [ "${1:-}" = massa ]; then
  # Confirmacao: com o tronco de massa normal a configuracao atual anda?
  for REP in $(seq 1 ${NREP:-4}); do CORRIDAS+=("6 1e10 $REP"); done
else
  for REP in $(seq 1 ${NREP:-4}); do
    for TAU in 1000 6; do for MU in 1e10 1.0; do CORRIDAS+=("$TAU $MU $REP"); done; done
  done
fi
echo "$(date +%H:%M:%S)  campanha pe: ${#CORRIDAS[@]} corridas"
N=0
for C in "${CORRIDAS[@]}"; do
  read -r TAU MU REP <<< "$C"
  N=$((N + 1))
  TAG="tms${TMS:-1.0}_tau${TAU}_mu${MU}_r${REP}"
  echo "$(date +%H:%M:%S)  [$N/${#CORRIDAS[@]}] $TAG"
  timeout 300 docker run --rm -v "$RAIZ":/ws -w /ws marta_bringup:1.0 bash -lc "
    source /opt/ros/noetic/setup.bash && source devel/setup.bash
    roslaunch marta_bringup stabilizer_test.launch gui:=false \
      stabilizer_enabled:=true heading_enabled:=true joint_effort:=$TAU \\
      foot_collision:=box foot_mu:=$MU z_com:=0.252 torso_mass_scale:=${TMS:-1.0} > /tmp/l.log 2>&1 &
    sleep 24
    python3 /ws/$EXP/registrar_pe.py _out:=/ws/$OUT/dados/${TAG}_pe.csv > /dev/null 2>&1 &
    rosrun marta_bringup walk_test.py _duration:=30 _velocity_x:=0.1 \
      _tag:=$TAG _tau:=$TAU _rep:=$REP _foot_mu:=$MU \
      _bag_out:=/tmp/$TAG.bag _campanha_csv:=/ws/$CAMP 2>&1 \
      | grep -E 'RESULTADO|avanco|tempo real'
    kill %2 2>/dev/null; sleep 1
    python3 /ws/$EXP/bag_para_csv.py /tmp/$TAG.bag /ws/$OUT/dados > /dev/null
    grep -E 'zCom=' ~/.ros/log/latest/controller_node*.log 2>/dev/null | head -1 | sed 's/.*\\] //'
  " 2>&1 | sed 's/^/      /' || echo "      FALHOU (segue)"
done
echo "$(date +%H:%M:%S)  fim."
