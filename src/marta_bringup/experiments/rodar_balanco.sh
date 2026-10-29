#!/bin/bash
# Experimento 1 da fila (ajustes_marcha.md): mais tempo de balanco reduz o
# atraso do joelho (pe sobe pouco e pousa de ponta)?
#
#   bash src/marta_bringup/experiments/rodar_balanco.sh           # fatorial
#   PER=0.6 DS=0.2 bash .../rodar_balanco.sh massa                 # tronco normal
#
# Configuracao atual da marcha (z_com 0,252 e defaults do launch), tau = 6,
# 0,1 m/s, pe em caixa, estabilizadores ligados. Fatorial:
#   step_period          0,4 (atual) x 0,5 x 0,6 s
#   double_support_ratio 0,3 (atual) x 0,2
# -> apoio simples de 0,28 a 0,48 s. Tronco pela metade (com o tronco normal a
# configuracao atual cai em ~3 s: rodar_pe.sh massa). n = 4, intercaladas.
# Grava o pe e as juntas no mesmo instante (registrar_pe.py).
set -u
RAIZ="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$RAIZ"
EXP=src/marta_bringup/experiments
OUT="$EXP/balanco"
CAMP="$OUT/campanha_balanco.csv"
mkdir -p "$OUT/dados"
CORRIDAS=()   # "periodo ds tronco repeticao"
if [ "${1:-}" = massa ]; then
  for REP in $(seq 1 ${NREP:-4}); do CORRIDAS+=("$PER $DS 1.0 $REP"); done
else
  for REP in $(seq 1 ${NREP:-4}); do
    for PER in 0.4 0.5 0.6; do for DS in 0.3 0.2; do CORRIDAS+=("$PER $DS 0.5 $REP"); done; done
  done
fi
echo "$(date +%H:%M:%S)  campanha balanco: ${#CORRIDAS[@]} corridas"
N=0
for C in "${CORRIDAS[@]}"; do
  read -r PER DS TMS REP <<< "$C"
  N=$((N + 1))
  TAG="per${PER}_ds${DS}_tms${TMS}_r${REP}"
  echo "$(date +%H:%M:%S)  [$N/${#CORRIDAS[@]}] $TAG"
  timeout 300 docker run --rm -v "$RAIZ":/ws -w /ws marta_bringup:1.0 bash -lc "
    source /opt/ros/noetic/setup.bash && source devel/setup.bash
    roslaunch marta_bringup stabilizer_test.launch gui:=false \
      stabilizer_enabled:=true heading_enabled:=true joint_effort:=6 \\
      foot_collision:=box z_com:=0.252 torso_mass_scale:=$TMS \\
      step_period:=$PER double_support_ratio:=$DS > /tmp/l.log 2>&1 &
    sleep 24
    python3 /ws/$EXP/registrar_pe.py _out:=/ws/$OUT/dados/${TAG}_pe.csv > /dev/null 2>&1 &
    rosrun marta_bringup walk_test.py _duration:=30 _velocity_x:=0.1 \
      _tag:=$TAG _tau:=6 _rep:=$REP \
      _bag_out:=/tmp/$TAG.bag _campanha_csv:=/ws/$CAMP 2>&1 \
      | grep -E 'RESULTADO|tempo real'
    kill %2 2>/dev/null; sleep 1
    python3 /ws/$EXP/bag_para_csv.py /tmp/$TAG.bag /ws/$OUT/dados > /dev/null
    grep -E 'zCom=' ~/.ros/log/latest/controller_node*.log 2>/dev/null | head -1 | sed 's/.*\\] //'
  " 2>&1 | sed 's/^/      /' || echo "      FALHOU (segue)"
done
echo "$(date +%H:%M:%S)  fim."
