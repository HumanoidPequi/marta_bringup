#!/bin/bash
# Campanha de postura (plano_comparacao_darwin.md, E0 + E1c).
#
#   bash src/marta_bringup/experiments/rodar_postura.sh            # campanha
#   bash src/marta_bringup/experiments/rodar_postura.sh piloto ZC  # 1 corrida, tau=1000
#
# Pergunta: de onde vem o torque que a marcha pede? Os pilotos (2026-10-15)
# mostraram dois efeitos opostos: esticar a perna (z_com maior) alivia o
# joelho de APOIO, mas dobra o torque da perna em BALANCO, que precisa
# levantar o pe z_step em 0,2 s. Fatorial 2x2:
#   z_com  0,2975 (atual, joelho ~110 graus) x 0,345 (~72 graus)
#   z_step 0,06 (atual) x 0,04 (a altura de pe da marcha do Darwin-OP)
# em tau = 1000 (torque PEDIDO, sem saturacao), 6 e 4 (sobrevivencia).
# z_com 0,36 (~56 graus) caiu em 2,5 s no piloto e ficou de fora.
# A celula (0,2975; 0,06) e a nova linha de base com o pe em caixa (E0).
#
# Regime de operacao: pe em caixa, estabilizador de pitch e malha de rumo
# ligados, 30 s a 0,1 m/s.
#
# Uma corrida por vez, condicoes intercaladas dentro de cada repeticao.
# Cada corrida grava um bag, extrai os CSV e apaga o bag.
set -u
RAIZ="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$RAIZ"
EXP=src/marta_bringup/experiments
OUT="$EXP/postura"
CAMP="$OUT/campanha_postura.csv"
mkdir -p "$OUT/dados"

CORRIDAS=()   # "z_com z_step tau repeticao"
if [ "${1:-}" = piloto ]; then
  CORRIDAS+=("${2:-0.36} ${3:-0.06} 1000 0")
else
  for REP in $(seq 1 ${NREP:-6}); do
    for ZC in 0.2975 0.345; do
      for ZS in 0.06 0.04; do
        for TAU in 1000 6 4; do CORRIDAS+=("$ZC $ZS $TAU $REP"); done
      done
    done
  done
fi

TOTAL=${#CORRIDAS[@]}
echo "$(date +%H:%M:%S)  campanha postura: $TOTAL corridas"
N=0
for C in "${CORRIDAS[@]}"; do
  read -r ZC ZS TAU REP <<< "$C"
  N=$((N + 1))
  TAG="zc${ZC}_zs${ZS}_tau${TAU}_r${REP}"
  echo "$(date +%H:%M:%S)  [$N/$TOTAL] $TAG"
  timeout 300 docker run --rm -v "$RAIZ":/ws -w /ws marta_bringup:1.0 bash -lc "
    source /opt/ros/noetic/setup.bash && source devel/setup.bash
    roslaunch marta_bringup stabilizer_test.launch gui:=false \
      stabilizer_enabled:=true heading_enabled:=true joint_effort:=$TAU \\
      foot_collision:=box z_com:=$ZC z_step:=$ZS > /tmp/l.log 2>&1 &
    sleep 24
    rosrun marta_bringup walk_test.py _duration:=30 _velocity_x:=0.1 \
      _tag:=$TAG _tau:=$TAU _rep:=$REP \
      _bag_out:=/tmp/$TAG.bag _campanha_csv:=/ws/$CAMP 2>&1 \
      | grep -E 'RESULTADO|avanco|VERDADE pitch|tempo real'
    python3 /ws/$EXP/bag_para_csv.py /tmp/$TAG.bag /ws/$OUT/dados > /dev/null
    grep -E 'zCom=|tZMP=' ~/.ros/log/latest/controller_node*.log /tmp/l.log 2>/dev/null | head -2 | sed 's/.*\] //'
  " 2>&1 | sed 's/^/      /' || echo "      FALHOU (segue)"
done
echo "$(date +%H:%M:%S)  fim. $(( $(wc -l < "$CAMP") - 1 )) linhas em $CAMP"
