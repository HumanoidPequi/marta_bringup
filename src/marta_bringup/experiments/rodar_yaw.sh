#!/bin/bash
# Campanha E2 da investigacao do desvio de direcao (caracterizacao).
#
#   bash src/marta_bringup/experiments/rodar_yaw.sh          # E2
#   bash src/marta_bringup/experiments/rodar_yaw.sh caixa    # E3: pe em caixa
#   bash src/marta_bringup/experiments/rodar_yaw.sh tau6     # caixa, tau=6, stab off x on
#   bash src/marta_bringup/experiments/rodar_yaw.sh espelho  # caixa, tau=6, stab on,
#                                                            # partida pelo pe dir x esq
#   bash src/marta_bringup/experiments/rodar_yaw.sh suave    # idem, com partida suave
#                                                            # em 4 passos (start_ramp_steps)
#   bash src/marta_bringup/experiments/rodar_yaw.sh rumo     # caixa, tau=6, stab on,
#                                                            # malha de rumo off x on
#   bash src/marta_bringup/experiments/rodar_yaw.sh agacha   # caixa, tau=6, stab on: com x
#                                                            # sem os passos de ajuste pos-Agacha
#   bash src/marta_bringup/experiments/rodar_yaw.sh ganho    # caixa, tau=6, stab on, malha de
#                                                            # rumo com kp/sat em 3 niveis
#
# O default do pe virou "box" em 2026-10-07; o E2 passa foot_collision
# explicitamente, entao continua reproduzindo a malha.
#
# tau in {1000, 6} x 10 repeticoes, estabilizador DESLIGADO (mede o sistema sem
# realimentacao), 30 s a 0,1 m/s; mais 3 corridas com foot_mu = 0,5 a tau = 6,
# o controle positivo da regua de giro do pe (condicao de Kajita 2001).
#
# Uma corrida por vez (RTF e variavel controlada). As condicoes sao
# INTERCALADAS dentro de cada repeticao, para que uma deriva lenta da maquina
# nao se confunda com o efeito do torque.
#
# Cada corrida grava um bag, extrai os CSV (telemetria + juntas) e apaga o bag:
# os CSV sao tudo que analisar_yaw.py precisa, e 40 bags passariam de 800 MB.
set -u
RAIZ="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$RAIZ"
EXP=src/marta_bringup/experiments
OUT="$EXP/yaw"
CAMP="$OUT/campanha_yaw.csv"
mkdir -p "$OUT/dados"

# Cada corrida: "tau foot_mu foot_collision estabilizador repeticao [partida [rampa [rumo]]]"
CORRIDAS=()
if [ "${1:-}" = ganho ]; then
  # Sintonia da malha de rumo: um ganho maior segura o giro dos primeiros
  # passos? Campos extras: kp_heading max_heading_rate.
  for REP in $(seq 1 8); do
    CORRIDAS+=("6 1e10 box true $REP dir 0 true false 1.0 0.3"
               "6 1e10 box true $REP dir 0 true false 2.0 0.6"
               "6 1e10 box true $REP dir 0 true false 3.0 0.6")
  done
elif [ "${1:-}" = agacha ]; then
  # Os passos no lugar logo apos o Agacha() causam o giro inicial?
  for REP in $(seq 1 10); do
    CORRIDAS+=("6 1e10 box true $REP dir 0 false false" "6 1e10 box true $REP dir 0 false true")
  done
elif [ "${1:-}" = rumo ]; then
  # A malha de rumo (heading_enabled) endireita a marcha?
  for REP in $(seq 1 10); do
    CORRIDAS+=("6 1e10 box true $REP dir 0 false" "6 1e10 box true $REP dir 0 true")
  done
elif [ "${1:-}" = suave ]; then
  # A partida suave remove a curva? Mesmo desenho do espelho, com rampa de 4
  # passos; compara com as corridas _partdir/_partesq (sem rampa).
  for REP in $(seq 1 10); do
    CORRIDAS+=("6 1e10 box true $REP dir 4" "6 1e10 box true $REP esq 4")
  done
elif [ "${1:-}" = espelho ]; then
  # Teste do espelho: a curva para a direita troca de lado quando a marcha
  # parte pelo outro pe? (start_left_swing)
  for REP in $(seq 1 10); do
    CORRIDAS+=("6 1e10 box true $REP dir" "6 1e10 box true $REP esq")
  done
elif [ "${1:-}" = tau6 ]; then
  # Nova linha de base com o pe em caixa, no regime informativo (tau=6):
  # giro em yaw, e se o ganho do estabilizador de pitch se mantem.
  for REP in $(seq 1 10); do
    CORRIDAS+=("6 1e10 box false $REP" "6 1e10 box true $REP")
  done
elif [ "${1:-}" = caixa ]; then
  # E3, ramo H1: o giro vem da resolucao do contato de MALHA no ODE? Mesma
  # sola, colisao em caixa. Compara com as corridas tau=1000 de malha do E2.
  for REP in $(seq 1 10); do CORRIDAS+=("1000 1e10 box false $REP"); done
else
  for REP in $(seq 1 10); do
    CORRIDAS+=("1000 1e10 mesh false $REP" "6 1e10 mesh false $REP")
  done
  for REP in 1 2 3; do CORRIDAS+=("6 0.5 mesh false $REP"); done
fi

TOTAL=${#CORRIDAS[@]}
echo "$(date +%H:%M:%S)  campanha yaw: $TOTAL corridas"
N=0
for C in "${CORRIDAS[@]}"; do
  read -r TAU MU COL STAB REP PARTIDA RAMPA RUMO PULA KP SAT <<< "$C"
  RAMPA=${RAMPA:-0}
  # Sufixo de rumo so quando o campo e explicito (nao colide com tags antigos).
  SUFR=""
  [ -n "${RUMO:-}" ] && SUFR=$([ "$RUMO" = true ] && echo _rumo || echo _semrumo)
  RUMO=${RUMO:-false}
  SUFP=""
  [ -n "${PULA:-}" ] && SUFP=$([ "$PULA" = true ] && echo _semajuste || echo _comajuste)
  PULA=${PULA:-false}
  SUFK=""
  [ -n "${KP:-}" ] && SUFK="_kp${KP}_sat${SAT}"
  KP=${KP:-1.0}
  SAT=${SAT:-0.3}
  # Sufixo so quando a partida e explicita, para nao colidir com tags antigos.
  SUF=${PARTIDA:+_part$PARTIDA}
  PARTIDA=${PARTIDA:-dir}
  N=$((N + 1))
  TAG="tau${TAU}_mu${MU}$([ "$COL" = box ] && echo _caixa)$([ "$STAB" = true ] && echo _stab)${SUF}$([ "$RAMPA" != 0 ] && echo _suave$RAMPA)${SUFR}${SUFP}${SUFK}_r${REP}"
  echo "$(date +%H:%M:%S)  [$N/$TOTAL] $TAG"
  timeout 300 docker run --rm -v "$RAIZ":/ws -w /ws marta_bringup:1.0 bash -lc "
    source /opt/ros/noetic/setup.bash && source devel/setup.bash
    roslaunch marta_bringup stabilizer_test.launch gui:=false \
      stabilizer_enabled:=$STAB joint_effort:=$TAU foot_mu:=$MU foot_collision:=$COL \\
      start_left_swing:=$([ "$PARTIDA" = esq ] && echo true || echo false) \\
      start_ramp_steps:=$RAMPA heading_enabled:=$RUMO skip_initial_stop_steps:=$PULA \\
      kp_heading:=$KP max_heading_rate:=$SAT > /tmp/l.log 2>&1 &
    sleep 24
    rosrun marta_bringup walk_test.py _duration:=30 _velocity_x:=0.1 \
      _tag:=$TAG _tau:=$TAU _rep:=$REP _foot_mu:=$MU \
      _bag_out:=/tmp/$TAG.bag _campanha_csv:=/ws/$CAMP 2>&1 \
      | grep -E 'RESULTADO|giro do corpo|avanco|caranguejo|VERDADE pitch|tempo real'
    python3 /ws/$EXP/bag_para_csv.py /tmp/$TAG.bag /ws/$OUT/dados > /dev/null
  " 2>&1 | sed 's/^/      /' || echo "      FALHOU (segue)"
done
echo "$(date +%H:%M:%S)  fim. $(( $(wc -l < "$CAMP") - 1 )) linhas em $CAMP"
