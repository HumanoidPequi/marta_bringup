#!/bin/bash
# Compensacao de gravidade com o servo MX-28 e a massa real (ajustes_marcha.md).
#
#   bash src/marta_bringup/experiments/rodar_gravidade.sh piloto
#   bash src/marta_bringup/experiments/rodar_gravidade.sh          # campanha
#
# Servo MX-28 (p = 12,74, amortecimento implicito 0,43, sem limite de
# velocidade artificial), tau = 2,5 (stall a 12 V), tronco de massa normal,
# marcha padrao (step_period 0,6). Sem compensacao a Marta cai no 1o passo
# (o joelho de apoio cede ~20 graus = tau/Kp). Condicoes:
#   off           compensacao desligada
#   g1            hip_roll,knee, ganho 1,0 (modelo estatico)
#   g175          hip_roll,knee, ganho 1,75 (razao medido/previsto do joelho, E1)
#   todas         hip_roll,hip_pitch,knee,ank_pitch,ank_roll, ganho 1,0
# n = 4, intercaladas, uma simulacao por vez.
set -u
RAIZ="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$RAIZ"
EXP=src/marta_bringup/experiments
OUT="$EXP/gravidade_mx28"
CAMP="$OUT/campanha_gravidade.csv"
mkdir -p "$OUT/dados"
MX="joint_velocity:=100 implicit_damping:=true joint_damping:=0.43 gains:=/ws/src/marta_description/config/controller_mx28.yaml"
# Condicoes (lista em CONDS), tau (TAU) e tronco (TMS) por ambiente. Ex.:
#   CONDS="off g1" TAU=1000 TMS=1.0 NREP=3 ...   (o mecanismo funciona?)
#   CONDS="off g1 g175" TAU=2.5 TMS=0.5 NREP=4   (ajuda onde o torque cabe?)
TAU=${TAU:-2.5}
TMS=${TMS:-1.0}
OUT="$OUT/tau${TAU}_tms${TMS}"
CAMP="$OUT/campanha_gravidade.csv"
mkdir -p "$OUT/dados"
CORRIDAS=()
if [ "${1:-}" = piloto ]; then
  CORRIDAS+=("g1 0")
else
  for R in $(seq 1 ${NREP:-4}); do for C in ${CONDS:-off g1 g175 todas}; do CORRIDAS+=("$C $R"); done; done
fi
echo "$(date +%H:%M:%S)  gravidade: ${#CORRIDAS[@]} corridas"
N=0
for C in "${CORRIDAS[@]}"; do
  read -r ROT REP <<< "$C"
  case $ROT in
    off)   G="gravity_comp_enabled:=false" ;;
    g1)    G="gravity_comp_enabled:=true gravity_comp_joints:=hip_roll,knee gravity_comp_gain:=1.0" ;;
    g175)  G="gravity_comp_enabled:=true gravity_comp_joints:=hip_roll,knee gravity_comp_gain:=1.75" ;;
    todas) G="gravity_comp_enabled:=true gravity_comp_joints:=hip_roll,hip_pitch,knee,ank_pitch,ank_roll gravity_comp_gain:=1.0" ;;
    todas05) G="gravity_comp_enabled:=true gravity_comp_joints:=hip_roll,hip_pitch,knee,ank_pitch,ank_roll gravity_comp_gain:=0.5" ;;
    todas_x)   G="gravity_comp_enabled:=true gravity_comp_joints:=hip_roll,hip_pitch,knee,ank_pitch,ank_roll gravity_comp_gain:=1.0 hip_offset_x:=0.0036" ;;
    todas05_x) G="gravity_comp_enabled:=true gravity_comp_joints:=hip_roll,hip_pitch,knee,ank_pitch,ank_roll gravity_comp_gain:=0.5 hip_offset_x:=0.0036" ;;
    off_x)     G="gravity_comp_enabled:=false hip_offset_x:=0.0036" ;;
    g05_x)     G="gravity_comp_enabled:=true gravity_comp_joints:=hip_roll,knee gravity_comp_gain:=0.5 hip_offset_x:=0.0036" ;;
  esac
  N=$((N + 1))
  TAG="${ROT}_r${REP}"
  echo "$(date +%H:%M:%S)  [$N/${#CORRIDAS[@]}] $TAG"
  timeout 300 docker run --rm -v "$RAIZ":/ws -w /ws marta_bringup:1.0 bash -lc "
    source /opt/ros/noetic/setup.bash && source devel/setup.bash
    roslaunch marta_bringup stabilizer_test.launch gui:=false \
      stabilizer_enabled:=true heading_enabled:=true joint_effort:=$TAU \\
      foot_collision:=box torso_mass_scale:=$TMS $MX $G > /tmp/l.log 2>&1 &
    sleep 24
    python3 /ws/$EXP/registrar_pe.py _out:=/ws/$OUT/dados/${TAG}_pe.csv > /dev/null 2>&1 &
    rosrun marta_bringup walk_test.py _duration:=30 _velocity_x:=0.1 \
      _tag:=$TAG _tau:=$TAU _rep:=$REP \
      _bag_out:=/tmp/$TAG.bag _campanha_csv:=/ws/$CAMP 2>&1 \
      | grep -E 'RESULTADO|tempo real'
    kill %2 2>/dev/null; sleep 1
    python3 /ws/$EXP/bag_para_csv.py /tmp/$TAG.bag /ws/$OUT/dados > /dev/null
    grep -h 'GravityComp' /tmp/l.log ~/.ros/log/latest/controller_node*.log 2>/dev/null | head -1 | sed 's/.*\\] //'
  " 2>&1 | sed 's/^/      /' || echo "      FALHOU (segue)"
done
echo "$(date +%H:%M:%S)  fim."
