#!/bin/bash
# Experimento: o tZMP do gerador esta descasado da altura real do CoM?
#
# comSwingFactor entra so em tZMP = sqrt(zCom*csf/g). Com csf=1.0 o gerador
# assume zc=0.2975 m (distancia torso-pe da IK); a telemetria mediu o CoM real a
# 0.2184 m, o que da csf=0.734 para casar. Estabilizador DESLIGADO de proposito:
# queremos a qualidade do GERADOR, sem a malha de realimentacao mascarando.
#
# Dois niveis de torque: nominal isola o efeito de modelo, 6 N.m mostra se ele
# importa no regime realista.
set -u
RAIZ="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$RAIZ"
EXP=src/marta_bringup/experiments
CAMP="$EXP/campanha_csf.csv"
REPS=${REPS:-4}

echo "$(date +%H:%M:%S)  experimento comSwingFactor: $((2*2*REPS)) corridas"
N=0
for TAU in 1000 6; do
  for CSF in 1.0 0.734; do
    for R in $(seq 1 "$REPS"); do
      N=$((N+1))
      TAG="tau${TAU}_csf${CSF}_r${R}"
      echo "$(date +%H:%M:%S)  [$N/$((2*2*REPS))] $TAG"
      timeout 300 docker run --rm -v "$RAIZ":/ws -w /ws marta_bringup:1.0 bash -lc "
        source /opt/ros/noetic/setup.bash && source devel/setup.bash
        roslaunch marta_bringup stabilizer_test.launch gui:=false \
          stabilizer_enabled:=false joint_effort:=$TAU \
          com_swing_factor:=$CSF > /tmp/l.log 2>&1 &
        sleep 24
        rosrun marta_bringup walk_test.py _duration:=20 _velocity_x:=0.1 \
          _tag:=$TAG _tau:=$TAU _rep:=$R _campanha_csv:=/ws/$CAMP 2>&1 | tail -2
      " 2>&1 | sed 's/^/      /' || echo "      FALHOU (segue)"
    done
  done
done
echo "$(date +%H:%M:%S)  fim: $(( $(wc -l < "$CAMP") - 1 )) linhas"
