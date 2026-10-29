#!/bin/bash
# Bateria do panorama de parametros (docs/panorama_parametros.md).
#
#   bash src/marta_bringup/experiments/rodar_panorama.sh <desenho.csv> [semente]
#
# <desenho.csv>: uma condicao por linha, colunas
#   base,torso_mass_scale,z_com,hip_offset_x,gain,nrep,extra
# (extra = args adicionais do roslaunch, separados por espaco, ou vazio).
# Cada condicao vira nrep corridas com tag <base>_r<k>. A lista completa e
# embaralhada com a semente (registrada no log) e as tags ja presentes no CSV
# da campanha sao puladas (retomada). Para depois de 3 falhas seguidas ou com
# menos de 2 GB livres.
#
# Fixos em todas as corridas: servo MX-28 (p = 12,74 via controller_mx28.yaml,
# amortecimento implicito 0,43, joint_velocity 100, limite 2,5 N.m),
# compensacao de gravidade ligada (hip_roll,knee), pe em caixa, estabilizador de
# pitch e malha de rumo ligados (salvo extra), marcha padrao, 30 s a 0,1 m/s.
# registrar_pe.py grava pe + juntas + Delta da compensacao no mesmo instante.
set -u
RAIZ="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$RAIZ"
EXP=src/marta_bringup/experiments
OUT="$EXP/panorama"
CAMP="$OUT/campanha_panorama.csv"
DESENHO="$1"
SEMENTE="${2:-20261028}"
mkdir -p "$OUT/dados"
MX="joint_effort:=2.5 joint_velocity:=100 implicit_damping:=true joint_damping:=0.43 gains:=/ws/src/marta_description/config/controller_mx28.yaml"

LISTA=$(python3 - "$DESENHO" "$SEMENTE" "$CAMP" <<'PY'
import csv, os, random, sys
des, sem, camp = sys.argv[1], int(sys.argv[2]), sys.argv[3]
feitas = set()
if os.path.exists(camp):
    feitas = {r['tag'] for r in csv.DictReader(open(camp))}
runs = []
for r in csv.DictReader(open(des)):
    for k in range(1, int(r['nrep']) + 1):
        tag = '%s_r%d' % (r['base'], k)
        if tag not in feitas:
            runs.append('|'.join([tag, r['torso_mass_scale'], r['z_com'], r['hip_offset_x'], r['gain'], r['extra']]))
random.Random(sem).shuffle(runs)
print('\n'.join(runs))
PY
)
TOTAL=$(echo -n "$LISTA" | grep -c '^' || true)
echo "$(date +%H:%M:%S)  panorama: $DESENHO, semente $SEMENTE, $TOTAL corridas a fazer"
N=0; FALHAS=0
while IFS='|' read -r TAG TMS ZC HX G EXTRA; do
  [ -z "$TAG" ] && continue
  LIVRE=$(df --output=avail -k "$RAIZ" | tail -1)
  if [ "$LIVRE" -lt 2000000 ]; then echo "PAROU: menos de 2 GB livres"; exit 1; fi
  N=$((N + 1))
  echo "$(date +%H:%M:%S)  [$N/$TOTAL] $TAG"
  timeout 300 docker run --rm -v "$RAIZ":/ws -w /ws marta_bringup:1.0 bash -lc "
    source /opt/ros/noetic/setup.bash && source devel/setup.bash
    PYTHONUNBUFFERED=1 roslaunch marta_bringup stabilizer_test.launch gui:=false \
      stabilizer_enabled:=true heading_enabled:=true foot_collision:=box $MX \\
      torso_mass_scale:=$TMS z_com:=$ZC hip_offset_x:=$HX \\
      gravity_comp_enabled:=true gravity_comp_joints:=hip_roll,knee gravity_comp_gain:=$G \\
      $EXTRA > /tmp/l.log 2>&1 &
    sleep 24
    python3 /ws/$EXP/registrar_pe.py _out:=/ws/$OUT/dados/${TAG}_pe.csv > /dev/null 2>&1 &
    rosrun marta_bringup walk_test.py _duration:=30 _velocity_x:=0.1 \
      _tag:=$TAG _tau:=2.5 \
      _bag_out:=/tmp/$TAG.bag _campanha_csv:=/ws/$CAMP 2>&1 \
      | grep -E 'RESULTADO|tempo real'
    kill %2 2>/dev/null; sleep 1
    python3 /ws/$EXP/bag_para_csv.py /tmp/$TAG.bag /ws/$OUT/dados > /dev/null
    cp /tmp/l.log /ws/$OUT/dados/${TAG}_launch.log 2>/dev/null
  " 2>&1 | sed 's/^/      /'
  if grep -q "^$TAG," "$CAMP" 2>/dev/null; then FALHAS=0; else
    FALHAS=$((FALHAS + 1)); echo "      FALHOU ($FALHAS seguidas)"
    if [ $FALHAS -ge 3 ]; then echo "PAROU: 3 falhas seguidas"; exit 1; fi
  fi
done <<< "$LISTA"
echo "$(date +%H:%M:%S)  fim."
