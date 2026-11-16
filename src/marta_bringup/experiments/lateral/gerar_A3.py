#!/usr/bin/env python3
"""Gera lateral/desenho_A3.csv (docs/marcha_lateral.md sec. 6).

  python3 gerar_A3.py <vy>    # |vy| escolhido pela regra da A2
y_separation {0,05; 0,06; 0,07} x step_period {0,5; 0,6; 0,7} x +-vy x 2 massas, pe D.
Base copiada da linha A2 de |vy| 0,10 (mesma massa, sentido, pe D), trocando so walk_velocity_y.
"""
import csv
import os
import sys

aqui = os.path.dirname(os.path.abspath(__file__))
vy = float(sys.argv[1])
rows = list(csv.DictReader(open(os.path.join(aqui, 'desenho_A2.csv'))))
by = {r['base']: r for r in rows}
out = []
for M in ('M33', 'M38'):
    for sg in ('+', '-'):
        ref = None
        for r0 in rows:
            v0 = float(r0['extra'].split('walk_velocity_y:=')[1].split()[0])
            if r0['base'].startswith('A2_%s_' % M) and r0['base'].endswith('_pD') and abs(v0) == 0.1 \
                    and (v0 > 0) == (sg == '+'):
                ref = dict(r0)
                ref['extra'] = ref['extra'].replace('walk_velocity_y:=%s ' % r0['extra'].split('walk_velocity_y:=')[1].split()[0],
                                                    'walk_velocity_y:=%s%g ' % ('' if sg == '+' else '-', vy))
        assert ref is not None and ('walk_velocity_y:=%s%g ' % ('' if sg == '+' else '-', vy)) in ref['extra']
        for ys in ('0.05', '0.06', '0.07'):
            for T in ('0.5', '0.6', '0.7'):
                r = dict(ref)
                r['base'] = 'A3_%s_lat%s%g_y%s_T%s_pD' % (M, sg, vy, ys, T)
                r['extra'] = ref['extra'] + ' y_separation:=%s step_period:=%s' % (ys, T)
                out.append(r)
w = csv.DictWriter(open(os.path.join(aqui, 'desenho_A3.csv'), 'w', newline=''), fieldnames=rows[0].keys())
w.writeheader()
w.writerows(out)
print(len(out), 'linhas; exemplo:', out[0]['base'], '|', out[0]['extra'][-60:])
