#!/usr/bin/env python3
"""Diagnostico exploratorio da A2: oscilacao de pitch travada na fase do lateral.

1) pitch verdadeiro medio por fase de apoio (telemetria; support 1 = apoio E,
   2 = apoio D) e oscilacao (desvio padrao) entre 3 e 28 s de marcha;
2) pitch e roll de cada pe em relacao ao tronco por FK do URDF, com os
   set-points (comando) e com os process_value (realizado), entre 8 e 20 s.

  python3 diag_fase_pitch.py        (roda do diretorio experiments/lateral)
"""
import csv
import os
import sys

import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(AQUI, '..', 'bracos'))
sys.argv = sys.argv[:1]
import bracos_offline as b  # noqa: E402

EXP = os.path.join(AQUI, '..')


def fase(tag, d):
    T = list(csv.DictReader(open(os.path.join(EXP, d, 'dados', tag + '_telemetria.csv'))))
    t = np.array([float(r['t']) for r in T])
    pt = np.degrees([float(r['pitch_true']) for r in T])
    sp = np.array([int(float(r['support'])) for r in T])
    i0 = np.argmax(sp != sp[0])
    m = (t > t[i0] + 3) & (t < t[i0] + 28)
    if m.sum() < 100 or np.abs(pt[m]).max() > 30:   # sem marcha ou janela inclui queda
        return None
    return pt[m & (sp == 1)].mean(), pt[m & (sp == 2)].mean(), np.std(pt[m])


def eul(R):
    return np.degrees(np.arctan2(R[2, 1], R[2, 2])), np.degrees(-np.arcsin(R[2, 0]))


def pes(mod, tag, d, col):
    por = {}
    for x in csv.DictReader(open(os.path.join(EXP, d, 'dados', tag + '_juntas.csv'))):
        t = float(x['t'])
        if 8 < t < 20:
            por.setdefault(round(t, 3), {})[x['junta'] + '_link'] = float(x[col])
    T = list(csv.DictReader(open(os.path.join(EXP, d, 'dados', tag + '_telemetria.csv'))))
    tt = np.array([float(r['t']) for r in T])
    sp = np.array([int(float(r['support'])) for r in T])
    res = {1: [], 2: []}
    for t, q in sorted(por.items()):
        if len(q) < 20:
            continue
        k = sp[min(np.searchsorted(tt, t), len(sp) - 1)]
        if k in res:
            F = mod.fk(q)
            (lr, lp), (rr, rp) = eul(F['l_feet_1'][0]), eul(F['r_feet_1'][0])
            res[k].append((lp, rp, lr, rr))
    return {k: np.array(v).mean(0) for k, v in res.items()}


def main():
    print('1) pitch medio por fase [graus] (positivo = para frente)')
    casos = [('A1_%s_%s_%s_pD_r1' % (M, br, mv), 'lateral') for M in ('M33', 'M38')
             for br in ('ref', 'hpo0', 'hx0', 's0', 'gc0', 'rumo0', 'pitch0')
             for mv in ('lugar', 'lat+0.05', 'lat-0.05')]
    casos += [('A2_%s_lat%s_pD_r1' % (M, v), 'lateral') for M in ('M33', 'M38') for v in ('+0.075', '-0.075')]
    casos += [('B1_M33_P0_s1_reta0.10_pD_r1', 'bracos')]
    for tag, d in casos:
        r = fase(tag, d) if os.path.exists(os.path.join(EXP, d, 'dados', tag + '_telemetria.csv')) else None
        print('  %-30s %s' % (tag, '(caiu ou sem marcha na janela)' if r is None else
                                'apoio E %+5.2f  apoio D %+5.2f  E - D %+5.2f  oscilacao %.2f' % (r[0], r[1], r[0] - r[1], r[2])))
    print('\n2) orientacao dos pes em relacao ao tronco por FK [graus], por fase')
    mod = b.Modelo(os.path.join(AQUI, '..', 'bracos', 'b0', 'marta_tms0.5.urdf'))
    for tag in ('A1_M33_pitch0_lugar_pD_r1', 'A1_M33_pitch0_lat+0.05_pD_r1', 'A1_M33_pitch0_lat-0.05_pD_r1'):
        for col in ('set_point', 'process_value'):
            r = pes(mod, tag, 'lateral', col)
            print('  %-30s %-13s apoio E: pitch pe E %+5.2f D %+5.2f | apoio D: pitch pe E %+5.2f D %+5.2f' % (
                tag, col, r[1][0], r[1][1], r[2][0], r[2][1]))


if __name__ == '__main__':
    main()
