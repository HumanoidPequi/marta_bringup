#!/usr/bin/env python3
"""E4 da marcha omnidirecional: piloto do estabilizador de roll
(docs/marcha_omnidirecional.md).

Le omni/resumo_omni.csv. Tags E4_<massa>_<config>_<mov>_<pe>_r1, com config
  cint_K<K>     so quadril: kp_h = K, kd_h = 0,15 K
  quatorn_K<K>  quadril + tornozelo: K/2 cada, kd 0,075 K cada
  torn_K<K>     so tornozelo: kp_a = K, kd_a = 0,15 K
  soD_kd<kd>    so derivativo (quadril e tornozelo)
O braco "off" e a corrida da E1 na mesma condicao (E1_<massa>_<mov>_<pe>_h1);
as replicas da E2 (mesma configuracao) entram para mostrar o ruido.

Portao (plano, E4): A_roll -30%, sem mais quedas, pitch RMS <= +10%, pe plano.
Aqui: por configuracao, quantas condicoes completaram, sobrevida, A_roll e
roll RMS (so nas que completaram nos dois bracos, ou ate a queda), pitch RMS,
saturacao de hip_roll e ank_roll.

  python3 analisar_e4.py [omni] [prefixo E4_]
"""
import csv
import math
import os
import re
import sys
from collections import defaultdict

DUR = 30.0


def f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return float('nan')


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), 'omni')
    pref = sys.argv[2] if len(sys.argv) > 2 else 'E4_'
    R = {r['tag']: r for r in csv.DictReader(open(os.path.join(d, 'resumo_omni.csv')))}
    cfg = defaultdict(dict)
    conds = set()
    for t, r in R.items():
        m = re.match(r'^%s(M\d+)_(\S+?)_((?:reta|giro|curva|lat|diag|comb)[+-]?[\d.]+)_(p[DE])_r\d+$' % pref, t)
        if not m:
            continue
        massa, c, mov, pe = m.groups()
        cfg[c][(massa, mov, pe)] = r
        conds.add((massa, mov, pe))
    for c in conds:
        r = R.get('E1_%s_%s_%s_h1_r1' % c)
        if r:
            cfg['off(E1)'][c] = r
    conds = sorted(conds)

    def cel(r):
        if r is None:
            return '-'
        s = 'ok' if r['desfecho'] == 'completou' else '%.1f' % f(r['sobrevida'])
        return '%s a%.1f r%.1f p%.1f' % (s, f(r['a_roll']), f(r['roll_rms_o']), f(r['pitch_rms']))

    print('%-16s' % 'config' + ''.join('%-28s' % ('%s %s %s' % c) for c in conds) + ' compl  sat_hr sat_ar')
    for c in sorted(cfg, key=lambda k: (k != 'off(E1)', k)):
        rs = cfg[c]
        comp = sum(rs[k]['desfecho'] == 'completou' for k in rs)
        sh = max((f(rs[k]['sat_hip_roll']) for k in rs), default=float('nan'))
        sa = max((f(rs[k]['sat_ank_roll']) for k in rs), default=float('nan'))
        print('%-16s' % c + ''.join('%-28s' % cel(rs.get(k)) for k in conds) + ' %d/%d  %5.1f %5.1f' % (comp, len(rs), sh, sa))
    print('\na = A_roll [graus], r = roll RMS [graus], p = pitch RMS [graus] (ate a queda nas que cairam)')


if __name__ == '__main__':
    main()
