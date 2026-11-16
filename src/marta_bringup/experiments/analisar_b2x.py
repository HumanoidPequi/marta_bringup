#!/usr/bin/env python3
"""B2x (exploratoria, post hoc): por que a P2 sobrevive sem roll (docs/bracos.md, sec. 7).

Le bracos/resumo_omni.csv: B2x_<M>_<P1|P2c|P0f>_s1_<mov>_<pe>_r1 e, como
referencia, B2_<M>_<P0|P2>_s1_<mov>_<pe>_r1. Par = (massa, mov, pe), 12.
Sobrevivencia de cada braco contra P0 s1 (sinal, Holm sobre 3) e contra P2 s1
(descritivo), mais pitch_marcha mediano e tabela de leitura H1/H2/H3.

  python3 analisar_b2x.py [bracos]
"""
import csv
import os
import re
import sys

from analisar_e5 import f, holm, sinal
from analisar_e7 import med, ok


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'bracos')
    P = {}
    for r in csv.DictReader(open(os.path.join(d, 'resumo_omni.csv'))):
        m = re.match(r'^(B2x?)_(M\d+)_(P\w+?)_s1_((?:giro|lat)[+-][\d.]+)_(p[DE])_r\d+$', r['tag'])
        if m:
            camp, massa, post, mov, pe = m.groups()
            if camp == 'B2' and post not in ('P0', 'P2'):
                continue
            P.setdefault((massa, mov, pe), {})[post] = r
    bracos_ = ('P0', 'P2', 'P1', 'P2c', 'P0f')
    print('Completou (de 12) e pitch_marcha mediano nas que completam / dur. mediana:')
    for b in bracos_:
        rs = [v[b] for v in P.values() if b in v]
        print('  %-4s %2d/%2d  pitch_marcha %+.2f  roll RMS %.2f  queda med %4.1f s  direcoes %s' % (
            b, sum(ok(r) for r in rs), len(rs),
            med([f(r['pitch_marcha']) for r in rs if ok(r)]),
            med([f(r['roll_rms_o']) for r in rs if ok(r)]),
            med([f(r['sobrevida']) for r in rs if not ok(r)]),
            ' '.join(sorted(r['direcao'] for r in rs if not ok(r)))))
        por_mov = {}
        for (m, mv, pe), v in P.items():
            if b in v:
                por_mov.setdefault(mv, []).append(ok(v[b]))
        print('        ' + '  '.join('%s %d/%d' % (mv, sum(x), len(x)) for mv, x in sorted(por_mov.items())))
    print('\nContra P0 s1 (sinal; Holm sobre 3) e contra P2 s1 (descritivo):')
    ps, lin = [], []
    for b in ('P1', 'P2c', 'P0f'):
        pp = [(v['P0'], v[b], v['P2']) for v in P.values() if 'P0' in v and b in v and 'P2' in v]
        sb = sum(ok(y) and not ok(x) for x, y, _ in pp)
        sa = sum(ok(x) and not ok(y) for x, y, _ in pp)
        tb = sum(ok(y) and not ok(z) for _, y, z in pp)
        tz = sum(ok(z) and not ok(y) for _, y, z in pp)
        p = sinal(sb, sa)
        ps.append(p)
        lin.append((b, len(pp), sb, sa, p, tb, tz))
    adj = holm(ps)
    for (b, n, sb, sa, p, tb, tz), a in zip(lin, adj):
        print('  %-4s (%d pares): vs P0 so %s %d, so P0 %d, p %.2g, Holm %.2g | vs P2 so %s %d, so P2 %d, p %.2g' % (
            b, n, b, sb, sa, p, a, b, tb, tz, sinal(tb, tz)))


if __name__ == '__main__':
    main()
