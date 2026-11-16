#!/usr/bin/env python3
"""B2 dos bracos: estresse sem roll (docs/bracos.md, sec. 7; criterio pre-registrado).

Le bracos/resumo_omni.csv. Tags B2_<massa>_<P>_s<s>_<mov>_<pe>_r1, P em P0|P2,
s em 0|1, mov em giro+0.175|lat+0.05|lat-0.05. Par = (massa, mov, pe): 12.

Contrastes (Holm sobre os 6 p-valores):
  C1 postura   P2 s1 contra P0 s1
  C2 balanco   P0 s1 contra P0 s0
  C3 balanco   P2 s1 contra P2 s0
Em cada um: sinal na sobrevivencia (pares discordantes) e Wilcoxon no tempo de
queda (sobrevida, pares em que os dois caem). Positivo se um dos dois tem Holm
p < 0,05 a favor do braco testado (mais completas ou queda mais tarde).
Descritivo: direcao da queda, "durou mais" (censurado em 30 s), roll RMS.

  python3 analisar_b2.py [bracos]
"""
import csv
import math
import os
import re
import sys

from analisar_e5 import f, holm, sinal, wil
from analisar_e7 import med, ok

CONTRASTES = (('C1 postura P2 contra P0 (s1)', ('P0', 1), ('P2', 1)),
              ('C2 balanco s1 contra s0 (P0)', ('P0', 0), ('P0', 1)),
              ('C3 balanco s1 contra s0 (P2)', ('P2', 0), ('P2', 1)))


def ler(d):
    P = {}
    for r in csv.DictReader(open(os.path.join(d, 'resumo_omni.csv'))):
        m = re.match(r'^B2_(M\d+)_(P\d)_s(\d)_((?:giro|lat)[+-][\d.]+)_(p[DE])_r\d+$', r['tag'])
        if m:
            massa, post, s, mov, pe = m.groups()
            P.setdefault((massa, mov, pe), {})[(post, int(s))] = r
    return P


def dur(r):
    return 30.0 if ok(r) else f(r['sobrevida'])


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'bracos')
    P = ler(d)
    print('%d pares (massa, mov, pe)\n' % len(P))

    print('Desfecho por condicao (completou/total; sobrevida mediana dos que caem; direcoes):')
    for post in ('P0', 'P2'):
        for s in (0, 1):
            for mov in sorted({k[1] for k in P}):
                rs = [v[(post, s)] for (m, mv, pe), v in P.items() if mv == mov and (post, s) in v]
                if not rs:
                    continue
                caem = [r for r in rs if not ok(r)]
                print('  %s s%d %-11s %d/%d  queda med %5.1f s  %s' % (
                    post, s, mov, sum(ok(r) for r in rs), len(rs),
                    med([f(r['sobrevida']) for r in caem]) if caem else float('nan'),
                    ' '.join(sorted(r['direcao'] for r in caem))))

    print()
    ps, res = [], []
    for nome, a, b in CONTRASTES:
        pp = [(v[a], v[b]) for v in P.values() if a in v and b in v]
        sb = sum(ok(y) and not ok(x) for x, y in pp)
        sa = sum(ok(x) and not ok(y) for x, y in pp)
        p1 = sinal(sb, sa)
        both = [(x, y) for x, y in pp if not ok(x) and not ok(y)]
        t0 = [f(x['sobrevida']) for x, _ in both]
        t1 = [f(y['sobrevida']) for _, y in both]
        dT, p2 = wil(t1, t0)
        mais = sum(dur(y) > dur(x) + 1e-6 for x, y in pp)
        menos = sum(dur(y) < dur(x) - 1e-6 for x, y in pp)
        dR = med([f(y['roll_rms_o']) - f(x['roll_rms_o']) for x, y in pp
                  if not (math.isnan(f(x['roll_rms_o'])) or math.isnan(f(y['roll_rms_o'])))] or [float('nan')])
        res.append((nome, pp, sb, sa, p1, both, dT, p2, mais, menos, dR))
        ps += [p1, 1.0 if math.isnan(p2) else p2]
    adj = holm(ps)
    for i, (nome, pp, sb, sa, p1, both, dT, p2, mais, menos, dR) in enumerate(res):
        print('%s (%d pares)' % (nome, len(pp)))
        print('    sobrevivencia: so testado %d, so referencia %d, sinal p %.2g, Holm %.2g' % (
            sb, sa, p1, adj[2 * i]))
        print('    tempo de queda (n %d, ambos caem): mediana (testado - ref) %+.2f s, p %.2g, Holm %.2g' % (
            len(both), dT, p2, adj[2 * i + 1]))
        print('    descritivo: durou mais %d, menos %d, empate %d (censura 30 s); roll RMS %+.2f graus' % (
            mais, menos, len(pp) - mais - menos, dR))
        pos = (adj[2 * i] < 0.05 and sb > sa) or (adj[2 * i + 1] < 0.05 and dT > 0)
        neg = (adj[2 * i] < 0.05 and sb < sa) or (adj[2 * i + 1] < 0.05 and dT < 0)
        print('    -> %s' % ('POSITIVO' if pos else 'NEGATIVO (piora)' if neg else 'sem efeito significativo'))


if __name__ == '__main__':
    main()
