#!/usr/bin/env python3
"""Analise do experimento 3: Darwin-OP com o modelo de servo MX-28.

Por condicao (tau): quedas, avanco, e para joelho (j_tibia), quadril pitch
(j_thigh2) e tornozelo pitch (j_ankle1) das duas pernas: comando (antes da
saturacao) RMS e p99 [N.m], fracao do tempo com |comando| > 2,5 N.m (stall do
MX-28) e >= tau (saturado), e erro de rastreamento RMS [graus]. Janela: de 2 s
apos o inicio da marcha ate o fim ou 0,5 s antes da queda.

  python3 analisar_darwin.py [dir]   (default: ./darwin_mx28)
"""
import csv
import math
import os
import statistics as st
import sys

JUNTAS = {'joelho': 'tibia', 'quadril pitch': 'thigh2', 'tornozelo pitch': 'ankle1'}


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), 'darwin_mx28')
    cel = {}
    for r in csv.DictReader(open(os.path.join(d, 'campanha_darwin.csv'))):
        if r['tag'].endswith('_r0'):
            continue
        cel.setdefault(r['tau'], []).append(r)
    ordem = sorted(cel, key=lambda k: (k == 'atual', float(k) if k != 'atual' else 0))
    print('%-7s %7s %-22s %8s %6s' % ('tau', 'quedas', 't_queda (s)', 'avanco', 'rtf'))
    for k in ordem:
        v = cel[k]
        q = [x for x in v if x['caiu'] == '1']
        ok = [x for x in v if x['caiu'] != '1']
        print('%-7s %3d/%-3d %-22s %8s %6.3f' % (
            k, len(q), len(v), ' '.join(x['t_queda'] for x in q)[:22] or '-',
            ('%.2f m' % st.mean(float(x['avanco']) for x in ok)) if ok else '-',
            min(float(x['rtf']) for x in v)))
    print('\ncomando RMS / p99 [N.m], %% do tempo > 2,5 N.m, %% saturado, erro RMS [graus] (media das corridas com dado)')
    for nome, j in JUNTAS.items():
        print(nome)
        for k in ordem:
            res = []
            for x in cel[k]:
                f = os.path.join(d, 'dados', x['tag'] + '.csv')
                if not os.path.exists(f):
                    continue
                t_fim = float(x['t_queda']) - 0.5 if x['caiu'] == '1' else 1e9
                rows = [r for r in csv.DictReader(open(f)) if 2.0 <= float(r['t']) <= t_fim]
                if len(rows) < 50:
                    continue
                c = [float(r['j_%s_%s_cmd' % (j, s)]) for r in rows for s in 'lr']
                e = [float(r['j_%s_%s_sp' % (j, s)]) - float(r['j_%s_%s_pv' % (j, s)]) for r in rows for s in 'lr']
                a = sorted(abs(v) for v in c)
                lim = 10.0 if k == 'atual' else float(k)
                res.append((math.sqrt(st.mean([v * v for v in c])), a[int(0.99 * (len(a) - 1))],
                            100 * sum(v > 2.5 for v in a) / len(a), 100 * sum(v >= lim - 1e-6 for v in a) / len(a),
                            math.degrees(math.sqrt(st.mean([v * v for v in e])))))
            if res:
                m = [st.mean(z) for z in zip(*res)]
                print('  tau %-6s n=%d  %5.2f / %5.2f  %4.0f%%  %4.0f%%  %5.1f°' % (k, len(res), *m))


if __name__ == '__main__':
    main()
