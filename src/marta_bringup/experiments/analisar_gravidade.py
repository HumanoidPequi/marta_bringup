#!/usr/bin/env python3
"""Analise da campanha da compensacao de gravidade (rodar_gravidade.sh).

Por condicao:
  - quedas (e se a queda veio antes da marcha: walk_test.py reporta ~2,5 s);
  - cedencia do joelho e do quadril roll de APOIO em relacao ao alvo da IK:
    real - (alvo publicado - Delta), no miolo (25-75%) do apoio simples [graus];
  - Delta medio aplicado ao joelho de apoio [graus];
  - pe de balanco: pico, raspou, ponta (analisar_pe.py).

  python3 analisar_gravidade.py <dir com campanha_gravidade.csv>
"""
import csv
import os
import statistics as st
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analisar_pe as P  # noqa: E402


def cedencia(prefixo, linha):
    rows = list(csv.DictReader(open(prefixo + '_pe.csv')))
    ini = next((float(r['t']) for r in rows if r['support'] != '0'), None)
    if ini is None or 'l_knee_dg' not in rows[0]:
        return None
    t_fim = ini + (float(linha['t_queda']) if linha['caiu'] == '1' else 1e9) - 0.3
    ced, dg = [], []
    for lado_balanco, c in P.passos([r for r in rows if float(r['t']) >= ini + 0.5], t_fim):
        apoio = 'l' if lado_balanco == 'r' else 'r'
        n = len(c)
        for r in c[n // 4: 3 * n // 4]:
            ik = float(r[apoio + '_knee_sp']) - float(r[apoio + '_knee_dg'])
            ced.append(float(r[apoio + '_knee_pv']) - ik)
            dg.append(float(r[apoio + '_knee_dg']))
    if not ced:
        return None
    return st.mean(ced), st.mean(dg), len(ced)


def main():
    d = sys.argv[1]
    cel = {}
    for r in csv.DictReader(open(os.path.join(d, 'campanha_gravidade.csv'))):
        if r['tag'].endswith('_r0'):
            continue
        k = r['tag'].rsplit('_r', 1)[0]
        pre = os.path.join(d, 'dados', r['tag'])
        if not os.path.exists(pre + '_pe.csv'):
            continue
        cel.setdefault(k, []).append((r, cedencia(pre, r), P.analisa(pre, r)))
    print('%-8s %8s %-22s %16s %14s %9s %7s %7s' % (
        'cond', 'quedas', 't_queda (s)', 'cedencia joelho', 'Delta joelho', 'pico mm', 'raspou', 'ponta'))
    for k in sorted(cel):
        v = cel[k]
        q = [x for x in v if x[0]['caiu'] == '1']
        C = [x[1] for x in v if x[1]]
        A = [x[2] for x in v if x[2]]
        f = lambda xs: st.mean(xs) if xs else float('nan')
        print('%-8s %3d/%-3d %-22s %+14.1f° %+12.1f° %9.1f %6.0f%% %6.0f%%' % (
            k, len(q), len(v), ' '.join('%.1f' % float(x[0]['t_queda']) for x in q)[:22] or '-',
            f([c[0] for c in C]), f([c[1] for c in C]), f([a['pico'] for a in A]),
            f([a['raspou'] for a in A]), f([a['ponta'] for a in A])))
    print('\ncedencia = joelho de apoio real - alvo da IK (+ = mais dobrado que a IK quer),'
          ' miolo do apoio simples; quedas em ~2,5 s = antes da marcha medida')


if __name__ == '__main__':
    main()
