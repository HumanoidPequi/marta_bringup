#!/usr/bin/env python3
"""Analise dos experimentos com registrar_pe.py (balanco e servo).

Para cada condicao (tag sem o _rN final):
  - quedas e tempos de queda (walk_test.py, verdade de solo);
  - pe de balanco: pico, % de passos que rasparam, % que pousaram de ponta
    (mesmas definicoes de analisar_pe.py);
  - atraso do joelho de balanco (real - alvo, graus) no meio (40-60% do
    apoio simples) e no fim (80-100%), do registro sincronizado;
  - joelho de balanco: erro RMS e fracao saturada (|comando| >= tau).

  python3 analisar_balanco.py <dir> [tau]     (tau default 6)
"""
import csv
import glob
import os
import statistics as st
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analisar_pe as P  # noqa: E402


def atraso_joelho(prefixo, linha):
    rows = list(csv.DictReader(open(prefixo + '_pe.csv')))
    if 'l_knee_pv' not in rows[0]:
        return None
    ini = next((float(r['t']) for r in rows if r['support'] != '0'), None)
    if ini is None:
        return None
    t_fim = ini + (float(linha['t_queda']) if linha['caiu'] == '1' else 1e9) - 0.3
    meio, fim = [], []
    for lado, c in P.passos([r for r in rows if float(r['t']) >= ini + 1.0], t_fim):
        n = len(c)
        if n < 10:
            continue
        for i, r in enumerate(c):
            e = float(r[lado + '_knee_pv']) - float(r[lado + '_knee_sp'])
            if 0.4 <= i / n < 0.6:
                meio.append(e)
            elif i / n >= 0.8:
                fim.append(e)
    if not meio:
        return None
    return st.mean(meio), st.mean(fim)


def main():
    d = sys.argv[1]
    tau = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0
    cel = {}
    for r in csv.DictReader(open(glob.glob(os.path.join(d, 'campanha_*.csv'))[0])):
        k = r['tag'].rsplit('_r', 1)[0]
        pre = os.path.join(d, 'dados', r['tag'])
        if not os.path.exists(pre + '_pe.csv'):
            continue
        t_fim = float(r['t_queda']) - 0.3 if r['caiu'] == '1' else 1e9
        tau_r = float(r['tau']) if r.get('tau') else tau
        cel.setdefault(k, []).append((r, P.analisa(pre, r), atraso_joelho(pre, r), P.juntas(pre, tau_r, t_fim)))
    print('%-34s %7s %-16s %9s %7s %7s %13s %13s' % (
        'condicao', 'quedas', 't_queda (s)', 'pico mm', 'raspou', 'ponta', 'joelho m/f', 'joelho e/sat'))
    for k in sorted(cel):
        v = cel[k]
        q = [x for x in v if x[0]['caiu'] == '1']
        A = [x[1] for x in v if x[1]]
        K = [x[2] for x in v if x[2]]
        J = [x[3]['knee'] for x in v if x[3] and 'knee' in x[3]]
        f = lambda xs: st.mean(xs) if xs else float('nan')
        print('%-34s %3d/%-3d %-16s %9.1f %6.0f%% %6.0f%% %+6.1f/%+5.1f° %5.1f°/%3.0f%%' % (
            k, len(q), len(v), ' '.join('%.1f' % float(x[0]['t_queda']) for x in q)[:16] or '-',
            f([a['pico'] for a in A]), f([a['raspou'] for a in A]), f([a['ponta'] for a in A]),
            f([a[0] for a in K]), f([a[1] for a in K]), f([j[0] for j in J]), f([j[1] for j in J])))
    print('\njoelho m/f = atraso do joelho de balanco (real - alvo) no meio / fim do apoio simples;'
          ' e/sat = erro RMS no balanco / fracao do tempo com |comando| >= tau')


if __name__ == '__main__':
    main()
