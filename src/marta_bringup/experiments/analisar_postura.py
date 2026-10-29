#!/usr/bin/env python3
"""Analise da campanha de postura (rodar_postura.sh; plano_comparacao_darwin.md).

Fatorial 2x2 z_com x z_step em tau = 1000, 6 e 4. Para cada celula:
  - quedas e tempo ate a queda (verdade de solo, walk_test.py), Fisher contra a
    linha de base (z_com 0,2975; z_step 0,06) no mesmo tau;
  - torque PEDIDO (state.command, antes da saturacao) por junta e por fase
    (perna de apoio, perna em balanco, apoio duplo): RMS e p99;
  - fracao do tempo com |comando| acima de 2,5 N.m (stall do MX-28 a 12 V);
  - erro de rastreamento RMS por fase;
  - altura real do CoM, pitch verdadeiro RMS e avanco.
Corridas com RTF < 0,85 sao descartadas, EXCETO as que cairam: nesta campanha
o walk_test.py ainda media o RTF incluindo o pos-queda (robo deitado deixa a
fisica ~2x mais lenta), entao toda queda cedo saia com RTF ~0,5. Medido em
2026-10-15: o RTF ate a queda era 1,00, e as 29 corridas sem queda deram
>= 0,999. O walk_test.py foi corrigido depois da campanha. Janela de analise: t em [5, 28] s,
cortada no instante da queda. Uso:

  python3 analisar_postura.py [dir_postura]   (default: ./postura)
"""
import bisect
import csv
import math
import os
import statistics as st
import sys

try:
    from scipy import stats
except ImportError:  # sem scipy: so estatistica descritiva
    stats = None

STALL = 2.5
T0, T1 = 5.0, 28.0
JUNTAS = ('knee', 'hip_pitch', 'ank_pitch', 'hip_roll', 'ank_roll')
BASE = ('0.2975', '0.06')


def celula(tag):
    """'zc0.2975_zs0.06_tau1000_r1' -> ('0.2975', '0.06', '1000')."""
    zc, zs, tau = tag.split('_')[:3]
    return zc[2:], zs[2:], tau[3:]


def torques(prefixo, t_fim):
    sup = [(float(r['t']), int(float(r['support'])))
           for r in csv.DictReader(open(prefixo + '_estabilizador.csv'))]
    ts = [s[0] for s in sup]
    tel = [r for r in csv.DictReader(open(prefixo + '_telemetria.csv'))
           if T0 <= float(r['t']) <= min(T1, t_fim)]
    out = {}
    for r in csv.DictReader(open(prefixo + '_juntas.csv')):
        t = float(r['t'])
        if not (T0 <= t <= min(T1, t_fim)):
            continue
        j = r['junta']
        nome = j[2:]
        if nome not in JUNTAS:
            continue
        s = sup[max(0, bisect.bisect_right(ts, t) - 1)][1]
        fase = 'duplo' if s == 0 else ('apoio' if (s == 1) == (j[0] == 'l') else 'balanco')
        out.setdefault((nome, fase), []).append((float(r['command']), float(r['error'])))
        out.setdefault((nome, 'todas'), []).append((float(r['command']), float(r['error'])))
    zc = [float(r['z_com_meas']) for r in tel if r['z_com_meas'] not in ('', 'nan')]
    return out, (st.mean(zc) if zc else float('nan'))


def resumo(v):
    c = sorted(abs(x) for x, _ in v)
    return (math.sqrt(st.mean([x * x for x, _ in v])), c[int(0.99 * (len(c) - 1))],
            sum(1 for x in c if x > STALL) / len(c),
            math.degrees(math.sqrt(st.mean([e * e for _, e in v]))))


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), 'postura')
    linhas = [r for r in csv.DictReader(open(os.path.join(d, 'campanha_postura.csv')))
              if not r['tag'].endswith('_r0')]
    cel = {}
    descartadas = 0
    for r in linhas:
        if float(r['rtf']) < 0.85 and r['caiu'] != '1':
            descartadas += 1
            continue
        k = celula(r['tag'])
        dur = float(r['t_queda']) if r['caiu'] == '1' else 1e9
        tq, zc = torques(os.path.join(d, 'dados', r['tag']), dur)
        cel.setdefault(k, []).append({'r': r, 'tq': tq, 'zc': zc})
    print('corridas validas: %d; descartadas por RTF < 0,85 (sem queda): %d\n' % (
        sum(len(v) for v in cel.values()), descartadas))

    print('== Sobrevivencia (Fisher bicaudal contra z_com 0,2975 / z_step 0,06 no mesmo tau)')
    print('%-8s %-7s %-5s %6s %-22s %8s %8s %9s %8s' % (
        'z_com', 'z_step', 'tau', 'quedas', 't_queda (s)', 'p', 'avanco', 'pitch RMS', 'z_CoM'))
    for k in sorted(cel, key=lambda k: (-float(k[2]), k[0], -float(k[1]))):
        v = cel[k]
        q = [x for x in v if x['r']['caiu'] == '1']
        tqs = sorted(float(x['r']['t_queda']) for x in q)
        p = ''
        base = cel.get((BASE[0], BASE[1], k[2]))
        if stats and base and k[:2] != BASE:
            qb = sum(1 for x in base if x['r']['caiu'] == '1')
            p = '%.3f' % stats.fisher_exact([[len(q), len(v) - len(q)], [qb, len(base) - qb]])[1]
        sobre = [x for x in v if x['r']['caiu'] != '1']
        av = st.mean([float(x['r']['avanco']) for x in sobre]) if sobre else float('nan')
        pr = st.mean([float(x['r']['rms_pitch_v']) for x in sobre]) if sobre else float('nan')
        zc = st.mean([x['zc'] for x in v if not math.isnan(x['zc'])] or [float('nan')])
        print('%-8s %-7s %-5s %3d/%-2d %-22s %8s %8.2f %9.2f %8.4f' % (
            k[0], k[1], k[2], len(q), len(v), ' '.join('%.1f' % t for t in tqs)[:22] or '-', p, av, pr, zc))

    print('\n== Torque PEDIDO (tau = 1000): media entre corridas de RMS / p99 [N.m],'
          ' fracao > %.1f N.m, erro RMS [graus]' % STALL)
    ks = [k for k in cel if k[2] == '1000']
    ks.sort(key=lambda k: (k[0], -float(k[1])))
    for nome in JUNTAS[:3]:
        for fase in ('apoio', 'balanco', 'duplo', 'todas'):
            cols = []
            for k in ks:
                res = [resumo(x['tq'][(nome, fase)]) for x in cel[k] if (nome, fase) in x['tq']]
                if not res:
                    cols.append('-')
                    continue
                m = [st.mean(c) for c in zip(*res)]
                cols.append('%5.2f/%5.2f %3.0f%% %4.1f°' % (m[0], m[1], 100 * m[2], m[3]))
            print('%-9s %-7s %s' % (nome, fase, ' | '.join(cols)))
    # n efetivo: so contam corridas com dado na janela (quem caiu antes de T0 fica fora)
    print('colunas: ' + ' | '.join('z_com %s z_step %s (n=%d de %d)' % (
        k[0], k[1], sum(1 for x in cel[k] if ('knee', 'todas') in x['tq']), len(cel[k])) for k in ks))

    if stats:
        print('\n== Efeitos sobre o RMS do joelho, todas as fases (tau = 1000; Welch)')
        def amostra(k, fase='todas'):
            return [resumo(x['tq'][('knee', fase)])[0] for x in cel.get(k, []) if ('knee', fase) in x['tq']]
        for fase in ('apoio', 'balanco', 'todas'):
            for a, b, rot in (((BASE[0], '0.06', '1000'), ('0.345', '0.06', '1000'), 'postura (z_step 0,06)'),
                              ((BASE[0], '0.04', '1000'), ('0.345', '0.04', '1000'), 'postura (z_step 0,04)'),
                              ((BASE[0], '0.06', '1000'), (BASE[0], '0.04', '1000'), 'altura do pe (z_com 0,2975)'),
                              (('0.345', '0.06', '1000'), ('0.345', '0.04', '1000'), 'altura do pe (z_com 0,345)')):
                xa, xb = amostra(a, fase), amostra(b, fase)
                if len(xa) > 1 and len(xb) > 1:
                    t = stats.ttest_ind(xa, xb, equal_var=False)
                    print('  %-7s %-28s %5.2f -> %5.2f N.m (%+5.0f%%), p = %.2g' % (
                        fase, rot, st.mean(xa), st.mean(xb), 100 * (st.mean(xb) / st.mean(xa) - 1), t.pvalue))


if __name__ == '__main__':
    main()
