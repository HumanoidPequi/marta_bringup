#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Analisa o experimento do comSwingFactor: o tZMP do gerador esta descasado da
altura real do CoM, e corrigi-lo melhora a marcha?

    python3 analisar_csf.py [campanha_csf.csv] [dir_figuras]

csf = 1.000 -> tZMP = 0.1742 s  (o gerador assume zc = 0.2975 m, torso-pe da IK)
csf = 0.734 -> tZMP = 0.1492 s  (casa com o CoM real medido, 0.2184 m)
"""
import csv, math, os, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy import stats

A, B = '#B45309', '#2E6F8E'
plt.rcParams.update({'font.size': 10.5, 'axes.grid': True, 'grid.alpha': 0.25,
                     'axes.spines.top': False, 'axes.spines.right': False,
                     'axes.axisbelow': True})


RTF_MIN = 0.85   # abaixo disso a corrida foi limitada por CPU, nao por fisica


def carregar(p):
    """
    Descarta corridas invalidas por dois criterios PRE-DECLARADOS:
    divergencia numerica e RTF abaixo de RTF_MIN.

    O filtro de RTF nao e cosmetico. Numa campanha, as 4 corridas com RTF entre
    0.52 e 0.66 foram as UNICAS 4 que cairam, enquanto as 12 com RTF > 0.85 nao
    tiveram queda nenhuma. Um teste controlado depois mostrou que a telemetria
    nao causa essa degradacao (RTF 0.995 com ela contra 0.998 sem): era
    contencao de CPU na maquina. Sem o filtro, essas quedas entrariam no
    resultado como se fossem fisica.
    """
    d = []
    descartadas = 0
    with open(p) as f:
        for r in csv.DictReader(f):
            if int(r.get('divergiu', 0) or 0):
                descartadas += 1
                continue
            if r.get('rtf') and float(r['rtf']) < RTF_MIN:
                descartadas += 1
                continue
            g = lambda k: float(r[k]) if r.get(k) else float('nan')
            d.append(dict(tau=g('tau'), csf=g('csf'), caiu=int(r['caiu']),
                          rms=g('rms_pitch_v'), der=g('deriva_energia'),
                          dist=g('dist'), nf=g('n_fases')))
    if descartadas:
        print('%d corrida(s) descartada(s): divergencia ou RTF < %.2f' %
              (descartadas, RTF_MIN))
    return d


def tabela(d):
    print('%-7s %-7s %3s %7s  %-20s %-20s %-10s' %
          ('tau', 'csf', 'n', 'quedas', 'deriva E_x [m2/s2]',
           'pitch RMS real [deg]', 'dist [m]'))
    for tau in sorted({x['tau'] for x in d}, reverse=True):
        for csf in sorted({x['csf'] for x in d}, reverse=True):
            s = [x for x in d if x['tau'] == tau and x['csf'] == csf]
            v = [x for x in s if not x['caiu']]
            if not s:
                continue
            f = lambda k: ('%.4f ± %.4f' % (np.mean([x[k] for x in v]),
                           np.std([x[k] for x in v], ddof=1) if len(v) > 1 else 0)
                           ) if v else 'n/d'
            print('%-7g %-7g %3d %4d/%-3d  %-20s %-20s %-10.3f' %
                  (tau, csf, len(s), sum(x['caiu'] for x in s), len(s),
                   f('der'), f('rms'),
                   np.mean([x['dist'] for x in s])))


def comparar(d, tau, campo):
    a = [x[campo] for x in d if x['tau'] == tau and x['csf'] > 0.9 and not x['caiu']]
    b = [x[campo] for x in d if x['tau'] == tau and x['csf'] < 0.9 and not x['caiu']]
    if len(a) < 2 or len(b) < 2:
        return None
    t, p = stats.ttest_ind(a, b, equal_var=False)
    return np.mean(a), np.mean(b), t, p, len(a), len(b)


def figura(d, saida):
    taus = sorted({x['tau'] for x in d}, reverse=True)
    fig, axes = plt.subplots(1, 2, figsize=(9.8, 4.4))
    for ax, campo, rot in ((axes[0], 'der', 'deriva de $E_x$ por fase  [m²/s²]'),
                           (axes[1], 'rms', 'RMS do pitch real  [graus]')):
        x = np.arange(len(taus)); w = 0.35
        for i, (csf, cor, lab) in enumerate(((1.0, A, 'csf = 1,000  ($T_c$=0,174 s)'),
                                             (0.734, B, 'csf = 0,734  ($T_c$=0,149 s)'))):
            m, e, n = [], [], []
            for tau in taus:
                v = [y[campo] for y in d if y['tau'] == tau
                     and abs(y['csf'] - csf) < 0.01 and not y['caiu']]
                m.append(np.mean(v) if v else np.nan)
                e.append(np.std(v, ddof=1) / math.sqrt(len(v)) if len(v) > 1 else 0)
                n.append(len(v))
            bars = ax.bar(x + (i - .5) * w, m, w, yerr=e, capsize=4,
                          color=cor, label=lab)
            for r, k in zip(bars, n):
                if not math.isnan(r.get_height()):
                    ax.annotate('n=%d' % k, (r.get_x() + r.get_width() / 2,
                                r.get_height()), textcoords='offset points',
                                xytext=(0, 3), ha='center', fontsize=8)
        ax.set_xticks(x)
        ax.set_xticklabels(['%g N·m' % t for t in taus])
        ax.set_ylabel(rot)
        ax.set_xlabel('limite de torque')
    axes[0].set_title('Aderência ao modelo', loc='left', fontweight='bold')
    axes[1].set_title('Qualidade da marcha', loc='left', fontweight='bold')
    axes[0].legend(frameon=False, fontsize=9)
    fig.suptitle('Casar o $T_c$ do gerador com a altura real do CoM melhora a marcha?',
                 x=0.09, ha='left', fontweight='bold', fontsize=12.5)
    for ext in ('png', 'pdf'):
        fig.savefig(os.path.join(saida, 'fig_csf_experimento.' + ext),
                    bbox_inches='tight', dpi=200)
    plt.close(fig)
    print('  fig_csf_experimento')


if __name__ == '__main__':
    p = sys.argv[1] if len(sys.argv) > 1 else 'src/marta_bringup/experiments/campanha_csf.csv'
    saida = sys.argv[2] if len(sys.argv) > 2 else 'docs/figuras_marcha'
    os.makedirs(saida, exist_ok=True)
    d = carregar(p)
    print('%d corridas\n' % len(d))
    tabela(d)
    print()
    for tau in sorted({x['tau'] for x in d}, reverse=True):
        for campo, rot in (('der', 'deriva E_x'), ('rms', 'pitch RMS')):
            r = comparar(d, tau, campo)
            if r:
                ma, mb, t, pv, na, nb = r
                sinal = 'MELHOROU' if mb < ma else 'piorou'
                print('tau=%-6g %-11s csf1.0 %.4f (n=%d)  csf0.734 %.4f (n=%d)  '
                      '%s %.0f%%  t=%.2f p=%.3f'
                      % (tau, rot, ma, na, mb, nb, sinal,
                         abs(100 * (mb - ma) / ma), t, pv))
    print()
    figura(d, saida)
