#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Gera as figuras da campanha a partir do CSV. Roda no HOST (precisa de
matplotlib/numpy/scipy), nao dentro do container.

    python3 src/marta_bringup/experiments/plot_campanha.py \
            src/marta_bringup/experiments/campanha_preliminar.csv \
            docs/figuras

Cada figura sai em PNG (200 dpi, para slide) e PDF (vetorial, para relatorio).
"""
import csv, math, os, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SEM, COM = '#B45309', '#2E6F8E'          # ambar = sem, azul = com
plt.rcParams.update({
    'font.size': 11, 'axes.grid': True, 'grid.alpha': 0.25,
    'axes.spines.top': False, 'axes.spines.right': False,
    'figure.dpi': 110, 'axes.axisbelow': True,
})


def carregar(caminho):
    linhas = []
    with open(caminho) as f:
        for row in csv.DictReader(l for l in f if not l.startswith('#')):
            def num(k):
                v = row[k].strip()
                return float(v) if v else float('nan')
            linhas.append(dict(tau=num('tau'), stab=row['stab'].strip(),
                               caiu=num('caiu'), rms=num('rms_verdade'),
                               rms_est=num('rms_est'), roll=num('roll_verdade'),
                               dist=num('dist')))
    return linhas


def agrupar(dados, stab, campo):
    """{tau: [valores]} para uma condicao, ignorando NaN."""
    out = {}
    for d in dados:
        if d['stab'] == stab and not math.isnan(d[campo]):
            out.setdefault(d['tau'], []).append(d[campo])
    return out


def salvar(fig, saida, nome):
    for ext in ('png', 'pdf'):
        fig.savefig(os.path.join(saida, '%s.%s' % (nome, ext)),
                    bbox_inches='tight', dpi=200)
    plt.close(fig)
    print('  %s.png / .pdf' % nome)


# --- Figura 1: curva dose-resposta -----------------------------------------
def fig_dose_resposta(dados, saida):
    fig, ax = plt.subplots(figsize=(7.2, 4.4))
    for stab, cor, rot in ((('off'), SEM, 'sem estabilizador'),
                           (('on'), COM, 'com estabilizador')):
        g = agrupar(dados, stab, 'rms')
        taus = sorted(g, reverse=True)
        m = [np.mean(g[t]) for t in taus]
        s = [np.std(g[t], ddof=1) if len(g[t]) > 1 else 0.0 for t in taus]
        ax.errorbar(taus, m, yerr=s, marker='o', ms=7, lw=2.2, capsize=5,
                    color=cor, label=rot)
        for t, mm, n in zip(taus, m, [len(g[t]) for t in taus]):
            ax.annotate('n=%d' % n, (t, mm), textcoords='offset points',
                        xytext=(0, 11), ha='center', fontsize=8, color=cor)
    ax.set_xscale('log')
    ax.invert_xaxis()
    ax.set_xticks([1000, 6, 4])
    ax.set_xticklabels(['1000\n(original)', '6\n(realista)', '4'])
    ax.set_xlabel('limite de torque das juntas [N·m]  —  dificuldade crescente →')
    ax.set_ylabel('RMS do pitch REAL [graus]')
    ax.set_title('Efeito do estabilizador em função da dificuldade da simulação',
                 loc='left', fontweight='bold')
    ax.axvspan(5, 3.5, color='0.85', zorder=0)
    ax.annotate('nenhuma\ncondição sobrevive', (4, 26), ha='center',
                fontsize=8.5, color='0.35')
    ax.legend(frameon=False, loc='upper left')
    salvar(fig, saida, 'fig1_dose_resposta')


# --- Figura 2: taxa de queda ------------------------------------------------
def fig_quedas(dados, saida):
    fig, ax = plt.subplots(figsize=(6.4, 4.0))
    taus = sorted({d['tau'] for d in dados}, reverse=True)
    larg = 0.36
    x = np.arange(len(taus))
    for i, (stab, cor, rot) in enumerate((('off', SEM, 'sem estabilizador'),
                                          ('on', COM, 'com estabilizador'))):
        fr, ns = [], []
        for t in taus:
            sub = [d['caiu'] for d in dados if d['tau'] == t and d['stab'] == stab]
            fr.append(100.0 * sum(sub) / len(sub) if sub else 0.0)
            ns.append(len(sub))
        b = ax.bar(x + (i - 0.5) * larg, fr, larg, color=cor, label=rot)
        for r, n, f in zip(b, ns, fr):
            ax.annotate('%d/%d' % (round(f * n / 100), n),
                        (r.get_x() + r.get_width() / 2, f + 2),
                        ha='center', fontsize=9)
    ax.set_xticks(x)
    ax.set_xticklabels(['%g N·m' % t for t in taus])
    ax.set_ylim(0, 118)
    ax.set_ylabel('corridas que caíram [%]')
    ax.set_xlabel('limite de torque  —  dificuldade crescente →')
    ax.set_title('Taxa de queda', loc='left', fontweight='bold')
    ax.legend(frameon=False, loc='upper left')
    salvar(fig, saida, 'fig2_taxa_queda')


# --- Figura 4: distribuicao no ponto informativo ----------------------------
def fig_distribuicao(dados, saida, tau=6.0):
    from scipy import stats
    fig, ax = plt.subplots(figsize=(5.6, 4.4))
    grupos, cores, rots = [], [SEM, COM], ['sem', 'com']
    for stab in ('off', 'on'):
        grupos.append([d['rms'] for d in dados
                       if d['tau'] == tau and d['stab'] == stab
                       and not math.isnan(d['rms'])])
    bp = ax.boxplot(grupos, widths=0.5, patch_artist=True, showfliers=False,
                    medianprops=dict(color='black', lw=1.6))
    for caixa, cor in zip(bp['boxes'], cores):
        caixa.set(facecolor=cor, alpha=0.22, edgecolor=cor, lw=1.6)
    for i, (g, cor) in enumerate(zip(grupos, cores), start=1):
        ax.scatter(np.random.normal(i, 0.045, len(g)), g, color=cor,
                   zorder=3, s=42, edgecolor='white', lw=1.0)
    t, p = stats.ttest_ind(grupos[0], grupos[1], equal_var=False)
    red = 100 * (np.mean(grupos[0]) - np.mean(grupos[1])) / np.mean(grupos[0])
    ax.set_xticklabels(['%s\n(n=%d)' % (r, len(g)) for r, g in zip(rots, grupos)])
    ax.set_ylabel('RMS do pitch REAL [graus]')
    ax.set_title('Distribuição a %g N·m' % tau, loc='left', fontweight='bold')
    ax.annotate('−%.0f%%\nt=%.2f, p=%.3f' % (red, t, p),
                xy=(0.97, 0.94), xycoords='axes fraction', ha='right', va='top',
                fontsize=10, bbox=dict(boxstyle='round,pad=0.45',
                                       fc='white', ec='0.8'))
    salvar(fig, saida, 'fig4_distribuicao_6Nm')


# --- Figura 5: a armadilha da metrica ---------------------------------------
def fig_metrica(dados, saida):
    fig, ax = plt.subplots(figsize=(6.8, 4.2))
    taus = [1000.0, 6.0]
    x = np.arange(len(taus))
    larg = 0.36
    for i, (campo, cor, rot) in enumerate(
            (('rms_est', '0.62', 'medido pela ESTIMATIVA\n(o sinal que a malha regula)'),
             ('rms', '#1B7F5C', 'medido pela VERDADE de solo'))):
        red = []
        for t in taus:
            o = [d[campo] for d in dados if d['tau'] == t and d['stab'] == 'off'
                 and not math.isnan(d[campo])]
            n = [d[campo] for d in dados if d['tau'] == t and d['stab'] == 'on'
                 and not math.isnan(d[campo])]
            red.append(100 * (np.mean(o) - np.mean(n)) / np.mean(o))
        b = ax.bar(x + (i - 0.5) * larg, red, larg, color=cor, label=rot)
        for r, v in zip(b, red):
            ax.annotate('%.0f%%' % v, (r.get_x() + r.get_width() / 2, v + 1),
                        ha='center', fontsize=10, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(['1000 N·m (original)', '6 N·m (realista)'])
    ax.set_ylabel('redução aparente do RMS de pitch [%]')
    ax.set_title('Por que a métrica precisa ser independente do controlador',
                 loc='left', fontweight='bold')
    ax.legend(frameon=False, fontsize=9, loc='upper left')
    ax.set_ylim(0, 46)
    salvar(fig, saida, 'fig5_estimativa_vs_verdade')


if __name__ == '__main__':
    csv_path = sys.argv[1] if len(sys.argv) > 1 else \
        'src/marta_bringup/experiments/campanha_preliminar.csv'
    saida = sys.argv[2] if len(sys.argv) > 2 else 'docs/figuras'
    os.makedirs(saida, exist_ok=True)
    d = carregar(csv_path)
    print('%d corridas carregadas. Gerando em %s/' % (len(d), saida))
    fig_dose_resposta(d, saida)
    fig_quedas(d, saida)
    fig_distribuicao(d, saida)
    fig_metrica(d, saida)
