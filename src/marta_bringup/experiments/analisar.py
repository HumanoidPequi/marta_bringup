#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Analise da campanha: tabelas + figuras. Roda no HOST.

    python3 src/marta_bringup/experiments/analisar.py \
            src/marta_bringup/experiments/campanha.csv docs/figuras
"""
import csv, math, os, sys
from collections import defaultdict
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy import stats

SEM, COM = '#B45309', '#2E6F8E'
plt.rcParams.update({'font.size': 11, 'axes.grid': True, 'grid.alpha': 0.25,
                     'axes.spines.top': False, 'axes.spines.right': False,
                     'axes.axisbelow': True})
DUR = 20.0   # janela nominal de marcha [s]


def carregar(p):
    out = []
    with open(p) as f:
        for r in csv.DictReader(l for l in f if not l.startswith('#')):
            if int(r.get('divergiu', 0) or 0):
                continue                      # divergencia numerica
            # RTF baixo = corrida limitada por CPU, nao por fisica. Numa
            # campanha, as unicas quedas observadas foram exatamente as
            # corridas com RTF < 0.7; sem este filtro elas contaminam o
            # resultado como se fossem efeito real.
            if r.get('rtf') and float(r['rtf']) < 0.85:
                continue
            def num(k, d=float('nan')):
                v = (r.get(k) or '').strip()
                return float(v) if v else d
            out.append(dict(tau=num('tau'), stab=int(r['stab']),
                            caiu=int(r['caiu']), motivo=r['motivo'],
                            t_queda=num('t_queda'), dur=num('dur'),
                            rms=num('rms_pitch_v'), pico=num('pico_pitch_v'),
                            media=num('media_pitch_v'), rms_roll=num('rms_roll_v'),
                            rms_est=num('rms_pitch_est'), vies=num('vies'),
                            dist=num('dist'), delta=num('delta_pico'),
                            sat=num('frac_sat'), rtf=num('rtf')))
    return out


def por_celula(d, campo, so_sobreviventes=False):
    g = defaultdict(list)
    for x in d:
        if so_sobreviventes and x['caiu']:
            continue
        if not math.isnan(x[campo]):
            g[(x['tau'], x['stab'])].append(x[campo])
    return g


def salvar(fig, saida, nome):
    for ext in ('png', 'pdf'):
        fig.savefig(os.path.join(saida, nome + '.' + ext),
                    bbox_inches='tight', dpi=200)
    plt.close(fig)
    print('  ' + nome)


# --- Fig 1: dose-resposta ---------------------------------------------------
def fig1(d, saida):
    """
    So os niveis em que AS DUAS condicoes completaram todas as corridas.
    Incluir niveis com queda produziria vies de sobrevivente: a media seria
    calculada so sobre quem sobreviveu, que nao e uma amostra comparavel.
    Os niveis com queda aparecem nas figuras 2 e 3, que e onde eles informam.
    """
    completos = []
    for t in sorted({x['tau'] for x in d}, reverse=True):
        ok = all(not y['caiu'] for y in d if y['tau'] == t)
        if ok:
            completos.append(t)
    g = por_celula(d, 'rms')
    fig, ax = plt.subplots(figsize=(7.4, 4.6))
    x = np.arange(len(completos))
    for stab, cor, rot in ((0, SEM, 'sem estabilizador'), (1, COM, 'com estabilizador')):
        m = [np.mean(g[(t, stab)]) for t in completos]
        e = [np.std(g[(t, stab)], ddof=1) / math.sqrt(len(g[(t, stab)])) for t in completos]
        ax.errorbar(x, m, yerr=e, marker='o', ms=8, lw=2.4, capsize=5,
                    color=cor, label=rot)
    for i, t in enumerate(completos):
        ax.annotate('n=%d' % len(g[(t, 0)]), (i, 1.72), ha='center',
                    fontsize=8, color='0.4')
    # significancia por nivel
    for i, t in enumerate(completos):
        a, b = g[(t, 0)], g[(t, 1)]
        _, pv = stats.ttest_ind(a, b, equal_var=False)
        alto = max(np.mean(a), np.mean(b))
        ax.annotate('p=%.3f' % pv if pv >= 1e-3 else 'p<0.001',
                    (i, alto + 0.42), ha='center', fontsize=9,
                    fontweight='bold' if pv < 0.05 else 'normal',
                    color='#1B7F5C' if pv < 0.05 else '0.55')
    ax.set_xticks(x)
    ax.set_xticklabels(['%g N·m' % t for t in completos])
    ax.set_xlim(-0.4, len(completos) - 0.6)
    ax.set_ylim(1.6, 6.6)
    ax.axvspan(len(completos) - 1.4, len(completos) - 0.6, color='#FDF3E7', zorder=0)
    ax.annotate('regime realista\npara servo KidSize',
                (len(completos) - 1, 6.25), ha='center', fontsize=9, color='#B45309')
    ax.set_xlabel('limite de torque das juntas   —   dificuldade crescente →')
    ax.set_ylabel('RMS do pitch real [graus]')
    ax.set_title('Oscilação do tronco em função da dificuldade da simulação\n'
                 'níveis em que todas as corridas completaram os 20 s',
                 loc='left', fontweight='bold', fontsize=12)
    ax.legend(frameon=False, loc='upper left')
    salvar(fig, saida, 'fig1_dose_resposta')


# --- Fig 2: taxa de queda ---------------------------------------------------
def fig2(d, saida):
    taus = sorted({x['tau'] for x in d}, reverse=True)
    fig, ax = plt.subplots(figsize=(7.0, 4.2))
    x = np.arange(len(taus)); w = 0.36
    for i, (stab, cor, rot) in enumerate(((0, SEM, 'sem estabilizador'),
                                          (1, COM, 'com estabilizador'))):
        fr, ns, qs = [], [], []
        for t in taus:
            sub = [y['caiu'] for y in d if y['tau'] == t and y['stab'] == stab]
            ns.append(len(sub)); qs.append(sum(sub))
            fr.append(100.0 * sum(sub) / len(sub) if sub else 0)
        b = ax.bar(x + (i - .5) * w, fr, w, color=cor, label=rot)
        for r, q, n in zip(b, qs, ns):
            ax.annotate('%d/%d' % (q, n), (r.get_x() + r.get_width() / 2,
                        r.get_height() + 2), ha='center', fontsize=9)
    ax.set_xticks(x); ax.set_xticklabels(['%g N·m' % t for t in taus])
    ax.set_ylim(0, 118); ax.set_ylabel('corridas que caíram [%]')
    ax.set_xlabel('limite de torque   —   dificuldade crescente →')
    ax.set_title('Taxa de queda', loc='left', fontweight='bold')
    ax.legend(frameon=False, loc='upper left')
    salvar(fig, saida, 'fig2_taxa_queda')


# --- Fig 3: sobrevivencia (Kaplan-Meier) ------------------------------------
def fig3(d, saida, tau=5.0):
    fig, ax = plt.subplots(figsize=(6.6, 4.3))
    for stab, cor, rot in ((0, SEM, 'sem estabilizador'), (1, COM, 'com estabilizador')):
        sub = [y for y in d if y['tau'] == tau and y['stab'] == stab]
        if not sub:
            continue
        # tempo observado: instante da queda, ou fim da janela (censurado)
        obs = sorted((y['t_queda'] if y['caiu'] else y['dur'], y['caiu']) for y in sub)
        n = len(obs); s = 1.0
        ts, ss = [0.0], [1.0]
        em_risco = n
        for t, ev in obs:
            if ev:
                s *= (em_risco - 1) / em_risco
                ts.append(t); ss.append(s)
            em_risco -= 1
        ts.append(DUR); ss.append(s)
        ax.step(ts, [v * 100 for v in ss], where='post', lw=2.4, color=cor,
                label='%s (n=%d)' % (rot, n))
        cens = [y['dur'] for y in sub if not y['caiu']]
        if cens:
            ax.scatter(cens, [s * 100] * len(cens), marker='|', s=140,
                       color=cor, zorder=3)
    ax.set_xlim(0, DUR + 0.4); ax.set_ylim(-4, 104)
    ax.set_xlabel('tempo de marcha [s]')
    ax.set_ylabel('robôs ainda em pé [%]')
    ax.set_title('Curva de sobrevivência a %g N·m\n'
                 '(traços verticais = corridas que terminaram sem cair)' % tau,
                 loc='left', fontweight='bold', fontsize=12)
    ax.legend(frameon=False, loc='lower left')
    salvar(fig, saida, 'fig3_sobrevivencia_5Nm')


# --- Fig 4: distribuicao no ponto informativo -------------------------------
def fig4(d, saida, tau=6.0):
    grupos = []
    for stab in (0, 1):
        grupos.append([y['rms'] for y in d if y['tau'] == tau
                       and y['stab'] == stab and not y['caiu']])
    if not all(grupos):
        return
    fig, ax = plt.subplots(figsize=(5.4, 4.4))
    bp = ax.boxplot(grupos, widths=.5, patch_artist=True, showfliers=False,
                    medianprops=dict(color='black', lw=1.6))
    for c, cor in zip(bp['boxes'], (SEM, COM)):
        c.set(facecolor=cor, alpha=.22, edgecolor=cor, lw=1.6)
    for i, (g, cor) in enumerate(zip(grupos, (SEM, COM)), 1):
        ax.scatter(np.random.normal(i, .045, len(g)), g, color=cor, s=44,
                   zorder=3, edgecolor='white', lw=1)
    t, p = stats.ttest_ind(*grupos, equal_var=False)
    red = 100 * (np.mean(grupos[0]) - np.mean(grupos[1])) / np.mean(grupos[0])
    ax.set_xticklabels(['sem\n(n=%d)' % len(grupos[0]), 'com\n(n=%d)' % len(grupos[1])])
    ax.set_ylabel('RMS do pitch real [graus]')
    ax.set_title('Distribuição a %g N·m' % tau, loc='left', fontweight='bold')
    ax.annotate('−%.0f%%\nt=%.2f   p=%.4f' % (red, t, p), xy=(.96, .95),
                xycoords='axes fraction', ha='right', va='top', fontsize=10,
                bbox=dict(boxstyle='round,pad=.45', fc='white', ec='.8'))
    salvar(fig, saida, 'fig4_distribuicao_6Nm')


# --- Fig 5: estimativa x verdade --------------------------------------------
def fig5(d, saida):
    taus = sorted({x['tau'] for x in d}, reverse=True)
    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    v = [np.mean([y['vies'] for y in d if y['tau'] == t and not math.isnan(y['vies'])])
         for t in taus]
    ax.bar(range(len(taus)), v, .55, color='#1B7F5C')
    for i, val in enumerate(v):
        ax.annotate('%+.1f°' % val, (i, val + .4), ha='center', fontweight='bold')
    ax.set_xticks(range(len(taus)))
    ax.set_xticklabels(['%g N·m' % t for t in taus])
    ax.set_ylabel('viés do estimador [graus]\n(verdade − estimativa)')
    ax.set_xlabel('limite de torque   —   dificuldade crescente →')
    ax.set_title('O estimador se degrada com a dificuldade\n'
                 'por isso a métrica usa verdade de solo',
                 loc='left', fontweight='bold', fontsize=12)
    salvar(fig, saida, 'fig5_vies_estimador')


# --- Fig 6: serie temporal ---------------------------------------------------
def fig6(saida, dirserie, tau=6):
    import glob
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(7.6, 5.4), sharex=True,
                                 gridspec_kw=dict(height_ratios=[2, 1]))
    for stab, cor, rot in (('off', SEM, 'sem estabilizador'),
                           ('on', COM, 'com estabilizador')):
        fs = sorted(glob.glob(os.path.join(dirserie, 'tau%d_%s_r*.csv' % (tau, stab))))
        if not fs:
            continue
        t, p, dl = [], [], []
        with open(fs[0]) as f:
            for r in csv.DictReader(f):
                t.append(float(r['t'])); p.append(float(r['pitch_verdade']))
                dl.append(float(r['delta_quadril']))
        a1.plot(t, p, lw=1.5, color=cor, label=rot)
        a2.plot(t, dl, lw=1.3, color=cor)
    a1.axhline(0, color='0.6', lw=.8)
    a1.set_ylabel('pitch real [graus]')
    a1.set_title('Uma corrida de cada, a %d N·m' % tau, loc='left', fontweight='bold')
    a1.legend(frameon=False, ncol=2, fontsize=9)
    a2.axhline(0, color='0.6', lw=.8)
    a2.set_ylabel('Δθ no quadril\n[graus]'); a2.set_xlabel('tempo [s]')
    salvar(fig, saida, 'fig6_serie_temporal')


def fig7(d, saida, tau=6.0):
    """
    Decompoe RMS^2 = media^2 + variancia. Distingue duas afirmacoes diferentes:
    "o tronco fica mais perto da vertical em media" (postura) e "o tronco
    balanca menos a cada passo" (oscilacao). Nao sao a mesma coisa.
    """
    fig, ax = plt.subplots(figsize=(6.4, 4.4))
    rots, post, osc = [], [], []
    for stab, rot in ((0, 'sem\nestabilizador'), (1, 'com\nestabilizador')):
        sub = [y for y in d if y['tau'] == tau and y['stab'] == stab and not y['caiu']]
        rots.append(rot)
        post.append(np.mean([abs(y['media']) for y in sub]))
        osc.append(np.mean([math.sqrt(max(y['rms'] ** 2 - y['media'] ** 2, 0))
                            for y in sub]))
    x = np.arange(2); w = 0.34
    b1 = ax.bar(x - w / 2, post, w, color='#B45309', label='postura  |média do pitch|')
    b2 = ax.bar(x + w / 2, osc, w, color='#2E6F8E', label='oscilação  desvio por passo')
    for bs in (b1, b2):
        for r in bs:
            ax.annotate('%.2f°' % r.get_height(),
                        (r.get_x() + r.get_width() / 2, r.get_height() + .08),
                        ha='center', fontsize=10, fontweight='bold')
    ax.set_xticks(x); ax.set_xticklabels(rots)
    ax.set_ylabel('graus')
    ax.set_ylim(0, max(post + osc) * 1.28)
    ax.set_title('O que exatamente melhorou, a %g N·m\n'
                 'a correção age sobretudo na POSTURA' % tau,
                 loc='left', fontweight='bold', fontsize=12)
    ax.legend(frameon=False, fontsize=9.5)
    salvar(fig, saida, 'fig7_postura_vs_oscilacao')


def tabela(d):
    print('\n%-8s %-5s %3s  %7s  %14s  %12s  %10s  %7s' %
          ('tau', 'stab', 'n', 'quedas', 'RMS pitch real', 'RMS roll', 'dist [m]', 'viés'))
    for t in sorted({x['tau'] for x in d}, reverse=True):
        for stab in (0, 1):
            sub = [y for y in d if y['tau'] == t and y['stab'] == stab]
            if not sub:
                continue
            viv = [y for y in sub if not y['caiu']]
            r = [y['rms'] for y in viv]
            print('%-8g %-5s %3d  %3d/%-3d  %6.2f ± %-5.2f  %6.2f       %8.3f  %+6.1f' % (
                t, 'com' if stab else 'sem', len(sub),
                sum(y['caiu'] for y in sub), len(sub),
                np.mean(r) if r else float('nan'),
                np.std(r, ddof=1) if len(r) > 1 else 0,
                np.mean([y['rms_roll'] for y in viv]) if viv else float('nan'),
                np.mean([y['dist'] for y in sub]),
                np.mean([y['vies'] for y in sub])))


if __name__ == '__main__':
    p = sys.argv[1] if len(sys.argv) > 1 else 'src/marta_bringup/experiments/campanha.csv'
    saida = sys.argv[2] if len(sys.argv) > 2 else 'docs/figuras'
    os.makedirs(saida, exist_ok=True)
    d = carregar(p)
    print('%d corridas validas' % len(d))
    tabela(d)
    print('\nfiguras:')
    fig1(d, saida); fig2(d, saida); fig3(d, saida)
    fig4(d, saida); fig5(d, saida); fig7(d, saida)
    fig6(saida, os.path.join(os.path.dirname(p), 'series'))
