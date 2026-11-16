#!/usr/bin/env python3
"""Figuras do estudo do caranguejo e dos bracos (docs/marcha_lateral.md, docs/bracos.md).

Gera em docs/figuras_marcha/:
  fig_lateral_avanco.png    A1/A1x: vx_b (avanco espurio) por braco da ablacao
  fig_lateral_envelope.png  A2: corridas completas por |vy| e massa;
                            A3: grade y_separation x step_period a |vy| 0,075
  fig_lateral_fase.png      pitch verdadeiro no tempo, M33 lat +0,075, default
                            (0,06; 0,6) contra (0,05; 0,5), fundo = fase de apoio
  fig_bracos_b1.png         B1: roll RMS (P - P0) por par; hc_rms na reta por s
  fig_bracos_b2.png         B2/B2x: completas sem roll por braco e pitch medio
  fig_bracos_b3.png         B3: limiar de empurrao por postura (se existir)

  python3 fig_caranguejo_bracos.py
"""
import csv
import os
import re

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from analisar_e5 import f  # noqa: E402
from analisar_e7 import med, ok  # noqa: E402

AQUI = os.path.dirname(os.path.abspath(__file__))
SAIDA = os.environ.get('FIG_SAIDA', os.path.join(AQUI, '..', '..', '..', 'docs', 'figuras_marcha'))
AZUL = ['#cde2fb', '#b7d3f6', '#9ec5f4', '#86b6ef', '#6da7ec', '#5598e7', '#3987e5',
        '#2a78d6', '#256abf', '#1c5cab', '#184f95', '#104281', '#0d366b']
CAT = {'M33': '#2a78d6', 'M38': '#eb6834'}
VERDE = '#1baf7a'
SUP, TXT, TXT2, MUDO, GRADE = '#fcfcfb', '#0b0b0b', '#52514e', '#898781', '#e1e0d9'
plt.rcParams.update({
    'figure.facecolor': SUP, 'axes.facecolor': SUP, 'savefig.facecolor': SUP,
    'font.size': 9, 'axes.edgecolor': MUDO, 'axes.labelcolor': TXT2,
    'xtick.color': TXT2, 'ytick.color': TXT2, 'axes.titlecolor': TXT,
    'axes.spines.top': False, 'axes.spines.right': False,
})


def resumo(d):
    return {r['tag']: r for r in csv.DictReader(open(os.path.join(AQUI, d, 'resumo_omni.csv')))}


def salvar(fig, nome):
    fig.savefig(os.path.join(SAIDA, nome), dpi=160, bbox_inches='tight')
    plt.close(fig)
    print('  ', nome)


def grade(ax):
    ax.grid(axis='y', color=GRADE, lw=0.6)
    ax.set_axisbelow(True)


def fig_avanco(L):
    bracos = [('ref', 'A1', 'ref'), ('s0', 'A1', 'sem balanço'), ('rumo0', 'A1', 'sem rumo'),
              ('pitch0', 'A1', 'sem estab. pitch'), ('hx0', 'A1', 'hip_offset_x 0'),
              ('hpo0', 'A1', 'hip_pitch_offset 0'), ('gc0', 'A1', 'sem comp. gravidade'),
              ('ff', 'A1x', 'ff vx (A1x)')]
    movs = (('lugar', 'o', 'no lugar'), ('lat+0.05', '^', 'lat +0,05'), ('lat-0.05', 'v', 'lat −0,05'))
    fig, ax = plt.subplots(figsize=(7.2, 3.4))
    for i, (b, camp, _) in enumerate(bracos):
        for j, M in enumerate(('M33', 'M38')):
            for k, (mv, mk, _) in enumerate(movs):
                r = L.get('%s_%s_%s_%s_pD_r1' % (camp, M, b, mv))
                if r is None:
                    continue
                x = i + (j - 0.5) * 0.32 + (k - 1) * 0.08
                if ok(r):
                    ax.plot(x, 1000 * f(r['vx_b']), mk, color=CAT[M], ms=6, mec=SUP, mew=0.8)
                elif r['desfecho'] == 'caiu_antes':
                    ax.plot(x, -6, 'x', color=MUDO, ms=5)
                else:
                    v = f(r['vx_b'])
                    if not np.isnan(v):
                        ax.plot(x, 1000 * v, mk, color=CAT[M], ms=6, mfc='none')
    ax.axhline(0, color=MUDO, lw=0.8)
    ax.set_xticks(range(len(bracos)))
    ax.set_xticklabels([n for _, _, n in bracos], rotation=25, ha='right')
    ax.set_ylabel('avanço espúrio vx_b [mm/s]')
    ax.set_title('A1: o avanço vem da marcha; nenhuma ablação o remove, o ff em vx sim', loc='left', fontsize=9.5)
    from matplotlib.lines import Line2D
    h = [Line2D([], [], ls='', marker='s', color=CAT[M], label=M) for M in CAT]
    h += [Line2D([], [], ls='', marker=mk, color=TXT2, label=n) for _, mk, n in movs]
    h += [Line2D([], [], ls='', marker='x', color=MUDO, label='caiu antes da marcha (−6)')]
    ax.legend(handles=h, frameon=False, fontsize=7.5, ncol=3, loc='upper left')
    grade(ax)
    salvar(fig, 'fig_lateral_avanco.png')


def celula(ax, x, y, n, tot, txt=None):
    fr = n / tot if tot else 0
    cor = AZUL[int(round(fr * (len(AZUL) - 1)))] if tot else '#ffffff'
    ax.add_patch(plt.Rectangle((x - 0.46, y - 0.46), 0.92, 0.92, color=cor, lw=0))
    ax.text(x, y, txt or '%d/%d' % (n, tot), ha='center', va='center', fontsize=8,
            color='#ffffff' if fr > 0.55 else TXT)


def fig_envelope(L, O):
    fig, axs = plt.subplots(1, 3, figsize=(8.6, 2.8), gridspec_kw={'width_ratios': [1.0, 1.25, 1.25]})
    ax = axs[0]
    vys = (0.05, 0.075, 0.10)
    for j, M in enumerate(('M33', 'M38')):
        for i, v in enumerate(vys):
            rs = []
            for sg in ('+', '-'):
                for pe in ('pD', 'pE'):
                    if v == 0.05:
                        rs.append(O.get('E7_%s_kd06_lat%s0.05_%s_r1' % (M, sg, pe)))
                    else:
                        rs.append(L.get('A2_%s_lat%s%s_%s_r1' % (M, sg, '%.3f' % v if v == 0.075 else '0.10', pe)))
            rs = [r for r in rs if r]
            celula(ax, j, i, sum(ok(r) for r in rs), len(rs))
    ax.set_xticks([0, 1]); ax.set_xticklabels(['M33', 'M38'])
    ax.set_yticks(range(3)); ax.set_yticklabels(['0,05', '0,075', '0,10'])
    ax.set_ylabel('|vy| [m/s]')
    ax.set_xlim(-0.5, 1.5); ax.set_ylim(-0.5, 2.5)
    ax.set_title('A2: completas (de 4)\nT 0,6; y 0,06', loc='left', fontsize=9)
    for k, M in enumerate(('M33', 'M38')):
        ax = axs[k + 1]
        for i, ys in enumerate(('0.05', '0.06', '0.07')):
            for j, T in enumerate(('0.5', '0.6', '0.7')):
                rs = [L.get('A3_%s_lat%s0.075_y%s_T%s_pD_r1' % (M, sg, ys, T)) for sg in ('+', '-')]
                rs = [r for r in rs if r]
                celula(ax, j, i, sum(ok(r) for r in rs), len(rs))
        ax.add_patch(plt.Rectangle((1 - 0.48, 1 - 0.48), 0.96, 0.96, fill=False, ec=TXT, lw=1.2))
        ax.set_xticks(range(3)); ax.set_xticklabels(['0,5', '0,6', '0,7'])
        ax.set_yticks(range(3)); ax.set_yticklabels(['0,05', '0,06', '0,07'])
        ax.set_xlabel('step_period [s]')
        if k == 0:
            ax.set_ylabel('y_separation [m]')
        ax.set_xlim(-0.5, 2.5); ax.set_ylim(-0.5, 2.5)
        ax.set_title('A3 %s: completas (de 2)\n|vy| 0,075; quadro = default' % M, loc='left', fontsize=9)
    for a in axs:
        a.spines['left'].set_visible(False); a.spines['bottom'].set_visible(False)
        a.tick_params(length=0)
    fig.tight_layout()
    salvar(fig, 'fig_lateral_envelope.png')


def fig_fase():
    casos = (('A2_M33_lat+0.075_pD_r1', 'default: y 0,06; T 0,6'),
             ('A3_M33_lat+0.075_y0.05_T0.5_pD_r1', 'y 0,05; T 0,5'))
    fig, axs = plt.subplots(2, 1, figsize=(7.2, 3.8), sharex=True, sharey=True)
    for ax, (tag, nome) in zip(axs, casos):
        T = list(csv.DictReader(open(os.path.join(AQUI, 'lateral', 'dados', tag + '_telemetria.csv'))))
        t = np.array([float(r['t']) for r in T])
        pt = np.degrees([float(r['pitch_true']) for r in T])
        sp = np.array([int(float(r['support'])) for r in T])
        i0 = np.argmax(sp != sp[0])
        tt = t - t[i0]
        m = (tt > 10) & (tt < 14)
        for k, cor in ((1, '#fde7dc'), (2, '#dceafb')):
            on = m & (sp == k)
            ax.fill_between(tt, -9, 9, where=on, color=cor, lw=0, step='mid')
        ax.plot(tt[m], pt[m], color=TXT, lw=1.4)
        ax.axhline(0, color=MUDO, lw=0.6)
        ax.set_ylim(-9, 9)
        ax.set_ylabel('pitch [°]')
        ax.set_title('M33, lateral +0,075 m/s, %s' % nome, loc='left', fontsize=9)
    axs[-1].set_xlabel('tempo de marcha [s]   (fundo laranja = apoio esquerdo; azul = apoio direito)')
    axs[-1].set_xlim(10, 14)
    fig.tight_layout()
    salvar(fig, 'fig_lateral_fase.png')


def fig_b1(B):
    R = {}
    for tag, r in B.items():
        m = re.match(r'^B1_(M\d+)_(P\d)_s(\d)_(\S+?)_(p[DE])_r\d+$', tag)
        if m:
            R[m.groups()] = r
    fig, axs = plt.subplots(1, 2, figsize=(8.0, 3.0), gridspec_kw={'width_ratios': [1.1, 1]})
    ax = axs[0]
    nomes = {'P1': 'P1 junto', 'P3': 'P3 cot. 45°', 'P2': 'P2 cot. 90°'}
    for i, P in enumerate(('P1', 'P3', 'P2')):
        for (M, PP, s, mv, pe), r in R.items():
            if PP != P or s != '1':
                continue
            r0 = R.get((M, 'P0', '1', mv, pe))
            if r0 and ok(r) and ok(r0):
                x = i + (0.12 if M == 'M38' else -0.12) + (sum(map(ord, mv + pe)) % 7 - 3) * 0.02
                ax.plot(x, f(r['roll_rms_o']) - f(r0['roll_rms_o']), 'o', color=CAT[M], ms=4.5, mec=SUP, mew=0.6)
    ax.axhline(0, color=MUDO, lw=0.8)
    ax.set_xticks(range(3)); ax.set_xticklabels([nomes[p] for p in ('P1', 'P3', 'P2')])
    ax.set_ylabel('roll RMS (P − P0) [°]')
    ax.set_title('Q1: nenhuma postura reduz o roll (s = 1, com roll)', loc='left', fontsize=9)
    grade(ax)
    ax = axs[1]
    cores = {'P0': AZUL[11], 'P1': AZUL[8], 'P3': AZUL[5], 'P2': AZUL[3]}
    for P in ('P0', 'P1', 'P3', 'P2'):
        ys = [med([f(r['hc_rms']) for (M, PP, s, mv, pe), r in R.items()
                   if PP == P and s == str(k) and mv.startswith('reta') and ok(r)]) for k in (0, 1, 2)]
        ax.plot([0, 1, 2], [1000 * y for y in ys], '-o', color=cores[P], lw=2, ms=6, mec=SUP, label=P)
        ax.text(2.08, 1000 * ys[2] + {'P0': 0.25, 'P1': -0.25}.get(P, 0), P, va='center', fontsize=8, color=TXT2)
    ax.set_xticks([0, 1, 2]); ax.set_xticklabels(['s 0', 's 1', 's 2'])
    ax.set_xlim(-0.2, 2.35)
    ax.set_ylabel('hc_rms na reta [mrad/s]')
    ax.set_title('Q2/Q2b: o balanço reduz o esforço de rumo', loc='left', fontsize=9)
    grade(ax)
    fig.tight_layout()
    salvar(fig, 'fig_bracos_b1.png')


def fig_b2(B):
    bracos = (('P0', 'B2', 'P0 aberto\n(ref.)'), ('P1', 'B2x', 'P1 junto'),
              ('P2c', 'B2x', 'P2c contraída\nCoM anulado'), ('P0f', 'B2x', 'P0f aberto\nCoM +2,7 mm'),
              ('P2', 'B2', 'P2 contraída\nCoM +2,7 mm'))
    fig, axs = plt.subplots(1, 2, figsize=(8.4, 3.0), gridspec_kw={'width_ratios': [1.5, 1]})
    ns, pm = [], []
    for b, camp, _ in bracos:
        rs = [r for t, r in B.items() if re.match(r'^%s_M\d+_%s_s1_' % (camp, b), t)]
        ns.append((sum(ok(r) for r in rs), len(rs)))
        pm.append(med([f(r['pitch_marcha']) for r in rs if ok(r)]))
    ax = axs[0]
    for i, (n, t) in enumerate(ns):
        cor = AZUL[10] if bracos[i][0] in ('P0f', 'P2') else AZUL[3]
        ax.bar(i, n, width=0.6, color=cor)
        ax.text(i, n + 0.3, '%d/%d' % (n, t), ha='center', fontsize=8, color=TXT)
    ax.set_xticks(range(len(bracos))); ax.set_xticklabels([n for _, _, n in bracos], fontsize=7.5)
    ax.set_ylim(0, 13.5)
    ax.set_ylabel('completaram 30 s (de 12)')
    ax.set_title('Sem roll: sobrevive quem desloca o CoM para frente', loc='left', fontsize=9)
    grade(ax)
    ax = axs[1]
    for i, (b, _, _) in enumerate(bracos):
        for t, r in ((t, r) for t, r in B.items() if re.match(r'^B2x?_M\d+_%s_s1_' % b, t)):
            pmr = f(r['pitch_parado'])
            ax.plot(pmr, 30 if ok(r) else f(r['sobrevida']), 'o' if '_M33_' in t else 's',
                    color=AZUL[10] if b in ('P0f', 'P2') else AZUL[3], ms=4.5, mec=SUP, mew=0.6)
    from matplotlib.lines import Line2D
    ax.legend(handles=[Line2D([], [], ls='', marker='o', color=TXT2, label='M33'),
                       Line2D([], [], ls='', marker='s', color=TXT2, label='M38'),
                       Line2D([], [], ls='', marker='s', color=AZUL[10], label='CoM +2,7 mm'),
                       Line2D([], [], ls='', marker='s', color=AZUL[3], label='CoM 0')],
              frameon=False, fontsize=7, loc='center right')
    ax.set_xlabel('pitch parado antes da marcha [°] (+ = frente)')
    ax.set_ylabel('sobrevida [s] (30 = completou)')
    ax.set_title('cada corrida', loc='left', fontsize=9)
    grade(ax)
    fig.tight_layout()
    salvar(fig, 'fig_bracos_b2.png')


def fig_b3():
    arq = os.path.join(AQUI, 'bracos', 'b3', 'limiares.csv')
    if not os.path.exists(arq):
        return
    L = list(csv.DictReader(open(arq)))
    fig, ax = plt.subplots(figsize=(6.0, 2.9))
    posts = ('P0', 'P1', 'P3', 'P2')
    for r in L:
        m = re.match(r'^B3_(M\d+)_(P\d)_y([+-])$', r['base'])
        if not m:
            continue
        M, P, sg = m.groups()
        x = posts.index(P) + (-0.15 if M == 'M33' else 0.15)
        y = f(r['limiar_N'])
        ax.errorbar(x + (-0.04 if sg == '+' else 0.04), y, yerr=[[0], [f(r['resolucao_N'])]],
                    fmt='^' if sg == '+' else 'v', color=CAT[M], ms=6, capsize=2, lw=1)
    ax.set_xticks(range(4)); ax.set_xticklabels(posts)
    ax.set_ylabel('maior empurrão tolerado [N]\n(barra = até a 1ª força que derruba)')
    from matplotlib.lines import Line2D
    h = [Line2D([], [], ls='', marker='s', color=CAT[M], label=M) for M in CAT]
    h += [Line2D([], [], ls='', marker='^', color=TXT2, label='+y (para o pé de apoio)'), Line2D([], [], ls='', marker='v', color=TXT2, label='−y (para o pé em balanço)')]
    ax.legend(handles=h, frameon=False, fontsize=7, ncol=2, loc='upper right')
    ax.set_ylim(4, 17.5)
    ax.set_title('B3: empurrão lateral de 0,1 s no apoio esquerdo, marcha no lugar (com roll, s = 1)', loc='left', fontsize=9)
    grade(ax)
    salvar(fig, 'fig_bracos_b3.png')


def main():
    L, O, B = resumo('lateral'), resumo('omni'), resumo('bracos')
    fig_avanco(L)
    fig_envelope(L, O)
    fig_fase()
    fig_b1(B)
    fig_b2(B)
    fig_b3()


if __name__ == '__main__':
    main()
