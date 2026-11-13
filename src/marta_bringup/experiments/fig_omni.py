#!/usr/bin/env python3
"""Figuras da marcha omnidirecional (docs/marcha_omnidirecional.md).

Le omni/resumo_omni.csv (analisar_omni.py) e gera em docs/figuras_marcha/:

  fig_omni_sobrevivencia_<P>.png
      um painel por massa; linhas = movimento e velocidade, colunas = braco
      (o que vem depois do movimento na tag: pe inicial, malha de rumo,
      roll...); cor = sobrevida / 30 s (azul sequencial; o mais escuro =
      completou); texto = "ok" ou o instante da queda [s]
  fig_omni_omega_<P>.png
      taxa de giro realizada x comandada (giro, curva, combinado), por massa;
      marca cheia = completou, vazada = caiu (taxa medida ate 0,5 s antes da
      queda, janela minima de 3 s; corridas que cairam antes disso ficam de
      fora); a diagonal tracejada e o ideal
  fig_omni_roll_<P>.png
      so quando ha braco "off_<pe>" e um braco de roll "<cfg>_<pe>" (E5):
      roll RMS verdadeiro sem roll (x) contra com roll (y), um ponto por par
      (massa, movimento, pe inicial); marca cheia = o par sem roll completou,
      vazada = caiu (RMS ate 0,5 s antes da queda); abaixo da diagonal = o
      roll reduziu a oscilacao. Substitui a figura de omega nesse caso.

  python3 fig_omni.py [resumo_omni.csv] [prefixo, ex. E1_] [bracos, ex. pD_h1,pE_h1,pD_h0]
"""
import csv
import math
import os
import re
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

AQUI = os.path.dirname(os.path.abspath(__file__))
SAIDA = os.environ.get('FIG_SAIDA', os.path.join(AQUI, '..', '..', '..', 'docs', 'figuras_marcha'))
AZUL = ['#cde2fb', '#b7d3f6', '#9ec5f4', '#86b6ef', '#6da7ec', '#5598e7', '#3987e5',
        '#2a78d6', '#256abf', '#1c5cab', '#184f95', '#104281', '#0d366b']
CAT = {'M33': '#2a78d6', 'M38': '#eb6834'}
NOME = {'M33': '3,3 kg (M33)', 'M38': '3,8 kg (M38)'}
SUP, TXT, TXT2, MUDO, GRADE = '#fcfcfb', '#0b0b0b', '#52514e', '#898781', '#e1e0d9'
DUR = 30.0
# movimento: tipo + velocidade com sinal (reta0.10, giro+0.175, curva-0.25...)
RE_MOV = re.compile(r'^(reta|giro|curva|lat|diag|comb)([+-]?[\d.]+)$')
ORDEM = ['reta', 'giro', 'curva', 'lat', 'diag', 'comb']
ROTULO = {'reta': 'reta vx', 'giro': 'giro ω', 'curva': 'curva (0,1; ω)',
          'lat': 'lateral vy', 'diag': 'diagonal (0,1; vy)', 'comb': 'comb. (0,1; 0,03; ω)'}

plt.rcParams.update({
    'figure.facecolor': SUP, 'axes.facecolor': SUP, 'savefig.facecolor': SUP,
    'font.size': 9, 'axes.edgecolor': MUDO, 'axes.labelcolor': TXT2,
    'xtick.color': TXT2, 'ytick.color': TXT2, 'axes.titlecolor': TXT,
    'axes.spines.top': False, 'axes.spines.right': False,
})


def f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return float('nan')


def le(caminho, prefixo):
    """{(massa, movimento, braco): linha}; tag = <P>_<massa>_[<cfg>_]<mov>_<braco>_r<n>."""
    out = {}
    for r in csv.DictReader(open(caminho)):
        if not r['tag'].startswith(prefixo):
            continue
        partes = r['tag'][len(prefixo):].rsplit('_r', 1)[0].split('_')
        massa = partes[0]
        i = next((k for k, p in enumerate(partes) if RE_MOV.match(p)), None)
        if i is None:
            continue
        braco = '_'.join(partes[1:i] + partes[i + 1:])
        out[(massa, partes[i], braco)] = r
    return out


def chave_mov(m):
    t, v = RE_MOV.match(m).groups()
    return (ORDEM.index(t), abs(float(v)), -float(v))


def mapa(dados, prefixo, bracos):
    massas = sorted({k[0] for k in dados})
    movs = sorted({k[1] for k in dados}, key=chave_mov)
    cmap = LinearSegmentedColormap.from_list('azul', AZUL)
    fig, axs = plt.subplots(1, len(massas), figsize=(2.2 + 1.1 * len(bracos) * len(massas),
                                                       0.3 * len(movs) + 1.3),
                            sharey=True, squeeze=False)
    for ax, massa in zip(axs[0], massas):
        for i, mv in enumerate(movs):
            for j, b in enumerate(bracos):
                r = dados.get((massa, mv, b))
                if r is None:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, fc=SUP, ec=GRADE, lw=0.5))
                    continue
                ok = r['desfecho'] == 'completou'
                s = DUR if ok else min(f(r['sobrevida']), DUR)
                c = cmap(s / DUR)
                ax.add_patch(plt.Rectangle((j + 0.04, i + 0.06), 0.92, 0.88, fc=c, ec=SUP, lw=1.0))
                ax.text(j + 0.5, i + 0.5, 'ok' if ok else '%.1f' % s, ha='center', va='center',
                        fontsize=7.5, color='white' if s / DUR > 0.55 else TXT)
        ax.set_xlim(0, len(bracos))
        ax.set_ylim(len(movs), 0)
        ax.set_xticks([j + 0.5 for j in range(len(bracos))])
        ax.set_xticklabels(bracos, fontsize=8)
        ax.xaxis.tick_top()
        ax.tick_params(length=0)
        for s in ax.spines.values():
            s.set_visible(False)
        ax.set_title(NOME.get(massa, massa), pad=22, fontsize=10)
    ax0 = axs[0][0]
    ax0.set_yticks([i + 0.5 for i in range(len(movs))])
    rot = []
    for mv in movs:
        t, v = RE_MOV.match(mv).groups()
        rot.append('%s %s' % (ROTULO[t], v.replace('.', ',')))
    ax0.set_yticklabels(rot, fontsize=8)
    fig.text(0.01, 0.005, 'cor: sobrevida/30 s (escuro = andou os 30 s); número = instante da queda [s]',
             fontsize=7.5, color=TXT2)
    fig.tight_layout(rect=(0, 0.02, 1, 1))
    nome = os.path.join(SAIDA, 'fig_omni_sobrevivencia_%s.png' % prefixo.rstrip('_'))
    fig.savefig(nome, dpi=150)
    plt.close(fig)
    return nome


def omega(dados, prefixo):
    fig, ax = plt.subplots(figsize=(5.0, 4.4))
    lim = 0.0
    for (massa, mv, b), r in sorted(dados.items()):
        if not mv.startswith(('giro', 'curva', 'comb')):
            continue
        wc, wr = f(r['w_cmd']), f(r['w_real'])
        if math.isnan(wr):
            continue
        ok = r['desfecho'] == 'completou'
        ax.plot(wc, wr, 'o', ms=6.5, mfc=CAT[massa] if ok else SUP, mec=CAT[massa], mew=1.4,
                alpha=0.9)
        lim = max(lim, abs(wc), abs(wr))
    lim *= 1.12
    ax.plot([-lim, lim], [-lim, lim], '--', color=MUDO, lw=1.0, zorder=0)
    ax.axhline(0, color=GRADE, lw=0.8, zorder=0)
    ax.axvline(0, color=GRADE, lw=0.8, zorder=0)
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_aspect('equal')
    ax.set_xlabel('ω comandado [rad/s]')
    ax.set_ylabel('ω realizado [rad/s]')
    leg = [Line2D([], [], ls='', marker='o', ms=6.5, mfc=CAT[m], mec=CAT[m], label=NOME[m])
           for m in ('M33', 'M38')]
    leg += [Line2D([], [], ls='', marker='o', ms=6.5, mfc=MUDO, mec=MUDO, label='andou 30 s'),
            Line2D([], [], ls='', marker='o', ms=6.5, mfc=SUP, mec=MUDO, label='caiu (até a queda)')]
    ax.legend(handles=leg, frameon=False, fontsize=8, loc='upper left')
    fig.tight_layout()
    nome = os.path.join(SAIDA, 'fig_omni_omega_%s.png' % prefixo.rstrip('_'))
    fig.savefig(nome, dpi=150)
    plt.close(fig)
    return nome


def roll_pareado(dados, prefixo):
    pares = []
    for (massa, mv, b), r in dados.items():
        if not b.startswith('off_'):
            continue
        pe = b[len('off_'):]
        on = [v for (m2, mv2, b2), v in dados.items()
              if m2 == massa and mv2 == mv and b2 != b and b2.endswith('_' + pe)]
        if len(on) == 1 and not (math.isnan(f(r['roll_rms_o'])) or math.isnan(f(on[0]['roll_rms_o']))):
            pares.append((massa, r, on[0]))
    if not pares:
        return None
    fig, ax = plt.subplots(figsize=(5.0, 4.4))
    lim = 1.08 * max(max(f(a['roll_rms_o']), f(b['roll_rms_o'])) for _, a, b in pares)
    for massa, a, b in pares:
        ok = a['desfecho'] == 'completou'
        ax.plot(f(a['roll_rms_o']), f(b['roll_rms_o']), 'o', ms=6.5, mfc=CAT[massa] if ok else SUP,
                mec=CAT[massa], mew=1.4, alpha=0.9)
    ax.plot([0, lim], [0, lim], '--', color=MUDO, lw=1.0, zorder=0)
    ax.set_xlim(0, lim)
    ax.set_ylim(0, lim)
    ax.set_aspect('equal')
    ax.set_xlabel('roll RMS sem estabilizador de roll [graus]')
    ax.set_ylabel('roll RMS com estabilizador de roll [graus]')
    leg = [Line2D([], [], ls='', marker='o', ms=6.5, mfc=CAT[m], mec=CAT[m], label=NOME[m])
           for m in ('M33', 'M38')]
    leg += [Line2D([], [], ls='', marker='o', ms=6.5, mfc=MUDO, mec=MUDO, label='sem roll: andou 30 s'),
            Line2D([], [], ls='', marker='o', ms=6.5, mfc=SUP, mec=MUDO, label='sem roll: caiu (até a queda)')]
    ax.legend(handles=leg, frameon=False, fontsize=8, loc='upper left')
    fig.tight_layout()
    nome = os.path.join(SAIDA, 'fig_omni_roll_%s.png' % prefixo.rstrip('_'))
    fig.savefig(nome, dpi=150)
    plt.close(fig)
    return nome


def main():
    caminho = sys.argv[1] if len(sys.argv) > 1 else os.path.join(AQUI, 'omni', 'resumo_omni.csv')
    prefixo = sys.argv[2] if len(sys.argv) > 2 else 'E1_'
    dados = le(caminho, prefixo)
    if len(sys.argv) > 3:
        bracos = sys.argv[3].split(',')
    else:
        bracos = sorted({k[2] for k in dados})
    os.makedirs(SAIDA, exist_ok=True)
    print(mapa(dados, prefixo, bracos))
    print(roll_pareado(dados, prefixo) or omega(dados, prefixo))


if __name__ == '__main__':
    main()
