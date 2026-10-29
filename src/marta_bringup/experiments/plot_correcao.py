#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Correcao da malha de rumo (correction em ControllerNode::headingCorrection)
ao longo da marcha.

    python3 plot_correcao.py <dir> <saida.png> rumo_atual=1.0 rumo_kp0.5=0.5 rumo_kp0.3=0.3

Le <tag>_estabilizador.csv (bag_para_csv.py): heading_error [rad] e
heading_correction [rad/s], que e o que a malha soma ao comando de rotacao.
"""
import glob
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from plot_rumo import ler, SURFACE, INK, INK2, MUTED, GRID, AZUL, LARANJA

SAT = 0.3  # max_heading_rate [rad/s], default da launch


def carregar(caminho):
    e = ler(caminho)
    ativo = np.abs(e['heading_correction']) > 1e-9
    i0 = np.argmax(ativo)
    i1 = len(ativo) - 1 - np.argmax(ativo[::-1])
    # Para 1 s antes de qualquer queda (atitude ou altura, pela verdade de solo
    # da telemetria, mesmo criterio do plot_rumo.py). Nao usar o tamanho do erro
    # de rumo como sinal de queda: com kp baixo ele passa de 45 graus andando.
    tel = ler(caminho.replace('_estabilizador.csv', '_telemetria.csv'))
    caido = (np.abs(tel['pitch_true']) > np.radians(30)) | \
            (np.abs(tel['roll_true']) > np.radians(30)) | (tel['height'] < 0.2)
    caido &= tel['t'] >= e['t'][i0]
    if caido.any():
        t_fim = tel['t'][caido][0] - 1.0
        i1 = min(i1, int(np.searchsorted(e['t'], t_fim)))
    t = e['t'][i0:i1 + 1] - e['t'][i0]
    return t, e['heading_correction'][i0:i1 + 1], np.degrees(e['heading_error'][i0:i1 + 1])


def estilo(ax):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)


def main():
    """
    python3 plot_correcao.py <dir> <saida.png> <prefixo>=<kp> [<prefixo>=<kp> ...]
    Uma linha de paineis por ganho: correction (esq.) e heading_error (dir.).
    """
    dirc, saida = sys.argv[1], sys.argv[2]
    grupos = [a.split('=') for a in sys.argv[3:]] or [('rumo_atual', '1.0')]

    plt.rcParams.update({'font.size': 10, 'axes.edgecolor': MUTED,
                         'axes.labelcolor': INK2, 'xtick.color': INK2,
                         'ytick.color': INK2, 'axes.titlecolor': INK,
                         'axes.titleweight': 'bold', 'axes.titlesize': 10.5,
                         'axes.titlelocation': 'left'})
    fig, axs = plt.subplots(len(grupos), 2, figsize=(13, 3.3 * len(grupos)),
                            sharex=True, squeeze=False, facecolor=SURFACE,
                            gridspec_kw={'hspace': 0.45, 'wspace': 0.15,
                                         'width_ratios': [1.15, 1]})
    print('%-6s %-16s %8s %8s %8s %9s %9s' % ('kp', 'corrida', 'media', 'rms',
          'saturado', 'erro rms', 'erro med'))
    for lin, (prefixo, kp) in enumerate(grupos):
        arqs = sorted(glob.glob(os.path.join(dirc, prefixo + '_r*_estabilizador.csv')))
        runs = [(os.path.basename(a).replace('_estabilizador.csv', ''), *carregar(a))
                for a in arqs]
        stats = []
        for tag, t, c, e in runs:
            st = (c.mean(), np.sqrt(np.mean(c ** 2)),
                  100 * np.mean(np.abs(c) >= SAT - 1e-6),
                  np.sqrt(np.mean(e[t >= 10] ** 2)), np.mean(e[t >= 10]))
            stats.append(st)
            print('%-6s %-16s %+7.3f %8.3f %7.1f%% %8.1f° %+8.1f°' % ((kp, tag) + st))
        m = np.mean(stats, axis=0)
        print('%-6s %-16s %+7.3f %8.3f %7.1f%% %8.1f° %+8.1f°  <- media' % ((kp, '') + tuple(m)))

        # Destaque: corrida de erro RMS mediano (representativa, nao a melhor).
        k = int(np.argsort([s[3] for s in stats])[len(stats) // 2])
        tag, t, c, e = runs[k]
        a, b = axs[lin]
        estilo(a); estilo(b)
        for v in (SAT, -SAT):
            a.axhline(v, color=MUTED, linewidth=1, linestyle='--')
        a.axhline(0, color=INK, linewidth=1)
        a.plot(t, c, color=AZUL, linewidth=1.0)
        a.set_ylim(-SAT * 1.25, SAT * 1.25)
        a.set_ylabel('correction [rad/s]')
        a.set_title('kp = %s  (%s)   RMS %.3f rad/s, saturado %.0f%% do tempo'
                    % (kp, tag, m[1], m[2]))
        b.axhline(0, color=INK, linewidth=1)
        b.plot(t, e, color=LARANJA, linewidth=1.0)
        b.set_ylim(-45, 45)
        b.set_ylabel('heading_error [°]')
        b.set_title('erro de rumo   RMS %.1f°  (média de %d corridas, t ≥ 10 s)'
                    % (m[3], len(runs)))
    for ax in axs[-1]:
        ax.set_xlabel('tempo desde o início da marcha [s]')
        ax.set_xlim(0, 60)
    fig.suptitle('Malha de rumo com três ganhos: correction = clamp(−kp·erro, ±%.1f rad/s)'
                 % SAT, x=0.06, ha='left', fontsize=12, color=INK, fontweight='bold')
    fig.savefig(saida, dpi=160, facecolor=SURFACE, bbox_inches='tight')
    print('figura: %s' % saida)


if __name__ == '__main__':
    main()
