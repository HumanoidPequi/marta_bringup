#!/usr/bin/env python3
"""Figura-resumo da campanha de postura (plano_comparacao_darwin.md §2.3).

A: torque PEDIDO pelo joelho na marcha atual (tau = 1000), por fase, contra o
   stall do MX-28 (2,5 N.m) e o limite em que a Marta anda hoje (6 N.m).
B: esticar a perna (z_com 0,2975 -> 0,345) alivia o joelho de apoio e
   sobrecarrega o de balanco. z_step = 0,06 nas duas condicoes.
Barras = media das corridas; pontos = cada corrida; tau = 1000; t em [5, 28] s.

  python3 fig_postura.py [dir_postura] [saida.png]
"""
import os
import statistics as st
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analisar_postura as A  # noqa: E402
import csv  # noqa: E402

AZUL, LARANJA, CINZA = '#2a78d6', '#eb6834', '#8c8b86'
TINTA, TINTA2, GRADE = '#0b0b0b', '#52514e', '#e4e3df'
STALL, LIMITE = 2.5, 6.0


def rms_por_corrida(d, zc, zs, fase):
    out = []
    for r in csv.DictReader(open(os.path.join(d, 'campanha_postura.csv'))):
        if r['tag'].endswith('_r0') or A.celula(r['tag']) != (zc, zs, '1000'):
            continue
        dur = float(r['t_queda']) if r['caiu'] == '1' else 1e9
        tq, _ = A.torques(os.path.join(d, 'dados', r['tag']), dur)
        if ('knee', fase) in tq:
            out.append(A.resumo(tq[('knee', fase)])[0])
    return out


def barras(ax, x, vals, cor, rotulo=None):
    m = st.mean(vals)
    ax.bar(x, m, width=0.36, color=cor, label=rotulo, zorder=2)
    ax.scatter([x] * len(vals), vals, s=22, color='white', edgecolor=TINTA, linewidth=0.8, zorder=3)
    ax.text(x, max([m] + vals) + 0.3, ('%.1f' % m).replace('.', ','), ha='center', va='bottom', fontsize=10,
            color=TINTA, fontweight='bold')


def limites(ax, x0=None):
    ax.axhline(STALL, color=TINTA, lw=1.2, ls='--', zorder=4)
    ax.axhline(LIMITE, color=TINTA2, lw=1.0, ls=':', zorder=4)



def estilo(ax):
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color(GRADE)
    ax.tick_params(colors=TINTA2, labelsize=10)
    ax.yaxis.grid(True, color=GRADE, lw=0.8, zorder=0)
    ax.set_axisbelow(True)


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), 'postura')
    saida = sys.argv[2] if len(sys.argv) > 2 else os.path.join(d, 'fig_postura.png')
    plt.rcParams['font.family'] = 'DejaVu Sans'
    fig, (a, b) = plt.subplots(1, 2, figsize=(12, 5.2), sharey=True, gridspec_kw={'wspace': 0.08})
    fig.patch.set_facecolor('white')

    fases = [('apoio', 'perna de apoio'), ('balanco', 'perna em balanço'), ('duplo', 'apoio duplo')]
    for i, (f, nome) in enumerate(fases):
        barras(a, i + 0.1, rms_por_corrida(d, '0.2975', '0.06', f), CINZA)
    a.set_xlim(-0.5, 2.5)
    a.set_xticks([0.1, 1.1, 2.1])
    a.set_xticklabels([n for _, n in fases])
    limites(a)
    a.set_ylabel('torque pedido pelo joelho, RMS (N·m)', color=TINTA2, fontsize=10)
    a.set_title('A. Marcha atual: o joelho pede 2–3× o stall do servo', loc='left', fontsize=12, color=TINTA, pad=12)
    a.text(0, -0.2, 'Pé em caixa, 6 corridas. Com limite de 6 N·m: 0/6 quedas; com 4 N·m: 6/6 quedas.',
           transform=a.transAxes, fontsize=9, color=TINTA2, va='top')
    estilo(a)

    pares = [('apoio', 'perna de apoio', 'p = 0,009'), ('balanco', 'perna em balanço', 'p = 0,001')]
    for i, (f, nome, p) in enumerate(pares):
        va = rms_por_corrida(d, '0.2975', '0.06', f)
        vb = rms_por_corrida(d, '0.345', '0.06', f)
        barras(b, i - 0.2, va, AZUL, 'postura atual (joelho ~114°)' if i == 0 else None)
        barras(b, i + 0.2, vb, LARANJA, 'perna esticada (joelho ~78°)' if i == 0 else None)
        delta = 100 * (st.mean(vb) / st.mean(va) - 1)
        top = max(va + vb)
        b.text(i, top + 1.6, '%+.0f%%  (%s)' % (delta, p), ha='center', fontsize=10.5, color=TINTA)
    b.set_xticks([0, 1])
    b.set_xticklabels([n for _, n, _ in pares])
    b.set_xlim(-0.6, 1.6)
    limites(b)
    from matplotlib.lines import Line2D
    h, l = b.get_legend_handles_labels()
    h += [Line2D([], [], color=TINTA, lw=1.2, ls='--'), Line2D([], [], color=TINTA2, lw=1.0, ls=':')]
    l += ['stall do MX-28 (2,5 N·m)', 'limite em que a Marta anda hoje (6 N·m)']
    b.legend(h, l, loc='upper left', frameon=False, fontsize=9.5, labelcolor=TINTA2)
    b.set_title('B. Esticar a perna: o apoio alivia, o balanço dobra', loc='left', fontsize=12, color=TINTA, pad=12)
    b.text(0, -0.2, 'Perna esticada: n = 5 de 6; a marcha cai até com torque ilimitado (4/6).',
           transform=b.transAxes, fontsize=9, color=TINTA2, va='top')
    estilo(b)

    a.set_ylim(0, 16.5)
    fig.suptitle('Por que a Marta precisa de 6 N·m com um servo de 2,5 N·m: o gargalo é a perna em balanço',
                 x=0.07, ha='left', fontsize=13.5, fontweight='bold', color=TINTA, y=1.0)
    fig.text(0.07, 0.935, 'Gazebo/ODE, torque pedido pelo PID antes da saturação (limite 1000 N·m), 30 s a 0,1 m/s; '
             'barras = média, pontos = corridas.', fontsize=9.5, color=TINTA2)
    fig.subplots_adjust(left=0.07, right=0.99, top=0.83, bottom=0.19)
    fig.savefig(saida, dpi=200, facecolor='white')
    print(saida)


if __name__ == '__main__':
    main()
