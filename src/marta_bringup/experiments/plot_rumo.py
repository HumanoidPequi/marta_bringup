#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Oscilacao de yaw em torno do rumo desejado, com a malha de rumo ligada.

    python3 plot_rumo.py <dir_com_csvs> [prefixo] [saida.png]

Le os pares <tag>_telemetria.csv / <tag>_estabilizador.csv gerados por
bag_para_csv.py. So precisa de numpy e matplotlib (roda fora do container).

O que e o "rumo desejado" em cada corrida
-----------------------------------------
A malha de rumo guarda a referencia no yaw ESTIMADO (giro integrado) no
instante em que a marcha e ligada; o erro que ela corrige e
e(t) = psi_est(t) - psi_ref. A figura mostra a verdade de solo do Gazebo, nao o
sinal regulado. Para levar a referencia para o frame da verdade:
psi_ref_verdade = mediana(psi_verdade(t) - e(t)). Se o estimador nao derivar,
psi_verdade - e e constante; o desvio-padrao dela sai no relatorio como
checagem.

Hoje o rumo desejado e o rumo de partida (rotacao comandada = 0). Com uma
rotacao comandada (ex.: vinda da visao, apontando para a bola) a referencia se
move, e o mesmo calculo continua valendo: psi_verdade - e acompanha a
referencia movel.
"""
import csv
import glob
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

PASSADA = 0.8  # s: dois passos de 0,4 s (um ciclo esquerdo + direito)

# Paleta de referencia (skill dataviz), modo claro.
SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK2 = '#52514e'
MUTED = '#8a8984'
GRID = '#e6e5e1'
AZUL = '#2a78d6'
LARANJA = '#eb6834'


def ler(caminho):
    with open(caminho) as f:
        linhas = list(csv.DictReader(f))
    return {k: np.array([float(l[k]) for l in linhas]) for k in linhas[0]}


def media_movel(t, y, janela):
    """Media sobre uma passada: tira a oscilacao passo a passo e deixa a deriva."""
    dt = np.median(np.diff(t))
    n = max(1, int(round(janela / dt)))
    k = np.ones(n) / n
    return np.convolve(y, k, mode='same'), n


def carregar(dirc, tag):
    tel = ler(os.path.join(dirc, tag + '_telemetria.csv'))
    est = ler(os.path.join(dirc, tag + '_estabilizador.csv'))

    psi = np.unwrap(tel['yaw_true'])
    # Erro da malha interpolado nos instantes da telemetria.
    e = np.interp(tel['t'], est['t'], np.unwrap(est['heading_error']))
    corr = np.interp(tel['t'], est['t'], est['heading_correction'])

    # Marcha ativa: a malha so publica erro/correcao diferente de zero andando.
    ativo = np.abs(est['heading_correction']) > 1e-9
    if not ativo.any():
        raise RuntimeError('%s: malha de rumo nunca atuou (heading_enabled?)' % tag)
    t_ini = est['t'][np.argmax(ativo)]
    # Fim: ultimo instante com a malha atuando, e 1 s antes de qualquer queda
    # (a orientacao depois da queda nao e marcha).
    t_fim = est['t'][len(ativo) - 1 - np.argmax(ativo[::-1])]
    caido = (np.abs(tel['pitch_true']) > np.radians(30)) | \
            (np.abs(tel['roll_true']) > np.radians(30)) | (tel['height'] < 0.2)
    caiu = bool(caido[tel['t'] >= t_ini].any())
    if caiu:
        t_fim = min(t_fim, tel['t'][caido & (tel['t'] >= t_ini)][0] - 1.0)
    m = (tel['t'] >= t_ini) & (tel['t'] <= t_fim)

    off = psi[m] - e[m]
    ref = np.median(off)
    t = tel['t'][m] - t_ini
    desvio = np.degrees(psi[m] - ref)
    lento, n = media_movel(t, desvio, PASSADA)
    # Bordas da media movel ficam com meia janela: descarta.
    borda = n // 2
    return dict(tag=tag, t=t, desvio=desvio, lento=lento, borda=borda, caiu=caiu,
                corr=corr[m], suporte=tel['support'][m],
                deriva_est=np.degrees(np.std(off)),
                drift_est=np.degrees(off[-1] - off[0]) if len(off) else np.nan)


def metricas(r, t_regime=10.0):
    reg = r['t'] >= t_regime
    lento = r['lento'][r['borda']:-r['borda'] or None]
    t_l = r['t'][r['borda']:-r['borda'] or None]
    osc = r['desvio'] - r['lento']
    return dict(
        pico_partida=r['desvio'][r['t'] < t_regime][np.argmax(np.abs(r['desvio'][r['t'] < t_regime]))],
        media_regime=np.mean(r['desvio'][reg]),
        rms_regime=np.sqrt(np.mean(r['desvio'][reg] ** 2)),
        osc_pp=np.percentile(osc[reg], 97.5) - np.percentile(osc[reg], 2.5),
        lento_max=np.max(np.abs(lento[t_l >= t_regime])),
        final=r['desvio'][-1])


def figura(rs, saida, destaque=0):
    plt.rcParams.update({'font.size': 10, 'axes.edgecolor': MUTED,
                         'axes.labelcolor': INK2, 'xtick.color': INK2,
                         'ytick.color': INK2, 'axes.titlecolor': INK,
                         'axes.titleweight': 'bold', 'axes.titlesize': 11,
                         'axes.titlelocation': 'left'})
    fig = plt.figure(figsize=(11, 8.2), facecolor=SURFACE)
    gs = fig.add_gridspec(2, 2, height_ratios=[1.25, 1], hspace=0.42, wspace=0.18)
    a = fig.add_subplot(gs[0, :])
    b = fig.add_subplot(gs[1, 0])
    c = fig.add_subplot(gs[1, 1])
    for ax in (a, b, c):
        ax.set_facecolor(SURFACE)
        ax.grid(True, color=GRID, linewidth=0.8)
        ax.set_axisbelow(True)
        for s in ('top', 'right'):
            ax.spines[s].set_visible(False)

    r = rs[destaque]
    sl = slice(r['borda'], -r['borda'] or None)

    # --- (a) uma corrida: yaw verdadeiro, media por passada e referencia ---
    a.axhline(0, color=INK, linewidth=1.4, zorder=3)
    a.text(0.3, 0.6, 'rumo desejado (ψ_ref)', ha='left', va='bottom',
           color=INK, fontsize=9.5)
    a.plot(r['t'], r['desvio'], color=AZUL, linewidth=1.0, alpha=0.75,
           label='yaw do tronco (verdade do Gazebo)')
    a.plot(r['t'][sl], r['lento'][sl], color=LARANJA, linewidth=2.2,
           label='média por passada (0,8 s)')
    a.set_xlim(0, r['t'][-1])
    a.set_xlabel('tempo desde o início da marcha [s]')
    a.set_ylabel('ψ − ψ_ref [°]')
    a.set_title('(a) Uma corrida: o tronco oscila a cada passo em torno do rumo '
                'desejado')
    a.legend(loc='lower right', frameon=False, fontsize=9)

    # --- (b) zoom de 4 s em regime, com o pe de apoio ---
    t0z = 20.0
    z = (r['t'] >= t0z) & (r['t'] <= t0z + 4.0)
    tz = r['t'][z]
    sup = r['suporte'][z]
    for valor, cor, nome in ((1, '#dbe7f7', 'apoio esquerdo'),
                             (2, '#fbe1d6', 'apoio direito')):
        dentro = sup == valor
        b.fill_between(tz, 0, 1, where=dentro, transform=b.get_xaxis_transform(),
                       color=cor, linewidth=0, step='mid', label=nome)
    b.axhline(0, color=INK, linewidth=1.4)
    b.plot(tz, r['desvio'][z], color=AZUL, linewidth=1.6)
    b.plot(tz, r['lento'][z], color=LARANJA, linewidth=2.2)
    b.set_xlim(tz[0], tz[-1])
    b.set_xlabel('tempo [s]')
    b.set_ylabel('ψ − ψ_ref [°]')
    b.set_title('(b) Zoom de 4 s: um vaivém por passo')
    b.legend(loc='upper right', frameon=False, fontsize=8.5, ncol=2)

    # --- (c) todas as corridas, media por passada ---
    c.axhline(0, color=INK, linewidth=1.4)
    for i, ri in enumerate(rs):
        s = slice(ri['borda'], -ri['borda'] or None)
        if i == destaque:
            continue
        c.plot(ri['t'][s], ri['lento'][s], color=MUTED, linewidth=1.2)
    c.plot(r['t'][sl], r['lento'][sl], color=LARANJA, linewidth=2.2,
           label='corrida do painel (a)')
    c.plot([], [], color=MUTED, linewidth=1.2, label='demais corridas')
    c.set_xlim(0, max(ri['t'][-1] for ri in rs))
    c.set_xlabel('tempo desde o início da marcha [s]')
    c.set_ylabel('média por passada [°]')
    c.set_title('(c) %d corridas, média por passada' % len(rs))
    c.legend(loc='lower right', frameon=False, fontsize=8.5)

    fig.suptitle('Marta: yaw em torno do rumo desejado, com malha de rumo '
                 '(pé em caixa, τ = 6 N·m, estabilizador de pitch ligado, '
                 '0,1 m/s)', x=0.06, ha='left', fontsize=12, color=INK,
                 fontweight='bold')
    fig.savefig(saida, dpi=160, facecolor=SURFACE, bbox_inches='tight')
    print('figura: %s' % saida)


def main():
    dirc = sys.argv[1]
    prefixo = sys.argv[2] if len(sys.argv) > 2 else 'rumo_atual'
    saida = sys.argv[3] if len(sys.argv) > 3 else os.path.join(dirc, prefixo + '.png')
    tags = sorted(os.path.basename(p)[:-len('_estabilizador.csv')]
                  for p in glob.glob(os.path.join(dirc, prefixo + '*_estabilizador.csv')))
    rs = [carregar(dirc, t) for t in tags]

    print('%-16s %9s %9s %9s %9s %9s %9s %9s  queda' % (
        'corrida', 'partida', 'media', 'rms', 'osc p-p', 'lento max', 'final',
        'est desv'))
    ms = []
    for r in rs:
        mm = metricas(r)
        ms.append(mm)
        print(('%-16s %+8.1f° %+8.1f° %8.1f° %8.1f° %8.1f° %+8.1f° %8.2f°' % (
            r['tag'], mm['pico_partida'], mm['media_regime'], mm['rms_regime'],
            mm['osc_pp'], mm['lento_max'], mm['final'], r['deriva_est']))
              + ('  caiu no fim' if r['caiu'] else ''))
    for k in ('pico_partida', 'media_regime', 'rms_regime', 'osc_pp', 'lento_max'):
        v = np.array([m[k] for m in ms])
        print('  %-14s media %+6.2f  dp %5.2f' % (k, v.mean(), v.std(ddof=1) if len(v) > 1 else 0))

    # Destaque: a corrida de rms de regime mediano (representativa, nao a melhor).
    ordem = np.argsort([m['rms_regime'] for m in ms])
    figura(rs, saida, destaque=int(ordem[len(ordem) // 2]))


if __name__ == '__main__':
    main()
