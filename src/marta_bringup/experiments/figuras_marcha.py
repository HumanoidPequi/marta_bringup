#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Figuras da marcha a partir dos CSV extraidos por bag_para_csv.py.

    python3 figuras_marcha.py <prefixo> [dir_figuras]

onde <prefixo> e o caminho sem o sufixo, por exemplo
    .../dados/demo_tau6_com
esperando encontrar <prefixo>_telemetria.csv e <prefixo>_juntas.csv.

Nao precisa de ROS: so numpy e matplotlib. Roda no host ou no container.

Referencias de equacao: Kajita et al., "The 3D Linear Inverted Pendulum Mode",
IROS 2001.
"""
import csv
import math
import os
import sys
from collections import defaultdict

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

G = 9.80665
SEM, COM = '#B45309', '#2E6F8E'
REF, REAL = '#9AA4AE', '#1B3A4B'
plt.rcParams.update({'font.size': 10.5, 'axes.grid': True, 'grid.alpha': 0.25,
                     'axes.spines.top': False, 'axes.spines.right': False,
                     'axes.axisbelow': True})

# Juntas das pernas, na ordem anatomica - e assim que o mapa de calor fica
# legivel: quadril em cima, pe embaixo, esquerda e direita separadas.
# Largura da janela temporal das figuras do paper, em segundos. O artigo mostra
# ~4 s (7 passos); com 20 s de marcha seriam 50 passos e nada seria legivel.
LARGURA_JANELA = float(os.environ.get('JANELA_S', '4.0'))

PERNAS = ['l_hip_yaw', 'l_hip_roll', 'l_hip_pitch', 'l_knee', 'l_ank_pitch',
          'l_ank_roll', 'r_hip_yaw', 'r_hip_roll', 'r_hip_pitch', 'r_knee',
          'r_ank_pitch', 'r_ank_roll']


def recortar(d, t_ini, t_fim):
    """Recorta a telemetria numa janela de tempo."""
    m = (d['t'] >= t_ini) & (d['t'] <= t_fim)
    return {k: v[m] for k, v in d.items()}


def janela_central(d, largura):
    """
    Janela de 'largura' segundos no meio da corrida. As figuras 4 e 5 do artigo
    mostram 7 passos em ~4 s; com 20 s de marcha seriam 50 passos ilegiveis.
    """
    t = d['t']
    meio = 0.5 * (t[0] + t[-1])
    return max(t[0], meio - largura / 2.0), min(t[-1], meio + largura / 2.0)


def ler_telemetria(p):
    col = defaultdict(list)
    with open(p) as f:
        for r in csv.DictReader(f):
            for k, v in r.items():
                col[k].append(float(v))
    return {k: np.array(v) for k, v in col.items()}


def ler_juntas(p):
    por = defaultdict(lambda: defaultdict(list))
    with open(p) as f:
        for r in csv.DictReader(f):
            j = por[r['junta']]
            j['t'].append(float(r['t']))
            j['error'].append(float(r['error']))
            j['command'].append(float(r['command']))
    return {k: {kk: np.array(vv) for kk, vv in v.items()} for k, v in por.items()}


def trocas_de_apoio(t, support):
    """Instantes em que o estado de apoio muda - as linhas verticais da fig. 5."""
    return [t[i] for i in range(1, len(support)) if support[i] != support[i - 1]]


def janelas_apoio_simples(t, support):
    """Intervalos (t_ini, t_fim, lado) de cada fase de apoio simples."""
    jan, ini, lado = [], None, None
    for i in range(len(support)):
        s = support[i]
        if s in (1, 2):
            if ini is None or s != lado:
                if ini is not None:
                    jan.append((ini, t[i - 1], lado))
                ini, lado = t[i], s
        else:
            if ini is not None:
                jan.append((ini, t[i - 1], lado))
                ini, lado = None, None
    if ini is not None:
        jan.append((ini, t[-1], lado))
    return [j for j in jan if j[1] - j[0] > 0.05]


def salvar(fig, saida, nome):
    for ext in ('png', 'pdf'):
        fig.savefig(os.path.join(saida, nome + '.' + ext),
                    bbox_inches='tight', dpi=200)
    plt.close(fig)
    print('  ' + nome)


# ---------------------------------------------------------------- figura 5 --
def fig_paper5(d, saida):
    """
    Reproduz a figura 5 de Kajita 2001: posicao e velocidade do CoM contra o
    tempo, COM A ORIGEM NO PE DE APOIO. E por isso que as curvas saltam a cada
    troca de apoio - e o mesmo efeito descrito na legenda do artigo.
    Aqui, alem do planejado, vai o medido sobreposto.
    """
    d = recortar(d, *janela_central(d, LARGURA_JANELA))
    t, sup = d['t'], d['support']
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(8.2, 5.6), sharex=True)

    for x in trocas_de_apoio(t, sup):
        for a in (a1, a2):
            a.axvline(x, color='0.75', lw=0.7, ls=':', zorder=0)

    a1.plot(t, d['ref_x'], lw=1.0, color=REF, label='x planejado')
    a1.plot(t, d['ref_y'], lw=1.0, color=REF, ls='--', label='y planejado')
    a1.plot(t, d['sup_x'], lw=1.8, color=REAL, label='x medido')
    a1.plot(t, d['sup_y'], lw=1.8, color=COM, label='y medido')
    a1.set_ylabel('posição do CoM [m]\n(origem no pé de apoio)')
    a1.legend(frameon=False, ncol=4, fontsize=8.5, loc='upper center')
    a1.set_title('Figura 5 do artigo, com o medido sobreposto ao planejado',
                 loc='left', fontweight='bold')

    a2.plot(t, d['com_vx'], lw=1.6, color=REAL, label='vx medido')
    a2.plot(t, d['com_vy'], lw=1.6, color=COM, label='vy medido')
    a2.set_ylabel('velocidade do CoM [m/s]')
    a2.set_xlabel('tempo [s]')
    a2.legend(frameon=False, ncol=2, fontsize=8.5)
    salvar(fig, saida, 'fig_paper5_com_tempo')


# ---------------------------------------------------------------- figura 4 --
def fig_paper4(d, saida):
    """
    Reproduz a figura 4: o traco do CoM no plano XY com as pegadas. No artigo
    os arcos do apoio simples sao hiperboles e o apoio duplo e reta; aqui os
    trechos sao coloridos por fase de apoio para que a mesma leitura valha.
    """
    d = recortar(d, *janela_central(d, LARGURA_JANELA * 2))
    t, sup = d['t'], d['support']
    fig, ax = plt.subplots(figsize=(8.6, 4.4))

    simples = np.isin(sup, (1, 2))
    x, y = d['com_x'], d['com_y']
    # quebra em segmentos contiguos para nao ligar trechos de fases diferentes
    ini = 0
    for i in range(1, len(t) + 1):
        if i == len(t) or simples[i] != simples[ini]:
            sl = slice(ini, i)
            ax.plot(x[sl], y[sl], lw=2.0 if simples[ini] else 1.0,
                    ls='-' if simples[ini] else ':',
                    color=REAL if simples[ini] else '0.6', zorder=2)
            ini = i

    # pegadas: posicao do pe de apoio no instante de cada troca
    marcados = []
    for i in range(1, len(t)):
        if sup[i] != sup[i - 1] and sup[i] in (1, 2):
            p = (d['foot_x'][i], d['foot_y'][i])
            if all(math.hypot(p[0] - q[0], p[1] - q[1]) > 0.02 for q in marcados):
                marcados.append(p)
    if marcados:
        ax.scatter([p[0] for p in marcados], [p[1] for p in marcados],
                   s=70, facecolor='white', edgecolor=SEM, lw=1.8, zorder=3,
                   label='pegadas')

    ax.plot([], [], color=REAL, lw=2.0, label='CoM, apoio simples')
    ax.plot([], [], color='0.6', lw=1.0, ls=':', label='CoM, apoio duplo')
    ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]')
    ax.set_aspect('equal', adjustable='datalim')
    ax.legend(frameon=False, fontsize=9, loc='upper left')
    ax.set_title('Figura 4 do artigo: trajetória do CoM no plano e as pegadas',
                 loc='left', fontweight='bold')
    salvar(fig, saida, 'fig_paper4_plano_xy')


# ------------------------------------------------------- energia orbital ----
def fig_energia(d, saida):
    """
    Energia orbital, eq. (28)(29). Sob o 3D-LIPM ela e invariante no tempo.

    Painel de cima: valor absoluto. Ele SALTA a cada troca de apoio porque a
    origem do frame muda de pe - e o mesmo efeito que produz os saltos na figura
    5 do artigo, nao um defeito. Esses saltos, porem, escondem o que interessa.

    Painel de baixo: a mesma energia MENOS o seu valor no inicio de cada apoio
    simples. Assim cada fase comeca em zero e o que se ve e so a deriva dentro
    dela - que e a medida de quanto a realidade viola o modelo. Sob um LIPM
    perfeito, este painel seria uma linha reta em zero.
    """
    d = recortar(d, *janela_central(d, LARGURA_JANELA))
    t, sup = d['t'], d['support']
    jan = janelas_apoio_simples(t, sup)

    fig, (a1, a2) = plt.subplots(2, 1, figsize=(8.2, 6.4), sharex=True,
                                 gridspec_kw=dict(height_ratios=[1, 1], hspace=0.42))
    for (a, b, lado) in jan:
        for ax in (a1, a2):
            ax.axvspan(a, b, color='#EAF2F6', zorder=0)

    a1.plot(t, d['E_x'], lw=1.5, color=REAL, label='$E_x$ (sagital)')
    a1.plot(t, d['E_y'], lw=1.5, color=SEM, label='$E_y$ (frontal)')
    a1.axhline(0, color='0.7', lw=0.8)
    a1.set_ylabel('energia orbital\n[m²/s²]')
    a1.legend(frameon=False, ncol=2, fontsize=9)
    a1.set_title('Energia orbital — invariante sob o 3D-LIPM, eq. (28)(29)\n'
                 'faixas claras = apoio simples',
                 loc='left', fontweight='bold')

    # deriva: subtrai o valor no inicio de cada fase de apoio simples
    derivas = []
    for (a, b, lado) in jan:
        m = (t >= a) & (t <= b)
        if m.sum() < 5:
            continue
        for chave, cor in (('E_x', REAL), ('E_y', SEM)):
            e = d[chave][m]
            a2.plot(t[m], e - e[0], lw=1.5, color=cor)
        derivas.append(float(np.ptp(d['E_x'][m])))
    a2.axhline(0, color='0.7', lw=0.8)
    a2.set_ylabel('deriva dentro da fase\n[m²/s²]')
    a2.set_xlabel('tempo [s]')
    med = float(np.mean(derivas)) if derivas else float('nan')
    a2.set_title('Deriva relativa ao início de cada apoio simples — sob um LIPM '
                 'perfeito seria zero\namplitude média de $E_x$ por fase: '
                 '%.4f m²/s² (n=%d fases)' % (med, len(derivas)),
                 loc='left', fontsize=9.5, pad=8)
    salvar(fig, saida, 'fig_energia_orbital')


# --------------------------------------------------------- plano de fase ----
def fig_fase(d, saida):
    """
    Plano de fase (x, xdot) no frame do apoio. Sob o LIPM as orbitas sao
    hiperboles xdot^2 - (g/zc)x^2 = 2E, eq. (40). As curvas cinza sao as
    hiperboles previstas para a energia media de cada apoio simples.
    """
    t, sup = d['t'], d['support']
    zc = float(np.mean(d['z_com_meas']))
    w2 = G / zc
    fig, ax = plt.subplots(figsize=(6.4, 5.0))

    for (a, b, lado) in janelas_apoio_simples(t, sup):
        m = (t >= a) & (t <= b)
        if m.sum() < 5:
            continue
        ax.plot(d['sup_x'][m], d['com_vx'][m], lw=1.3,
                color=COM if lado == 1 else SEM, alpha=0.8)
        # hiperbole prevista para a energia media dessa janela
        E = float(np.mean(d['E_x'][m]))
        xs = np.linspace(d['sup_x'][m].min(), d['sup_x'][m].max(), 80)
        arg = 2 * E + w2 * xs ** 2
        ok = arg >= 0
        if ok.sum() > 3:
            ax.plot(xs[ok], np.sqrt(arg[ok]), lw=0.8, color='0.65', zorder=0)

    ax.plot([], [], color=COM, lw=1.3, label='apoio esquerdo')
    ax.plot([], [], color=SEM, lw=1.3, label='apoio direito')
    ax.plot([], [], color='0.65', lw=0.8, label='hipérbole prevista, eq. (40)')
    ax.set_xlabel('x do CoM no frame do apoio [m]')
    ax.set_ylabel('velocidade $\\dot{x}$ [m/s]')
    ax.set_title('Plano de fase sagital\n$z_c$ medido = %.3f m' % zc,
                 loc='left', fontweight='bold')
    ax.legend(frameon=False, fontsize=9)
    salvar(fig, saida, 'fig_plano_fase')


# ------------------------------------------------ torque e desvio por junta --
def _mapa(juntas, campo, titulo, rotulo, nome, saida, limite=None):
    presentes = [j for j in PERNAS if j in juntas]
    if not presentes:
        return
    t_ref = juntas[presentes[0]]['t']
    grade = np.vstack([np.interp(t_ref, juntas[j]['t'], juntas[j][campo])
                       for j in presentes])
    fig, ax = plt.subplots(figsize=(9.0, 4.6))
    vmax = np.abs(grade).max()
    im = ax.imshow(grade, aspect='auto', cmap='RdBu_r', vmin=-vmax, vmax=vmax,
                   extent=[t_ref[0], t_ref[-1], len(presentes) - 0.5, -0.5],
                   interpolation='nearest')
    ax.set_yticks(range(len(presentes)))
    ax.set_yticklabels(presentes, fontsize=8.5, family='monospace')
    ax.set_xlabel('tempo [s]')
    ax.grid(False)
    fig.canvas.draw_idle()
    cb = fig.colorbar(im, ax=ax, pad=0.015)
    cb.set_label(rotulo)
    sub = ''
    if limite:
        n = int((np.abs(grade) >= limite * 0.999).sum())
        sub = '\nno limite de %.0f N·m em %.1f%% das amostras' % (
            limite, 100.0 * n / grade.size)
    ax.set_title(titulo + sub, loc='left', fontweight='bold')
    salvar(fig, saida, nome)


def fig_torque(juntas, saida, limite=None):
    _mapa(juntas, 'command', 'Torque pedido pelo controlador, por junta',
          'torque [N·m]', 'fig_torque_juntas', saida, limite)


def fig_desvio(juntas, saida):
    juntas_deg = {j: {'t': v['t'], 'error': np.degrees(v['error'])}
                  for j, v in juntas.items()}
    _mapa(juntas_deg, 'error', 'Desvio de rastreamento, por junta',
          'erro [graus]', 'fig_desvio_juntas', saida)


def fig_pior_junta(juntas, saida, limite=None):
    """Ranking: qual junta pede mais torque e qual mais deixa de rastrear."""
    presentes = [j for j in PERNAS if j in juntas]
    if not presentes:
        return
    tq = [np.abs(juntas[j]['command']).max() for j in presentes]
    er = [np.degrees(np.abs(juntas[j]['error'])).max() for j in presentes]
    ordem = np.argsort(tq)
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(9.6, 4.4), sharey=True)
    y = np.arange(len(presentes))
    a1.barh(y, [tq[i] for i in ordem], color=SEM)
    if limite:
        a1.axvline(limite, color='black', lw=1.4, ls='--')
        a1.annotate('limite %.0f N·m' % limite, (limite, len(y) - 0.4),
                    fontsize=8.5, ha='right', rotation=90, va='top')
    a1.set_yticks(y)
    a1.set_yticklabels([presentes[i] for i in ordem], fontsize=8.5,
                       family='monospace')
    a1.set_xlabel('torque de pico pedido [N·m]')
    a1.set_title('Quem pede mais torque', loc='left', fontweight='bold')
    a2.barh(y, [er[i] for i in ordem], color=COM)
    a2.set_xlabel('desvio de pico [graus]')
    a2.set_title('Quem mais deixa de rastrear', loc='left', fontweight='bold')
    salvar(fig, saida, 'fig_ranking_juntas')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    pref = sys.argv[1]
    saida = sys.argv[2] if len(sys.argv) > 2 else 'docs/figuras'
    os.makedirs(saida, exist_ok=True)
    limite = float(os.environ.get('LIMITE_TORQUE', '0')) or None

    d = ler_telemetria(pref + '_telemetria.csv')
    juntas = ler_juntas(pref + '_juntas.csv')
    print('%d amostras de telemetria, %d juntas' % (len(d['t']), len(juntas)))
    print('figuras:')
    fig_paper5(d, saida)
    fig_paper4(d, saida)
    fig_energia(d, saida)
    fig_fase(d, saida)
    fig_torque(juntas, saida, limite)
    fig_desvio(juntas, saida)
    fig_pior_junta(juntas, saida, limite)
