#!/usr/bin/env python3
"""Figuras do panorama de parametros (docs/panorama_parametros.md).

Le panorama/resumo_corridas.csv (analisar_panorama.py) e gera em
docs/figuras_marcha/:
  fig_panorama_mapa.png    um painel por massa; z_com (linhas) x hip_offset_x
                           (colunas); cor = fracao das corridas que completaram
                           os 30 s; texto = completou/n e sobrevida media
  fig_panorama_pender.png  (a) pitch parado x massa; (b) pitch parado x
                           cedencia somada (quadril + joelho + tornozelo pitch)
                           parado - teste da hipotese H1
  fig_panorama_torque.png  torque que o joelho de apoio comanda na metade do 1o
                           apoio simples, Kp*(real - alvo), x massa e x z_com,
                           por desfecho, contra o stall (H4)
  fig_panorama_offset.png  fase E (+ braco -5 da fase B): pitch parado e
                           desfecho por hip_pitch_offset_deg; uma marca por
                           condicao (massa, celula, ganho)
                           (python3 fig_panorama.py <resumo> E)

  python3 fig_panorama.py [resumo_corridas.csv] [prefixos, ex. A_,C_] [ganho] [sufixo]
  (sem ganho: todas as figuras; com ganho: so o mapa daquele ganho)
"""
import csv
import os
import re
import statistics as st
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402

AQUI = os.path.dirname(os.path.abspath(__file__))
SAIDA = os.environ.get('FIG_SAIDA', os.path.join(AQUI, '..', '..', '..', 'docs', 'figuras_marcha'))
# massa total medida na telemetria (total_mass) no piloto: 0,5 -> 3,298;
# 0,67 -> 3,802; 1,0 -> 4,781 kg; linear na escala do tronco
MASSA = {0.35: 2.85, 0.5: 3.30, 0.59: 3.56, 0.67: 3.80, 0.75: 4.04, 0.83: 4.28, 1.0: 4.78}
AZUL = ['#cde2fb', '#b7d3f6', '#9ec5f4', '#86b6ef', '#6da7ec', '#5598e7', '#3987e5',
        '#2a78d6', '#256abf', '#1c5cab', '#184f95', '#104281', '#0d366b']
SUP, TXT, TXT2, MUDO, GRADE = '#fcfcfb', '#0b0b0b', '#52514e', '#898781', '#e1e0d9'
RE = re.compile(r'_m([\d.]+)_z([\d.]+)_x(-?[\d.]+)_g([\d.]+)')

plt.rcParams.update({
    'figure.facecolor': SUP, 'axes.facecolor': SUP, 'savefig.facecolor': SUP,
    'font.size': 9, 'axes.edgecolor': MUDO, 'axes.labelcolor': TXT2,
    'xtick.color': TXT2, 'ytick.color': TXT2, 'axes.titlecolor': TXT,
    'axes.spines.top': False, 'axes.spines.right': False,
})


def le(caminho, prefixo):
    out = []
    for r in csv.DictReader(open(caminho)):
        m = RE.search(r['base'])
        if not m or not r['base'].startswith(tuple(prefixo.split(','))):
            continue
        r['m'], r['z'], r['x'], r['g'] = (float(v) for v in m.groups())
        out.append(r)
    return out


def num(v):
    try:
        f = float(v)
    except (TypeError, ValueError):
        return float('nan')
    return f


def mapa(corridas, arq):
    ms = sorted({r['m'] for r in corridas})
    zs = sorted({r['z'] for r in corridas})
    xs = sorted({r['x'] for r in corridas})
    cmap = LinearSegmentedColormap.from_list('azul', AZUL)
    # ate 4 paineis por linha; paineis que sobram ficam vazios
    nc = min(4, len(ms))
    nl = (len(ms) + nc - 1) // nc
    fig, grade = plt.subplots(nl, nc, figsize=(2.3 * nc + 0.9, 3.0 * nl + 0.4), squeeze=False,
                              constrained_layout=True)
    axs = list(grade.flat)
    for ax in axs[len(ms):]:
        ax.set_visible(False)
    axs = axs[:len(ms)]
    for ax, m in zip(axs, ms):
        for i, z in enumerate(zs):
            for j, x in enumerate(xs):
                c = [r for r in corridas if (r['m'], r['z'], r['x']) == (m, z, x)]
                if not c:
                    ax.add_patch(plt.Rectangle((j - .5, i - .5), 1, 1, fc=SUP, ec=GRADE, lw=1))
                    ax.text(j, i, 'sem dado', ha='center', va='center', color=MUDO, fontsize=7)
                    continue
                k = sum(r['desfecho'] == 'completou' for r in c)
                f = k / len(c)
                # 2 px de superficie entre celulas: borda na cor do fundo
                ax.add_patch(plt.Rectangle((j - .5, i - .5), 1, 1, fc=cmap(f), ec=SUP, lw=2))
                cor = '#ffffff' if f > 0.55 else TXT
                sob = st.mean(num(r['sobrevida']) for r in c)
                ax.text(j, i + .13, '%d/%d' % (k, len(c)), ha='center', va='center', color=cor,
                        fontsize=9, fontweight='bold')
                ax.text(j, i - .2, '%.0f s' % sob, ha='center', va='center', color=cor, fontsize=7)
                # desfecho das quedas: letra da direcao (f/t/l) e 'a' se antes da marcha
                q = ''.join(('a' if r['desfecho'] == 'caiu_antes' else '') + r['direcao'][:1]
                            for r in c if r['desfecho'] != 'completou')
                if q:
                    ax.text(j, i - .36, q, ha='center', va='center', color=cor, fontsize=6)
        ax.set_xlim(-.5, len(xs) - .5)
        ax.set_ylim(-.5, len(zs) - .5)
        ax.set_xticks(range(len(xs)))
        ax.set_xticklabels(['%g' % (1000 * x) for x in xs])
        ax.set_yticks(range(len(zs)))
        ax.set_yticklabels(['%g' % (1000 * z) for z in zs])
        ax.set_title('%.2f kg (tronco ×%g)' % (MASSA.get(m, float('nan')), m), fontsize=9)
        ax.set_xlabel('hip_offset_x [mm]')
        ax.tick_params(length=0)
        for s in ax.spines.values():
            s.set_visible(False)
    for i, ax in enumerate(axs):
        if i % nc == 0:
            ax.set_ylabel('z_com [mm]')
        else:
            ax.set_yticklabels([])
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(0, 1))
    cb = fig.colorbar(sm, ax=axs, shrink=.8, pad=.02)
    cb.set_label('fração que completou 30 s', color=TXT2)
    cb.outline.set_visible(False)
    g = sorted({r['g'] for r in corridas})
    fig.suptitle('Corridas que completaram 30 s de marcha (MX-28, compensação ganho %s)'
                 % '/'.join('%g' % v for v in g), color=TXT, fontsize=10)
    fig.savefig(arq, dpi=160)
    print('->', arq)


def pender(corridas, arq):
    ms = sorted({r['m'] for r in corridas})
    # massa e ordenada: rampa sequencial, um passo por massa (do 300 ao 700)
    passos = [AZUL[int(round(4 + 8 * i / max(1, len(ms) - 1)))] for i in range(len(ms))]
    fig, (a, b) = plt.subplots(1, 2, figsize=(9, 3.6), constrained_layout=True)
    for m, cor in zip(ms, passos):
        c = [r for r in corridas if r['m'] == m and num(r['pitch_parado']) == num(r['pitch_parado'])]
        if not c:
            continue
        mk = MASSA.get(m, m)
        p = [num(r['pitch_parado']) for r in c]
        a.scatter([mk] * len(p), p, s=22, color=cor, edgecolor=SUP, lw=1, zorder=3)
        a.plot([mk - .12, mk + .12], [st.median(p)] * 2, color=TXT, lw=2, zorder=4)
        b.scatter([num(r['ced_soma_parado']) for r in c], p, s=22, color=cor, edgecolor=SUP, lw=1,
                  zorder=3, label='%.2f kg' % mk)
    for ax in (a, b):
        ax.axhline(0, color=GRADE, lw=1, zorder=1)
        ax.grid(axis='y', color=GRADE, lw=.6)
        ax.set_axisbelow(True)
    a.set_xlabel('massa total [kg]')
    a.set_ylabel('pitch do tronco parado [°]  (+ = para a frente)')
    a.set_title('(a) inclinação parada no fim do agachamento; traço = mediana', fontsize=9)
    b.set_xlabel('cedência parada [°]\nΣ(real − alvo) de quadril, joelho e tornozelo pitch')
    b.set_title('(b) inclinação × cedência dos servos (H1)', fontsize=9)
    # identidade cinematica (eixos pitch paralelos, pe plano): pitch = -soma(alvo) - cedencia
    al = st.mean(num(r['soma_alvo_parado']) for r in corridas)
    xs = [num(r['ced_soma_parado']) for r in corridas]
    b.plot([min(xs), max(xs)], [-al - min(xs), -al - max(xs)], color=MUDO, lw=1, ls=(0, (4, 3)), zorder=2)
    b.text(max(xs), -al, 'pitch = %.1f° − cedência\n(−Σalvo = %.1f°: o −5° no hip pitch)' % (-al, -al),
           ha='right', va='top', color=TXT2, fontsize=8)
    b.legend(frameon=False, fontsize=8, title='massa', title_fontsize=8)
    fig.savefig(arq, dpi=160)
    print('->', arq)


# desfecho e identidade: 3 primeiros slots categoricos (validados todos-os-pares)
# mais forma como codificacao secundaria
DESF = (('completou', 'completou 30 s', '#2a78d6', 'o'),
        ('caiu_marcha', 'caiu na marcha', '#eb6834', 's'),
        ('caiu_antes', 'caiu antes da marcha', '#1baf7a', '^'))


def torque(corridas, arq, stall=2.5):
    fig, (a, b) = plt.subplots(1, 2, figsize=(9, 3.6), constrained_layout=True, sharey=True)
    ms = sorted({r['m'] for r in corridas})
    zs = sorted({r['z'] for r in corridas})
    for k, rot, cor, mk in DESF:
        c = [r for r in corridas if r['desfecho'] == k and num(r['tau_joelho_1o']) == num(r['tau_joelho_1o'])]
        if not c:
            continue
        # deslocamento horizontal por desfecho, para nao sobrepor as nuvens
        d = {'completou': -1, 'caiu_marcha': 0, 'caiu_antes': 1}[k]
        a.scatter([MASSA[r['m']] + 0.07 * d for r in c], [num(r['tau_joelho_1o']) for r in c], s=22,
                  color=cor, marker=mk, edgecolor=SUP, lw=.8, zorder=3, label=rot)
        h = [r for r in c if r['m'] >= 0.67]
        b.scatter([1000 * r['z'] + 1.6 * d for r in h], [num(r['tau_joelho_1o']) for r in h], s=22,
                  color=cor, marker=mk, edgecolor=SUP, lw=.8, zorder=3)
    for ax in (a, b):
        ax.axhline(stall, color=TXT, lw=1, ls=(0, (4, 3)), zorder=2)
        ax.grid(axis='y', color=GRADE, lw=.6)
        ax.set_axisbelow(True)
    a.text(MASSA[ms[0]] - .2, stall + .08, 'limite do servo (%.1f N·m)' % stall, color=TXT2, fontsize=8)
    a.set_xticks([MASSA[m] for m in ms])
    a.set_xlabel('massa total [kg]')
    a.set_ylabel('torque comandado no joelho de apoio [N·m]\nKp·(real − alvo), metade do 1º apoio')
    a.set_title('(a) todas as corridas da fase A', fontsize=9)
    a.legend(frameon=False, fontsize=8, loc='upper left')
    b.set_xticks([1000 * z for z in zs])
    b.set_xlabel('z_com [mm]  (mais alto = joelho mais esticado)')
    b.set_title('(b) massas de 3,80 a 4,78 kg', fontsize=9)
    fig.savefig(arq, dpi=160)
    print('->', arq)


CEL_E = {('0.5', '0.250', '0.0036'), ('0.67', '0.239', '0.009'), ('0.83', '0.269', '0.009'),
         ('1.0', '0.260', '0.013'), ('0.5', '0.260', '0.0036'), ('0.67', '0.260', '0.0036'),
         ('0.83', '0.250', '0.013'), ('1.0', '0.269', '0.013')}


def offset(res, arq):
    def campo(tag, k):
        s = tag.split('_' + k)
        return s[1].split('_')[0] if len(s) > 1 else None
    cond = {}
    for r in csv.DictReader(open(res)):
        t = r['tag']
        if t[:2] not in ('B_', 'E_') or (campo(t, 'm'), campo(t, 'z'), campo(t, 'x')) not in CEL_E:
            continue
        o = float(campo(t, 'o') or -5)
        cond.setdefault((o, campo(t, 'm'), campo(t, 'z'), campo(t, 'g')), []).append(r)
    os_ = sorted({k[0] for k in cond}, reverse=True)
    fig, (a, b) = plt.subplots(1, 2, figsize=(9, 3.6), constrained_layout=True,
                               gridspec_kw={'width_ratios': [1.3, 1]})
    cont = {o: {k: 0 for k, *_ in DESF} for o in os_}
    for (o, m, z, g), c in sorted(cond.items()):
        # desfecho da condicao = o da maioria das corridas (empate: o pior)
        ordem = [k for k, *_ in DESF]
        d = max(ordem, key=lambda k: (sum(r['desfecho'] == k for r in c), ordem.index(k)))
        cont[o][d] += 1
        _, rot, cor, mk = next(x for x in DESF if x[0] == d)
        jit = (MASSA[float(m)] - 4.0) * 0.35 + (float(g) - 0.5) * 0.6
        a.scatter(o + jit, st.mean(num(r['pitch_parado']) for r in c), s=26, color=cor, marker=mk,
                  edgecolor=SUP, lw=.8, zorder=3)
    for k, rot, cor, mk in DESF:
        a.scatter([], [], color=cor, marker=mk, s=26, label=rot)
    a.axhline(0, color=GRADE, lw=1, zorder=1)
    a.grid(axis='y', color=GRADE, lw=.6)
    a.set_axisbelow(True)
    a.set_xticks(os_)
    a.set_xticklabels(['%g°' % o for o in os_])
    a.set_xlabel('hip_pitch_offset_deg  (−5 = valor fixo original)')
    a.set_ylabel('pitch do tronco parado [°]  (+ = para a frente)')
    a.set_title('(a) postura parada e desfecho; uma marca por condição', fontsize=9)
    a.legend(frameon=False, fontsize=8, loc='lower left')
    # (b) barras empilhadas: numero de condicoes por desfecho
    for i, o in enumerate(os_):
        base = 0
        for k, rot, cor, mk in DESF:
            v = cont[o][k]
            if v:
                b.barh(i, v, left=base, color=cor, edgecolor=SUP, lw=2, height=.6)
                b.text(base + v / 2, i, str(v), ha='center', va='center',
                       color='#ffffff' if k != 'caiu_antes' else TXT, fontsize=8)
            base += v
    b.set_yticks(range(len(os_)))
    b.set_yticklabels(['%g°' % o for o in os_])
    b.invert_yaxis()
    b.set_xlabel('condições (4 massas × 2 células × 3 ganhos)')
    b.set_title('(b) desfecho por offset', fontsize=9)
    for s in ('left',):
        b.spines[s].set_visible(False)
    b.tick_params(axis='y', length=0)
    fig.savefig(arq, dpi=160)
    print('->', arq)


def main():
    res = sys.argv[1] if len(sys.argv) > 1 else os.path.join(AQUI, 'panorama', 'resumo_corridas.csv')
    pre = sys.argv[2] if len(sys.argv) > 2 else 'A_'
    if pre == 'E':
        os.makedirs(SAIDA, exist_ok=True)
        return offset(res, os.path.join(SAIDA, 'fig_panorama_offset.png'))
    ganho = float(sys.argv[3]) if len(sys.argv) > 3 else None
    corridas = [r for r in le(res, pre) if ganho is None or r['g'] == ganho]
    if not corridas:
        sys.exit('nenhuma corrida com prefixo %s em %s' % (pre, res))
    os.makedirs(SAIDA, exist_ok=True)
    tag = sys.argv[4] if len(sys.argv) > 4 else ('' if pre == 'A_' else '_' + pre.replace(',', '').replace('_', ''))
    if ganho is not None:
        mapa(corridas, os.path.join(SAIDA, 'fig_panorama_mapa%s.png' % tag))
        return
    mapa(corridas, os.path.join(SAIDA, 'fig_panorama_mapa%s.png' % tag))
    pender(corridas, os.path.join(SAIDA, 'fig_panorama_pender%s.png' % tag))
    torque(corridas, os.path.join(SAIDA, 'fig_panorama_torque%s.png' % tag))


if __name__ == '__main__':
    main()
