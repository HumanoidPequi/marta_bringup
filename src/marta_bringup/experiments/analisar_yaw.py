#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Decomposicao do desvio de direcao por passo: o giro entra pelo CHAO ou pela
PERNA?  So numpy; roda em qualquer lugar sobre os CSV do bag_para_csv.py.

    python3 analisar_yaw.py <prefixo> [<prefixo> ...]

<prefixo> e o caminho sem o sufixo: le <prefixo>_telemetria.csv e
<prefixo>_juntas.csv.

A identidade usada
------------------
Numerando os apoios em ordem (o pe k pousa, fica no chao, o pe k+1 pousa, ...),
com psi_in(k) o yaw do pe k no instante em que ele toca o chao:

    psi_in(k+1) - psi_in(k) = s_k + r_k        (exato, telescopico)

    s_k = psi_k(toque de k+1) - psi_in(k)
          giro do pe k EM CONTATO com o chao entre o seu toque e o toque do
          proximo pe. Com o pe parado no chao isso seria zero: e escorregamento
          rotacional (hipotese H1).
    r_k = psi_in(k+1) - psi_k(toque de k+1)
          yaw relativo com que o pe novo pousa em relacao ao que esta no chao.
          O gerador comanda zero; o que sobra vem da cadeia de juntas
          (cedencia da hip_yaw, hipotese H2) ou de giro no ar.

Somando, o giro das pegadas fica separado em "chao" (sum s) e "perna" (sum r).
A predicao cinematica de r_k usa so as hip_yaw (eixo -z nas duas pernas, entao
psi_pe ~ psi_tronco - q):  r_hat_k = -(q_novo - q_antigo) no toque. Pitch e
roll das outras juntas contribuem so em segunda ordem.

Deteccao de contato: pe abaixo de uma altura E com o ponto de contato quase
parado no plano. Os limiares sao arbitrarios; por isso a decomposicao e
repetida com tres combinacoes e a conclusao so vale se nao mudar entre elas.

AVISO (validacao de 2026-10-07, tau=6): a deteccao por contato NAO e confiavel
nesse regime - o pe em balanco sobe so ~1,8 cm (contra zStep = 6 cm comandado)
e se arrasta, entao os limiares partem e perdem apoios (9 a 102 apoios para
~80 reais). A contabilidade POR FASE PLANEJADA, abaixo, nao depende de
detectar contato e e a regua principal; a decomposicao s/r fica para o regime
em que o pe sai do chao de verdade.

Contabilidade por fase planejada
--------------------------------
O giro de cada pe e do corpo e somado por fase do plano (apoio simples
esquerdo, direito, duplo), por diferenca de ANGULO amostra a amostra. As
parcelas somam exatamente o giro total. "Pe de apoio girou X graus por janela
de apoio simples" e a medida direta de H1: com o pe preso, seria zero.
"""
import csv
import sys

import numpy as np

RAD2DEG = 180.0 / np.pi
# (folga de altura acima do pe mais baixo [m], velocidade horizontal max [m/s])
LIMIARES = [(0.004, 0.03), (0.006, 0.05), (0.003, 0.02)]
MIN_APOIO_S = 0.08   # contato mais curto que isso e quique, nao apoio


def ler_telemetria(prefixo):
    linhas = list(csv.DictReader(open(prefixo + '_telemetria.csv')))
    cols = ['t', 'yaw_true', 'l_foot_yaw', 'r_foot_yaw', 'l_foot_x', 'l_foot_y',
            'l_foot_z', 'r_foot_x', 'r_foot_y', 'r_foot_z', 'support']
    if 'l_foot_yaw' not in linhas[0]:
        sys.exit('%s: bag anterior a instrumentacao de yaw' % prefixo)
    return {c: np.array([float(l[c]) for l in linhas]) for c in cols}


def ler_hip_yaw(prefixo):
    q = {}
    for l in csv.DictReader(open(prefixo + '_juntas.csv')):
        if l['junta'] in ('l_hip_yaw', 'r_hip_yaw'):
            q.setdefault(l['junta'], []).append((float(l['t']), float(l['process_value'])))
    return {k: np.array(v) for k, v in q.items()}


def em_contato(T, lado, folga, vmax):
    t = T['t']
    x, y, z = T[lado + '_foot_x'], T[lado + '_foot_y'], T[lado + '_foot_z']
    v = np.hypot(np.gradient(x, t), np.gradient(y, t))
    chao = np.percentile(z, 2)
    return (z < chao + folga) & (v < vmax)


def intervalos(mascara, t):
    """Trechos contiguos verdadeiros, como (i_inicio, i_fim) inclusivos."""
    out, i = [], None
    for k, m in enumerate(mascara):
        if m and i is None:
            i = k
        elif not m and i is not None:
            out.append((i, k - 1))
            i = None
    if i is not None:
        out.append((i, len(mascara) - 1))
    return [(a, b) for a, b in out if t[b] - t[a] >= MIN_APOIO_S]


def decompor(T, Q, folga, vmax):
    t = T['t']
    yaw = {lado: np.unwrap(T[lado + '_foot_yaw']) for lado in ('l', 'r')}
    apoios = []
    for lado in ('l', 'r'):
        for a, b in intervalos(em_contato(T, lado, folga, vmax), t):
            apoios.append((t[a], lado, a, b))
    apoios.sort()

    # So interessa a alternancia: dois apoios seguidos do mesmo pe (o pe
    # levantou pouco e a deteccao partiu um apoio) viram um so.
    seq = []
    for ap in apoios:
        if seq and seq[-1][1] == ap[1]:
            seq[-1] = (seq[-1][0], ap[1], seq[-1][2], ap[3])
        else:
            seq.append(ap)

    s, r, r_hat = [], [], []
    for (t_k, lk, ak, bk), (t_n, ln, an, bn) in zip(seq, seq[1:]):
        psi_in_k = yaw[lk][ak]
        psi_k_no_toque = np.interp(t_n, t, yaw[lk])
        psi_in_n = yaw[ln][an]
        s.append(psi_k_no_toque - psi_in_k)
        r.append(psi_in_n - psi_k_no_toque)
        if Q:
            q = {lado: np.interp(t_n, Q[lado + '_hip_yaw'][:, 0], Q[lado + '_hip_yaw'][:, 1])
                 for lado in ('l', 'r')}
            r_hat.append(-(q[ln] - q[lk]))
    return seq, np.array(s) * RAD2DEG, np.array(r) * RAD2DEG, np.array(r_hat) * RAD2DEG


def por_fase(T):
    """Giro do corpo e de cada pe somado por fase planejada (ver docstring)."""
    sup = T['support'].astype(int)[:-1]
    d = {nome: np.diff(np.unwrap(T[col])) * RAD2DEG
         for nome, col in (('corpo', 'yaw_true'), ('l', 'l_foot_yaw'), ('r', 'r_foot_yaw'))}
    nomes = {0: 'duplo', 1: 'apoio esq', 2: 'apoio dir'}
    print('  contabilidade por fase planejada [graus]:')
    print('    %-8s %9s %11s %11s %11s' % ('', 'total', nomes[1], nomes[2], nomes[0]))
    for k in ('corpo', 'l', 'r'):
        print('    %-8s %+9.1f %+11.1f %+11.1f %+11.1f'
              % ({'l': 'pe esq', 'r': 'pe dir'}.get(k, k), d[k].sum(),
                 d[k][sup == 1].sum(), d[k][sup == 2].sum(), d[k][sup == 0].sum()))

    # Giro do pe de APOIO em cada janela de apoio simples: medida direta de H1.
    sup_full = T['support'].astype(int)
    for lado, code, nome in (('l', 1, 'esq'), ('r', 2, 'dir')):
        yaw = np.unwrap(T[lado + '_foot_yaw']) * RAD2DEG
        giros, i = [], None
        for k, v in enumerate(sup_full):
            if v == code and i is None:
                i = k
            elif v != code and i is not None:
                giros.append(yaw[k] - yaw[i])
                i = None
        g = np.array(giros)
        if len(g):
            print('  pe %s em apoio simples: %d janelas, giro por janela mediana %+.2f '
                  'media %+.2f dp %.2f graus' % (nome, len(g), np.median(g), g.mean(), g.std()))


def relatorio(prefixo):
    T = ler_telemetria(prefixo)
    try:
        Q = ler_hip_yaw(prefixo)
    except FileNotFoundError:
        Q = {}
    t = T['t']
    corpo = np.unwrap(T['yaw_true']) * RAD2DEG
    print('=' * 72)
    print(prefixo)
    print('  corpo: %+.1f graus em %.1f s (inicio da gravacao %+.1f graus)'
          % (corpo[-1] - corpo[0], t[-1] - t[0], corpo[0]))
    for lado in ('l', 'r'):
        z = T[lado + '_foot_z']
        print('  pe %s: altura do contato p5 %.4f / p95 %.4f m  (subida ~ %.1f cm; zStep = 6 cm)'
              % (lado, np.percentile(z, 5), np.percentile(z, 95),
                 100 * (np.percentile(z, 95) - np.percentile(z, 5))))
    por_fase(T)

    print('\n  decomposicao s/r por contato detectado (ver AVISO no cabecalho):')
    for folga, vmax in LIMIARES:
        seq, s, r, r_hat = decompor(T, Q, folga, vmax)
        if len(s) < 4:
            print('  limiar (%.3f m, %.2f m/s): so %d apoios - deteccao falhou'
                  % (folga, vmax, len(seq)))
            continue
        total = s.sum() + r.sum()
        duracao = seq[-1][0] - seq[0][0]
        print('\n  limiar folga=%.3f m  v<%.2f m/s:  %d apoios em %.1f s (%.2f s/apoio)'
              % (folga, vmax, len(seq), duracao, duracao / max(1, len(seq) - 1)))
        print('    pegadas giraram %+7.1f graus  =  chao (sum s) %+7.1f  +  perna (sum r) %+7.1f'
              % (total, s.sum(), r.sum()))
        print('    fracao do giro liquido pelo chao: %.0f%%'
              % (100.0 * s.sum() / total if abs(total) > 1e-9 else float('nan')))
        print('    por apoio: s media %+.2f dp %.2f | r media %+.2f dp %.2f graus'
              % (s.mean(), s.std(), r.mean(), r.std()))
        # Por pe que esta no chao: o giro pelo chao e simetrico?
        for lado, nome in (('l', 'esq'), ('r', 'dir')):
            m = np.array([ap[1] == lado for ap in seq[:-1]])
            if m.any():
                print('      pe %s no chao: s media %+.2f (n=%d)  r media %+.2f'
                      % (nome, s[m].mean(), m.sum(), r[m].mean()))
        if len(r_hat) == len(r) and len(r) > 2:
            a, b = np.polyfit(r_hat, r, 1)
            rho = np.corrcoef(r_hat, r)[0, 1]
            print('    perna explicada pelas hip_yaw: r = %.2f*r_hat %+.2f  (R2 = %.2f);'
                  ' sum r_hat %+.1f graus' % (a, b, rho ** 2, r_hat.sum()))


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    for p in sys.argv[1:]:
        relatorio(p)
