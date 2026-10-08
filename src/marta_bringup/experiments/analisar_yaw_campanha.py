#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Analise da campanha E2 (rodar_yaw.sh): responde as perguntas de decisao do
plano de investigacao do desvio de direcao.

    python3 analisar_yaw_campanha.py [dir_campanha]     (padrao: yaw/)

Le <dir>/campanha_yaw.csv (uma linha por corrida, do walk_test.py) e
<dir>/dados/<tag>_telemetria.csv (serie temporal). Corridas com RTF < 0,85,
com queda ou com fisica divergida ficam FORA - filtro obrigatorio (README,
achado H).

Perguntas:
  1. giro ou caranguejo?        psi_final x caranguejo (rumo - yaw por passada)
  2. sistematico ou aleatorio?  teste do sinal entre repeticoes; media e DP de
                                psi(t) em funcao de t (deriva: media ~ t;
                                passeio aleatorio: DP ~ sqrt(t))
  3. a saturacao importa?       Mann-Whitney em |psi_taxa|, tau=6 x tau=1000
  4. o pe segura yaw?           giro do pe de apoio por janela de apoio simples
                                (mediana), e quanto o pe sobe no balanco
  controle positivo: foot_mu=0,5 deve AUMENTAR o giro do pe de apoio; se nao
                                aumentar, a regua do pe nao detecta escorregamento
"""
import csv
import os
import sys

import numpy as np
from scipy import stats

RAD2DEG = 180.0 / np.pi
RTF_MIN = 0.85
INSTANTES = [5, 10, 15, 20, 25, 30]


def carregar(dir_c):
    linhas = list(csv.DictReader(open(os.path.join(dir_c, 'campanha_yaw.csv'))))
    validas, descartadas = [], []
    for l in linhas:
        rtf = float(l['rtf']) if l['rtf'] else 0.0
        if rtf < RTF_MIN or l['caiu'] == '1' or l['divergiu'] == '1':
            descartadas.append((l['tag'], rtf, l['caiu']))
        else:
            validas.append(l)
    return validas, descartadas


def condicao(l):
    # o rosparam devolve 1e10 como 10000000000.0; %g normaliza. A geometria do
    # pe nao e coluna (mudaria o cabecalho da campanha): vem do tag.
    return 'tau=%g mu=%g pe=%s stab=%s' % (float(l['tau']), float(l['foot_mu']),
                                           'caixa' if '_caixa' in l['tag'] else 'malha',
                                           l['stab'])


def serie(dir_c, tag):
    p = os.path.join(dir_c, 'dados', tag + '_telemetria.csv')
    if not os.path.exists(p):
        return None
    R = list(csv.DictReader(open(p)))
    return {k: np.array([float(r[k]) for r in R])
            for k in ('t', 'yaw_true', 'l_foot_yaw', 'r_foot_yaw', 'l_foot_z',
                      'r_foot_z', 'support')}


def giro_pe_apoio(T):
    """Mediana do giro do pe de apoio por janela de apoio simples [graus]."""
    sup = T['support'].astype(int)
    out = {}
    for lado, code in (('l', 1), ('r', 2)):
        yaw = np.unwrap(T[lado + '_foot_yaw']) * RAD2DEG
        g, i = [], None
        for k, v in enumerate(sup):
            if v == code and i is None:
                i = k
            elif v != code and i is not None:
                g.append(yaw[k] - yaw[i])
                i = None
        out[lado] = np.median(g) if g else np.nan
    return out


def fracao_duplo(T):
    """Fracao do giro liquido do corpo que acontece no apoio duplo planejado."""
    d = np.diff(np.unwrap(T['yaw_true']))
    sup = T['support'].astype(int)[:-1]
    tot = d.sum()
    return d[sup == 0].sum() / tot if abs(tot) > 1e-6 else np.nan


def main(dir_c):
    validas, descartadas = carregar(dir_c)
    print('corridas validas: %d; descartadas: %d %s' % (
        len(validas), len(descartadas),
        ', '.join('%s(rtf=%.2f,caiu=%s)' % d for d in descartadas)))

    grupos = {}
    for l in validas:
        grupos.setdefault(condicao(l), []).append(l)

    taxas, pitch_rms = {}, {}
    for nome in sorted(grupos):
        G = grupos[nome]
        f = lambda c: np.array([float(l[c]) for l in G])
        psi, taxa, car = f('psi_final'), f('psi_taxa'), f('caranguejo')
        av, lat = f('avanco'), f('lateral')
        taxas[nome] = taxa
        pitch_rms[nome] = f('rms_pitch_v')
        n_pos = int((psi > 0).sum())
        p_sinal = stats.binomtest(n_pos, len(psi), 0.5).pvalue
        print('\n== %s  (n=%d)' % (nome, len(G)))
        print('  1. giro final %+.1f +- %.1f graus | caranguejo %+.1f +- %.1f graus'
              % (psi.mean(), psi.std(ddof=1) if len(psi) > 1 else 0, car.mean(),
                 car.std(ddof=1) if len(car) > 1 else 0))
        print('     avanco %.2f m, lateral %+.2f m (frame do rumo inicial)'
              % (av.mean(), lat.mean()))
        print('  2. sinal: %d/%d para a esquerda (teste do sinal p=%.3g); taxa %+.2f +- %.2f graus/s'
              % (n_pos, len(psi), p_sinal, taxa.mean(),
                 taxa.std(ddof=1) if len(taxa) > 1 else 0))

        series = [serie(dir_c, l['tag']) for l in G]
        series = [s for s in series if s is not None]
        if not series:
            continue
        # psi(t) relativo ao inicio da gravacao, em instantes fixos
        M = []
        for T in series:
            t = T['t'] - T['t'][0]
            psi_t = (np.unwrap(T['yaw_true']) - np.unwrap(T['yaw_true'])[0]) * RAD2DEG
            M.append([np.interp(x, t, psi_t) if x <= t[-1] else np.nan for x in INSTANTES])
        M = np.array(M)
        med, dp = np.nanmean(M, 0), np.nanstd(M, 0, ddof=1) if len(M) > 1 else np.zeros(len(INSTANTES))
        print('     psi(t) media: ' + '  '.join('%ds %+6.1f' % (x, m) for x, m in zip(INSTANTES, med)))
        print('     psi(t) DP   : ' + '  '.join('%ds %6.1f' % (x, s) for x, s in zip(INSTANTES, dp)))

        gp = [giro_pe_apoio(T) for T in series]
        subida = [100 * (np.percentile(T[k], 95) - np.percentile(T[k], 5))
                  for T in series for k in ('l_foot_z', 'r_foot_z')]
        fd = [fracao_duplo(T) for T in series]
        print('  4. pe de apoio gira por apoio simples (mediana das corridas): '
              'esq %+.2f, dir %+.2f graus'
              % (np.nanmedian([g['l'] for g in gp]), np.nanmedian([g['r'] for g in gp])))
        print('     subida do pe no balanco %.1f cm (zStep = 6 cm); '
              'fracao do giro liquido no apoio duplo %.0f%%'
              % (np.mean(subida), 100 * np.nanmedian(fd)))

    a, b = 'tau=1000 mu=1e+10 pe=malha stab=0', 'tau=6 mu=1e+10 pe=malha stab=0'
    if a in taxas and b in taxas:
        u = stats.mannwhitneyu(np.abs(taxas[b]), np.abs(taxas[a]))
        print('\n3. saturacao: |taxa| tau=6 %.2f x tau=1000 %.2f graus/s '
              '(razao %.2f, Mann-Whitney p=%.3g)'
              % (np.abs(taxas[b]).mean(), np.abs(taxas[a]).mean(),
                 np.abs(taxas[b]).mean() / np.abs(taxas[a]).mean(), u.pvalue))
    c = 'tau=1000 mu=1e+10 pe=caixa stab=0'
    if a in taxas and c in taxas:
        # Com sinal: a pergunta e se o giro SISTEMATICO para a esquerda some.
        u = stats.mannwhitneyu(taxas[c], taxas[a])
        print('E3 caixa x malha (tau=1000): taxa %+.2f x %+.2f graus/s '
              '(Mann-Whitney com sinal p=%.3g)'
              % (taxas[c].mean(), taxas[a].mean(), u.pvalue))


    # Linha de base nova (pe em caixa, tau=6): o estabilizador de pitch ainda
    # ganha? Referencia antiga (malha, 20 s): 5,54 -> 3,18 graus (-43%).
    off, on = 'tau=6 mu=1e+10 pe=caixa stab=0', 'tau=6 mu=1e+10 pe=caixa stab=1'
    if off in pitch_rms and on in pitch_rms:
        u = stats.mannwhitneyu(pitch_rms[on], pitch_rms[off])
        print('caixa tau=6: pitch RMS real sem %.2f -> com %.2f graus (%+.0f%%, '
              'Mann-Whitney p=%.3g)' % (pitch_rms[off].mean(), pitch_rms[on].mean(),
                                       100 * (pitch_rms[on].mean() / pitch_rms[off].mean() - 1),
                                       u.pvalue))
    d = 'tau=6 mu=1e+10 pe=malha stab=0'
    if d in taxas and off in taxas:
        u = stats.mannwhitneyu(taxas[off], taxas[d])
        print('caixa x malha (tau=6): taxa %+.2f x %+.2f graus/s (p=%.3g)'
              % (taxas[off].mean(), taxas[d].mean(), u.pvalue))


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else
         os.path.join(os.path.dirname(os.path.abspath(__file__)), 'yaw'))
