#!/usr/bin/env python3
"""E2 da marcha omnidirecional: os ganhos de pitch e de rumo estao bem
sintonizados em toda a faixa testada? (docs/marcha_omnidirecional.md)

Le omni/resumo_omni.csv (analisar_omni.py) e os _telemetria.csv /
_estabilizador.csv em omni/dados. Tags:
  E2p_<massa>_<mov>_kp<kp>_kd<kd>      familia do pitch (atual: kp0.2 kd0.03)
  E2h_<massa>_<mov>_kh<kh>_sat<sat>    familia do rumo  (atual: kh1.0 sat0.3)

Uma observacao por condicao (massa x movimento): a comparacao de cada
combinacao contra a atual e PAREADA por condicao (8 no pitch, 10 no rumo).

Metricas por corrida:
  sob        sobrevida [s], 30 se completou (desfecho principal)
  pitch_rms  pitch verdadeiro RMS na marcha [graus] (do resumo)
  rumo_rms   RMS do erro de rumo VERDADEIRO na janela [3 s, fim]:
             yaw_true desenrolado - (yaw(3 s) + w_cmd (t - 3 s)) [graus]
  razao_w    w realizado / comandado (do resumo; so com w_cmd != 0)

Testes (por combinacao contra a atual, pareados por condicao):
  sobrevida: teste do sinal sobre os pares que diferem (> 0,5 s);
  pitch_rms ou rumo_rms: Wilcoxon pareado nos pares em que as DUAS
  completaram (senao a metrica mede tempos diferentes).
  Holm sobre as combinacoes de cada familia, por metrica.

Criterio pre-registrado (plano): os ganhos atuais estao "bem sintonizados" se
nenhuma combinacao os supera com Holm p < 0,05 em uma metrica sem piorar
as outras.

  python3 analisar_e2.py [omni]
"""
import csv
import math
import os
import re
import sys
from math import comb

from scipy.stats import wilcoxon

DUR = 30.0
T0 = 3.0


def f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return float('nan')


def sinal(a, b):
    n, k = a + b, min(a, b)
    return min(1.0, 2 * sum(comb(n, i) for i in range(k + 1)) / 2 ** n) if n else 1.0


def holm(ps):
    ordem = sorted(range(len(ps)), key=lambda i: ps[i])
    adj, m, acc = [1.0] * len(ps), len(ps), 0.0
    for r, i in enumerate(ordem):
        acc = max(acc, min(1.0, (m - r) * ps[i]))
        adj[i] = acc
    return adj


def rumo_rms(pre, w_cmd, t_fim):
    """RMS do erro de rumo verdadeiro em [T0, t_fim] [graus]."""
    try:
        d = list(csv.DictReader(open(pre + '_telemetria.csv')))
    except OSError:
        return float('nan')
    t, y, prev, off = [], [], None, 0.0
    for r in d:
        ti, yi = f(r['t']), f(r['yaw_true'])
        if math.isnan(yi):
            continue
        if prev is not None:
            dy = yi - prev
            if dy > math.pi:
                off -= 2 * math.pi
            elif dy < -math.pi:
                off += 2 * math.pi
        prev = yi
        if T0 <= ti <= t_fim:
            t.append(ti)
            y.append(yi + off)
    if len(t) < 50:
        return float('nan')
    e = [yi - (y[0] + w_cmd * (ti - t[0])) for ti, yi in zip(t, y)]
    return math.degrees(math.sqrt(sum(v * v for v in e) / len(e)))


def le(d):
    out = {}
    for r in csv.DictReader(open(os.path.join(d, 'resumo_omni.csv'))):
        m = re.match(r'^(E2[ph])_(M\d+)_(\S+?)_(k[ph]\S+)_r\d+$', r['tag'])
        if not m:
            continue
        fam, massa, mov, combo = m.groups()
        ok = r['desfecho'] == 'completou'
        sob = DUR if ok else f(r['sobrevida'])
        t_fim = DUR if ok else max(T0, sob - 0.5)
        pre = os.path.join(d, 'dados', r['tag'])
        out[(fam, combo, massa, mov)] = {
            'ok': ok, 'sob': sob, 'pitch_rms': f(r['pitch_rms']),
            'razao_w': f(r['razao_w']),
            'rumo_rms': rumo_rms(pre, f(r['w_cmd']), t_fim) if t_fim - T0 >= 3.0 else float('nan')}
    return out


def familia(dados, fam, atual, metrica):
    combos = sorted({k[1] for k in dados if k[0] == fam})
    conds = sorted({(k[2], k[3]) for k in dados if k[0] == fam})
    print('\n=== %s (atual %s; metrica secundaria %s) ===' % (fam, atual, metrica))
    print('condicoes:', ', '.join('%s %s' % c for c in conds))
    linhas = ['%-18s' % 'condicao' + ''.join('%14s' % c for c in combos)]
    for c in conds:
        cel = []
        for cb in combos:
            x = dados.get((fam, cb, c[0], c[1]))
            if x is None:
                cel.append('%14s' % '-')
            else:
                s = 'ok' if x['ok'] else '%.1f' % x['sob']
                v = x[metrica]
                cel.append('%14s' % ('%s|%.2f' % (s, v) if not math.isnan(v) else s))
        linhas.append('%-18s' % ('%s %s' % c) + ''.join(cel))
    print('\n'.join(linhas))
    res = []
    for cb in combos:
        if cb == atual:
            continue
        melhor = pior = 0
        da, db = [], []
        for c in conds:
            x, a = dados.get((fam, cb, c[0], c[1])), dados.get((fam, atual, c[0], c[1]))
            if x is None or a is None:
                continue
            if x['sob'] > a['sob'] + 0.5:
                melhor += 1
            elif a['sob'] > x['sob'] + 0.5:
                pior += 1
            if x['ok'] and a['ok'] and not (math.isnan(x[metrica]) or math.isnan(a[metrica])):
                da.append(x[metrica])
                db.append(a[metrica])
        p_s = sinal(melhor, pior)
        if len(da) >= 3 and any(u != v for u, v in zip(da, db)):
            p_m = wilcoxon(da, db).pvalue
            dif = sorted(u - v for u, v in zip(da, db))[len(da) // 2]
        else:
            p_m, dif = 1.0, float('nan')
        sob_tot = sum(dados[(fam, cb, c[0], c[1])]['ok'] for c in conds if (fam, cb, c[0], c[1]) in dados)
        res.append([cb, sob_tot, melhor, pior, p_s, len(da), dif, p_m])
    adj_s = holm([r[4] for r in res])
    adj_m = holm([r[7] for r in res])
    a_tot = sum(dados[(fam, atual, c[0], c[1])]['ok'] for c in conds if (fam, atual, c[0], c[1]) in dados)
    print('\natual %s: completou %d/%d' % (atual, a_tot, len(conds)))
    print('%-14s %5s %6s %5s %8s %8s %4s %9s %8s %8s' % ('combinacao', 'compl', 'melhor', 'pior', 'p_sinal', 'Holm', 'n', 'dif_med', 'p_wilc', 'Holm'))
    for r, hs, hm in zip(res, adj_s, adj_m):
        print('%-14s %5d %6d %5d %8.4f %8.4f %4d %9.3f %8.4f %8.4f' % (r[0], r[1], r[2], r[3], r[4], hs, r[5], r[6], r[7], hm))
    return res


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), 'omni')
    dados = le(d)
    if not dados:
        sys.exit('nenhuma corrida E2 em %s' % d)
    familia(dados, 'E2p', 'kp0.2_kd0.03', 'pitch_rms')
    familia(dados, 'E2h', 'kh1.0_sat0.3', 'rumo_rms')


if __name__ == '__main__':
    main()
