#!/usr/bin/env python3
"""E5 da marcha omnidirecional: avaliacao confirmatoria do estabilizador de roll
(docs/marcha_omnidirecional.md).

Le omni/resumo_omni.csv, tags E5_<massa>_<braco>_<mov>_<pe>_r1, com braco
'off' ou a configuracao do roll. Os dois bracos foram INTERCALADOS (ordem
embaralhada na mesma campanha). Par = (massa, movimento, pe inicial): 44 pares.

Contrastes primarios, pre-registrados no plano, com Holm sobre os 3:
  1. sobrevivencia: teste do sinal sobre os pares em que so um completou
     (equivalente exato do McNemar);
  2. A_roll [graus]: Wilcoxon pareado, nos pares em que os dois bracos tem
     A_roll (janela >= 3 s); como as quedas encurtam a janela, mostra tambem
     o roll RMS;
  3. erro de seguimento: |w_real/w_cmd - 1| nos giros e curvas, e
     |erro_dir| [graus] nos movimentos com translacao, nos pares em que os
     dois bracos tem a metrica; as duas entram como testes separados
     (Wilcoxon) e o contraste 3 usa o pior p dos dois.

  python3 analisar_e5.py [omni] [prefixo E5_]
"""
import csv
import math
import os
import re
import sys
from math import comb

from scipy.stats import wilcoxon


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


def wil(a, b):
    if len(a) < 5 or all(x == y for x, y in zip(a, b)):
        return float('nan'), float('nan')
    d = sorted(x - y for x, y in zip(a, b))
    return d[len(d) // 2], wilcoxon(a, b).pvalue


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), 'omni')
    pref = sys.argv[2] if len(sys.argv) > 2 else 'E5_'
    P = {}
    for r in csv.DictReader(open(os.path.join(d, 'resumo_omni.csv'))):
        m = re.match(r'^%s(M\d+)_(\w+?)_((?:reta|giro|curva|lat|diag|comb)[+-]?[\d.]+)_(p[DE])_r\d+$' % pref, r['tag'])
        if m:
            massa, braco, mov, pe = m.groups()
            P.setdefault((massa, mov, pe), {})[braco == 'off' and 'off' or 'on'] = r
    pares = {k: v for k, v in P.items() if 'off' in v and 'on' in v}
    print('%d pares completos (de %d)\n' % (len(pares), len(P)))

    print('%-26s %-12s %-12s %7s %7s %7s %7s' % ('par', 'off', 'roll', 'Aoff', 'Aon', 'RMSoff', 'RMSon'))
    so_on = so_off = ambos = nenhum = 0
    A0, A1, R0, R1, W0, W1, D0, D1 = [], [], [], [], [], [], [], []
    for k in sorted(pares):
        o, n = pares[k]['off'], pares[k]['on']
        oo, no = o['desfecho'] == 'completou', n['desfecho'] == 'completou'
        so_on += no and not oo
        so_off += oo and not no
        ambos += oo and no
        nenhum += not (oo or no)
        ds = lambda r: 'ok' if r['desfecho'] == 'completou' else 'caiu %.1f' % f(r['sobrevida'])
        print('%-26s %-12s %-12s %7.2f %7.2f %7.2f %7.2f' % (
            ' '.join(k), ds(o), ds(n), f(o['a_roll']), f(n['a_roll']), f(o['roll_rms_o']), f(n['roll_rms_o'])))
        if not (math.isnan(f(o['a_roll'])) or math.isnan(f(n['a_roll']))):
            A0.append(f(o['a_roll']))
            A1.append(f(n['a_roll']))
        if not (math.isnan(f(o['roll_rms_o'])) or math.isnan(f(n['roll_rms_o']))):
            R0.append(f(o['roll_rms_o']))
            R1.append(f(n['roll_rms_o']))
        if not (math.isnan(f(o['razao_w'])) or math.isnan(f(n['razao_w']))):
            W0.append(abs(f(o['razao_w']) - 1))
            W1.append(abs(f(n['razao_w']) - 1))
        if not (math.isnan(f(o['erro_dir'])) or math.isnan(f(n['erro_dir']))):
            D0.append(abs(f(o['erro_dir'])))
            D1.append(abs(f(n['erro_dir'])))

    p1 = sinal(so_on, so_off)
    dA, p2 = wil(A1, A0)
    dR, pR = wil(R1, R0)
    dW, pW = wil(W1, W0)
    dD, pD = wil(D1, D0)
    p3 = max(x for x in (pW, pD) if not math.isnan(x)) if not (math.isnan(pW) and math.isnan(pD)) else float('nan')
    adj = holm([p1, p2 if not math.isnan(p2) else 1.0, p3 if not math.isnan(p3) else 1.0])
    tot_off = sum(v['off']['desfecho'] == 'completou' for v in pares.values())
    tot_on = sum(v['on']['desfecho'] == 'completou' for v in pares.values())
    print('\ncompletou: off %d/%d, roll %d/%d' % (tot_off, len(pares), tot_on, len(pares)))
    print('1. sobrevivencia: so roll %d, so off %d, ambos %d, nenhum %d; sinal p = %.2g; Holm %.2g'
          % (so_on, so_off, ambos, nenhum, p1, adj[0]))
    print('2. A_roll: n %d, mediana(roll - off) %+.2f graus, p = %.2g; Holm %.2g'
          % (len(A0), dA, p2, adj[1]))
    print('   roll RMS (secundaria): n %d, mediana(roll - off) %+.2f graus, p = %.2g' % (len(R0), dR, pR))
    print('3. |w/w_cmd - 1|: n %d, mediana(roll - off) %+.3f, p = %.2g' % (len(W0), dW, pW))
    print('   |erro_dir|:     n %d, mediana(roll - off) %+.2f graus, p = %.2g' % (len(D0), dD, pD))
    print('   contraste 3 (pior p) = %.2g; Holm %.2g' % (p3, adj[2]))
    print('\nPor massa e movimento (completou off / roll):')
    tipos = {}
    for (massa, mov, pe), v in pares.items():
        t = (massa, re.sub(r'[+-]', '±', mov))
        x = tipos.setdefault(t, [0, 0, 0])
        x[0] += v['off']['desfecho'] == 'completou'
        x[1] += v['on']['desfecho'] == 'completou'
        x[2] += 1
    for t in sorted(tipos):
        print('  %s %-14s %d/%d  ->  %d/%d' % (t[0], t[1], tipos[t][0], tipos[t][2], tipos[t][1], tipos[t][2]))


if __name__ == '__main__':
    main()
