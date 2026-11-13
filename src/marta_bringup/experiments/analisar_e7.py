#!/usr/bin/env python3
"""E7 da marcha omnidirecional: kd de pitch e hip_offset_x com roll quatorn K1,
no campo livre (docs/marcha_omnidirecional.md, secao E7; criterio pre-registrado).

Le omni/resumo_omni.csv, tags E7_<massa>_<braco>_<mov>_<pe>_r1, braco em
ref | kd06 | x5 | kd06x5. Par = (massa, movimento, pe inicial): 44 pares.

Cada braco contra ref:
  primarias (Holm sobre 3 bracos x 2 metricas = 6 testes)
    1. sobrevivencia: teste do sinal nos pares discordantes;
    2. pitch_rms [graus]: Wilcoxon nos pares em que os dois completam;
  vetos (mediana das diferencas pareadas, braco - ref)
    roll RMS > +10 %; |erro_dir| > +2 graus; |w/w_cmd - 1| > +0,02;
    sep_min_mm < 90 em alguma corrida;
  secundaria: oscilacao de pitch, sqrt(rms^2 - media^2).
Regra: adota se nao completa menos que ref, melhora uma primaria com
Holm p < 0,05 e nao aciona veto; se varios passam, o de menor pitch_rms;
se nenhum passa, fica ref.

  python3 analisar_e7.py [omni]
"""
import csv
import math
import os
import re
import sys

from analisar_e5 import f, holm, sinal, wil

BRACOS = ('kd06', 'x5', 'kd06x5')


def med(xs):
    xs = sorted(xs)
    n = len(xs)
    if not n:
        return float('nan')
    return xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])


def ok(r):
    return r['desfecho'] == 'completou'


def osc(r):
    rms, m = f(r['pitch_rms']), f(r['pitch_marcha'])
    return math.sqrt(max(rms * rms - m * m, 0.0))


def pareado(pares, b, chave, cond=lambda o, n: True):
    a0, a1 = [], []
    for v in pares.values():
        o, n = v['ref'], v[b]
        x0, x1 = chave(o), chave(n)
        if cond(o, n) and not (math.isnan(x0) or math.isnan(x1)):
            a0.append(x0)
            a1.append(x1)
    return a0, a1


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'omni')
    P = {}
    for r in csv.DictReader(open(os.path.join(d, 'resumo_omni.csv'))):
        m = re.match(r'^E7_(M\d+)_(ref|kd06|x5|kd06x5)_((?:reta|giro|curva|lat|diag|comb)'
                     r'[+-]?[\d.]+)_(p[DE])_r\d+$', r['tag'])
        if m:
            massa, braco, mov, pe = m.groups()
            P.setdefault((massa, mov, pe), {})[braco] = r
    pares = {k: v for k, v in P.items() if all(b in v for b in ('ref',) + BRACOS)}
    print('%d pares completos (de %d)\n' % (len(pares), len(P)))

    ds = lambda r: 'ok' if ok(r) else 'caiu %.1f' % f(r['sobrevida'])
    print('%-24s %-10s %-10s %-10s %-10s' % (('par',) + ('ref',) + BRACOS))
    for k in sorted(pares):
        print('%-24s %-10s %-10s %-10s %-10s' % ((' '.join(k),) + tuple(
            ds(pares[k][b]) for b in ('ref',) + BRACOS)))

    res, ps = {}, []
    for b in BRACOS:
        so_b = sum(ok(v[b]) and not ok(v['ref']) for v in pares.values())
        so_r = sum(ok(v['ref']) and not ok(v[b]) for v in pares.values())
        p1 = sinal(so_b, so_r)
        ambos = lambda o, n: ok(o) and ok(n)
        P0, P1 = pareado(pares, b, lambda r: f(r['pitch_rms']), ambos)
        dP, p2 = wil(P1, P0)
        res[b] = dict(so_b=so_b, so_r=so_r, p1=p1, nP=len(P0), dP=dP, p2=p2, P0=P0, P1=P1)
        ps += [p1, 1.0 if math.isnan(p2) else p2]
    adj = holm(ps)

    print('\nref completou %d/%d' % (sum(ok(v['ref']) for v in pares.values()), len(pares)))
    aprovados = []
    for i, b in enumerate(BRACOS):
        r = res[b]
        h1, h2 = adj[2 * i], adj[2 * i + 1]
        n_b = sum(ok(v[b]) for v in pares.values())
        n_r = sum(ok(v['ref']) for v in pares.values())
        print('\n== %s ==  completou %d/%d' % (b, n_b, len(pares)))
        print('  1. sobrevivencia: so %s %d, so ref %d; sinal p = %.2g; Holm %.2g'
              % (b, r['so_b'], r['so_r'], r['p1'], h1))
        print('  2. pitch_rms: n %d, mediana(%s - ref) %+.2f graus (ref med %.2f), p = %.2g; Holm %.2g'
              % (r['nP'], b, r['dP'], med(r['P0']), r['p2'], h2))
        O0, O1 = pareado(pares, b, osc, lambda o, n: ok(o) and ok(n))
        dO, pO = wil(O1, O0)
        print('     oscilacao de pitch (secundaria): mediana %+.2f graus (ref med %.2f), p = %.2g'
              % (dO, med(O0), pO))
        R0, R1 = pareado(pares, b, lambda x: f(x['roll_rms_o']))
        rel = med([(y - x) / x for x, y in zip(R0, R1) if x > 0])
        D0, D1 = pareado(pares, b, lambda x: abs(f(x['erro_dir'])))
        W0, W1 = pareado(pares, b, lambda x: abs(f(x['razao_w']) - 1))
        dD = med([y - x for x, y in zip(D0, D1)])
        dW = med([y - x for x, y in zip(W0, W1)])
        sep = min((f(v[b]['sep_min_mm']) for v in pares.values()
                   if not math.isnan(f(v[b]['sep_min_mm']))), default=float('nan'))
        vetos = []
        if rel > 0.10:
            vetos.append('roll RMS %+.0f%%' % (100 * rel))
        if dD > 2.0:
            vetos.append('|erro_dir| %+.1f graus' % dD)
        if dW > 0.02:
            vetos.append('|w/w_cmd-1| %+.3f' % dW)
        if sep < 90:
            vetos.append('sep_min %.0f mm' % sep)
        print('  vetos: roll RMS %+.1f%% (n %d), |erro_dir| %+.2f graus (n %d), '
              '|w/w_cmd-1| %+.3f (n %d), sep_min %.0f mm -> %s'
              % (100 * rel, len(R0), dD, len(D0), dW, len(W0), sep,
                 ', '.join(vetos) if vetos else 'nenhum'))
        melhora = (h1 < 0.05 and r['so_b'] > r['so_r']) or (h2 < 0.05 and r['dP'] < 0)
        passa = n_b >= n_r and melhora and not vetos
        print('  -> %s' % ('PASSA' if passa else 'nao passa'))
        if passa:
            aprovados.append((med(r['P1']), b))

    print('\nDECISAO pelo criterio: %s' % (min(aprovados)[1] if aprovados else 'ref (nenhum braco passou)'))

    print('\nPor massa e movimento (completou ref / kd06 / x5 / kd06x5):')
    tipos = {}
    for (massa, mov, pe), v in pares.items():
        t = (massa, re.sub(r'[+-]', '±', mov))
        x = tipos.setdefault(t, [0, 0, 0, 0, 0])
        for j, b in enumerate(('ref',) + BRACOS):
            x[j] += ok(v[b])
        x[4] += 1
    for t in sorted(tipos):
        x = tipos[t]
        print('  %s %-14s %d / %d / %d / %d  (de %d)' % (t[0], t[1], x[0], x[1], x[2], x[3], x[4]))

    print('\npitch_rms mediano por massa e braco (pares em que todos completam):')
    for massa in ('M33', 'M38'):
        sel = [v for (m, _, _), v in pares.items() if m == massa
               and all(ok(v[b]) for b in ('ref',) + BRACOS)]
        print('  %s (n %d): ' % (massa, len(sel)) + '  '.join(
            '%s %.2f' % (b, med([f(v[b]['pitch_rms']) for v in sel])) for b in ('ref',) + BRACOS))
        print('  %s reta:   ' % massa + '  '.join(
            '%s %s' % (b, '/'.join('%.2f' % f(v[b]['pitch_rms'])
                                    for (m, mv, _), v in sorted(pares.items())
                                    if m == massa and mv.startswith('reta')))
            for b in ('ref',) + BRACOS))


if __name__ == '__main__':
    main()
