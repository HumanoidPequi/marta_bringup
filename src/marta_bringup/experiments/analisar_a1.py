#!/usr/bin/env python3
"""A1 do caranguejo: de onde vem o avanco espurio (docs/marcha_lateral.md, sec. 4).

Le lateral/resumo_omni.csv (analisar_omni.py lateral). Tags
A1_<massa>_<braco>_<mov>_pD_r1, braco em ref|hpo0|hx0|s0|gc0|rumo0|pitch0,
mov em lugar|lat+0.05|lat-0.05.

Criterio pre-registrado:
  Q1  vx_b(lugar, ref) / vx_b(lateral, ref): >= 0,5 => o avanco e da marcha,
      nao do lateral;
  Q2  fracao de vx_b na colocacao do passo (vx_ar) e no escorregamento (vx_chao);
  Q3  um fator explica o avanco se a ablacao reduz |vx_b| em >= 50 % contra a
      ref nas 6 combinacoes (3 mov x 2 massas) com metrica; sinal 6/6 -> p 0,031.

  python3 analisar_a1.py [lateral]
"""
import csv
import math
import os
import re
import sys

from analisar_e5 import f, sinal

BRACOS = ('ref', 'hpo0', 'hx0', 's0', 'gc0', 'rumo0', 'pitch0')
MOVS = ('lugar', 'lat+0.05', 'lat-0.05')


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'lateral')
    R = {}
    for r in csv.DictReader(open(os.path.join(d, 'resumo_omni.csv'))):
        m = re.match(r'^A1_(M\d+)_(\w+?)_(lugar|lat[+-][\d.]+)_pD_r\d+$', r['tag'])
        if m:
            R[m.groups()] = r
    print('%d corridas A1\n' % len(R))

    def g(k, c):
        r = R.get(k)
        return f(r[c]) if r else float('nan')

    for massa in ('M33', 'M38'):
        print('== %s ==  (mm/s; desfecho)' % massa)
        print('  %-8s ' % 'braco' + ''.join('%-44s' % mv for mv in MOVS))
        print('  %-8s ' % '' + ''.join('%-44s' % 'vx_b   vx_ar  vx_chao  vy_b   desfecho' for _ in MOVS))
        for b in BRACOS:
            cel = []
            for mv in MOVS:
                k = (massa, b, mv)
                r = R.get(k)
                if not r:
                    cel.append('%-44s' % '(falta)')
                    continue
                cel.append('%-44s' % ('%+6.1f %+6.1f %+6.1f %+6.1f  %s' % (
                    1000 * g(k, 'vx_b'), 1000 * g(k, 'vx_ar'), 1000 * g(k, 'vx_chao'),
                    1000 * g(k, 'vy_b'), r['desfecho'] if r['desfecho'] == 'completou'
                    else 'caiu %.1f %s' % (f(r['sobrevida']), r['direcao']))))
            print('  %-8s ' % b + ''.join(cel))
        print()

    print('Q1: vx_b no lugar / vx_b no lateral (ref)')
    for massa in ('M33', 'M38'):
        lug = g((massa, 'ref', 'lugar'), 'vx_b')
        lat = [g((massa, 'ref', mv), 'vx_b') for mv in MOVS[1:]]
        lat_m = sum(lat) / len(lat)
        print('  %s: lugar %+.1f mm/s, lateral %+.1f / %+.1f mm/s -> razao %.2f -> %s' % (
            massa, 1000 * lug, 1000 * lat[0], 1000 * lat[1], lug / lat_m,
            'o avanco e da MARCHA (>= 0,5)' if lug / lat_m >= 0.5 else 'especifico do lateral'))

    print('\nQ2: fracao de vx_b na colocacao (ar) e no escorregamento (chao), ref')
    for massa in ('M33', 'M38'):
        for mv in MOVS:
            k = (massa, 'ref', mv)
            vb, ar, ch = g(k, 'vx_b'), g(k, 'vx_ar'), g(k, 'vx_chao')
            print('  %s %-9s ar %4.0f %%  chao %4.0f %%  (soma pes %+.1f vs CoM %+.1f mm/s)' % (
                massa, mv, 100 * ar / vb, 100 * ch / vb, 1000 * (ar + ch), 1000 * vb))

    print('\nQ3: reducao de |vx_b| pela ablacao (>= 50 % nas 6 combinacoes)')
    for b in BRACOS[1:]:
        red, n = [], 0
        for massa in ('M33', 'M38'):
            for mv in MOVS:
                v0, v1 = g((massa, 'ref', mv), 'vx_b'), g((massa, b, mv), 'vx_b')
                if math.isnan(v0) or math.isnan(v1):
                    red.append(None)
                    continue
                n += 1
                red.append(1 - abs(v1) / abs(v0))
        sim = sum(1 for x in red if x is not None and x >= 0.5)
        nao = sum(1 for x in red if x is not None and x < 0.5)
        p = sinal(sim, nao) if n else float('nan')
        txt = ' '.join('  --  ' if x is None else '%+5.0f%%' % (100 * x) for x in red)
        veredito = ('EXPLICA' if n == 6 and sim == 6 else
                    'tendencia' if sim >= 5 else 'nao explica')
        print('  %-7s %s   %d/%d >= 50%%, sinal p = %.3f -> %s' % (b, txt, sim, n, p, veredito))
    print('  (ordem: M33 lugar, lat+, lat-, M38 lugar, lat+, lat-)')


if __name__ == '__main__':
    main()
