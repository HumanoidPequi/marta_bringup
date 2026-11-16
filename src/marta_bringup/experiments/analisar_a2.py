#!/usr/bin/env python3
"""A2 do caranguejo: envelope de vy (docs/marcha_lateral.md, sec. 5).

Le lateral/resumo_omni.csv (tags A2_<massa>_lat<+-vy>_<pe>_r1) e, como
referencia, omni/resumo_omni.csv (E7_<massa>_kd06_lat+-0.05_<pe>_r1, mesma
configuracao base com kd 0,06).

Criterio pre-registrado: vy "anda" numa massa se as 4 corridas (2 sentidos x
2 pes) completam 30 s. Registro tambem razao_v, |erro_dir|, sep_min_mm,
roll RMS e direcao da queda.

  python3 analisar_a2.py [lateral] [omni]
"""
import csv
import os
import re
import sys

from analisar_e5 import f
from analisar_e7 import med, ok


def ler(arq, pref):
    R = {}
    for r in csv.DictReader(open(arq)):
        m = re.match(r'^%s(M\d+)_(?:kd06_)?lat([+-][\d.]+)_(p[DE])_r\d+$' % pref, r['tag'])
        if m:
            massa, vy, pe = m.groups()
            R[(massa, float(vy), pe)] = r
    return R


def main():
    aqui = os.path.dirname(os.path.abspath(__file__))
    d = sys.argv[1] if len(sys.argv) > 1 else os.path.join(aqui, 'lateral')
    o = sys.argv[2] if len(sys.argv) > 2 else os.path.join(aqui, 'omni')
    R = ler(os.path.join(d, 'resumo_omni.csv'), 'A2_')
    R.update(ler(os.path.join(o, 'resumo_omni.csv'), 'E7_'))
    vys = sorted({abs(k[1]) for k in R})
    print('%d corridas (A2 + referencia E7 kd06 em 0,05)\n' % len(R))
    for massa in ('M33', 'M38'):
        print('== %s ==' % massa)
        for v in vys:
            rs = [(k, r) for k, r in sorted(R.items()) if k[0] == massa and abs(k[1]) == v]
            if not rs:
                continue
            n = sum(ok(r) for _, r in rs)
            print('  |vy| %.3f: %d/%d completaram -> %s' % (v, n, len(rs), 'ANDA' if n == len(rs) == 4 else 'nao anda'))
            for (m, vy, pe), r in rs:
                des = 'completou' if ok(r) else 'caiu %4.1f s %-6s' % (f(r['sobrevida']), r['direcao'])
                print('     %+.3f %s  %-18s razao_v %.2f  erro_dir %+6.1f  sep_min %5.1f  roll RMS %.2f  pitch RMS %.2f  vx_b %+5.1f mm/s' % (
                    vy, pe, des, f(r['razao_v']), f(r['erro_dir']), f(r['sep_min_mm']),
                    f(r['roll_rms_o']), f(r['pitch_rms']), 1000 * f(r['vx_b'])))
        print()


if __name__ == '__main__':
    main()
