#!/usr/bin/env python3
"""A3 do caranguejo: y_separation x step_period no lateral (docs/marcha_lateral.md, sec. 6).

Le lateral/resumo_omni.csv, tags A3_<massa>_lat<+-vy>_y<ys>_T<T>_pD_r1.
Celula = (massa, ys, T), 2 corridas (+-vy). Criterio pre-registrado, nesta ordem:
  1. sobrevivencia (corridas completas, de 2);
  2. |erro_dir| (media das 2);
  3. roll RMS (roll_rms_o, media das 2).
Vetos: sep_min < 90 mm em alguma corrida, ou pitch RMS > +10 % contra a celula
(0,06; 0,6) da mesma massa (so avaliado se a referencia completou).

  python3 analisar_a3.py [lateral]
"""
import csv
import math
import os
import re
import sys

from analisar_e5 import f
from analisar_e7 import ok


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'lateral')
    C = {}
    for r in csv.DictReader(open(os.path.join(d, 'resumo_omni.csv'))):
        m = re.match(r'^A3_(M\d+)_lat([+-][\d.]+)_y([\d.]+)_T([\d.]+)_pD_r\d+$', r['tag'])
        if m:
            massa, vy, ys, T = m.groups()
            C.setdefault((massa, ys, T), {})[vy] = r
    print('%d celulas\n' % len(C))

    def media(rs, col):
        xs = [abs(f(r[col])) if col == 'erro_dir' else f(r[col]) for r in rs if ok(r)]
        return sum(xs) / len(xs) if xs else float('nan')

    for massa in ('M33', 'M38'):
        ref = C.get((massa, '0.06', '0.6'), {})
        ref_ok = len(ref) == 2 and all(ok(r) for r in ref.values())
        p_ref = media(list(ref.values()), 'pitch_rms') if ref_ok else float('nan')
        print('== %s ==  (referencia 0,06 / 0,6: %s; pitch RMS %.2f)' % (
            massa, 'completou 2/2' if ref_ok else 'NAO completou', p_ref))
        linhas = []
        for (m, ys, T), v in sorted(C.items()):
            if m != massa:
                continue
            rs = list(v.values())
            n = sum(ok(r) for r in rs)
            ed, rr, pr = media(rs, 'erro_dir'), media(rs, 'roll_rms_o'), media(rs, 'pitch_rms')
            sep = min((f(r['sep_min_mm']) for r in rs if ok(r)), default=float('nan'))
            vet = []
            if not math.isnan(sep) and sep < 90:
                vet.append('sep_min %.0f' % sep)
            if ref_ok and not math.isnan(pr) and (pr - p_ref) / p_ref > 0.10:
                vet.append('pitch RMS %+.0f%%' % (100 * (pr - p_ref) / p_ref))
            quedas = ' '.join('%s caiu %.1f s %s' % (vy, f(r['sobrevida']), r['direcao'])
                              for vy, r in sorted(v.items()) if not ok(r))
            linhas.append((-n, ed if not math.isnan(ed) else 1e9, rr if not math.isnan(rr) else 1e9,
                           ys, T, n, len(rs), ed, rr, pr, sep, vet, quedas))
            print('  y %s T %s: %d/%d  |erro_dir| %5.1f  roll RMS %.2f  pitch RMS %.2f  sep_min %5.1f  vetos: %s  %s' % (
                ys, T, n, len(rs), ed, rr, pr, sep, ', '.join(vet) or 'nenhum', quedas))
        cand = sorted(x for x in linhas if not x[11])
        if cand and cand[0][5] > 0:
            b = cand[0]
            print('  -> melhor celula sem veto: y %s T %s (%d/%d, |erro_dir| %.1f, roll RMS %.2f)' % (
                b[3], b[4], b[5], b[6], b[7], b[8]))
        else:
            print('  -> nenhuma celula sem veto completa alguma corrida')
        print()


if __name__ == '__main__':
    main()
