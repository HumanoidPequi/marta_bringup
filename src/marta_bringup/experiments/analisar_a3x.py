#!/usr/bin/env python3
"""A3x (exploratoria, post hoc): docs/marcha_lateral.md sec. 7.

(a) dissociacao frequencia x comprimento do passo:
    T 0,5 a vy +-0,09 (45 mm) e T 0,6 a vy +-0,0625 (37,5 mm), y 0,05;
    leitura: completou e assimetria de fase E - D (|.| < 1,5 baixa; > 3 alta).
(b) validacao de y 0,05 / T 0,5 fora do lateral contra a configuracao base:
    "nao prejudica" se completa, pitch RMS e roll RMS <= +10 % da referencia
    e, em giro/curva, |w/wc - 1| <= referencia + 0,02.

  python3 analisar_a3x.py
"""
import csv
import os
import sys

from analisar_e5 import f
from analisar_e7 import ok

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(AQUI, 'lateral'))
from diag_fase_pitch import fase  # noqa: E402


def ler(d):
    return {r['tag']: r for r in csv.DictReader(open(os.path.join(AQUI, d, 'resumo_omni.csv')))}


def main():
    L, B, O = ler('lateral'), ler('bracos'), ler('omni')

    print('(a) dissociacao (assimetria de fase com +vy; com -vy o sinal inverte)')
    for M in ('M33', 'M38'):
        for vy, T, mm in (('0.09', '0.5', 45), ('0.0625', '0.6', 37.5)):
            for sg in ('+', '-'):
                tag = 'A3x_%s_lat%s%s_y0.05_T%s_pD_r1' % (M, sg, vy, T)
                r = L.get(tag)
                if r is None:
                    print('  %s (falta)' % tag)
                    continue
                fz = fase(tag, 'lateral') if ok(r) else None
                txt = 'E - D %+5.2f osc %.2f' % (fz[0] - fz[1], fz[2]) if fz else ''
                print('  %-36s passo %4.1f mm  %-22s %s' % (
                    tag, mm, 'completou' if ok(r) else 'caiu %.1f s %s' % (f(r['sobrevida']), r['direcao']), txt))

    print('\n(b) validacao de y 0,05 / T 0,5 contra a base (0,06 / 0,6)')
    refs = {'lugar': ('lateral', 'A1_%s_ref_lugar_pD_r1'),
            'reta0.10': ('bracos', 'B1_%s_P0_s1_reta0.10_pD_r1'),
            'giro+0.175': ('bracos', 'B1_%s_P0_s1_giro+0.175_pD_r1'),
            'curva+0.175': ('omni', 'E7_%s_kd06_curva+0.175_pD_r1'),
            'lat+0.1': ('lateral', 'A2_%s_lat+0.10_pD_r1'),
            'lat-0.1': ('lateral', 'A2_%s_lat-0.10_pD_r1')}
    fonte = {'lateral': L, 'bracos': B, 'omni': O}
    for M in ('M33', 'M38'):
        for mov, (d, pat) in refs.items():
            n = L.get('A3x_%s_%s_y0.05_T0.5_pD_r1' % (M, mov))
            r = fonte[d].get(pat % M)
            if n is None or r is None:
                print('  %s %-11s (falta %s)' % (M, mov, 'nova' if n is None else pat % M))
                continue

            def des(x):
                return 'completou' if ok(x) else 'caiu %.1f %s' % (f(x['sobrevida']), x['direcao'])
            linha = '  %s %-11s ref %-14s -> novo %-14s' % (M, mov, des(r), des(n))
            if ok(n) and ok(r):
                dp = (f(n['pitch_rms']) - f(r['pitch_rms'])) / f(r['pitch_rms'])
                dr = (f(n['roll_rms_o']) - f(r['roll_rms_o'])) / f(r['roll_rms_o'])
                linha += ' pitch RMS %.2f -> %.2f (%+.0f%%)  roll RMS %.2f -> %.2f (%+.0f%%)' % (
                    f(r['pitch_rms']), f(n['pitch_rms']), 100 * dp, f(r['roll_rms_o']), f(n['roll_rms_o']), 100 * dr)
                bad = dp > 0.10 or dr > 0.10
                if mov.startswith(('giro', 'curva')):
                    w0, w1 = abs(f(r['razao_w']) - 1), abs(f(n['razao_w']) - 1)
                    linha += '  |w/wc-1| %.3f -> %.3f' % (w0, w1)
                    bad = bad or w1 > w0 + 0.02
                if mov.startswith(('reta', 'curva')):
                    linha += '  vx_b %.1f -> %.1f mm/s' % (1000 * f(r['vx_b']), 1000 * f(n['vx_b']))
                linha += '  -> %s' % ('PREJUDICA' if bad else 'nao prejudica')
            elif ok(n):
                linha += ' (melhora: a referencia cai)  pitch RMS %.2f roll RMS %.2f' % (
                    f(n['pitch_rms']), f(n['roll_rms_o']))
            else:
                linha += ' -> %s' % ('PREJUDICA (a referencia completa)' if ok(r) else 'ambos caem')
            print(linha)


if __name__ == '__main__':
    main()
