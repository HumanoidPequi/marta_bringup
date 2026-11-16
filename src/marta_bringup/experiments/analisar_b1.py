#!/usr/bin/env python3
"""B1 dos bracos: postura x balanco (docs/bracos.md, sec. 6; criterio pre-registrado).

Le bracos/resumo_omni.csv. Tags B1_<massa>_<P>_s<s>_<mov>_<pe>_r1, P em
P0|P1|P2|P3 (e P*x para variantes com hip_offset_x recompensado), s em 0|1.
Par = (massa, movimento, pe): 16 pares por comparacao.

Q1 postura (s = 1, cada P contra P0): sobrevivencia (sinal) e roll_rms_o
   (Wilcoxon, pares em que os dois completam); Holm sobre os 2 x (no. de P)
   testes; vetos pitch RMS > +10 %, |erro_dir| > +2 graus, |w/w_cmd - 1| >
   +0,02, sep_min < 90 mm; passa se completa >= P0, melhora uma primaria com
   Holm p < 0,05 e nao aciona veto; varias -> a de menor roll RMS mediano.
Q2 balanco (s = 1 contra s = 0 em cada P): hc_rms e escorr_yaw (Wilcoxon);
   Holm sobre 2 x (no. de P) testes; "ajuda" se reduz uma primaria com Holm
   p < 0,05 em alguma postura sem piorar a sobrevivencia nem acionar vetos.
Q2b (adendo pre-registrado, exploratoria): na reta, s = 2 contra s = 1,
   juntando as 4 posturas (16 pares); hc_rms e escorr_yaw, Holm sobre 2;
   vetos de Q1. Fora do Holm de Q1/Q2.
Exploratorio (nao pre-registrado): s1 contra s0 por movimento (16 pares) e
   decomposicao do pitch RMS em vies medio (pitch_marcha) e oscilacao.

  python3 analisar_b1.py [bracos]
"""
import csv
import math
import os
import re
import sys

from analisar_e5 import f, holm, sinal, wil
from analisar_e7 import med, ok, osc


def ler(d):
    P = {}
    for r in csv.DictReader(open(os.path.join(d, 'resumo_omni.csv'))):
        m = re.match(r'^B1_(M\d+)_(P\w+?)_s(\d)_((?:reta|giro|curva|lat)[+-]?[\d.]+)_(p[DE])_r\d+$', r['tag'])
        if m:
            massa, post, s, mov, pe = m.groups()
            P.setdefault((massa, mov, pe), {})[(post, int(s))] = r
    return P


def pares(P, a, b):
    return [(v[a], v[b]) for v in P.values() if a in v and b in v]


def vetos(pp):
    def mdiff(fn):
        xs = [fn(y) - fn(x) for x, y in pp if not (math.isnan(fn(x)) or math.isnan(fn(y)))]
        return med(xs) if xs else float('nan')
    rel = med([(f(y['pitch_rms']) - f(x['pitch_rms'])) / f(x['pitch_rms']) for x, y in pp
               if ok(x) and ok(y) and f(x['pitch_rms']) > 0] or [float('nan')])
    dD = mdiff(lambda r: abs(f(r['erro_dir'])))
    dW = mdiff(lambda r: abs(f(r['razao_w']) - 1))
    sep = min((f(y['sep_min_mm']) for _, y in pp if not math.isnan(f(y['sep_min_mm']))), default=float('nan'))
    v = []
    if rel > 0.10:
        v.append('pitch RMS %+.0f%%' % (100 * rel))
    if dD > 2.0:
        v.append('|erro_dir| %+.1f' % dD)
    if dW > 0.02:
        v.append('|w/wc-1| %+.3f' % dW)
    if sep < 90:
        v.append('sep_min %.0f' % sep)
    return v, rel, dD, dW, sep


def contraste(pp, metrica):
    a0 = [f(x[metrica]) for x, y in pp if ok(x) and ok(y)]
    a1 = [f(y[metrica]) for x, y in pp if ok(x) and ok(y)]
    keep = [i for i in range(len(a0)) if not (math.isnan(a0[i]) or math.isnan(a1[i]))]
    a0, a1 = [a0[i] for i in keep], [a1[i] for i in keep]
    d, p = wil(a1, a0)
    return len(a0), d, p, med(a0)


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'bracos')
    P = ler(d)
    posts = sorted({k[0] for v in P.values() for k in v})
    print('%d pares (massa, mov, pe); posturas %s\n' % (len(P), ', '.join(posts)))

    print('Desfecho por postura e s (completou / total):')
    for post in posts:
        for s in (0, 1):
            rs = [v[(post, s)] for v in P.values() if (post, s) in v]
            if rs:
                print('  %-4s s%d: %2d/%2d   roll RMS med %.2f  pitch RMS med %.2f  hc_rms med %.4f' % (
                    post, s, sum(ok(r) for r in rs), len(rs),
                    med([f(r['roll_rms_o']) for r in rs if ok(r)]),
                    med([f(r['pitch_rms']) for r in rs if ok(r)]),
                    med([f(r['hc_rms']) for r in rs if ok(r)])))

    print('\nQ1: postura (s = 1) contra P0')
    outras = [p for p in posts if p != 'P0']
    ps, res = [], {}
    for post in outras:
        pp = pares(P, ('P0', 1), (post, 1))
        sb = sum(ok(y) and not ok(x) for x, y in pp)
        sr = sum(ok(x) and not ok(y) for x, y in pp)
        p1 = sinal(sb, sr)
        n, dR, p2, m0 = contraste(pp, 'roll_rms_o')
        res[post] = (pp, sb, sr, p1, n, dR, p2, m0)
        ps += [p1, 1.0 if math.isnan(p2) else p2]
    adj = holm(ps) if ps else []
    aprov = []
    for i, post in enumerate(outras):
        pp, sb, sr, p1, n, dR, p2, m0 = res[post]
        nb = sum(ok(y) for _, y in pp)
        nr = sum(ok(x) for x, _ in pp)
        v, rel, dD, dW, sep = vetos(pp)
        dO = med([osc(y) - osc(x) for x, y in pp if ok(x) and ok(y)] or [float('nan')])
        dA = med([f(y['a_roll']) - f(x['a_roll']) for x, y in pp if ok(x) and ok(y)
                  and not math.isnan(f(x['a_roll'])) and not math.isnan(f(y['a_roll']))] or [float('nan')])
        print('  %s: completou %d/%d (P0 %d/%d); so %s %d, so P0 %d, sinal p %.2g, Holm %.2g' % (
            post, nb, len(pp), nr, len(pp), post, sb, sr, p1, adj[2 * i]))
        print('      roll RMS: n %d, mediana (%s - P0) %+.2f graus (P0 med %.2f), p %.2g, Holm %.2g' % (
            n, post, dR, m0, p2, adj[2 * i + 1]))
        print('      vetos: pitch RMS %+.1f%%, |erro_dir| %+.2f, |w/wc-1| %+.3f, sep_min %.0f -> %s' % (
            100 * rel, dD, dW, sep, ', '.join(v) or 'nenhum'))
        dM = med([f(y['pitch_marcha']) - f(x['pitch_marcha']) for x, y in pp if ok(x) and ok(y)] or [float('nan')])
        print('      secundarias: oscilacao de pitch %+.2f graus, A_roll %+.2f graus' % (dO, dA))
        print('      decomposicao do pitch: vies medio (pitch_marcha) %+.2f graus, oscilacao %+.2f graus' % (dM, dO))
        melhora = (adj[2 * i] < 0.05 and sb > sr) or (adj[2 * i + 1] < 0.05 and dR < 0)
        passa = nb >= nr and melhora and not v
        print('      -> %s' % ('PASSA' if passa else 'nao passa'))
        if passa:
            aprov.append((med([f(y['roll_rms_o']) for _, y in pp if ok(y)]), post))
    print('  DECISAO Q1: %s' % (min(aprov)[1] if aprov else 'P0 (nenhuma postura passou)'))

    print('\nQ2: balanco s = 1 contra s = 0, por postura')
    ps2, r2 = [], {}
    for post in posts:
        pp = pares(P, (post, 0), (post, 1))
        a = contraste(pp, 'hc_rms')
        b = contraste(pp, 'escorr_yaw')
        r2[post] = (pp, a, b)
        ps2 += [1.0 if math.isnan(a[2]) else a[2], 1.0 if math.isnan(b[2]) else b[2]]
    adj2 = holm(ps2) if ps2 else []
    ajuda = []
    for i, post in enumerate(posts):
        pp, a, b = r2[post]
        n0 = sum(ok(x) for x, _ in pp)
        n1 = sum(ok(y) for _, y in pp)
        v = vetos(pp)[0]
        print('  %s: completou s0 %d/%d, s1 %d/%d' % (post, n0, len(pp), n1, len(pp)))
        print('      hc_rms: n %d, mediana (s1 - s0) %+.4f rad/s (s0 med %.4f), p %.2g, Holm %.2g' % (
            a[0], a[1], a[3], a[2], adj2[2 * i]))
        print('      escorr_yaw: n %d, mediana (s1 - s0) %+.3f graus/s (s0 med %.3f), p %.2g, Holm %.2g' % (
            b[0], b[1], b[3], b[2], adj2[2 * i + 1]))
        for mov in sorted({k[1] for k in P}):
            sub = [(x, y) for (m, mv, pe), vv in P.items() if mv == mov
                   for x, y in [(vv.get((post, 0)), vv.get((post, 1)))] if x and y and ok(x) and ok(y)]
            if sub:
                print('        %-12s hc_rms s0 %.4f -> s1 %.4f   escorr s0 %.3f -> s1 %.3f   (n %d)' % (
                    mov, med([f(x['hc_rms']) for x, _ in sub]), med([f(y['hc_rms']) for _, y in sub]),
                    med([f(x['escorr_yaw']) for x, _ in sub]), med([f(y['escorr_yaw']) for _, y in sub]), len(sub)))
        red = (adj2[2 * i] < 0.05 and a[1] < 0) or (adj2[2 * i + 1] < 0.05 and b[1] < 0)
        if red and n1 >= n0 and not v:
            ajuda.append(post)
        print('      vetos (s1 contra s0): %s' % (', '.join(v) or 'nenhum'))
    print('  DECISAO Q2: %s' % ('o balanco AJUDA em ' + ', '.join(ajuda) if ajuda else 'sem efeito significativo'))

    print('\nQ2b (adendo pre-registrado): reta, s = 2 contra s = 1, 4 posturas juntas')
    pp = [(v[(p, 1)], v[(p, 2)]) for (m, mv, pe), v in P.items() if mv.startswith('reta')
          for p in posts if (p, 1) in v and (p, 2) in v]
    if pp:
        a = contraste(pp, 'hc_rms')
        b = contraste(pp, 'escorr_yaw')
        adj3 = holm([1.0 if math.isnan(a[2]) else a[2], 1.0 if math.isnan(b[2]) else b[2]])
        v, rel, dD, dW, sep = vetos(pp)
        print('  completou s1 %d/%d, s2 %d/%d' % (sum(ok(x) for x, _ in pp), len(pp),
                                               sum(ok(y) for _, y in pp), len(pp)))
        print('  hc_rms: n %d, mediana (s2 - s1) %+.4f rad/s (s1 med %.4f), p %.2g, Holm %.2g; s2 < s1 em %d/%d' % (
            a[0], a[1], a[3], a[2], adj3[0], sum(f(y['hc_rms']) < f(x['hc_rms']) for x, y in pp), len(pp)))
        print('  escorr_yaw: n %d, mediana (s2 - s1) %+.3f graus/s (s1 med %.3f), p %.2g, Holm %.2g' % (
            b[0], b[1], b[3], b[2], adj3[1]))
        print('  vetos: pitch RMS %+.1f%%, |erro_dir| %+.2f, |w/wc-1| %+.3f, sep_min %.0f -> %s' % (
            100 * rel, dD, dW, sep, ', '.join(v) or 'nenhum'))
        for p in posts:
            xs = [v for (m, mv, pe), v in P.items() if mv.startswith('reta')]
            print('    %-3s hc_rms mediano  s0 %.4f  s1 %.4f  s2 %.4f' % (p, *[
                med([f(v[(p, k)]['hc_rms']) for v in xs if (p, k) in v and ok(v[(p, k)])] or [float('nan')])
                for k in (0, 1, 2)]))

    print('\nExploratorio (nao pre-registrado): s1 contra s0 por movimento, 4 posturas juntas')
    for mov in sorted({k[1] for k in P}):
        pp = [(v[(p, 0)], v[(p, 1)]) for (m, mv, pe), v in P.items() if mv == mov
              for p in posts if (p, 0) in v and (p, 1) in v]
        for c in ('hc_rms', 'escorr_yaw', 'roll_rms_o', 'pitch_rms'):
            n, dd, pv, m0 = contraste(pp, c)
            nm = sum(f(y[c]) < f(x[c]) for x, y in pp if ok(x) and ok(y))
            print('  %-11s %-10s n %2d  s0 med %.4f  mediana (s1 - s0) %+.4f (%+.0f%%)  p %.2g  s1 < s0 em %d' % (
                mov, c, n, m0, dd, 100 * dd / m0 if m0 else float('nan'), pv, nm))


if __name__ == '__main__':
    main()
