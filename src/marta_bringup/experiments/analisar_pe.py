#!/usr/bin/env python3
"""Analise da campanha do pe de balanco (rodar_pe.sh).

Por passo (cada trecho continuo de apoio simples), sobre o pe de BALANCO:
  - pico: maior altura do centro da sola acima do chao [mm];
  - raspou: ponta ou calcanhar a menos de RASPA mm do chao no miolo do balanco
    (fase s em [0,2; 0,8]);
  - pousou de ponta: no ultimo instante do balanco, a ponta mais baixa que o
    calcanhar por mais de PONTA graus.
O "chao" de cada corrida e a mediana da altura do pe de APOIO (o modelo tem
~9 mm de deslocamento constante entre a base da caixa e o plano).
Juntas da perna em balanco: erro RMS e fracao do tempo com |comando| >= tau.
Janela: do inicio da marcha + 1 s ate a queda (ou o fim).

  python3 analisar_pe.py [dir_pe]
"""
import bisect
import csv
import glob
import math
import os
import statistics as st
import sys

RASPA = 3.0      # mm
PONTA = 3.0      # graus
Z_STEP = 80.0    # mm planejados (z_step do launch)


def passos(rows, t_fim):
    """Lista de passos: (lado_balanco, [linhas]) em apoio simples."""
    out, cur, lado = [], [], None
    for r in rows:
        if float(r['t']) > t_fim:
            break
        s = int(r['support'])
        l = None if s == 0 else ('r' if s == 1 else 'l')
        if l != lado and cur:
            out.append((lado, cur))
            cur = []
        lado = l
        if l:
            cur.append(r)
    if cur and lado:
        out.append((lado, cur))
    return out


def analisa(prefixo, linha):
    rows = list(csv.DictReader(open(prefixo + '_pe.csv')))
    ini = next((float(r['t']) for r in rows if r['support'] != '0'), None)
    if ini is None:
        return None
    t_ini = ini + 1.0
    t_fim = ini + (float(linha['t_queda']) if linha['caiu'] == '1' else 1e9) - 0.3
    rows = [r for r in rows if float(r['t']) >= t_ini]
    ps = passos(rows, t_fim)
    apoio = [float(r['%s_z_centro' % ('l' if l == 'r' else 'r')]) for l, c in ps for r in c]
    if not ps or not apoio:
        return None
    chao = st.median(apoio)
    picos, raspou, ponta = [], 0, 0
    for l, c in ps:
        if len(c) < 5:
            continue
        zc = [1000 * (float(r[l + '_z_centro']) - chao) for r in c]
        picos.append(max(zc))
        n = len(c)
        miolo = c[int(0.2 * n):int(0.8 * n) + 1]
        folga = min(1000 * (min(float(r[l + '_z_ponta']), float(r[l + '_z_calcanhar'])) - chao) for r in miolo)
        raspou += folga < RASPA
        ponta += float(c[-1][l + '_pitch']) > PONTA
    n = len(picos)
    return {'n_passos': n, 'pico': st.mean(picos), 'pico_min': min(picos),
            'raspou': 100 * raspou / n, 'ponta': 100 * ponta / n}


def juntas(prefixo, tau, t_fim):
    sup = [(float(r['t']), int(float(r['support'])))
           for r in csv.DictReader(open(prefixo + '_estabilizador.csv'))]
    ts = [s[0] for s in sup]
    err, sat = {}, {}
    for r in csv.DictReader(open(prefixo + '_juntas.csv')):
        t = float(r['t'])
        if not (1.0 <= t <= t_fim):
            continue
        j = r['junta']
        if j[2:] not in ('hip_pitch', 'knee', 'ank_pitch'):
            continue
        s = sup[max(0, bisect.bisect_right(ts, t) - 1)][1]
        if s == 0 or (s == 1) == (j[0] == 'l'):
            continue
        err.setdefault(j[2:], []).append(float(r['error']))
        sat.setdefault(j[2:], []).append(abs(float(r['command'])) >= tau - 1e-6)
    return {k: (math.degrees(math.sqrt(st.mean([e * e for e in v]))), 100 * st.mean(sat[k]))
            for k, v in err.items()}


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), 'pe')
    cel = {}
    for r in csv.DictReader(open(os.path.join(d, 'campanha_pe.csv'))):
        tms, tau, mu = r['tag'].split('_')[:3]
        k = (tms[3:], tau[3:], mu[2:])
        pre = os.path.join(d, 'dados', r['tag'])
        if not os.path.exists(pre + '_pe.csv'):
            continue
        a = analisa(pre, r)
        t_fim = float(r['t_queda']) - 0.3 if r['caiu'] == '1' else 1e9
        cel.setdefault(k, []).append((r, a, juntas(pre, float(k[1]), t_fim)))
    print('pe planejado: %.0f mm; raspou = ponta ou calcanhar < %.0f mm do chao no miolo do balanco;'
          ' ponta = pousou com a ponta %.0f graus abaixo do calcanhar\n' % (Z_STEP, RASPA, PONTA))
    print('%-6s %-5s %-6s %7s %-12s %14s %8s %8s | %s' % (
        'tronco', 'tau', 'mu', 'quedas', 't_queda', 'pico (min) mm', 'raspou', 'ponta',
        'balanco: erro RMS / saturado (quadril, joelho, tornozelo)'))
    for k in sorted(cel, key=lambda k: (-float(k[0]), -float(k[1]), -float(k[2]))):
        v = cel[k]
        q = [x for x in v if x[0]['caiu'] == '1']
        A = [x[1] for x in v if x[1]]
        J = [x[2] for x in v if x[2]]
        jt = ', '.join('%.1f°/%.0f%%' % (st.mean(j[n][0] for j in J if n in j), st.mean(j[n][1] for j in J if n in j))
                       for n in ('hip_pitch', 'knee', 'ank_pitch') if any(n in j for j in J))
        print('%-6s %-5s %-6s %3d/%-3d %-12s %7.1f (%4.1f) %7.0f%% %7.0f%% | %s' % (
            k[0], k[1], k[2], len(q), len(v), ' '.join('%.1f' % float(x[0]['t_queda']) for x in q)[:12] or '-',
            st.mean(a['pico'] for a in A) if A else float('nan'), min(a['pico_min'] for a in A) if A else float('nan'),
            st.mean(a['raspou'] for a in A) if A else float('nan'), st.mean(a['ponta'] for a in A) if A else float('nan'), jt))


if __name__ == '__main__':
    main()
