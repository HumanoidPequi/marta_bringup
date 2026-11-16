#!/usr/bin/env python3
"""Bissecao da forca de empurrao tolerada (docs/bracos.md, B3).

Para cada condicao do desenho, procura a maior forca (pulso de push_duration s
no tronco, eixo e sinal do proprio desenho) que o robo aguenta sem cair, por
bissecao entre lo (aguenta) e hi (cai). Cada corrida e uma chamada do
rodar_panorama.sh com um desenho de uma linha, entao vale tudo daquele runner
(MX-28, compensacao, registro, retomada pelo CSV da campanha).

    python3 rodar_empurrao.py <desenho.csv> <saida> [--hi 40] [--passos 4]

<desenho.csv>: base,torso_mass_scale,z_com,hip_offset_x,gain,sinal,extra
  sinal = +1 ou -1 (sentido do empurrao no eixo y do corpo); extra = args do
  roslaunch e do walk_test (tokens com _), sem _push_force (posto aqui).
<saida>: diretorio da campanha, relativo a experiments/.

Regras (pre-registradas no doc):
  - hi inicial deve derrubar; se nao derrubar, dobra (ate 160 N) e registra;
  - lo inicial = 0 (marcha sem empurrao; precisa completar, senao a condicao
    e invalida e o limiar fica NaN);
  - corrida com RTF < 0,85 ou com o empurrao nao aplicado e refeita uma vez
    (tag _r2); se repetir, a condicao para com aviso;
  - resultado: limiar = lo final, resolucao = hi - lo.
Saida: <saida>/limiares.csv (uma linha por condicao) e o log das corridas.
"""
import argparse
import csv
import os
import subprocess
import sys

AQUI = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.dirname(AQUI)
RUNNER = os.path.join(EXP, 'rodar_panorama.sh')
RTF_MIN = 0.85


def ler_campanha(saida_abs):
    camp = os.path.join(saida_abs, 'campanha_panorama.csv')
    if not os.path.exists(camp):
        return {}
    return {r['tag']: r for r in csv.DictReader(open(camp))}


def empurrao_ok(saida_abs, tag):
    f = os.path.join(saida_abs, 'dados', tag + '_empurrao.csv')
    if not os.path.exists(f):
        return False
    linhas = list(csv.DictReader(open(f)))
    return bool(linhas) and linhas[-1]['ok'] == '1'


def corrida(cond, forca, saida_rel, saida_abs, log):
    """Roda (ou reaproveita) uma corrida; devolve True se caiu, None se invalida."""
    base = '%s_F%05.1f' % (cond['base'], forca)
    extra = cond['extra'].strip()
    if forca > 0:
        extra += ' _push_force:=%.2f' % (int(cond['sinal']) * forca)
    for k in (1, 2):
        tag = '%s_r%d' % (base, k)
        feitas = ler_campanha(saida_abs)
        if tag not in feitas:
            des = os.path.join(saida_abs, '_desenho_tmp.csv')
            with open(des, 'w') as f:
                w = csv.writer(f)
                w.writerow(['base', 'torso_mass_scale', 'z_com', 'hip_offset_x', 'gain', 'nrep', 'extra'])
                w.writerow([base, cond['torso_mass_scale'], cond['z_com'], cond['hip_offset_x'],
                            cond['gain'], k, extra])
            subprocess.run(['bash', RUNNER, des, '1', saida_rel], stdout=log, stderr=subprocess.STDOUT)
            log.flush()
            feitas = ler_campanha(saida_abs)
        r = feitas.get(tag)
        if r is None:
            print('  %s: sem linha na campanha' % tag, file=log)
            continue
        rtf = float(r['rtf']) if r.get('rtf') not in (None, '') else 0.0
        valida = rtf >= RTF_MIN and (forca == 0 or empurrao_ok(saida_abs, tag))
        if valida:
            return r['caiu'] == '1', tag
        print('  %s INVALIDA (rtf %.3f, empurrao %s); refazendo' % (
            tag, rtf, empurrao_ok(saida_abs, tag)), file=log)
    return None, base


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('desenho')
    ap.add_argument('saida')
    ap.add_argument('--hi', type=float, default=40.0)
    ap.add_argument('--passos', type=int, default=4)
    a = ap.parse_args()
    saida_abs = os.path.join(EXP, a.saida)
    os.makedirs(os.path.join(saida_abs, 'dados'), exist_ok=True)
    out = os.path.join(saida_abs, 'limiares.csv')
    prontas = {r['base'] for r in csv.DictReader(open(out))} if os.path.exists(out) else set()
    log = open(os.path.join(saida_abs, 'log_empurrao.txt'), 'a')
    for cond in csv.DictReader(open(a.desenho)):
        if cond['base'] in prontas:
            continue
        print('== %s' % cond['base'], file=log, flush=True)
        caiu0, _ = corrida(cond, 0.0, a.saida, saida_abs, log)
        lo, hi, nota = 0.0, a.hi, ''
        if caiu0 is None or caiu0:
            lo = hi = float('nan')
            nota = 'sem empurrao nao completa ou invalida'
        else:
            caiu, _ = corrida(cond, hi, a.saida, saida_abs, log)
            while caiu is False and hi < 160.0:
                lo, hi = hi, 2 * hi
                nota = 'hi expandido'
                caiu, _ = corrida(cond, hi, a.saida, saida_abs, log)
            if caiu is None:
                lo = hi = float('nan')
                nota = 'corrida invalida em hi'
            elif caiu is False:
                nota = 'nao caiu nem com %.0f N' % hi
                lo = hi
            else:
                for _ in range(a.passos):
                    meio = 0.5 * (lo + hi)
                    caiu, _ = corrida(cond, meio, a.saida, saida_abs, log)
                    if caiu is None:
                        nota = 'corrida invalida na bissecao'
                        break
                    if caiu:
                        hi = meio
                    else:
                        lo = meio
        novo = not os.path.exists(out)
        with open(out, 'a') as f:
            w = csv.writer(f)
            if novo:
                w.writerow(['base', 'limiar_N', 'cai_N', 'resolucao_N', 'nota'])
            w.writerow([cond['base'], '%.2f' % lo, '%.2f' % hi, '%.2f' % (hi - lo), nota])
        print('%s: limiar %.1f N (cai com %.1f) %s' % (cond['base'], lo, hi, nota), flush=True)
    log.close()


if __name__ == '__main__':
    sys.exit(main())
