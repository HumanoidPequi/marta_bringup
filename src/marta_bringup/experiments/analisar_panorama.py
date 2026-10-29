#!/usr/bin/env python3
"""Analise da bateria do panorama (rodar_panorama.sh; docs/panorama_parametros.md).

Por CORRIDA (resumo_corridas.csv):
  desfecho   completou (andou os 30 s; queda depois da parada nao conta),
             caiu_antes (antes da marcha medida: walk_test reporta ~2,5 s) ou
             caiu_marcha; direcao da queda: frente / tras (sinal do pitch
             verdadeiro, + = tronco para a frente) ou lado (roll);
  sobrevida  tempo de marcha ate a queda (30 s se completou; 0 se caiu antes);
  parado     pitch verdadeiro medio no ultimo 1 s antes do 1o passo
             (registrar_pe.py) - o "pender" estatico;
  marcha     (2 s ate o fim ou 0,5 s antes da queda) pitch medio e RMS, roll
             RMS, CoM - pe de apoio em x no apoio simples (sup_x), altura real
             do CoM, flexao media do joelho, cedencia do joelho de apoio contra
             a IK, fracao do tempo com |comando do joelho| >= 2,5 N.m, comando
             RMS do joelho, pico do pe, raspou, ponta, avanco.
Por CELULA (resumo_celulas.csv): a mesma lista, media das corridas, com n,
numero de corridas que completaram e tipos de queda.

  python3 analisar_panorama.py [dir] [campanha.csv]
"""
import csv
import glob
import math
import os
import statistics as st
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analisar_pe as P  # noqa: E402
import analisar_gravidade as G  # noqa: E402

DUR = 30.0
STALL = 2.5
KP = 12.74         # N.m/rad, MX-28 (controller_mx28.yaml)
JANELA_MIN = 3.0   # s de marcha minimos para as metricas de marcha


def nan():
    return float('nan')


def media(xs):
    xs = [x for x in xs if x == x]
    return st.mean(xs) if xs else nan()


def desfecho(r, pe_rows):
    if r['caiu'] != '1' or float(r['t_queda']) > DUR:
        return 'completou', '', DUR
    tq = float(r['t_queda'])
    antes = (r.get('rtf') in ('', '0.000') or float(r['rtf'] or 0) < 0.05) and tq < 2.7
    if r['motivo'] == 'roll':
        dirc = 'lado'
    else:
        dirc = ''
        for x in pe_rows:
            p = float(x['torso_pitch'])
            if abs(p) > 45:
                dirc = 'frente' if p > 0 else 'tras'
                break
        if not dirc:
            dirc = 'frente' if float(r['pico_pitch_v'] or 0) > 0 else 'tras'
    return ('caiu_antes' if antes else 'caiu_marcha'), dirc, (0.0 if antes else tq)


def resumo_corrida(d, r):
    pre = os.path.join(d, 'dados', r['tag'])
    out = {'tag': r['tag'], 'base': r['tag'].rsplit('_r', 1)[0], 'rtf': r['rtf']}
    pe_rows = list(csv.DictReader(open(pre + '_pe.csv'))) if os.path.exists(pre + '_pe.csv') else []
    out['desfecho'], out['direcao'], out['sobrevida'] = desfecho(r, pe_rows)
    out['avanco'] = float(r['avanco']) if out['desfecho'] == 'completou' and r['avanco'] else nan()
    # janela parada: ultimo 1 s antes do primeiro apoio simples
    ini = next((float(x['t']) for x in pe_rows if x['support'] != '0'), None)
    parado = [x for x in pe_rows if ini is not None and ini - 1.0 <= float(x['t']) < ini]
    out['pitch_parado'] = media([float(x['torso_pitch']) for x in parado])
    # cedencia (real - alvo) parado, media das duas pernas: no fim do agachamento
    # a compensacao ainda nao atua (Agacha() publica os alvos direto)
    out['ced_joelho_parado'] = media([float(x[l + '_knee_pv']) - float(x[l + '_knee_sp'])
                                      for x in parado for l in 'lr'])
    out['ced_soma_parado'] = media([sum(float(x['%s_%s_pv' % (l, j)]) - float(x['%s_%s_sp' % (l, j)])
                                        for j in ('hip_pitch', 'knee', 'ank_pitch'))
                                    for x in parado for l in 'lr'])
    # fechamento cinematico: eixos de quadril, joelho e tornozelo pitch paralelos
    # (+y no URDF), logo pitch_tronco = pitch_pe - soma(alvo) - soma(cedencia)
    out['soma_alvo_parado'] = media([sum(float(x['%s_%s_sp' % (l, j)]) for j in ('hip_pitch', 'knee', 'ank_pitch'))
                                     for x in parado for l in 'lr'])
    out['pe_pitch_parado'] = media([float(x[l + '_pitch']) for x in parado for l in 'lr'])
    # 1o apoio simples (passos no lugar, antes da marcha medida; so registrar_pe
    # cobre esse trecho): cedencia do joelho de apoio na METADE do apoio (antes
    # da queda, para nao medir efeito como causa) e o torque que o servo comanda
    # para segura-la, Kp*(real - alvo) (o alvo ja inclui Delta), contra o stall
    i1 = next((i for i, x in enumerate(pe_rows) if x['support'] in ('1', '2')), None)
    seg = []
    if i1 is not None:
        s1 = pe_rows[i1]['support']
        for x in pe_rows[i1:]:
            if x['support'] != s1:
                break
            seg.append(x)
    l1 = 'l' if seg and seg[0]['support'] == '1' else 'r'
    c1 = [float(x[l1 + '_knee_pv']) - float(x[l1 + '_knee_sp']) for x in seg]
    out['ced_joelho_1o'] = c1[len(c1) // 2] if c1 else nan()
    out['tau_joelho_1o'] = KP * math.radians(out['ced_joelho_1o']) if c1 else nan()
    out['pitch_fim_1o'] = float(seg[-1]['torso_pitch']) if seg else nan()
    t_fim = min(DUR, float(r['t_queda']) - 0.5) if r['caiu'] == '1' else DUR
    # metricas de marcha so com pelo menos JANELA_MIN s de marcha (2 s ate t_fim)
    if t_fim - 2.0 < JANELA_MIN:
        t_fim = -1.0
    # a mesma decomposicao na marcha, perna de apoio em apoio simples:
    # pitch = pe - soma(alvo - Delta) - soma(Delta) - soma(real - alvo); alvo - Delta
    # e IK + (-5 graus fixo no hip pitch) + estabilizador, Delta a compensacao
    ap = [(x, 'l' if x['support'] == '1' else 'r') for x in pe_rows
          if ini is not None and t_fim > 0 and x['support'] in ('1', '2')
          and ini + 2.0 <= float(x['t']) <= ini + t_fim]
    J = ('hip_pitch', 'knee', 'ank_pitch')
    g = lambda x, l, j, c: float(x['%s_%s_%s' % (l, j, c)])
    out['pitch_apoio'] = media([float(x['torso_pitch']) for x, l in ap])
    out['pe_pitch_apoio'] = media([float(x[l + '_pitch']) for x, l in ap])
    out['alvo_ik_apoio'] = media([sum(g(x, l, j, 'sp') - g(x, l, j, 'dg') for j in J) for x, l in ap])
    out['comp_apoio'] = media([sum(g(x, l, j, 'dg') for j in J) for x, l in ap])
    out['ced_apoio'] = media([sum(g(x, l, j, 'pv') - g(x, l, j, 'sp') for j in J) for x, l in ap])
    tel = []
    if os.path.exists(pre + '_telemetria.csv') and t_fim > 2.0:
        tel = [x for x in csv.DictReader(open(pre + '_telemetria.csv')) if 2.0 <= float(x['t']) <= t_fim]
    out['pitch_marcha'] = media([math.degrees(float(x['pitch_true'])) for x in tel])
    out['pitch_rms'] = math.sqrt(media([math.degrees(float(x['pitch_true'])) ** 2 for x in tel])) if tel else nan()
    out['roll_rms'] = math.sqrt(media([math.degrees(float(x['roll_true'])) ** 2 for x in tel])) if tel else nan()
    out['com_pe_mm'] = media([1000 * float(x['sup_x']) for x in tel if float(x['support']) > 0.5])
    out['z_com_mm'] = media([1000 * float(x['z_com_meas']) for x in tel if x['z_com_meas'] not in ('', 'nan')])
    # juntas: flexao e comando do joelho (todas as fases)
    flex, cmd = [], []
    if os.path.exists(pre + '_juntas.csv') and t_fim > 2.0:
        for x in csv.DictReader(open(pre + '_juntas.csv')):
            if x['junta'] in ('l_knee', 'r_knee') and 2.0 <= float(x['t']) <= t_fim:
                flex.append(abs(math.degrees(float(x['process_value']))))
                cmd.append(float(x['command']))
    out['joelho_flex'] = media(flex)
    out['joelho_rms'] = math.sqrt(media([c * c for c in cmd])) if cmd else nan()
    out['joelho_sat'] = 100 * media([float(abs(c) >= STALL - 1e-6) for c in cmd]) if cmd else nan()
    # cedencia: medida tambem nos passos no lugar, antes da marcha (diagnostico das quedas precoces)
    ced = G.cedencia(pre, r) if pe_rows else None
    out['cedencia_joelho'] = ced[0] if ced else nan()
    a = P.analisa(pre, r) if pe_rows and t_fim > 0 else None
    out['pe_pico'] = a['pico'] if a else nan()
    out['raspou'] = a['raspou'] if a else nan()
    out['ponta'] = a['ponta'] if a else nan()
    return out


CAMPOS = ['sobrevida', 'avanco', 'pitch_parado', 'ced_joelho_parado', 'ced_soma_parado',
          'soma_alvo_parado', 'pe_pitch_parado', 'pitch_apoio', 'pe_pitch_apoio', 'alvo_ik_apoio',
          'comp_apoio', 'ced_apoio', 'ced_joelho_1o', 'tau_joelho_1o', 'pitch_fim_1o', 'pitch_marcha', 'pitch_rms', 'roll_rms', 'com_pe_mm',
          'z_com_mm', 'joelho_flex', 'cedencia_joelho', 'joelho_sat', 'joelho_rms', 'pe_pico', 'raspou', 'ponta']


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), 'panorama')
    camp = sys.argv[2] if len(sys.argv) > 2 else os.path.join(d, 'campanha_panorama.csv')
    linhas = list(csv.DictReader(open(camp)))
    corridas = []
    for r in linhas:
        try:
            corridas.append(resumo_corrida(d, r))
        except Exception as e:  # corrida incompleta: registra e segue
            print('AVISO: %s nao analisada (%s)' % (r['tag'], e), file=sys.stderr)
    with open(os.path.join(d, 'resumo_corridas.csv'), 'w') as f:
        w = csv.DictWriter(f, fieldnames=['tag', 'base', 'rtf', 'desfecho', 'direcao'] + CAMPOS)
        w.writeheader()
        for c in corridas:
            w.writerow({k: (('%.3f' % v) if isinstance(v, float) else v) for k, v in c.items()})
    cel = {}
    for c in corridas:
        cel.setdefault(c['base'], []).append(c)
    with open(os.path.join(d, 'resumo_celulas.csv'), 'w') as f:
        w = csv.writer(f)
        w.writerow(['base', 'n', 'completou', 'caiu_antes', 'caiu_marcha', 'frente', 'tras', 'lado'] + CAMPOS)
        for b in sorted(cel):
            v = cel[b]
            cnt = lambda k, val: sum(1 for x in v if x[k] == val)
            w.writerow([b, len(v), cnt('desfecho', 'completou'), cnt('desfecho', 'caiu_antes'),
                        cnt('desfecho', 'caiu_marcha'), cnt('direcao', 'frente'), cnt('direcao', 'tras'),
                        cnt('direcao', 'lado')] + ['%.3f' % media([x[k] for x in v]) for k in CAMPOS])
    print('%d corridas, %d celulas -> resumo_corridas.csv, resumo_celulas.csv' % (len(corridas), len(cel)))


if __name__ == '__main__':
    main()
