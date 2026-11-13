#!/usr/bin/env python3
"""Analise das campanhas da marcha omnidirecional (docs/marcha_omnidirecional.md).

Le <dir>/campanha_panorama.csv (rodar_panorama.sh <desenho> <semente> <dir>) e,
por corrida, reaproveita analisar_panorama.resumo_corrida (desfecho, direcao da
queda, pitch, torque do joelho no 1o apoio...) e acrescenta:

  comando     vx, vy, w lidos do _launch.log (rosparam do controller_node), e
              a configuracao: start_left_swing, ganhos, roll, malha de rumo
  realizado   (janela de 3 s ate o fim ou 0,5 s antes da queda, minimo 3 s)
              w_real [rad/s]: inclinacao do yaw verdadeiro desenrolado;
              vx_b, vy_b [m/s]: deslocamento do CoM por passada completa
              (2 periodos, cancela o balanco lateral), girado para o frame do
              corpo pelo yaw medio da passada;
              razao_w, razao_v: realizado / comandado;
              erro_dir [graus]: direcao realizada - comandada no frame do corpo
              (so com |v comandada| > 0);
              deriva_m: no giro no lugar, distancia entre o CoM medio da
              primeira e da ultima passada da janela
  pe          escorr_yaw [graus/s]: |taxa de yaw do pe| media em apoio simples
              (o pe de apoio deveria ficar parado);
              sep_min_mm: menor separacao lateral pe esq - pe dir no frame do
              corpo (< 0: os pes se cruzaram; nao ha colisao entre pernas)
  roll        a_roll [graus]: roll verdadeiro no miolo (25-75%) de cada apoio
              simples, com sinal + = para o lado da perna em balanco, media dos
              apoios com t > 5 s (plano_fase3_roll.md par. 5); roll_rms;
              sat_hip_roll, sat_ank_roll, sat_hip_yaw [%]: tempo com
              |comando| >= limite (todas as fases, as duas pernas)

  python3 analisar_omni.py <dir> [campanha.csv]
"""
import csv
import math
import os
import re
import statistics as st
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analisar_panorama as AP  # noqa: E402

PERIODO = 0.6          # step_period do launch (um passo); a passada sao 2
T_INI = 3.0            # s: descarta a partida
MIN_JANELA = 3.0
PARAMS = ('walk_velocity_x', 'walk_velocity_y', 'walk_velocity_yaw', 'start_left_swing',
          'heading_enabled', 'kp_heading', 'max_heading_rate', 'kp_pitch', 'kd_pitch',
          'stabilizer_enabled', 'roll_stabilizer_enabled', 'kp_roll_hip', 'kd_roll_hip',
          'kp_roll_ankle', 'kd_roll_ankle', 'roll_washout_tau', 'torso_mass_scale',
          'gravity_comp_gain', 'z_com', 'hip_offset_x', 'joint_effort')


def nan():
    return float('nan')


def params_launch(pre):
    """rosparam do controller_node (e o effort do robot_description) no _launch.log."""
    out = {}
    if not os.path.exists(pre + '_launch.log'):
        return out
    for linha in open(pre + '_launch.log', errors='replace'):
        m = re.match(r'\s*\*\s*/controller_node/(\w+):\s*(\S+)', linha)
        if m and m.group(1) in PARAMS:
            out[m.group(1)] = m.group(2)
    return out


def num(d, k, padrao=0.0):
    v = d.get(k)
    if v is None:
        return padrao
    if v in ('True', 'true'):
        return 1.0
    if v in ('False', 'false'):
        return 0.0
    try:
        return float(v)
    except ValueError:
        return padrao


def unwrap(a):
    out = [a[0]]
    for i in range(1, len(a)):
        d = a[i] - a[i - 1]
        out.append(out[-1] + math.atan2(math.sin(d), math.cos(d)))
    return out


def inclinacao(t, y):
    mt, my = st.mean(t), st.mean(y)
    den = sum((a - mt) ** 2 for a in t)
    return sum((a - mt) * (b - my) for a, b in zip(t, y)) / den if den > 0 else nan()


def omni(pre, r, cmd, t_fim):
    out = {}
    tel = [x for x in csv.DictReader(open(pre + '_telemetria.csv'))] if os.path.exists(pre + '_telemetria.csv') else []
    jan = [x for x in tel if T_INI <= float(x['t']) <= t_fim]
    vazio = ('w_real', 'razao_w', 'vx_b', 'vy_b', 'razao_v', 'erro_dir', 'deriva_m',
             'escorr_yaw', 'sep_min_mm', 'a_roll', 'roll_rms_o')
    if t_fim - T_INI < MIN_JANELA or len(jan) < 50:
        return {k: nan() for k in vazio}
    t = [float(x['t']) for x in jan]
    yaw = unwrap([float(x['yaw_true']) for x in jan])
    cx = [float(x['com_x']) for x in jan]
    cy = [float(x['com_y']) for x in jan]
    vx, vy, w = cmd
    out['w_real'] = inclinacao(t, yaw)
    out['razao_w'] = out['w_real'] / w if abs(w) > 1e-6 else nan()
    # deslocamento por passada (2 periodos), no frame do corpo
    passo = 2 * PERIODO
    dt_med = (t[-1] - t[0]) / (len(t) - 1)
    k = max(1, int(round(passo / dt_med)))
    vb = []
    for i in range(0, len(t) - k, max(1, k // 4)):
        dx, dy = cx[i + k] - cx[i], cy[i + k] - cy[i]
        ym = 0.5 * (yaw[i] + yaw[i + k])
        c, s = math.cos(ym), math.sin(ym)
        vb.append(((c * dx + s * dy) / (t[i + k] - t[i]), (-s * dx + c * dy) / (t[i + k] - t[i])))
    out['vx_b'] = st.mean(v[0] for v in vb) if vb else nan()
    out['vy_b'] = st.mean(v[1] for v in vb) if vb else nan()
    vcmd = math.hypot(vx, vy)
    if vcmd > 1e-6 and vb:
        out['razao_v'] = math.hypot(out['vx_b'], out['vy_b']) / vcmd
        e = math.atan2(out['vy_b'], out['vx_b']) - math.atan2(vy, vx)
        out['erro_dir'] = math.degrees(math.atan2(math.sin(e), math.cos(e)))
    else:
        out['razao_v'] = out['erro_dir'] = nan()
    if vcmd < 1e-6 and len(t) > 2 * k:
        out['deriva_m'] = math.hypot(st.mean(cx[-k:]) - st.mean(cx[:k]), st.mean(cy[-k:]) - st.mean(cy[:k]))
    else:
        out['deriva_m'] = nan()
    # pe: escorregamento em yaw no apoio e separacao lateral no frame do corpo
    esc, sep = [], []
    for x in jan:
        s = float(x['support'])
        if s > 0.5:
            esc.append(abs(math.degrees(float(x['l_foot_yaw_rate' if s < 1.5 else 'r_foot_yaw_rate']))))
        dx = float(x['l_foot_x']) - float(x['r_foot_x'])
        dy = float(x['l_foot_y']) - float(x['r_foot_y'])
        y0 = float(x['yaw_true'])
        sep.append(1000 * (-math.sin(y0) * dx + math.cos(y0) * dy))
    out['escorr_yaw'] = st.mean(esc) if esc else nan()
    out['sep_min_mm'] = min(sep) if sep else nan()
    # A_roll: miolo de cada apoio simples, + = para o lado da perna em balanco
    # (roll > 0 tomba para a direita; apoio esquerdo = balanco direito)
    ar, seg = [], []
    for x in tel:
        tt = float(x['t'])
        if tt > t_fim:
            break
        s = int(round(float(x['support'])))
        if seg and s != seg[0][1]:
            if seg[0][1] in (1, 2) and seg[0][0] > 5.0 and len(seg) >= 8:
                n = len(seg)
                sinal = 1.0 if seg[0][1] == 1 else -1.0
                ar.append(sinal * st.mean(math.degrees(v) for _, _, v in seg[n // 4: 3 * n // 4]))
            seg = []
        seg.append((tt, s, float(x['roll_true'])))
    out['a_roll'] = st.mean(ar) if ar else nan()
    out['roll_rms_o'] = math.sqrt(st.mean(math.degrees(float(x['roll_true'])) ** 2 for x in jan))
    return out


def saturacao(pre, t_fim, effort):
    """% do tempo com |comando| >= limite, por grupo de junta (as duas pernas)."""
    grupos = {'hip_roll': [0, 0], 'ank_roll': [0, 0], 'hip_yaw': [0, 0]}
    if not os.path.exists(pre + '_juntas.csv'):
        return {'sat_' + g: nan() for g in grupos}
    for x in csv.DictReader(open(pre + '_juntas.csv')):
        g = x['junta'][2:]
        if g in grupos and T_INI <= float(x['t']) <= t_fim:
            grupos[g][1] += 1
            grupos[g][0] += abs(float(x['command'])) >= effort - 1e-6
    return {'sat_' + g: (100.0 * a / b if b else nan()) for g, (a, b) in grupos.items()}


CAMPOS_O = ['vx_cmd', 'vy_cmd', 'w_cmd', 'pe_esq', 'w_real', 'razao_w', 'vx_b', 'vy_b', 'razao_v',
            'erro_dir', 'deriva_m', 'escorr_yaw', 'sep_min_mm', 'a_roll', 'roll_rms_o',
            'sat_hip_roll', 'sat_ank_roll', 'sat_hip_yaw']


def resumo(d, r):
    pre = os.path.join(d, 'dados', r['tag'])
    out = AP.resumo_corrida(d, r)
    p = params_launch(pre)
    cmd = (num(p, 'walk_velocity_x'), num(p, 'walk_velocity_y'), num(p, 'walk_velocity_yaw'))
    out['vx_cmd'], out['vy_cmd'], out['w_cmd'] = cmd
    out['pe_esq'] = num(p, 'start_left_swing')
    out['config'] = ' '.join('%s=%s' % (k, p[k]) for k in PARAMS[3:] if k in p)
    t_fim = min(AP.DUR, float(r['t_queda']) - 0.5) if r['caiu'] == '1' else AP.DUR
    out.update(omni(pre, r, cmd, t_fim))
    out.update(saturacao(pre, t_fim, num(p, 'joint_effort', AP.STALL) or AP.STALL))
    return out


def main():
    d = sys.argv[1]
    camp = sys.argv[2] if len(sys.argv) > 2 else os.path.join(d, 'campanha_panorama.csv')
    corridas = []
    for r in csv.DictReader(open(camp)):
        try:
            corridas.append(resumo(d, r))
        except Exception as e:  # corrida incompleta: registra e segue
            print('AVISO: %s nao analisada (%s)' % (r['tag'], e), file=sys.stderr)
    campos = ['tag', 'base', 'rtf', 'desfecho', 'direcao'] + AP.CAMPOS + CAMPOS_O + ['config']
    with open(os.path.join(d, 'resumo_omni.csv'), 'w') as f:
        w = csv.DictWriter(f, fieldnames=campos, extrasaction='ignore')
        w.writeheader()
        for c in corridas:
            w.writerow({k: (('%.4f' % v) if isinstance(v, float) else v) for k, v in c.items()})
    print('%d corridas -> %s' % (len(corridas), os.path.join(d, 'resumo_omni.csv')))


if __name__ == '__main__':
    main()
