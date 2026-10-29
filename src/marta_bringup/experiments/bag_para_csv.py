#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Extrai um rosbag de marcha para dois CSV tidy. Roda DENTRO do container (precisa
do rosbag); as figuras sao geradas depois por figuras_marcha.py, que so precisa
de matplotlib e portanto roda em qualquer lugar.

    python3 bag_para_csv.py <arquivo.bag> [dir_saida]

Gera, ao lado do bag:
    <nome>_telemetria.csv   uma linha por amostra de /marta/walk/telemetry
    <nome>_juntas.csv       uma linha por amostra de cada */state (formato longo)
    <nome>_estabilizador.csv  uma linha por amostra de /marta/stabilizer_debug
"""
import csv
import os
import sys

import rosbag

JUNTAS = ['l_hip_yaw', 'l_hip_roll', 'l_hip_pitch', 'l_knee', 'l_ank_pitch',
          'l_ank_roll', 'r_hip_yaw', 'r_hip_roll', 'r_hip_pitch', 'r_knee',
          'r_ank_pitch', 'r_ank_roll', 'l_sho_pitch', 'l_sho_roll', 'l_el',
          'r_sho_pitch', 'r_sho_roll', 'r_el', 'head_pan', 'head_tilt']

TELEMETRIA = [
    ('t', None),
    ('com_x', lambda m: m.com.x), ('com_y', lambda m: m.com.y),
    ('com_z', lambda m: m.com.z),
    ('com_vx', lambda m: m.com_vel.x), ('com_vy', lambda m: m.com_vel.y),
    ('sup_x', lambda m: m.com_support.x), ('sup_y', lambda m: m.com_support.y),
    ('sup_z', lambda m: m.com_support.z),
    ('ref_x', lambda m: m.com_ref_support.x), ('ref_y', lambda m: m.com_ref_support.y),
    ('err_x', lambda m: m.com_error.x), ('err_y', lambda m: m.com_error.y),
    ('foot_x', lambda m: m.support_foot.x), ('foot_y', lambda m: m.support_foot.y),
    ('swing_x', lambda m: m.swing_foot.x), ('swing_y', lambda m: m.swing_foot.y),
    ('E_x', lambda m: m.orbital_energy_x), ('E_y', lambda m: m.orbital_energy_y),
    ('dcm_x', lambda m: m.dcm_x), ('dcm_y', lambda m: m.dcm_y),
    ('pitch', lambda m: m.pitch), ('pitch_true', lambda m: m.pitch_true),
    ('roll', lambda m: m.roll), ('roll_true', lambda m: m.roll_true),
    ('phase', lambda m: m.phase), ('support', lambda m: m.support_state),
    ('height', lambda m: m.height), ('z_com_meas', lambda m: m.z_com_measured),
    ('total_mass', lambda m: m.total_mass),
]


def _campo(caminho):
    """Leitor de campo que devolve NaN em bags gravados antes do campo existir."""
    def ler(m):
        try:
            for parte in caminho.split('.'):
                m = getattr(m, parte)
            return m
        except AttributeError:
            return float('nan')
    return ler


# Yaw e pes individuais (investigacao do desvio de direcao). Acrescentados no
# fim, para o CSV continuar compativel com quem le as colunas antigas.
TELEMETRIA += [(nome, _campo(c)) for nome, c in [
    ('yaw_true', 'yaw_true'), ('yaw_rate_true', 'yaw_rate_true'),
    ('l_foot_yaw', 'left_foot_yaw'), ('l_foot_yaw_rate', 'left_foot_yaw_rate'),
    ('r_foot_yaw', 'right_foot_yaw'), ('r_foot_yaw_rate', 'right_foot_yaw_rate'),
    ('l_foot_x', 'left_foot_contact.x'), ('l_foot_y', 'left_foot_contact.y'),
    ('l_foot_z', 'left_foot_contact.z'),
    ('r_foot_x', 'right_foot_contact.x'), ('r_foot_y', 'right_foot_contact.y'),
    ('r_foot_z', 'right_foot_contact.z'),
]]


def extrair(caminho_bag, saida_dir=None):
    base = os.path.splitext(os.path.basename(caminho_bag))[0]
    saida_dir = saida_dir or os.path.dirname(caminho_bag) or '.'
    os.makedirs(saida_dir, exist_ok=True)

    topicos_juntas = {'/marta/%s_position/state' % j: j for j in JUNTAS}
    bag = rosbag.Bag(caminho_bag)

    # t0 comum aos dois arquivos, para os tempos serem comparaveis entre eles
    t0 = None
    for _, _, t in bag.read_messages(topics=['/marta/walk/telemetry']):
        t0 = t.to_sec()
        break
    if t0 is None:
        print('  AVISO: nenhuma mensagem de /marta/walk/telemetry em %s' % caminho_bag)
        t0 = bag.get_start_time()

    # ---- telemetria de marcha ----
    p_tel = os.path.join(saida_dir, base + '_telemetria.csv')
    n_tel = 0
    with open(p_tel, 'w') as f:
        w = csv.writer(f)
        w.writerow([nome for nome, _ in TELEMETRIA])
        for _, msg, t in bag.read_messages(topics=['/marta/walk/telemetry']):
            linha = ['%.4f' % (t.to_sec() - t0)]
            linha += ['%.6f' % fn(msg) for nome, fn in TELEMETRIA[1:]]
            w.writerow(linha)
            n_tel += 1

    # ---- torque e desvio por junta, formato longo ----
    # Formato longo (uma linha por junta por amostra) em vez de largo: as juntas
    # chegam em topicos independentes, sem sincronia, e o formato longo evita
    # inventar um alinhamento que os dados nao tem.
    p_j = os.path.join(saida_dir, base + '_juntas.csv')
    n_j = 0
    with open(p_j, 'w') as f:
        w = csv.writer(f)
        w.writerow(['t', 'junta', 'set_point', 'process_value', 'error',
                    'command', 'p'])
        for topico, msg, t in bag.read_messages(topics=list(topicos_juntas)):
            w.writerow(['%.4f' % (t.to_sec() - t0), topicos_juntas[topico],
                        '%.6f' % msg.set_point, '%.6f' % msg.process_value,
                        '%.6f' % msg.error, '%.6f' % msg.command, '%.2f' % msg.p])
            n_j += 1

    # ---- estabilizador e malha de rumo (/marta/stabilizer_debug) ----
    # O que o CONTROLADOR enxerga: o erro de rumo vem do yaw estimado pelo giro,
    # nao da verdade de solo. Bags anteriores a malha de rumo tem 7 campos; os
    # dois ultimos saem NaN.
    p_e = os.path.join(saida_dir, base + '_estabilizador.csv')
    n_e = 0
    with open(p_e, 'w') as f:
        w = csv.writer(f)
        w.writerow(['t', 'delta_pitch', 'pitch', 'pitch_rate', 'support',
                    'saturated', 'heading_error', 'heading_correction'])
        for _, msg, t in bag.read_messages(topics=['/marta/stabilizer_debug']):
            d = list(msg.data) + [float('nan')] * (9 - len(msg.data))
            w.writerow(['%.4f' % (t.to_sec() - t0)]
                       + ['%.6f' % d[i] for i in (0, 3, 4, 5, 6, 7, 8)])
            n_e += 1

    bag.close()
    print('  %s -> %d amostras de telemetria, %d de juntas, %d do estabilizador'
          % (base, n_tel, n_j, n_e))
    return p_tel, p_j


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    extrair(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
