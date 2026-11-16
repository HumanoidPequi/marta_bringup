#!/usr/bin/env python3
"""Calculadora de parametros da marcha a partir das especificacoes do robo.

Consulta a bateria do panorama (docs/panorama_parametros.md, 354 corridas,
servo MX-28) e devolve postura, hip_offset_x, ganho da compensacao e a linha
do roslaunch; o bloco 6 da as velocidades seguras da marcha omnidirecional
(docs/marcha_omnidirecional.md). A mesma logica, em tabela, esta em docs/consulta_parametros.md.

  python3 consulta_parametros.py                 # pergunta cada valor (Enter = Marta atual)
  python3 consulta_parametros.py --massa 3.5 --coxa 95 --canela 95 -n

O que transfere de um robo para outro (e o que esta calculadora usa):
  I = m*g*L/stall   indice de carga, L = coxa + canela; decide se anda
  joelho-alvo       flexao do joelho parado; a postura em termos geometricos
  b/L               alavanca efetiva do joelho de apoio, por joelho-alvo
  CoM - pe          CoM a frente do pe de apoio em apoio simples (sup_x)
O que NAO transfere: os valores de z_com e hip_offset_x em si (dependem da
geometria) - aqui z_com e recalculado pela IK do robo informado.
"""
import argparse
import math
import sys

G = 9.81

# Marta atual (RobotPhysicalParameters.cpp, URDF, launch) [mm, kg, N.m, V]
MARTA = dict(massa=4.78, coxa=98.202, canela=97.4, tornozelo_sola=37.05,
             tronco_quadril_z=59.509, tronco_quadril_y=40.504, quadril_x=3.6,
             y_sep=60.0, stall=2.5, tensao=12.0)
# escala do tronco -> massa total, medida na telemetria (total_mass)
MASSA_FIXA, MASSA_TRONCO = 1.815, 2.966

# ---- dados da bateria (panorama/resumo_corridas.csv) -------------------------
# por nivel de massa: I, celulas da grade 4x4 (z_com x hip_offset_x) que andaram
# e, por joelho-alvo [graus], quantas das 4 colunas de hip_offset_x andaram e
# quais x [mm]; a nota diz de que ganho vem a grade e o que a fase B somou
BATERIA = [
    # massa, I,  anda/16, {joelho: (n de 4, [x que andaram])}, nota
    (2.85, 2.19, 13, {82: (4, [-4, 3.6, 9, 13]), 73: (4, [-4, 3.6, 9, 13]),
                      63: (3, [-4, 3.6, 13]), 53: (2, [-4, 3.6])},
     'grade com ganho 0,5; fase B: 73 graus/x 3,6 andou 3/3 com 0,3, 0,5 e 0,7'),
    (3.30, 2.53, 11, {82: (4, [-4, 3.6, 9, 13]), 73: (3, [-4, 3.6, 9]),
                      63: (2, [-4, 3.6]), 53: (2, [-4, 3.6])},
     'grade com ganho 0,5; fase B: 73 graus/x 3,6 andou 3/3 com 0,3, 2/3 com 0,5, 1/3 com 0,7'),
    (3.56, 2.73, 10, {82: (3, [3.6, 9, 13]), 73: (2, [9, 13]),
                      63: (3, [-4, 3.6, 9]), 53: (2, [-4, 3.6])},
     'grade com ganho 0,3 (com 0,5: 8/16)'),
    (3.80, 2.92, 2, {82: (1, [9]), 73: (0, []), 63: (1, [3.6]), 53: (0, [])},
     'grade com ganho 0,5; fase B: 82 graus/x 9 andou 3/3 com 0,3 e 0,5, 0/3 com 0,7'),
    (4.04, 3.10, 1, {82: (0, []), 73: (1, [9]), 63: (0, []), 53: (1, [9])},
     'grades com ganho 0,5 (73 graus/x 9) e 0,3 (53 graus/x 9): 1/16 cada'),
    (4.28, 3.29, 0, {82: (0, []), 73: (0, []), 63: (0, []), 53: (0, [])},
     'grade com ganho 0,5: 0/16; fase B: 53 graus/x 9 andou 2/3 so com ganho 0,3'),
    (4.78, 3.67, 0, {82: (0, []), 73: (0, []), 63: (0, []), 53: (0, [])},
     'nenhuma combinacao andou (grade 0,5; fase B com 0,3/0,5/0,7)'),
]
# alavanca efetiva do joelho de apoio, b = tau/(m*g), dividida por L (massas
# leves, metade do 1o apoio simples); cresce com a cedencia nas massas pesadas
B_SOBRE_L = {82: 0.302, 73: 0.271, 63: 0.240, 53: 0.204}
# CoM - pe de apoio [mm] nas corridas que andaram: mediana por massa
COM_PE = {2.85: 2.8, 3.30: 1.6, 3.56: 1.7, 3.80: 8.8, 4.04: 12.3}
# a IK estatica superestima o joelho parado da Marta (estado inicial do
# gerador): delta = joelho_ik - joelho_medido, linear no joelho medido
CORR_A, CORR_B, CORR_J0 = 1.73, 0.0457, 53.17


# ---- velocidades seguras (docs/marcha_omnidirecional.md: E1, E4, E5, E6env e
# diagnostico sem gol; docs/consulta_parametros.md passo 6) -------------------
# "anda" = 30 s nos dois pes iniciais. Dois niveis testados: M33 (I 2,53) e
# M38 (I 2,92). Notas: (1) 0,15 so sem o gol do campo; (2) lateral sai torto
# (avanco espurio do gerador, erro de direcao 12-34 graus); (3) quedas no gol
# e na bola, anda sem eles; (4) varia entre corridas; (5) lateral acima do
# limite: A2-A3x de docs/marcha_lateral.md (T 0,5 / y 0,05 anda +-0,09).
VEL_MOVS = [  # (movimento, faixa testada)
    ('reta vx [m/s]', '0,05-0,15'),
    ('giro no lugar w [rad/s]', '+-0,10 a +-0,35'),
    ('curva (vx 0,1; w)', '+-0,10 a +-0,25'),
    ('lateral vy [m/s]', '+-0,025 a +-0,10'),
    ('diagonal (0,1; vy)', '+-0,03'),
    ('combinado (0,1; 0,03; w)', '+-0,175'),
]
VEL = {  # nivel -> (I, com roll, sem roll), na ordem de VEL_MOVS
    'M33': (2.53, ['0,05-0,15 (1)', 'toda', 'toda', 'ate +-0,075 (2)(5)', 'toda', 'toda'],
            ['0,05-0,15 (1)', 'nenhuma', 'so +0,10', '+-0,025 (2)', 'toda', 'so +0,175']),
    'M38': (2.92, ['0,05-0,15 (1)', 'toda', 'toda', 'ate +-0,05 (2)(5)', 'toda (3)', '+0,175 (-0,175: 1 de 2)'],
            ['0,10-0,15 (1)', 'nenhuma', 'so -0,10', 'nenhuma', 'instavel (4)', 'so +0,175']),
}
ROLL_ARGS = ['roll_stabilizer_enabled:=true', 'kp_roll_hip:=0.5', 'kd_roll_hip:=0.075',
             'kp_roll_ankle:=0.5', 'kd_roll_ankle:=0.075']
# kd de pitch 0,06 com roll: E7 (docs/marcha_omnidirecional.md), -10% de pitch RMS,
# 41/44 pares melhores; o default do launch segue 0,03 para reproduzir E0-E7.
PITCH_ARGS = ['kd_pitch:=0.06']


def joelho_para_zcom(joelho, r):
    """z_com [m] que leva ao joelho-alvo parado `joelho` [graus] na IK do robo r."""
    a, b = r['coxa'] / 1000, r['canela'] / 1000
    alvo = joelho + CORR_A + CORR_B * (joelho - CORR_J0)
    c2 = a * a + b * b + 2 * a * b * math.cos(math.radians(alvo))
    y = (r['y_sep'] - r['tronco_quadril_y']) / 1000
    x = r['quadril_x'] / 1000
    if c2 - y * y - x * x <= 0:
        return float('nan')
    return math.sqrt(c2 - y * y - x * x) + (r['tronco_quadril_z'] + r['tornozelo_sola']) / 1000


def interp(xs, ys, x):
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    for i in range(len(xs) - 1):
        if xs[i] <= x <= xs[i + 1]:
            f = (x - xs[i]) / (xs[i + 1] - xs[i])
            return ys[i] + f * (ys[i + 1] - ys[i])


def eh_marta(r):
    return all(abs(r[k] - MARTA[k]) < 1e-6 for k in MARTA if k not in ('massa', 'stall', 'tensao'))


def calcula(r):
    stall = r['stall'] * r['tensao'] / 12.0       # stall ~ proporcional a tensao
    L = (r['coxa'] + r['canela']) / 1000
    I = r['massa'] * G * L / stall
    Is = [b[1] for b in BATERIA]
    anda = interp(Is, [b[2] for b in BATERIA], I)
    viz = min(BATERIA, key=lambda b: abs(b[1] - I))
    if I <= 2.6:
        classe = 'JANELA AMPLA'
    elif I <= 2.8:
        classe = 'JANELA MEDIA'
    elif I <= 3.0:
        classe = 'BORDA (poucas combinacoes andam)'
    else:
        classe = 'NAO ANDA com este servo (a bateria nao achou combinacao estavel)'
    posturas = []
    for j in (82, 73, 63, 53):
        n, xs = viz[3][j]
        tau = r['massa'] * G * B_SOBRE_L[j] * L
        posturas.append(dict(joelho=j, n=n, xs=xs, zcom=joelho_para_zcom(j, r), tau=tau,
                             frac=tau / stall))
    posturas.sort(key=lambda p: (-p['n'], -p['joelho']))
    com_pe = interp(sorted(COM_PE), [COM_PE[k] for k in sorted(COM_PE)], viz[0])
    return dict(stall=stall, L=L, I=I, anda=anda, viz=viz, classe=classe, posturas=posturas,
                com_pe=com_pe)


def relatorio(r, s):
    marta = eh_marta(r)
    p0 = s['posturas'][0]
    linhas = []
    w = linhas.append
    w('')
    w('=' * 72)
    w(' CONSULTA DE PARAMETROS DA MARCHA  (bateria do panorama, MX-28, 0,1 m/s)')
    w('=' * 72)
    w(' 1. Indice de carga')
    w('    stall efetivo  %.2f N.m  (%.2f a 12 V, escalado para %.1f V)' % (s['stall'], r['stall'], r['tensao']))
    w('    L = coxa + canela = %.1f mm' % (1000 * s['L']))
    w('    I = m.g.L/stall = %.2f   ->  %s' % (s['I'], s['classe']))
    w('    referencia: I 2,19 -> 13/16 combinacoes andaram; 2,53 -> 11/16; 2,73 -> 10/16;')
    w('                2,92 -> 2/16; 3,10 -> 1/16; >= 3,29 -> 0/16')
    w('    nivel testado mais proximo: %.2f kg da Marta (I = %.2f), %d/16 andaram'
      % (s['viz'][0], s['viz'][1], s['viz'][2]))
    w('      (%s)' % s['viz'][4])
    w('    interpolando em I: ~%.0f/16 combinacoes devem andar' % s['anda'])
    if s['I'] < BATERIA[0][1] - 0.05:
        w('    (I abaixo do menor testado: espera-se janela ainda maior; extrapolacao)')
    w('')
    w(' 2. Postura (joelho-alvo parado), ordenada pelo que andou no nivel mais proximo')
    w('    joelho | combinacoes que andaram | z_com p/ este robo | tau joelho apoio previsto')
    for p in s['posturas']:
        w('    %4d   |        %d de 4            |     %.3f m        |  %.2f N.m (%3.0f%% do stall)'
          % (p['joelho'], p['n'], p['zcom'], p['tau'], 100 * p['frac']))
    w('    tau previsto = m.g.b com b/L medido nas massas leves; acima de ~90% do stall')
    w('    o joelho de apoio satura no 1o passo e o robo cai para tras.')
    w('    Mais agachado (82-73 graus) cai menos de lado; mais esticado poupa torque.')
    w('')
    w(' 3. hip_offset_x (CoM a frente do pe de apoio)')
    w('    alvo: CoM - pe de apoio ~ %+.0f mm em apoio simples (telemetria sup_x);' % s['com_pe'])
    w('    faixa que andou: 0 a +10 mm; mais massa pede o CoM mais a frente.')
    if marta and p0['xs']:
        w('    nesta geometria, x que andaram com joelho %d: %s mm'
          % (p0['joelho'], ', '.join('%g' % x for x in p0['xs'])))
    elif marta:
        w('    nenhum x andou neste nivel de massa; o URDF (%.1f mm) e so referencia.'
          % r['quadril_x'])
    else:
        w('    geometria diferente da Marta: comece no x do URDF (%.1f mm) e ajuste ate o alvo.'
          % r['quadril_x'])
    w('')
    w(' 4. Compensacao e postura do tronco')
    w('    gravity_comp_gain     %s   (0,3 e 0,5 empatam; 0,7 derruba para a frente)' % (
        '0,3' if s['I'] > 2.6 else '0,3 a 0,5'))
    w('    gravity_comp_joints   hip_roll,knee')
    w('    hip_pitch_offset_deg  -5   (necessario: a compensacao so entra no 1o passo)')
    w('')
    if s['I'] > 3.0:
        w(' >> Com I > 3,0 nenhuma postura andou. Opcoes: servo mais forte (stall maior),')
        w('    tensao maior, menos massa ou perna mais curta. I = 2,6 pede massa <= %.2f kg'
          % (2.6 * s['stall'] / (G * s['L'])))
        w('')
    x_cmd = (min(p0['xs'], key=lambda x: abs(x - 3.6)) if marta and p0['xs'] else r['quadril_x']) / 1000
    ganho = '0.3' if s['I'] > 2.6 else '0.5'
    args = ['z_com:=%.3f' % p0['zcom'], 'hip_offset_x:=%.4f' % x_cmd,
            'gravity_comp_enabled:=true', 'gravity_comp_joints:=hip_roll,knee',
            'gravity_comp_gain:=%s' % ganho, 'hip_pitch_offset_deg:=-5',
            'joint_effort:=%.2f' % s['stall'], 'joint_velocity:=100', 'implicit_damping:=true',
            'joint_damping:=0.43', 'gains:=/ws/src/marta_description/config/controller_mx28.yaml',
            'foot_collision:=box', 'stabilizer_enabled:=true', 'heading_enabled:=true'] + ROLL_ARGS + PITCH_ARGS + [
            'walk_velocity_x:=0.1', 'walk_velocity_y:=0', 'walk_velocity_yaw:=0']
    if marta:
        esc = (r['massa'] - MASSA_FIXA) / MASSA_TRONCO
        args.insert(0, 'torso_mass_scale:=%.3f' % max(esc, 0.0))
    w(' 5. Linha do roslaunch (simulacao)')
    w('    roslaunch marta_bringup stabilizer_test.launch \\')
    for i, a in enumerate(args):
        w('      %s%s' % (a, ' \\' if i < len(args) - 1 else ''))
    if not marta:
        w('    (o modelo simulado continua sendo a Marta: torso_mass_scale e a geometria')
        w('     do URDF nao mudam com estas entradas; z_com/hip_offset_x sao para o robo novo)')
    w('    (roll quatorn K1 confirmado na E5; walk_velocity_* sao lidas so na partida:')
    w('     troque pela velocidade desejada, dentro da tabela do bloco 6)')
    w('')
    if s['I'] <= VEL['M33'][0] + 0.05:
        nivel, nota = 'M33', ('I abaixo do menor testado: espera-se ao menos isto (extrapolacao)'
                              if s['I'] < VEL['M33'][0] - 0.05 else 'nivel testado')
    else:
        nivel = 'M38'
        if abs(s['I'] - VEL['M38'][0]) <= 0.05:
            nota = 'nivel testado'
        elif s['I'] < VEL['M38'][0]:
            nota = 'entre os niveis testados: usa o mais pesado (conservador)'
        else:
            nota = 'EXTRAPOLACAO: I acima do maior testado (2,92); nenhuma velocidade garantida'
    w(' 6. Velocidades seguras (marcha omnidirecional; 30 s nos dois pes iniciais)')
    w('    tabela do nivel %s (I = %.2f): %s' % (nivel, VEL[nivel][0], nota))
    w('    %-26s %-16s %-24s %s' % ('movimento', 'faixa testada', 'com roll', 'sem roll'))
    for (mv, fx), com, sem in zip(VEL_MOVS, VEL[nivel][1], VEL[nivel][2]):
        w('    %-26s %-16s %-24s %s' % (mv, fx, com, sem))
    w('    (1) 0,15 so testada sem o gol do campo (world.world tem gol em x = 3 m)')
    w('    (2) o lateral sai torto: avanco espurio do gerador, erro de direcao 12-34 graus')
    w('    (3) quedas no gol e (inferencia) na bola; sem eles anda   (4) varia entre corridas')
    w('    (5) acima disso cai para tras; step_period:=0.5 y_separation:=0.05 anda +-0,09')
    w('        (proposta, nao default: docs/marcha_lateral.md A3/A3x; ff vx -0,010/-0,015)')
    w('    Sem roll o giro nao anda: use o estabilizador de roll fora da reta.')
    w('')
    w(' Avisos')
    if not marta:
        w('  - geometria diferente da Marta: os valores sao PONTO DE PARTIDA. A bateria variou')
        w('    so a massa do tronco de uma geometria; supoe mesma distribuicao de massa e')
        w('    proporcao coxa/canela parecida.')
        prop = r['coxa'] / r['canela']
        if abs(prop - MARTA['coxa'] / MARTA['canela']) > 0.1:
            w('  - proporcao coxa/canela %.2f muito diferente da Marta (%.2f): b/L pode mudar.'
              % (prop, MARTA['coxa'] / MARTA['canela']))
    w('  - z_com usa a IK do robo + correcao empirica da Marta (a IK estatica superestima o')
    w('    joelho parado em 1,7-3,1 graus); confira o joelho real no log/telemetria.')
    w('  - postura (bloco 2) medida sem roll; com roll, posturas mais esticadas nao foram testadas.')
    w('  - simulacao (Gazebo/ODE, modelo de servo da tese do Maximo); robo real tem folga,')
    w('    aquecimento e queda de tensao.')
    w('=' * 72)
    return '\n'.join(linhas)


CAMPOS = [  # (chave, flag, pergunta)
    ('massa', '--massa', 'massa total [kg]'),
    ('coxa', '--coxa', 'coxa: quadril -> joelho [mm]'),
    ('canela', '--canela', 'canela: joelho -> tornozelo [mm]'),
    ('tornozelo_sola', '--tornozelo-sola', 'tornozelo -> sola, vertical [mm]'),
    ('tronco_quadril_z', '--tronco-quadril-z', 'tronco -> quadril, vertical [mm]'),
    ('tronco_quadril_y', '--tronco-quadril-y', 'tronco -> quadril, lateral [mm]'),
    ('quadril_x', '--quadril-x', 'tronco -> quadril, x do URDF [mm]'),
    ('y_sep', '--y-sep', 'y_separation da marcha (tronco -> pe) [mm]'),
    ('stall', '--stall', 'stall do servo a 12 V [N.m]'),
    ('tensao', '--tensao', 'tensao de operacao [V]'),
]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    for k, flag, txt in CAMPOS:
        ap.add_argument(flag, dest=k, type=float, help='%s (Marta: %g)' % (txt, MARTA[k]))
    ap.add_argument('-n', '--sem-perguntas', action='store_true',
                    help='nao pergunta; o que faltar fica com o valor da Marta atual')
    op = ap.parse_args()
    r = {}
    interativo = not op.sem_perguntas and sys.stdin.isatty()
    if interativo:
        print('Especificacoes do robo (Enter mantem o valor da Marta atual entre colchetes)')
    for k, flag, txt in CAMPOS:
        v = getattr(op, k)
        while v is None and interativo:
            s = input('  %-46s [%g]: ' % (txt, MARTA[k])).strip().replace(',', '.')
            if not s:
                v = MARTA[k]
                break
            try:
                v = float(s)
            except ValueError:
                print('    numero invalido')
        r[k] = MARTA[k] if v is None else v
    print(relatorio(r, calcula(r)))


if __name__ == '__main__':
    main()
