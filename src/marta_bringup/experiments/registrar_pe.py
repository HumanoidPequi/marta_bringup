#!/usr/bin/env python3
"""Registra a geometria real dos pes (verdade de solo) a cada mensagem de telemetria.

Para investigar o pe de balanco que raspa ou pousa de ponta: alem da altura do
centro da sola, mede a altura da PONTA e do CALCANHAR (cantos de baixo da caixa
de colisao do pe, foot_collision:=box) e o pitch do pe.

Caixa do pe no frame de l_feet_1 / r_feet_1 (marta.xacro): centro em
(0,02445; 0; -0,013), tamanho 0,1399 x 0,070 x 0,048 m.

  rosrun ... registrar_pe.py _out:=/caminho/pe.csv
Colunas: t (s, tempo simulado), support (0 duplo, 1 esq no chao, 2 dir no chao),
torso_pitch (graus, verdade de solo), e para cada pe (l_, r_): z_centro,
z_ponta, z_calcanhar (m, base da sola), pitch (graus, + = ponta para baixo) e,
para hip_pitch, knee e ank_pitch daquele lado, o alvo (_sp) e o angulo real
(_pv) em graus, no MESMO instante (alinhamento entre arquivos nao basta: o pe
gira ~18 graus em ~100 ms no fim do balanco). Se a compensacao de gravidade
estiver publicando (/marta/gravity_comp), _dg e a correcao somada ao alvo
daquela junta (graus); o alvo da IK e _sp - _dg.
"""
import csv
import math

import rospy
from control_msgs.msg import JointControllerState
from gazebo_msgs.msg import LinkStates
from marta_msgs.msg import WalkTelemetry
from std_msgs.msg import Float64MultiArray

CX, CZ = 0.02445, -0.013
HX, HZ = 0.1399 / 2, 0.048 / 2
PES = (('l', 'marta::l_feet_1'), ('r', 'marta::r_feet_1'))
JUNTAS = ('hip_pitch', 'knee', 'ank_pitch')
# indice da junta no vetor Delta de /marta/gravity_comp (depois de w_esq, w_dir, rampa)
IDX_DG = {'hip_pitch': 2, 'knee': 3, 'ank_pitch': 4}


def rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


def z_mundo(pose, px, pz):
    r = rot(pose.orientation)
    return pose.position.z + r[2][0] * px + r[2][2] * pz


class Registrador:
    def __init__(self):
        self.poses = {}
        self.juntas = {}
        self.dg = None
        out = rospy.get_param('~out')
        self.f = open(out, 'w')
        self.w = csv.writer(self.f)
        cab = ['t', 'support', 'torso_pitch']
        for lado, _ in PES:
            cab += ['%s_z_centro' % lado, '%s_z_ponta' % lado, '%s_z_calcanhar' % lado, '%s_pitch' % lado]
            for j in JUNTAS:
                cab += ['%s_%s_sp' % (lado, j), '%s_%s_pv' % (lado, j), '%s_%s_dg' % (lado, j)]
                rospy.Subscriber('/marta/%s_%s_position/state' % (lado, j), JointControllerState,
                                 self.cb_junta, callback_args='%s_%s' % (lado, j), queue_size=1)
        self.w.writerow(cab)
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.cb_links, queue_size=1)
        rospy.Subscriber('/marta/walk/telemetry', WalkTelemetry, self.cb_tel, queue_size=10)
        rospy.Subscriber('/marta/gravity_comp', Float64MultiArray, self.cb_dg, queue_size=1)
        rospy.on_shutdown(self.f.close)

    def cb_links(self, m):
        for _, nome in PES:
            if nome in m.name:
                self.poses[nome] = m.pose[m.name.index(nome)]

    def cb_dg(self, m):
        self.dg = m.data

    def cb_junta(self, m, nome):
        self.juntas[nome] = (m.set_point, m.process_value)

    def cb_tel(self, m):
        if len(self.poses) < 2 or len(self.juntas) < 2 * len(JUNTAS):
            return
        linha = ['%.3f' % rospy.get_time(), m.support_state, '%.2f' % math.degrees(m.pitch_true)]
        for lado, nome in PES:
            p = self.poses[nome]
            zb = CZ - HZ
            z_c = z_mundo(p, CX, zb)
            z_p = z_mundo(p, CX + HX, zb)
            z_h = z_mundo(p, CX - HX, zb)
            pitch = math.degrees(math.asin(max(-1.0, min(1.0, (z_h - z_p) / (2 * HX)))))
            linha += ['%.4f' % z_c, '%.4f' % z_p, '%.4f' % z_h, '%.2f' % pitch]
            for j in JUNTAS:
                sp, pv = self.juntas['%s_%s' % (lado, j)]
                dg = 0.0
                if self.dg is not None and len(self.dg) >= 15:
                    dg = self.dg[3 + IDX_DG[j] + (0 if lado == 'l' else 6)]
                linha += ['%.2f' % math.degrees(sp), '%.2f' % math.degrees(pv), '%.2f' % math.degrees(dg)]
        self.w.writerow(linha)


if __name__ == '__main__':
    rospy.init_node('registrar_pe')
    Registrador()
    rospy.spin()
