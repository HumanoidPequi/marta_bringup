#!/usr/bin/env python3
"""Telemetria derivada para o Foxglove (docs/marcha_omnidirecional.md).

So le e publica; nao toca no controle. Publica tres topicos
(std_msgs/Float64MultiArray; os rotulos vao no layout da mensagem):

/marta/viz/torque_pernas  (13 valores, a ~rate Hz)
    |command| / effort das 12 juntas das pernas, mais 1,0 no fim (a linha do
    limite). command e o torque que o PID pede ANTES de o Gazebo cortar no
    limite do URDF: > 1 significa junta saturada.
/marta/viz/erro_pernas    (12 valores, a ~rate Hz)
    alvo - real [graus] de cada junta (o error do JointControllerState).

    Indices das juntas (0-5 perna esquerda, 6-11 direita):
      0 hip_yaw  1 hip_roll  2 hip_pitch  3 knee  4 ank_pitch  5 ank_roll

/marta/viz/lipm           (11 valores, a cada /marta/walk/telemetry)
    O 3D-LIPM (Kajita et al., IROS 2001) no frame do pe de APOIO, com eixos
    GIRADOS pelo yaw desse pe. A telemetria (WalkTelemetry) usa a origem no pe de
    apoio mas os eixos do mundo: em linha reta da no mesmo, mas no giro ou na
    curva x e y misturam avanco e lateral, e a energia orbital, o DCM e a
    comparacao com a referencia do gerador (que vem no frame do pe) deixam de
    valer. Aqui:
      0 x, 1 y        CoM - pe de apoio [m] (apoio duplo: ponto medio dos pes)
      2 ref_x, 3 ref_y   referencia do gerador no mesmo frame (com_ref_support)
      4 vx, 5 vy      velocidade do CoM [m/s]
      6 E_x, 7 E_y    energia orbital, eq. (28)(29): -(g/2z_c)x^2 + v^2/2
      8 dcm_x, 9 dcm_y   x + t_c vx (z_c e t_c do gerador, /marta/walk/reference)
      10 psi          yaw do frame [graus]

  rosrun marta_bringup viz_marcha.py _effort:=2.5 _rate:=50
"""
import math

import rospy
from control_msgs.msg import JointControllerState
from marta_msgs.msg import WalkReference, WalkTelemetry
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

G = 9.80665
JUNTAS = ['%s_%s' % (lado, j) for lado in 'lr'
          for j in ('hip_yaw', 'hip_roll', 'hip_pitch', 'knee', 'ank_pitch', 'ank_roll')]
LIPM = ['x', 'y', 'ref_x', 'ref_y', 'vx', 'vy', 'E_x', 'E_y', 'dcm_x', 'dcm_y', 'psi_graus']


def msg(dados, rotulo):
    m = Float64MultiArray()
    m.layout.dim = [MultiArrayDimension(label=rotulo, size=len(dados), stride=len(dados))]
    m.data = dados
    return m


def yaw_apoio(m):
    """Yaw do pe de apoio; no apoio duplo, a media circular dos dois pes."""
    if m.support_state == WalkTelemetry.SUPPORT_LEFT:
        return m.left_foot_yaw
    if m.support_state == WalkTelemetry.SUPPORT_RIGHT:
        return m.right_foot_yaw
    return math.atan2(math.sin(m.left_foot_yaw) + math.sin(m.right_foot_yaw),
                      math.cos(m.left_foot_yaw) + math.cos(m.right_foot_yaw))


class VizMarcha:
    def __init__(self):
        self.effort = rospy.get_param('~effort', 2.5)
        taxa = rospy.get_param('~rate', 50.0)
        self.cmd = [float('nan')] * len(JUNTAS)
        self.err = [float('nan')] * len(JUNTAS)
        self.z_c = self.t_c = None
        for i, j in enumerate(JUNTAS):
            rospy.Subscriber('/marta/%s_position/state' % j, JointControllerState,
                             self.cb_junta, callback_args=i, queue_size=1)
        rospy.Subscriber('/marta/walk/reference', WalkReference, self.cb_ref, queue_size=1)
        rospy.Subscriber('/marta/walk/telemetry', WalkTelemetry, self.cb_tel, queue_size=10)
        self.pub_t = rospy.Publisher('/marta/viz/torque_pernas', Float64MultiArray, queue_size=5)
        self.pub_e = rospy.Publisher('/marta/viz/erro_pernas', Float64MultiArray, queue_size=5)
        self.pub_l = rospy.Publisher('/marta/viz/lipm', Float64MultiArray, queue_size=10)
        rospy.Timer(rospy.Duration(1.0 / taxa), self.publica_juntas)
        rospy.loginfo('[viz_marcha] limite %.2f N.m, juntas a %.0f Hz', self.effort, taxa)

    def cb_junta(self, m, i):
        self.cmd[i] = m.command
        self.err[i] = m.error

    def cb_ref(self, m):
        self.z_c, self.t_c = m.z_com, m.t_c

    def publica_juntas(self, _):
        self.pub_t.publish(msg([abs(c) / self.effort for c in self.cmd] + [1.0],
                               ','.join(JUNTAS) + ',limite'))
        self.pub_e.publish(msg([math.degrees(e) for e in self.err], ','.join(JUNTAS)))

    def cb_tel(self, m):
        if not self.z_c or self.z_c <= 0.0:
            return
        psi = yaw_apoio(m)
        c, s = math.cos(psi), math.sin(psi)
        gira = lambda ax, ay: (c * ax + s * ay, -s * ax + c * ay)   # mundo -> frame do pe
        x, y = gira(m.com.x - m.support_foot.x, m.com.y - m.support_foot.y)
        vx, vy = gira(m.com_vel.x, m.com_vel.y)
        w2 = G / self.z_c
        dados = [x, y, m.com_ref_support.x, m.com_ref_support.y, vx, vy,
                 -0.5 * w2 * x * x + 0.5 * vx * vx, -0.5 * w2 * y * y + 0.5 * vy * vy,
                 x + self.t_c * vx, y + self.t_c * vy, math.degrees(psi)]
        self.pub_l.publish(msg(dados, ','.join(LIPM)))


if __name__ == '__main__':
    rospy.init_node('viz_marcha')
    VizMarcha()
    rospy.spin()
