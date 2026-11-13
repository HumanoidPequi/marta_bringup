#!/usr/bin/env python3
"""Validacao do teleop: reproduz em /marta/cmd_vel uma sequencia com as
transicoes que o teleop_marta.py pode gerar (troca direta reta/curva/giro,
inversao do giro, marcha no lugar, parar e voltar a andar).

    python3 sequencia_cmd.py <omega> [vx]

O relogio e o tempo de simulacao, contado a partir de walk_enabled = true
(o walk_test.py liga a marcha). Publica a 20 Hz, como o teleop.
"""
import sys

import rospy
from geometry_msgs.msg import Twist

W = float(sys.argv[1])
VX = float(sys.argv[2]) if len(sys.argv) > 2 else 0.1
# (inicio [s], rotulo, vx, w); 'PARA'/'ANDA' chaveiam walk_enabled
SEQ = [(0, 'reta', VX, 0), (8, 'curva+', VX, W), (16, 'reta', VX, 0),
       (22, 'giro+', 0, W), (30, 'giro-', 0, -W), (36, 'no lugar', 0, 0),
       (42, 'curva-', VX, -W), (50, 'giro+', 0, W), (56, 'reta', VX, 0),
       (62, 'no lugar', 0, 0), (68, 'PARA', 0, 0), (74, 'ANDA', 0, 0),
       (75, 'reta', VX, 0), (82, 'curva+', VX, W), (90, 'fim', 0, 0)]

rospy.init_node('sequencia_cmd')
pub = rospy.Publisher('/marta/cmd_vel', Twist, queue_size=1)
r = rospy.Rate(20)
while not rospy.is_shutdown() and not rospy.get_param('/controller_node/walk_enabled', False):
    pub.publish(Twist())
    r.sleep()
t0 = rospy.get_time()
i = -1
while not rospy.is_shutdown():
    t = rospy.get_time() - t0
    while i + 1 < len(SEQ) and t >= SEQ[i + 1][0]:
        i += 1
        rot = SEQ[i][1]
        print('%6.1f s  %s' % (t, rot), flush=True)
        if rot == 'PARA':
            rospy.set_param('/controller_node/walk_enabled', False)
        elif rot == 'ANDA':
            rospy.set_param('/controller_node/walk_enabled', True)
    _, rot, vx, w = SEQ[i]
    msg = Twist()
    if rot not in ('PARA', 'fim'):
        msg.linear.x, msg.angular.z = vx, w
    pub.publish(msg)
    r.sleep()
