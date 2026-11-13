#!/usr/bin/env python3
"""Registra a pose de todos os modelos do Gazebo a ~20 Hz (diagnostico bola/gol)."""
import rospy, sys
from gazebo_msgs.msg import ModelStates
out = open(sys.argv[1], 'w')
out.write('t,modelo,x,y,z\n')
ult = [-1.0]
def cb(m):
    t = rospy.get_time()
    if t - ult[0] < 0.05:
        return
    ult[0] = t
    for n, p in zip(m.name, m.pose):
        out.write('%.3f,%s,%.4f,%.4f,%.4f\n' % (t, n, p.position.x, p.position.y, p.position.z))
    out.flush()
rospy.init_node('log_modelos', anonymous=True)
rospy.Subscriber('/gazebo/model_states', ModelStates, cb, queue_size=1)
rospy.spin()
