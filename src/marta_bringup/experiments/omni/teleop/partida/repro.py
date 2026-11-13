import sys, time, math, rospy
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelStates
modo = sys.argv[1]
rospy.init_node('repro_partida')
t_ini = time.time()
def carregados():
    tops = {t for t, _ in rospy.get_published_topics()}
    return sum(1 for t in tops if t.startswith('/marta/') and t.endswith('_position/state'))
rospy.wait_for_service('/gazebo/unpause_physics', timeout=60)
if modo == 'func':
    sys.path.insert(0, '/ws/src/marta_bringup/scripts')
    import teleop_marta
    teleop_marta.esperar_controladores()
if modo == 'espera':
    while carregados() < 20:
        time.sleep(0.2)
print('despausa em %.1f s; controladores de junta carregados: %d' % (time.time() - t_ini, carregados()), flush=True)
rospy.ServiceProxy('/gazebo/unpause_physics', Empty)()
zs, ps = [], []
def cb(m):
    if 'marta' in m.name:
        p = m.pose[m.name.index('marta')]; q = p.orientation
        zs.append(p.position.z)
        ps.append(math.degrees(math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x))))))
rospy.Subscriber('/gazebo/model_states', ModelStates, cb)
time.sleep(25)
print('  25 s: z min %.3f fim %.3f m | pitch min %+.1f max %+.1f fim %+.1f graus | %s'
      % (min(zs), zs[-1], min(ps), max(ps), ps[-1], 'CAIU' if zs[-1] < 0.2 else 'em pe'), flush=True)
