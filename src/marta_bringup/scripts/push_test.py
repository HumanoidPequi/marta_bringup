#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Teste de rejeicao de perturbacao do estabilizador de pitch (Fase 2).

Aplica um pulso de forca no tronco da Marta no Gazebo e mede como o pitch
responde: pico, tempo de acomodacao, oscilacao residual e queda.

Uso (com o stabilizer_test.launch ja rodando):

    rosrun marta_bringup push_test.py                     # um empurrao
    rosrun marta_bringup push_test.py _compare:=true      # sem e com estabilizador
    rosrun marta_bringup push_test.py _force:=8 _duration:=0.15
    rosrun marta_bringup push_test.py _force:=-5          # empurrao por tras

Parametros (~):
    force      (double, 5.0)   forca no eixo x do mundo [N]; positivo empurra
                               o tronco para FRENTE
    duration   (double, 0.1)   duracao do pulso [s]
    settle     (double, 3.0)   tempo de observacao apos o pulso [s]
    tolerance  (double, 2.0)   |pitch| abaixo do qual se considera acomodado [graus]
    pre_wait   (double, 5.0)   espera fixa entre o laco entrar em regime e o pulso [s]
    compare    (bool, false)   roda duas vezes, com o estabilizador desligado e
                               ligado, e imprime as duas respostas lado a lado
    body       (string, marta::base_link)  corpo onde a forca e aplicada

Criterio de aprovacao da Fase 2:
    - o robo nao cai;
    - o pitch volta para dentro da tolerancia;
    - a amplitude das oscilacoes DECRESCE (nao ha divergencia);
    - com o estabilizador ligado, pico e tempo de acomodacao menores que sem.
"""

import math

import rospy
from geometry_msgs.msg import Point, Wrench
from gazebo_msgs.srv import ApplyBodyWrench
from std_msgs.msg import Float64MultiArray

RAD2DEG = 180.0 / math.pi
FALL_THRESHOLD_DEG = 45.0


class PushTest(object):
    def __init__(self):
        # 5 N por 0.1 s = 0.5 N.s. Para uma Marta de ~5 kg isso e ~0.1 m/s de
        # variacao de velocidade do tronco - dentro do que um robo de 30 cm de
        # altura consegue recuperar. Impulsos acima de ~1 N.s derrubam o robo
        # independentemente do estabilizador: nenhuma estrategia de quadril
        # recupera um empurrao que joga o CoM para fora da regiao de captura.
        self.force = rospy.get_param('~force', 5.0)
        self.duration = rospy.get_param('~duration', 0.1)
        self.settle = rospy.get_param('~settle', 3.0)
        self.tolerance = rospy.get_param('~tolerance', 2.0)
        self.compare = rospy.get_param('~compare', False)
        self.pre_wait = rospy.get_param('~pre_wait', 5.0)
        self.body = rospy.get_param('~body', 'marta::base_link')

        self.samples = []       # (t, pitch_deg, pitch_rate, delta_hip_deg, roll_deg)
        self.recording = False
        self.t0 = None
        self.last_delta_deg = 0.0
        self.last_delta_seen = False

        rospy.Subscriber('/marta/torso_attitude', Float64MultiArray, self.attitude_cb)
        rospy.Subscriber('/marta/stabilizer_debug', Float64MultiArray, self.debug_cb)

        rospy.loginfo('Aguardando /gazebo/apply_body_wrench ...')
        rospy.wait_for_service('/gazebo/apply_body_wrench')
        self.apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench',
                                               ApplyBodyWrench)

        self.unpause_gazebo()

    def unpause_gazebo(self):
        """
        O stabilizer_test.launch sobe o Gazebo pausado, para que os
        controladores de junta estejam no ar antes de a gravidade agir. Aqui a
        simulacao e liberada, ja com o robo sendo comandado.
        """
        try:
            from std_srvs.srv import Empty
            rospy.wait_for_service('/gazebo/unpause_physics', timeout=10.0)
            rospy.ServiceProxy('/gazebo/unpause_physics', Empty)()
            rospy.loginfo('Simulacao despausada.')
            rospy.sleep(1.0)
        except Exception as exc:
            rospy.logwarn('Nao foi possivel despausar o Gazebo: %s', exc)

    def attitude_cb(self, msg):
        if not self.recording or len(msg.data) < 4:
            return
        now = rospy.Time.now().to_sec()
        if self.t0 is None:
            self.t0 = now
        self.samples.append((now - self.t0,
                             msg.data[0] * RAD2DEG,
                             msg.data[1],
                             self.last_delta_deg,
                             msg.data[2] * RAD2DEG))

    def debug_cb(self, msg):
        if len(msg.data) >= 1:
            self.last_delta_deg = msg.data[0] * RAD2DEG
            self.last_delta_seen = True

    # ------------------------------------------------------------------

    def wait_for_control_loop(self, timeout=90.0):
        """
        Espera o laco de controle entrar em regime e da um tempo fixo de acomodacao.

        Depois de despausar, o controller_node passa ~10 s no Agacha(), rampando a
        escala de 0 a 1; so entao entra no laco principal. /marta/stabilizer_debug
        e publicado dentro desse laco, entao a primeira mensagem dele e o sinal de
        que a rampa acabou - melhor esperar esse evento do que um tempo chutado.
        """
        rospy.loginfo('Esperando o laco de controle entrar em regime ...')
        deadline = rospy.Time.now().to_sec() + timeout
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < deadline:
            if self.last_delta_seen:
                rospy.loginfo('Laco em regime; acomodando por %.1f s.', self.pre_wait)
                rospy.sleep(self.pre_wait)
                return True
            rate.sleep()
        rospy.logwarn('Nenhuma mensagem em /marta/stabilizer_debug em %.0f s. '
                      'O controller_node esta no ar?', timeout)
        return False

    def push(self):
        wrench = Wrench()
        wrench.force.x = self.force
        try:
            resp = self.apply_wrench(
                body_name=self.body,
                reference_frame='',
                reference_point=Point(0, 0, 0),
                wrench=wrench,
                start_time=rospy.Time(0),
                duration=rospy.Duration(self.duration))
            if not resp.success:
                rospy.logerr('apply_body_wrench falhou: %s', resp.status_message)
                return False
        except rospy.ServiceException as exc:
            rospy.logerr('Erro ao chamar apply_body_wrench: %s', exc)
            return False

        rospy.loginfo('Pulso aplicado: %.1f N por %.2f s em %s (impulso %.2f N.s)',
                      self.force, self.duration, self.body,
                      self.force * self.duration)
        return True

    def run_once(self, label):
        self.samples = []
        self.t0 = None
        self.recording = False

        self.wait_for_control_loop()

        self.samples = []
        self.t0 = None
        self.recording = True
        rospy.sleep(0.5)             # meio segundo de linha de base

        if not self.push():
            return None
        self.push_time = self.samples[-1][0] if self.samples else 0.0

        rospy.sleep(self.duration + self.settle)
        self.recording = False

        return self.analyse(label)

    # ------------------------------------------------------------------

    def analyse(self, label):
        if not self.samples:
            rospy.logerr('Nenhuma amostra de /marta/torso_attitude recebida.')
            return None

        # Só o que veio depois do pulso interessa: antes dele o robô está
        # apenas balançando na postura nominal.
        after = [s for s in self.samples if s[0] >= getattr(self, 'push_time', 0.0)]
        if not after:
            after = self.samples

        pitches = [s[1] for s in after]
        peak = max(pitches, key=abs)
        fell = abs(peak) > FALL_THRESHOLD_DEG
        max_delta = max((s[3] for s in after), key=abs)

        # Tempo de acomodacao: ultimo instante em que |pitch| saiu da tolerancia.
        settling = None
        for sample in reversed(after):
            if abs(sample[1]) > self.tolerance:
                settling = sample[0]
                break

        # Envelope: maior |pitch| em cada terco da janela pos-pulso. E o jeito
        # honesto de ver se a oscilacao decai, sem depender de deteccao de picos.
        t_start, t_end = after[0][0], after[-1][0]
        third = (t_end - t_start) / 3.0
        envelope = []
        roll_envelope = []
        for k in range(3):
            lo = t_start + k * third
            hi = lo + third
            window = [s for s in after if lo <= s[0] <= hi]
            envelope.append(max((abs(s[1]) for s in window), default=0.0))
            roll_envelope.append(max((abs(s[4]) for s in window), default=0.0))

        diverging = envelope[2] > envelope[0]

        print('')
        print('=' * 66)
        print('  %s' % label)
        print('=' * 66)
        print('  pico de pitch .............. %+7.2f graus' % peak)
        print('  correcao maxima no quadril .. %+7.2f graus' % max_delta)
        if fell:
            print('  RESULTADO .................. CAIU (|pitch| > %.0f graus)'
                  % FALL_THRESHOLD_DEG)
        elif settling is None:
            print('  acomodacao ................. nunca saiu da tolerancia')
        else:
            print('  tempo de acomodacao ........ %6.2f s  (|pitch| < %.1f graus)'
                  % (settling, self.tolerance))
        print('  envelope do pitch .......... %.1f -> %.1f -> %.1f graus'
              % tuple(envelope))
        print('  envelope do roll ........... %.1f -> %.1f -> %.1f graus  (sem'
              ' estabilizador ainda; Fase 3)' % tuple(roll_envelope))
        if diverging:
            verdict = 'CRESCENDO - reduza kp ou aumente kd'
        elif envelope[2] > 0.5 * envelope[0]:
            verdict = 'pouco amortecida - aumente kd'
        else:
            verdict = 'amortecida'
        print('  oscilacao .................. %s' % verdict)
        print('=' * 66)

        return {'peak': peak, 'settling': settling, 'fell': fell,
                'diverging': diverging, 'max_delta': max_delta,
                'envelope': envelope}

    # ------------------------------------------------------------------

    def run(self):
        if not self.compare:
            self.run_once('Empurrao de %.1f N por %.2f s' % (self.force, self.duration))
            return

        rospy.loginfo('Modo comparacao: primeiro SEM estabilizador.')
        rospy.set_param('/controller_node/stabilizer_enabled', False)
        rospy.sleep(1.5)             # o controlador rele os ganhos a cada 0,5 s
        without = self.run_once('SEM estabilizador')

        self.reset_robot()

        rospy.loginfo('Agora COM estabilizador.')
        rospy.set_param('/controller_node/stabilizer_enabled', True)
        rospy.sleep(1.5)
        with_stab = self.run_once('COM estabilizador')

        if without and with_stab:
            print('')
            print('  RESUMO')
            print('  %-28s %12s %12s' % ('', 'sem', 'com'))
            print('  %-28s %11.2f %12.2f' % ('pico de pitch [graus]',
                                             without['peak'], with_stab['peak']))
            print('  %-28s %11s %12s' % (
                'acomodacao [s]',
                'caiu' if without['fell'] else ('%.2f' % (without['settling'] or 0.0)),
                'caiu' if with_stab['fell'] else ('%.2f' % (with_stab['settling'] or 0.0))))
            print('')
            if with_stab['fell']:
                print('  >> O robo caiu COM o estabilizador. Revise o SINAL da correcao.')
            elif abs(with_stab['peak']) < abs(without['peak']):
                print('  >> Estabilizador reduziu o pico. Sinal correto.')
            else:
                print('  >> Pico nao melhorou: ganhos baixos demais, ou sinal invertido.')

    def reset_robot(self):
        """Recoloca o robo em pe entre as duas rodadas."""
        try:
            from gazebo_msgs.srv import SetModelState
            from gazebo_msgs.msg import ModelState
            rospy.wait_for_service('/gazebo/set_model_state', timeout=5.0)
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            state = ModelState()
            state.model_name = 'marta'
            state.pose.position.z = 0.3609
            state.pose.orientation.w = 1.0
            set_state(state)
            rospy.sleep(2.0)
        except Exception as exc:
            rospy.logwarn('Nao foi possivel reposicionar o robo: %s', exc)


if __name__ == '__main__':
    rospy.init_node('push_test')
    try:
        PushTest().run()
    except rospy.ROSInterruptException:
        pass
