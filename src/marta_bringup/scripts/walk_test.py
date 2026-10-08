#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Teste do estabilizador de pitch DURANTE A CAMINHADA (Fase 4).

Sequencia: despausa o Gazebo, espera o robo assentar em pe, liga a marcha em
tempo de execucao, mede por alguns segundos e manda parar.

Uso (com o stabilizer_test.launch ja rodando):

    rosrun marta_bringup walk_test.py
    rosrun marta_bringup walk_test.py _duration:=20 _velocity_x:=0.05

Para comparar com e sem estabilizador, suba a simulacao duas vezes - depois de
cair o robo nao volta a uma condicao limpa:

    roslaunch marta_bringup stabilizer_test.launch gui:=false stabilizer_enabled:=false
    roslaunch marta_bringup stabilizer_test.launch gui:=false stabilizer_enabled:=true

Parametros (~):
    duration    (double, 15.0) tempo de marcha medido [s]
    velocity_x  (double, 0.1)  velocidade de avanco [m/s]
    settle      (double, 5.0)  espera fixa entre o laco entrar em regime e a
                               marcha comecar [s]
    bag_out     (string, '')   se informado, grava um rosbag da corrida com toda
                               a telemetria (marcha, LIPM, torque e desvio por
                               junta). E a fonte das figuras do paper.
    align_roll  (bool, true)   espera ate 2 s a mais para o balanco lateral
                               chegar ao extremo (roll_rate ~ 0), para todas as
                               corridas partirem do mesmo ponto do ciclo

O que e medido, e por que:
    - caiu ou nao, e em quanto tempo;
    - distancia percorrida: um estabilizador que "segura" o robo as custas de
      travar a marcha nao serve;
    - RMS e pico de pitch: e o que o estabilizador deveria reduzir;
    - RMS e pico de roll: o plano frontal ainda NAO tem realimentacao (Fase 3),
      entao serve de referencia do que sobra;
    - correcao no quadril e fracao do tempo em saturacao: saturar muito durante
      a marcha significa que os ganhos estao altos para o regime dinamico;
    - trocas de apoio observadas: confirma que a maquina de estados esta
      ciclando e que a correcao esta migrando de perna.
"""

import math

import rospy
from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray
from marta_msgs.msg import WalkTelemetry

RAD2DEG = 180.0 / math.pi
FALL_THRESHOLD_DEG = 45.0
SUPPORT_NAMES = {0: 'duplo', 1: 'esquerdo', 2: 'direito'}


class WalkTest(object):
    def __init__(self):
        self.duration = rospy.get_param('~duration', 15.0)
        self.velocity_x = rospy.get_param('~velocity_x', 0.1)
        self.settle = rospy.get_param('~settle', 5.0)
        self.align_roll = rospy.get_param('~align_roll', True)

        self.samples = []      # (t, pitch_deg, roll_deg, delta_deg, sat, support)
        self.recording = False
        self.t0 = None
        self.debug = None
        self.position = None
        self.attitude = None
        self.sim_clock = None
        self.rtf_marks = None
        self.diverged = False
        # Estado bruto do LIPM, nao a energia ja pronta: a energia depende de
        # QUAL zc se usa como regua, e essa escolha pertence a analise.
        self.lipm = []        # (support, x, vx, y, vy, z_medido)
        self.fall_reason = None
        self.fall_state = None
        self.truth = None
        self.truth_samples = []
        # Yaw de verdade de solo e a trajetoria da base, para medir o desvio de
        # direcao: (t, x, y, yaw desenrolado em rad).
        self.yaw = None
        self.yaw0 = None
        self.traj = []
        self.rows = []
        self.csv_out = rospy.get_param('~csv_out', '')
        self.bag_out = rospy.get_param('~bag_out', '')
        self.bag_proc = None
        self.campanha_csv = rospy.get_param('~campanha_csv', '')
        self.tag = rospy.get_param('~tag', '')
        self.tau = rospy.get_param('~tau', float('nan'))
        self.rep = rospy.get_param('~rep', 0)

        rospy.Subscriber('/marta/torso_attitude', Float64MultiArray, self.attitude_cb)
        rospy.Subscriber('/marta/stabilizer_debug', Float64MultiArray, self.debug_cb)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_cb)
        rospy.Subscriber('/clock', Clock, self.clock_cb)
        rospy.Subscriber('/marta/walk/telemetry', WalkTelemetry, self.lipm_cb)

        self.unpause_gazebo()

    # ------------------------------------------------------------------

    def unpause_gazebo(self):
        try:
            from std_srvs.srv import Empty
            rospy.wait_for_service('/gazebo/unpause_physics', timeout=15.0)
            rospy.ServiceProxy('/gazebo/unpause_physics', Empty)()
            rospy.loginfo('Simulacao despausada.')
            rospy.sleep(1.0)
        except Exception as exc:
            rospy.logwarn('Nao foi possivel despausar o Gazebo: %s', exc)

    def attitude_cb(self, msg):
        if len(msg.data) < 4:
            return
        self.attitude = msg.data
        if not self.recording:
            return
        now = rospy.Time.now().to_sec()
        if self.t0 is None:
            self.t0 = now
        debug = self.debug or [0.0] * 7
        self.samples.append((now - self.t0,
                             msg.data[0] * RAD2DEG,
                             msg.data[2] * RAD2DEG,
                             debug[0] * RAD2DEG,
                             debug[6],
                             int(debug[5])))
        # Tabela alinhada para o CSV: a verdade de solo chega numa taxa
        # diferente, entao cada amostra de atitude leva a leitura mais recente.
        tp, tr = self.truth if self.truth else (float('nan'), float('nan'))
        z = self.position[2] if self.position else float('nan')
        self.rows.append((now - self.t0, msg.data[0] * RAD2DEG, tp,
                          msg.data[2] * RAD2DEG, tr, z,
                          debug[0] * RAD2DEG, int(debug[5]), int(debug[6] > 0.5)))

    def debug_cb(self, msg):
        if len(msg.data) >= 7:
            self.debug = msg.data

    def lipm_cb(self, msg):
        if self.recording:
            self.lipm.append((msg.support_state, msg.com_support.x, msg.com_vel.x,
                              msg.com_support.y, msg.com_vel.y, msg.z_com_measured))

    ZC_GERADOR = 0.297509   # o que o gerador assume (torso-pe da IK)

    def deriva_energia(self, zc=None, trim=0.2):
        """
        Amplitude media de E_x dentro de cada fase de apoio simples.

        Sob o 3D-LIPM a energia orbital e invariante no tempo, entao em apoio
        simples - ZMP fixo no pe, sem entrada - ela deveria ser constante. A
        amplitude que sobra mede o quanto a realidade viola o modelo.

        O zc importa e nao e obvio qual usar, porque sao DUAS perguntas
        diferentes:
          zc = 0.2975 (do gerador) -> "o robo obedece ao LIPM que o gerador
               assumiu?"  Mas esse zc esta errado em 36%, entao a grandeza nao
               seria invariante nem para um robo LIPM perfeito.
          zc = altura medida do CoM -> "o robo obedece a ALGUM LIPM?"  E a regua
               fisicamente correta, e a que deve ser usada para comparar
               variantes do gerador entre si.
        Usa-se a media da corrida, nao o valor instantaneo: a invariancia da
        eq. (28)(29) so vale para zc constante.

        As bordas de cada fase sao aparadas porque ali a origem do frame acabou
        de trocar de pe.
        """
        if not self.lipm:
            return float('nan'), 0
        if zc is None:
            zc = sum(r[5] for r in self.lipm) / len(self.lipm)
        w2 = 9.80665 / zc
        fases, atual, lado = [], [], None
        for s, x, vx, _y, _vy, _z in self.lipm:
            ex = -0.5 * w2 * x * x + 0.5 * vx * vx
            if s in (1, 2):
                if lado is None or s != lado:
                    if len(atual) > 12:
                        fases.append(atual)
                    atual, lado = [], s
                atual.append(ex)
            else:
                if len(atual) > 12:
                    fases.append(atual)
                atual, lado = [], None
        if len(atual) > 12:
            fases.append(atual)
        amplitudes = []
        for e in fases:
            k = int(len(e) * trim)
            e = e[k:len(e) - k] if k > 0 else e
            if e:
                amplitudes.append(max(e) - min(e))
        if not amplitudes:
            return float('nan'), 0
        return sum(amplitudes) / len(amplitudes), len(amplitudes)

    PASSADA_S = 0.8   # dois passos de period = 0.4 s: um ciclo completo esq+dir

    def metricas_yaw(self):
        """
        Desvio de direcao pela verdade de solo, relativo ao rumo no inicio da
        marcha (yaw0). Substitui hypot(dx, dy), que mistura avanco com deriva.

          psi_final, psi_pico   giro do corpo [graus]
          psi_taxa              inclinacao de psi(t) por minimos quadrados [graus/s]
          avanco, lateral       deslocamento da base no frame do rumo inicial [m]
          caranguejo            media de (rumo da trajetoria - yaw do corpo) por
                                passada [graus]. Perto de zero com psi grande ->
                                o robo GIRA; psi ~ 0 com lateral grande -> anda
                                de lado com o corpo alinhado.
        """
        nan = float('nan')
        vazio = dict(psi_final=nan, psi_pico=nan, psi_taxa=nan, avanco=nan,
                     lateral=nan, caranguejo=nan)
        if len(self.traj) < 2 or self.yaw0 is None:
            return vazio
        t0 = self.traj[0][0]
        ts = [r[0] - t0 for r in self.traj]
        psi = [(r[3] - self.yaw0) * RAD2DEG for r in self.traj]

        n = len(ts)
        mt, mp = sum(ts) / n, sum(psi) / n
        stt = sum((t - mt) ** 2 for t in ts)
        taxa = sum((t - mt) * (p - mp) for t, p in zip(ts, psi)) / stt if stt > 0 else nan

        c, s = math.cos(self.yaw0), math.sin(self.yaw0)
        dx = self.traj[-1][1] - self.traj[0][1]
        dy = self.traj[-1][2] - self.traj[0][2]

        # Rumo da trajetoria por passada inteira, para tirar o balanco lateral.
        angulos = []
        i = 0
        for j in range(n):
            if ts[j] - ts[i] < self.PASSADA_S:
                continue
            ddx = self.traj[j][1] - self.traj[i][1]
            ddy = self.traj[j][2] - self.traj[i][2]
            if math.hypot(ddx, ddy) > 0.005:
                chi = math.atan2(ddy, ddx)
                yaw_medio = sum(r[3] for r in self.traj[i:j + 1]) / (j + 1 - i)
                e = chi - yaw_medio
                angulos.append(math.atan2(math.sin(e), math.cos(e)) * RAD2DEG)
            i = j

        return dict(psi_final=psi[-1], psi_pico=max(psi, key=abs), psi_taxa=taxa,
                    avanco=c * dx + s * dy, lateral=-s * dx + c * dy,
                    caranguejo=sum(angulos) / len(angulos) if angulos else nan)

    def clock_cb(self, msg):
        self.sim_clock = msg.clock.to_sec()

    def model_cb(self, msg):
        if 'marta' not in msg.name:
            return
        pose = msg.pose[msg.name.index('marta')]
        self.position = (pose.position.x, pose.position.y, pose.position.z)

        # Verdade de solo da atitude, direto do Gazebo. E indispensavel quando o
        # estimador pode estar enviesado: com bias de giroscopio a estimativa
        # converge para zero enquanto o robo de verdade fica inclinado, e uma
        # metrica calculada sobre a estimativa fica cega para isso.
        q = pose.orientation
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        self.truth = (math.asin(sinp) * RAD2DEG, math.atan2(sinr, cosr) * RAD2DEG)

        # Yaw desenrolado: o robo pode girar mais de 180 graus numa corrida.
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self.yaw is not None:
            yaw = self.yaw + math.atan2(math.sin(yaw - self.yaw), math.cos(yaw - self.yaw))
        self.yaw = yaw

        if self.recording:
            self.truth_samples.append(self.truth)
            self.traj.append((rospy.Time.now().to_sec(), pose.position.x,
                              pose.position.y, yaw))

    # ------------------------------------------------------------------

    def wait_for_control_loop(self, timeout=90.0):
        """
        Espera o laco de controle entrar em regime.

        Depois de despausar, o controller_node passa ~10 s no Agacha(), rampando
        a escala de 0 a 1; so entao entra no laco principal. /marta/stabilizer_debug
        e publicado dentro desse laco, entao a primeira mensagem dele e o sinal
        de que a rampa acabou. E melhor esperar por esse evento do que por um
        tempo fixo chutado.
        """
        rospy.loginfo('Esperando o laco de controle entrar em regime ...')
        deadline = rospy.Time.now().to_sec() + timeout
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < deadline:
            if self.debug is not None:
                rospy.loginfo('Laco em regime; assentando por %.1f s.', self.settle)
                rospy.sleep(self.settle)
                self.align_lateral_phase()
                if self.attitude:
                    rospy.loginfo('Partindo com pitch = %.2f graus, roll = %.2f graus.',
                                  self.attitude[0] * RAD2DEG, self.attitude[2] * RAD2DEG)
                return True
            rate.sleep()
        rospy.logwarn('Nenhuma mensagem em /marta/stabilizer_debug em %.0f s. '
                      'O controller_node esta no ar?', timeout)
        return False

    def align_lateral_phase(self, rate_tolerance=0.15, timeout=2.0):
        """
        Alinha a FASE do balanco lateral antes de comecar a marcha.

        O balanco lateral do robo parado e um ciclo-limite: nao decai, so oscila -
        por isso nao adianta esperar ele "assentar". Pior: ele nao oscila em torno
        de zero, e sim em torno de um pendor de ~10 graus para a esquerda, entao
        esperar |roll| pequeno nunca converge. O que da para alinhar e a fase:
        espera a velocidade angular de roll cruzar zero, ou seja, o extremo do
        balanco. Custa menos de meio periodo (~0.4 s) e faz todas as corridas
        partirem do mesmo ponto do ciclo. Desligue com _align_roll:=false.
        """
        if not self.align_roll:
            return
        deadline = rospy.Time.now().to_sec() + timeout
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < deadline:
            if self.attitude and abs(self.attitude[3]) < rate_tolerance:
                return
            rate.sleep()
        rospy.logwarn('roll_rate nao cruzou zero em %.0f s; partindo assim mesmo.',
                      timeout)

    # Juntas cujo controlador publica torque e desvio. Os 20 topicos */state
    # existem desde sempre (effort_controllers/JointPositionController): sao a
    # fonte de M6 (torque, campo 'command') e M7 (desvio, campo 'error').
    JUNTAS = ['l_hip_yaw', 'l_hip_roll', 'l_hip_pitch', 'l_knee', 'l_ank_pitch',
              'l_ank_roll', 'r_hip_yaw', 'r_hip_roll', 'r_hip_pitch', 'r_knee',
              'r_ank_pitch', 'r_ank_roll', 'l_sho_pitch', 'l_sho_roll', 'l_el',
              'r_sho_pitch', 'r_sho_roll', 'r_el', 'head_pan', 'head_tilt']

    def start_bag(self):
        """
        Grava a corrida inteira. /gazebo/link_states fica de FORA de proposito:
        sao ~700 KB/s e o walk_telemetry_node ja destila dele tudo que importa.
        Os 20 topicos */state custam ~6 KB/s cada, o que e barato.
        """
        if not self.bag_out:
            return
        import subprocess, os
        os.makedirs(os.path.dirname(self.bag_out) or '.', exist_ok=True)
        topicos = ['/marta/walk/telemetry', '/marta/walk/reference',
                   '/marta/torso_attitude', '/marta/stabilizer_debug',
                   '/marta/joint_states', '/marta/imu', '/clock']
        topicos += ['/marta/%s_position/state' % j for j in self.JUNTAS]
        self.bag_proc = subprocess.Popen(
            ['rosbag', 'record', '-O', self.bag_out, '__name:=walk_bag'] + topicos,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        rospy.loginfo('Gravando %d topicos em %s', len(topicos), self.bag_out)
        rospy.sleep(1.5)   # deixa o rosbag assinar antes da marcha comecar

    def stop_bag(self):
        if self.bag_proc is None:
            return
        import subprocess
        # SIGINT para o rosbag fechar o arquivo com indice; matar com SIGKILL
        # deixaria o bag "ativo" e ilegivel sem reindex.
        subprocess.call(['rosnode', 'kill', '/walk_bag'],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            self.bag_proc.wait(timeout=10)
        except Exception:
            self.bag_proc.terminate()
        rospy.loginfo('Bag fechado: %s', self.bag_out)

    def set_walking(self, walking):
        rospy.set_param('/controller_node/walk_enabled', walking)
        # O controlador rele os parametros a cada 0,5 s.
        rospy.sleep(1.0)

    # ------------------------------------------------------------------

    def run(self):
        self.wait_for_control_loop()

        rospy.set_param('/controller_node/walk_velocity_x', self.velocity_x)
        start_pos = self.position

        self.start_attitude = self.attitude
        rospy.loginfo('Iniciando a marcha a %.2f m/s por %.0f s ...',
                      self.velocity_x, self.duration)
        self.samples = []
        self.truth_samples = []
        self.rows = []
        self.lipm = []
        self.traj = []
        self.yaw0 = self.yaw
        self.t0 = None
        self.recording = True
        self.start_bag()
        self.set_walking(True)

        # Marca de tempo simulado x tempo real. Se o RTF cair muito, a corrida
        # foi limitada por CPU e nao e comparavel com as outras - isso precisa
        # aparecer no relatorio, nao ser descoberto depois.
        import time as _time
        self.rtf_marks = (self.sim_clock, _time.time())

        # Observa em fatias, para detectar a queda cedo e nao esperar a janela toda.
        fell_at = None
        rate = rospy.Rate(10)
        deadline = rospy.Time.now().to_sec() + self.duration
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < deadline:
            # Deteccao de queda pela VERDADE de solo, nos tres eixos. Usar a
            # estimativa de pitch aqui seria duplamente errado: ela pode estar
            # deslocada em ate 20 graus no regime saturado, e e cega para queda
            # lateral. z e o arbitro final - se o tronco desceu, caiu.
            if self.truth is not None and self.position is not None:
                tp, tr = self.truth
                z = self.position[2]
                motivo = None
                if abs(tp) > FALL_THRESHOLD_DEG:
                    motivo = 'pitch'
                elif abs(tr) > FALL_THRESHOLD_DEG:
                    motivo = 'roll'
                elif z < 0.15:
                    motivo = 'altura'
                if motivo:
                    fell_at = self.rows[-1][0] if self.rows else 0.0
                    self.fall_reason = motivo
                    self.fall_state = (tp, tr, z)
                    rospy.logwarn('Caiu em t = %.1f s por %s '
                                  '(pitch %.1f, roll %.1f, z %.3f).',
                                  fell_at, motivo, tp, tr, z)
                    break
            if self.attitude and (math.isnan(self.attitude[0])
                                  or abs(self.attitude[0] * RAD2DEG) > 180.0):
                rospy.logerr('Atitude invalida (NaN ou |pitch| > 180 graus): a '
                             'fisica da simulacao divergiu. '
                             'Corrida INVALIDA - nao compare com as outras.')
                self.diverged = True
                break
            rate.sleep()

        self.recording = False
        end_pos = self.position
        self.set_walking(False)
        self.stop_bag()

        self.escrever_csv()
        self.report(start_pos, end_pos, fell_at)
        self.escrever_linha_campanha(start_pos, end_pos, fell_at)

    # ------------------------------------------------------------------

    def escrever_csv(self):
        """Grava a serie temporal da corrida, se ~csv_out foi informado."""
        if not self.csv_out or not self.rows:
            return
        import csv, os
        os.makedirs(os.path.dirname(self.csv_out) or '.', exist_ok=True)
        with open(self.csv_out, 'w') as f:
            w = csv.writer(f)
            w.writerow(['t', 'pitch_est', 'pitch_verdade', 'roll_est',
                        'roll_verdade', 'z', 'delta_quadril', 'apoio', 'saturado'])
            for r in self.rows:
                w.writerow(['%.4f' % r[0]] + ['%.4f' % v for v in r[1:7]]
                           + [r[7], r[8]])
        rospy.loginfo('Serie temporal gravada em %s (%d amostras).',
                      self.csv_out, len(self.rows))

    def escrever_linha_campanha(self, start_pos, end_pos, fell_at):
        """
        Acrescenta uma linha ao CSV da campanha. Escrever daqui, e nao parsear a
        saida de texto depois, evita que a analise dependa do formato do console.
        """
        if not self.campanha_csv or not self.rows:
            return
        import csv, os

        def rms(v):
            v = [x for x in v if not math.isnan(x)]
            return math.sqrt(sum(x * x for x in v) / len(v)) if v else float('nan')

        def media(v):
            v = [x for x in v if not math.isnan(x)]
            return sum(v) / len(v) if v else float('nan')

        pv = [r[2] for r in self.rows]
        rv = [r[4] for r in self.rows]
        pe = [r[1] for r in self.rows]
        zs = [r[5] for r in self.rows if not math.isnan(r[5])]
        dl = [r[6] for r in self.rows]

        dist = float('nan')
        if start_pos and end_pos:
            dist = math.hypot(end_pos[0] - start_pos[0], end_pos[1] - start_pos[1])

        trocas = sum(1 for i in range(1, len(self.rows))
                     if self.rows[i][7] != self.rows[i - 1][7])
        deriva_e, n_fases = self.deriva_energia()
        deriva_ger, _ = self.deriva_energia(zc=self.ZC_GERADOR)
        zc_med = (sum(r[5] for r in self.lipm) / len(self.lipm)) if self.lipm else float('nan')
        print('  deriva de E_x, zc medido ... %6.4f m2/s2  (zc=%.4f m, %d fases)'
              % (deriva_e, zc_med, n_fases))
        print('  deriva de E_x, zc gerador .. %6.4f m2/s2  (zc=%.4f m)'
              % (deriva_ger, self.ZC_GERADOR))

        rtf = self.real_time_factor()
        divergiu = int(getattr(self, 'diverged', False)
                       or any(math.isnan(x) or abs(x) > 180.0 for x in pv))

        linha = dict(
            tag=self.tag, tau=self.tau, stab=int(rospy.get_param(
                '/controller_node/stabilizer_enabled', True)), rep=self.rep,
            caiu=int(fell_at is not None), motivo=self.fall_reason or '',
            t_queda='%.2f' % fell_at if fell_at is not None else '',
            dur='%.2f' % self.rows[-1][0],
            rms_pitch_v='%.3f' % rms(pv), pico_pitch_v='%.3f' % max(pv, key=abs),
            media_pitch_v='%.3f' % media(pv),
            rms_roll_v='%.3f' % rms(rv), pico_roll_v='%.3f' % max(rv, key=abs),
            rms_pitch_est='%.3f' % rms(pe),
            vies='%.3f' % (media(pv) - media(pe)),
            dist='%.4f' % dist, z_min='%.4f' % (min(zs) if zs else float('nan')),
            delta_pico='%.3f' % max(dl, key=abs),
            frac_sat='%.4f' % (sum(r[8] for r in self.rows) / float(len(self.rows))),
            trocas=trocas, rtf='%.3f' % rtf if rtf else '', divergiu=divergiu,
            deriva_energia='%.5f' % self.deriva_energia()[0],
            deriva_energia_ger='%.5f' % self.deriva_energia(zc=self.ZC_GERADOR)[0],
            zc_medido='%.4f' % ((sum(r[5] for r in self.lipm) / len(self.lipm))
                                if self.lipm else float('nan')),
            n_fases=self.deriva_energia()[1],
            csf=rospy.get_param('/controller_node/com_swing_factor', 1.0))
        my = self.metricas_yaw()
        linha.update(
            psi_final='%.3f' % my['psi_final'], psi_pico='%.3f' % my['psi_pico'],
            psi_taxa='%.4f' % my['psi_taxa'], avanco='%.4f' % my['avanco'],
            lateral='%.4f' % my['lateral'], caranguejo='%.3f' % my['caranguejo'],
            foot_mu=rospy.get_param('~foot_mu', ''))

        # Uma campanha antiga tem cabecalho sem as colunas novas; acrescentar
        # linhas nela desalinharia tudo. Grava num arquivo irmao.
        destino = self.campanha_csv
        if os.path.exists(destino):
            with open(destino) as f:
                cabecalho = f.readline().strip().split(',')
            if cabecalho != list(linha.keys()):
                destino = os.path.splitext(destino)[0] + '_yaw.csv'
                rospy.logwarn('%s tem cabecalho antigo; gravando em %s',
                              self.campanha_csv, destino)
        self.campanha_csv = destino

        novo = not os.path.exists(self.campanha_csv)
        with open(self.campanha_csv, 'a') as f:
            w = csv.DictWriter(f, fieldnames=list(linha.keys()))
            if novo:
                w.writeheader()
            w.writerow(linha)
        rospy.loginfo('Linha acrescentada em %s', self.campanha_csv)

    def real_time_factor(self):
        """Tempo simulado por segundo de tempo real, medido na janela da marcha."""
        import time as _time
        if not self.rtf_marks or self.rtf_marks[0] is None or self.sim_clock is None:
            return None
        d_sim = self.sim_clock - self.rtf_marks[0]
        d_real = _time.time() - self.rtf_marks[1]
        if d_real <= 0.0:
            return None
        return d_sim / d_real

    def report(self, start_pos, end_pos, fell_at):
        if not self.samples:
            rospy.logerr('Nenhuma amostra recebida. O torso_attitude_node esta no ar?')
            return

        pitches = [s[1] for s in self.samples]
        rolls = [s[2] for s in self.samples]
        deltas = [s[3] for s in self.samples]
        sat_fraction = sum(1 for s in self.samples if s[4] > 0.5) / float(len(self.samples))

        transitions = 0
        for i in range(1, len(self.samples)):
            if self.samples[i][5] != self.samples[i - 1][5]:
                transitions += 1

        def rms(values):
            return math.sqrt(sum(v * v for v in values) / len(values))

        distance = 0.0
        if start_pos and end_pos:
            distance = math.hypot(end_pos[0] - start_pos[0], end_pos[1] - start_pos[1])

        enabled = rospy.get_param('/controller_node/stabilizer_enabled', True)
        kp = rospy.get_param('/controller_node/kp_pitch', 0.0)
        kd = rospy.get_param('/controller_node/kd_pitch', 0.0)

        print('')
        print('=' * 68)
        print('  MARCHA  -  estabilizador %s  (kp=%.3f  kd=%.3f)'
              % ('LIGADO' if enabled else 'DESLIGADO', kp, kd))
        print('=' * 68)
        if (getattr(self, 'diverged', False)
                or any(math.isnan(v) or abs(v) > 180.0 for v in pitches)):
            print('  RESULTADO .................. FISICA DIVERGIU (NaN) - corrida invalida')
        elif fell_at is not None:
            print('  RESULTADO .................. CAIU em %.1f s  por %s' 
                  % (fell_at, self.fall_reason or '?'))
            if self.fall_state:
                print('  estado na queda ............ pitch %+.1f  roll %+.1f  z %.3f'
                      % self.fall_state)
        else:
            print('  RESULTADO .................. andou %.0f s sem cair'
                  % self.samples[-1][0])
        print('  distancia percorrida ....... %6.3f m  (hypot; mistura avanco e deriva)'
              % distance)
        my = self.metricas_yaw()
        print('  avanco / lateral ........... %+6.3f / %+6.3f m  (frame do rumo inicial)'
              % (my['avanco'], my['lateral']))
        print('  giro do corpo final / pico . %+6.1f / %+6.1f graus  (%+.2f graus/s)'
              % (my['psi_final'], my['psi_pico'], my['psi_taxa']))
        print('  caranguejo (rumo - yaw) .... %+6.1f graus' % my['caranguejo'])
        print('  pitch  RMS / pico .......... %6.2f / %6.2f graus'
              % (rms(pitches), max(pitches, key=abs)))
        print('  roll   RMS / pico .......... %6.2f / %6.2f graus   (sem'
              ' estabilizador; Fase 3)' % (rms(rolls), max(rolls, key=abs)))
        print('  correcao no quadril pico ... %6.2f graus' % max(deltas, key=abs))
        print('  tempo em saturacao ......... %6.1f %%' % (100.0 * sat_fraction))
        print('  trocas de apoio ............ %d' % transitions)
        if self.truth_samples:
            tp = [t[0] for t in self.truth_samples]
            tr = [t[1] for t in self.truth_samples]
            print('  VERDADE pitch RMS / medio .. %6.2f / %+6.2f graus'
                  % (rms(tp), sum(tp) / len(tp)))
            print('  VERDADE roll  RMS / medio .. %6.2f / %+6.2f graus'
                  % (rms(tr), sum(tr) / len(tr)))
            vies = sum(tp) / len(tp) - sum(pitches) / len(pitches)
            print('  vies da estimativa ......... %+6.2f graus (verdade - estimado)'
                  % vies)

        deriva_e, n_fases = self.deriva_energia()
        deriva_ger, _ = self.deriva_energia(zc=self.ZC_GERADOR)
        zc_med = (sum(r[5] for r in self.lipm) / len(self.lipm)) if self.lipm else float('nan')
        print('  deriva de E_x, zc medido ... %6.4f m2/s2  (zc=%.4f m, %d fases)'
              % (deriva_e, zc_med, n_fases))
        print('  deriva de E_x, zc gerador .. %6.4f m2/s2  (zc=%.4f m)'
              % (deriva_ger, self.ZC_GERADOR))

        rtf = self.real_time_factor()
        if rtf is not None:
            flag = '' if rtf > 0.85 else '   <-- CORRIDA LIMITADA POR CPU'
            print('  fator de tempo real ........ %6.3f%s' % (rtf, flag))
        if getattr(self, 'start_attitude', None):
            print('  partiu de .................. pitch %+.2f  roll %+.2f graus'
                  % (self.start_attitude[0] * RAD2DEG, self.start_attitude[2] * RAD2DEG))
        print('=' * 68)


if __name__ == '__main__':
    rospy.init_node('walk_test')
    try:
        WalkTest().run()
    except rospy.ROSInterruptException:
        pass
