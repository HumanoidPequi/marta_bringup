#!/usr/bin/env python3
"""
Teleoperacao da Marta pelo teclado, numa janela Tk (so simulacao).

Sobe pelo stabilizer_test.launch com teleop:=true, que tambem liga
cmd_vel_enabled no controller_node. Precisa de DISPLAY/X11 no docker run
(README secao 5). A janela despausa a simulacao, espera o laco de controle
entrar em regime e fica pronta; Enter comeca a andar.

Teclas (segure para manter o comando; soltar tudo = marcha no lugar).
W A S D valem como as setas. Uma seta (ou WASD) com o robo em pe ja da a
partida; Enter tambem.
    seta cima            reta              (vx, 0, 0)
    seta esq / dir       giro no lugar     (0, 0, +-w)   esq = anti-horario
    cima + esq / dir     curva             (vx, 0, +-w)  ou giro no lugar no modo 'c'
    seta baixo           PARAR (walk_enabled = false; o gerador da ~3 passos e para)
    Enter                andar (de novo)
    c                    alterna a combinacao cima+lado entre curva e giro no lugar
    + / -                w = 0,35 / 0,175 rad/s
    q ou fechar          sai (para a marcha antes)

So emite comandos do envelope testado (docs/marcha_omnidirecional.md):
vx 0,1; w 0,175 ou 0,35; curva (0,1; +-w). Nao anda para tras nem de lado.
O controller_node le a velocidade a cada passo (0,6 s) e zera o comando se
/marta/cmd_vel ficar 0,5 s sem mensagem.

Parametros (~): vx (0.1), omega (0.175), omega_alto (0.35), rate (20 Hz).
"""
import threading
import time
import tkinter as tk

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty

def esperar_controladores(timeout=90.0, log=None):
    """Espera os controladores de junta (/marta/<junta>_position) estarem
    carregados ANTES de despausar o Gazebo.

    Despausar antes deixa o robo com as juntas soltas ate o spawner terminar:
    com a maquina carregada (GUI, Foxglove, outra simulacao) isso o derruba
    para tras na partida (reproduzido: 3/3 quedas contra 0/3 esperando,
    docs/marcha_omnidirecional.md, secao Teleop). Com a simulacao pausada o
    list_controllers fica bloqueado pelo switch_controller do spawner; por
    isso o sinal e o topico <junta>_position/state, que cada controlador
    anuncia ao ser carregado. O numero esperado vem do servidor de parametros.
    """
    import time as _t
    t0 = _t.time()
    esperado, n = 0, -1
    while not rospy.is_shutdown() and _t.time() - t0 < timeout:
        try:
            esperado = sum(1 for p in rospy.get_param_names()
                           if p.startswith('/marta/') and p.endswith('_position/type'))
            tops = {t for t, _ in rospy.get_published_topics()}
            n = sum(1 for t in tops if t.startswith('/marta/') and t.endswith('_position/state'))
        except Exception:  # noqa: BLE001
            pass
        if esperado and n >= esperado:
            _t.sleep(0.5)
            return True
        if log:
            log(n, esperado)
        _t.sleep(0.2)
    rospy.logwarn('Controladores de junta nao carregaram em %.0f s (%d de %d).', timeout, n, esperado)
    return False


SOLTURA_MS = 80   # filtro do auto-repeat do X11 (pares falsos soltar/apertar)
SETAS = ('Up', 'Down', 'Left', 'Right')
WASD = {'w': 'Up', 's': 'Down', 'a': 'Left', 'd': 'Right'}   # mesmas acoes das setas


class Teleop:
    def __init__(self):
        rospy.init_node('teleop_marta', disable_signals=True)
        self.vx = rospy.get_param('~vx', 0.1)
        self.w_baixo = rospy.get_param('~omega', 0.175)
        self.w_alto = rospy.get_param('~omega_alto', 0.35)
        self.w = self.w_baixo
        self.rate_hz = rospy.get_param('~rate', 20.0)
        self.modo_curva = True
        self.estado = 'iniciando'      # iniciando | pronto | andando | parando
        self.apertadas = set()
        self.soltar_pend = {}
        self.cmd = (0.0, 0.0, 0.0)
        self.em_regime = False
        self.rtf = float('nan')
        self._rtf_ref = None

        self.pub = rospy.Publisher('/marta/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/marta/stabilizer_debug', Float64MultiArray, self._debug_cb)

        self.root = tk.Tk()
        self.root.title('Marta - teleop')
        self.root.geometry('460x300')
        fonte = ('DejaVu Sans Mono', 11)
        self.lbl_estado = tk.Label(self.root, font=('DejaVu Sans Mono', 14, 'bold'))
        self.lbl_estado.pack(pady=(10, 4))
        self.lbl_cmd = tk.Label(self.root, font=fonte)
        self.lbl_cmd.pack()
        self.lbl_info = tk.Label(self.root, font=fonte)
        self.lbl_info.pack(pady=4)
        self.lbl_tecla = tk.Label(self.root, font=('DejaVu Sans Mono', 9), fg='#555555',
                                  text='ultima tecla recebida: (nenhuma)')
        self.lbl_tecla.pack()
        tk.Label(self.root, justify='left', font=('DejaVu Sans Mono', 9), text=(
            'cima/W: reta  esq/dir/A/D: giro  cima+lado: curva\n'
            'baixo/S: PARAR  seta ou Enter: andar  c: curva<->giro\n'
            '+/-: w 0,35/0,175 rad/s       q: sair\n'
            'soltar tudo = marcha no lugar\n'
            '(clique nesta janela para ela receber as teclas)')).pack(pady=8)
        self.root.bind('<KeyPress>', self._press)
        self.root.bind('<KeyRelease>', self._release)
        # Alguns gerenciadores de janela nao passam o teclado para a janela Tk
        # so pelo clique: forca o foco ao abrir e a cada clique.
        self.root.bind('<Button-1>', lambda _e: self.root.focus_force())
        self.root.after(500, self.root.focus_force)
        self.root.protocol('WM_DELETE_WINDOW', self.sair)

        threading.Thread(target=self._partida, daemon=True).start()
        self.root.after(int(1000 / self.rate_hz), self._tick)
        self.root.after(1000, self._mede_rtf)

    # --- partida: despausa e espera o laco ---------------------------------
    def _debug_cb(self, _msg):
        self.em_regime = True

    def _partida(self):
        self.msg_espera = 'carregando os controladores das juntas'
        esperar_controladores(log=lambda n, e: setattr(
            self, 'msg_espera', 'carregando os controladores das juntas (%d de %d)' % (max(n, 0), e)))
        self.msg_espera = 'aguardando o laco de controle (~15 s)'
        try:
            rospy.wait_for_service('/gazebo/unpause_physics', timeout=60.0)
            rospy.ServiceProxy('/gazebo/unpause_physics', Empty)()
        except Exception as e:  # noqa: BLE001
            rospy.logwarn('[teleop] nao consegui despausar: %s', e)
        while not rospy.is_shutdown() and not self.em_regime:
            time.sleep(0.2)
        time.sleep(3.0)    # assentar em pe, como o walk_test (settle)
        self.estado = 'pronto'
        rospy.loginfo('[teleop] pronto: Enter para andar.')

    # --- teclado -----------------------------------------------------------
    def _press(self, ev):
        k = WASD.get(ev.keysym.lower(), ev.keysym)
        self.lbl_tecla.config(text='ultima tecla recebida: %s' % ev.keysym)
        pend = self.soltar_pend.pop(k, None)
        if pend is not None:
            self.root.after_cancel(pend)       # auto-repeat: ignora o par falso
        if k in SETAS:
            if k not in self.apertadas:
                rospy.loginfo('[teleop] tecla %s', ev.keysym)
                if k == 'Down':
                    self._parar()
                else:
                    self._andar()              # seta com o robo em pe: partida
            self.apertadas.add(k)
        elif k == 'Return':
            self._andar()
        elif k in ('c', 'C'):
            self.modo_curva = not self.modo_curva
        elif k in ('plus', 'KP_Add', 'equal'):
            self.w = self.w_alto
        elif k in ('minus', 'KP_Subtract'):
            self.w = self.w_baixo
        elif k in ('q', 'Q', 'Escape'):
            self.sair()

    def _release(self, ev):
        k = WASD.get(ev.keysym.lower(), ev.keysym)
        if k in SETAS:
            self.soltar_pend[k] = self.root.after(SOLTURA_MS, self._soltou, k)

    def _soltou(self, k):
        self.soltar_pend.pop(k, None)
        self.apertadas.discard(k)

    # --- estados -----------------------------------------------------------
    def _andar(self):
        if self.estado in ('pronto', 'parando'):
            rospy.set_param('/controller_node/walk_enabled', True)
            self.estado = 'andando'
            rospy.loginfo('[teleop] andando.')

    def _parar(self):
        if self.estado == 'andando':
            rospy.set_param('/controller_node/walk_enabled', False)
            self.estado = 'parando'
            rospy.loginfo('[teleop] parada solicitada.')

    def comando(self):
        a = self.apertadas
        frente = 'Up' in a
        lado = ('Left' in a) - ('Right' in a)      # +1 esq, -1 dir, 0 nenhum/ambos
        if self.estado != 'andando':
            return (0.0, 0.0, 0.0)
        if frente and lado and not self.modo_curva:
            return (0.0, 0.0, lado * self.w)
        return (self.vx if frente else 0.0, 0.0, lado * self.w)

    # --- laco --------------------------------------------------------------
    def _tick(self):
        if rospy.is_shutdown():
            self.root.destroy()
            return
        self.cmd = self.comando()
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = self.cmd
        self.pub.publish(t)
        vx, _, w = self.cmd
        if self.estado == 'andando':
            mov = ('curva' if vx and w else 'reta' if vx else
                   'giro no lugar' if w else 'marcha no lugar')
        else:
            mov = {'iniciando': getattr(self, 'msg_espera', 'iniciando'),
                   'pronto': 'em pe - Enter para andar',
                   'parando': 'parado - Enter para andar'}[self.estado]
        self.lbl_estado.config(text=mov.upper() if self.estado == 'andando' else mov,
                               fg='#1b5e20' if self.estado == 'andando' else '#555555')
        self.lbl_cmd.config(text='vx %.2f m/s   w %+.3f rad/s' % (vx, w))
        self.lbl_info.config(text='combinacao: %s   w = %.3f   RTF %.2f' % (
            'curva' if self.modo_curva else 'giro', self.w, self.rtf))
        self.root.after(int(1000 / self.rate_hz), self._tick)

    def _mede_rtf(self):
        agora = (time.time(), rospy.get_time())
        if self._rtf_ref is not None and agora[0] > self._rtf_ref[0]:
            self.rtf = (agora[1] - self._rtf_ref[1]) / (agora[0] - self._rtf_ref[0])
        self._rtf_ref = agora
        self.root.after(2000, self._mede_rtf)

    def sair(self):
        try:
            self.pub.publish(Twist())
            if self.estado == 'andando':
                rospy.set_param('/controller_node/walk_enabled', False)
        finally:
            rospy.signal_shutdown('janela fechada')
            self.root.destroy()


if __name__ == '__main__':
    Teleop().root.mainloop()
