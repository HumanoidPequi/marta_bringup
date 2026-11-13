#!/usr/bin/env python3
"""Teste das teclas do teleop_marta.py com eventos injetados no Tk.

Precisa de um roscore e de DISPLAY (X11). Nao sobe simulacao: a thread de
partida fica esperando os controladores, sem efeito no teste.

    python3 teste_teclas.py      (dentro do container, com o workspace sourced)

Cobre: partida pela seta, auto-repeat do X11 (pares falsos soltar/apertar),
curva, modo giro ('c'), w alto/baixo, esq+dir, soltar tudo (marcha no lugar),
WASD, parar com S e voltar a andar com seta.
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', '..', 'scripts'))
import rospy  # noqa: E402
import teleop_marta as T  # noqa: E402

t = T.Teleop()
r = t.root
t.estado = 'pronto'
res = []


def ev(kind, k):
    r.event_generate('<%s>' % kind, keysym=k, when='now')


def run(ms):
    fim = time.time() + ms / 1000.0
    while time.time() < fim:
        r.update()
        time.sleep(0.005)


def chk(nome, esperado):
    c = t.comando()
    ok = all(abs(a - b) < 1e-9 for a, b in zip(c, esperado))
    res.append(ok)
    print('%-34s %-22s %s' % (nome, c, 'OK' if ok else 'FALHA (esperado %s)' % (esperado,)))


run(500)                                  # janela mapeada antes do 1o evento
chk('nada', (0, 0, 0))
ev('KeyPress', 'Up'); run(20); chk('cima da a partida', (0.1, 0, 0))
print('  walk_enabled =', rospy.get_param('/controller_node/walk_enabled', None))
for _ in range(5):
    ev('KeyRelease', 'Up'); ev('KeyPress', 'Up'); run(30)
chk('cima com auto-repeat', (0.1, 0, 0))
ev('KeyPress', 'Left'); run(20); chk('cima+esq (curva)', (0.1, 0, 0.175))
ev('KeyPress', 'c'); run(20); chk('cima+esq modo giro', (0, 0, 0.175))
ev('KeyRelease', 'c'); ev('KeyPress', 'c'); run(20); chk('modo curva de novo', (0.1, 0, 0.175))
ev('KeyRelease', 'Up'); run(150); chk('so esq (giro +)', (0, 0, 0.175))
ev('KeyPress', 'plus'); run(20); chk('esq com w alto', (0, 0, 0.35))
ev('KeyPress', 'minus'); run(20)
ev('KeyPress', 'Right'); run(20); chk('esq+dir (cancela)', (0, 0, 0))
ev('KeyRelease', 'Left'); run(150); chk('so dir (giro -)', (0, 0, -0.175))
ev('KeyRelease', 'Right'); run(150); chk('soltou tudo (no lugar)', (0, 0, 0))
ev('KeyPress', 'w'); run(20); chk('W = reta', (0.1, 0, 0))
ev('KeyPress', 'a'); run(20); chk('W+A = curva', (0.1, 0, 0.175))
ev('KeyRelease', 'w'); run(150); chk('A = giro', (0, 0, 0.175))
ev('KeyRelease', 'a'); run(150)
ev('KeyPress', 's'); run(20); chk('S para', (0, 0, 0))
print('  estado =', t.estado, ' walk_enabled =', rospy.get_param('/controller_node/walk_enabled', None))
ev('KeyRelease', 's'); run(150)
ev('KeyPress', 'Right'); run(20); chk('seta volta a andar (giro -)', (0, 0, -0.175))
print('  rotulo:', t.lbl_tecla.cget('text'))
print('TODOS OK (%d)' % len(res) if all(res) else 'HA FALHAS')
t.root.destroy()
rospy.signal_shutdown('fim do teste')
