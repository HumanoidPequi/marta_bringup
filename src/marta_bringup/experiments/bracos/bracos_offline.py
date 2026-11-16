#!/usr/bin/env python3
"""B0 do estudo dos bracos (docs/bracos.md): analise offline, sem simulacao.

So usa o URDF (gerado pelo xacro do container, com o torso_mass_scale pedido),
as malhas STL e os _juntas.csv de corridas ja gravadas (E7). Responde, ANTES de
qualquer campanha:

  1. sinais: por cinematica direta, para que lado cada junta do braco leva a
     mao (flexao do cotovelo, pitch e roll do ombro);
  2. viabilidade: folga minima braco <-> corpo (tronco, cintura, quadril, coxa,
     pescoco) pelas malhas, na postura e nos extremos do balanco; vertices do
     braco DENTRO do tronco (paridade de raio) = interpenetracao;
  3. por postura: CoM do corpo inteiro, deslocamento em relacao a postura
     historica, inercia centroidal (roll, pitch, yaw);
  4. autoridade do balanco: momento angular centroidal em z (yaw) gerado pelo
     movimento relativo dos membros, com o tronco parado, a partir das
     trajetorias de junta gravadas: pernas sozinhas x pernas + bracos com
     balanco s = 0, 1, 2 em cada postura. Fracao cancelada = 1 - RMS(H_total)
     / RMS(H_pernas).

    python3 bracos_offline.py [--massa 0.5] [--saida bracos/b0]

Le os _juntas.csv de omni/dados (E7_<M>_kd06_<mov>_pD_r1). Escreve CSVs em
<saida>/ e imprime as tabelas. Convencao das posturas: roll = abertura do
ombro (graus, positivo abre), cot = flexao do cotovelo (graus, positivo leva o
antebraco para frente), pit = pitch fixo do ombro (positivo = para frente) -
a mesma dos parametros arm_* do launch.
"""
import argparse
import csv
import math
import os
import struct
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import cKDTree

AQUI = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.dirname(AQUI)
RAIZ = os.path.abspath(os.path.join(EXP, '..', '..', '..'))
MESHES = os.path.join(RAIZ, 'src', 'marta_description', 'meshes')
D2R = math.pi / 180.0

BRACO = {'l': ['l_shoulder_1', 'l_arm_1', 'l_hand_1'], 'r': ['r_shoulder_1', 'r_arm_1', 'r_hand_1']}
CORPO = ['base_link', 'neck_1', 'l_hip_1', 'r_hip_1', 'l_waist_1', 'r_waist_1', 'l_thigh_1', 'r_thigh_1']
PERNAS = ['hip_yaw', 'hip_roll', 'hip_pitch', 'knee', 'ank_pitch', 'ank_roll']


# --- URDF ---------------------------------------------------------------------
def gerar_urdf(massa, destino):
    if os.path.exists(destino):
        return destino
    cmd = ("source /opt/ros/noetic/setup.bash && source devel/setup.bash && "
           "xacro src/marta_description/urdf/marta.xacro torso_mass_scale:=%s" % massa)
    out = subprocess.run(['docker', 'run', '--rm', '-v', RAIZ + ':/ws', '-w', '/ws',
                          'marta_bringup:1.0', 'bash', '-c', cmd],
                         check=True, capture_output=True, text=True).stdout
    with open(destino, 'w') as f:
        f.write(out)
    return destino


def vec(s, n=3):
    return np.array([float(x) for x in (s or ' '.join(['0'] * n)).split()])


def rpy2R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle(a, q):
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K


class Modelo:
    def __init__(self, urdf):
        root = ET.parse(urdf).getroot()
        self.links, self.joints, self.filhos = {}, {}, {}
        for l in root.findall('link'):
            d = {'m': 0.0, 'c': np.zeros(3), 'I': np.zeros((3, 3)), 'mesh': None}
            ine = l.find('inertial')
            if ine is not None:
                d['m'] = float(ine.find('mass').get('value'))
                o = ine.find('origin')
                d['c'] = vec(o.get('xyz')) if o is not None else np.zeros(3)
                i = ine.find('inertia')
                ixx, ixy, ixz, iyy, iyz, izz = (float(i.get(k)) for k in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz'))
                Rr = rpy2R(*vec(o.get('rpy'))) if o is not None else np.eye(3)
                d['I'] = Rr @ np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]]) @ Rr.T
            v = l.find('visual')
            if v is not None and v.find('geometry/mesh') is not None:
                m = v.find('geometry/mesh')
                o = v.find('origin')
                d['mesh'] = (os.path.basename(m.get('filename')), vec(m.get('scale') or '1 1 1'),
                             vec(o.get('xyz')) if o is not None else np.zeros(3),
                             rpy2R(*vec(o.get('rpy'))) if o is not None else np.eye(3))
            self.links[l.get('name')] = d
        for j in root.findall('joint'):
            o = j.find('origin')
            ax = j.find('axis')
            self.joints[j.get('name')] = dict(
                tipo=j.get('type'), pai=j.find('parent').get('link'), filho=j.find('child').get('link'),
                xyz=vec(o.get('xyz')) if o is not None else np.zeros(3),
                R=rpy2R(*vec(o.get('rpy'))) if o is not None else np.eye(3),
                eixo=vec(ax.get('xyz')) if ax is not None else np.array([1.0, 0, 0]))
            self.filhos.setdefault(j.find('parent').get('link'), []).append(j.get('name'))
        self._mesh = {}

    def fk(self, q):
        """q: {nome_junta_urdf: rad}. Devolve {link: (R, p)} no frame do base_link."""
        T = {'base_link': (np.eye(3), np.zeros(3))}
        pilha = ['base_link']
        while pilha:
            pai = pilha.pop()
            Rp, pp = T[pai]
            for jn in self.filhos.get(pai, []):
                j = self.joints[jn]
                Rj = j['R']
                if j['tipo'] in ('revolute', 'continuous'):
                    Rj = Rj @ axis_angle(j['eixo'], q.get(jn, 0.0))
                T[j['filho']] = (Rp @ Rj, pp + Rp @ j['xyz'])
                pilha.append(j['filho'])
        return T

    def com(self, T, nomes=None):
        nomes = nomes or [n for n in self.links if n in T]
        m = sum(self.links[n]['m'] for n in nomes)
        c = sum(self.links[n]['m'] * (T[n][1] + T[n][0] @ self.links[n]['c']) for n in nomes) / m
        return m, c

    def inercia(self, T, c):
        I = np.zeros((3, 3))
        for n, (R, p) in T.items():
            L = self.links[n]
            if L['m'] == 0:
                continue
            r = p + R @ L['c'] - c
            I += R @ L['I'] @ R.T + L['m'] * (r @ r * np.eye(3) - np.outer(r, r))
        return I

    def vertices(self, link, T, passo=1):
        nome, esc, o, Ro = self.links[link]['mesh']
        if nome not in self._mesh:
            self._mesh[nome] = ler_stl(os.path.join(MESHES, nome))
        tri = self._mesh[nome] * esc
        tri = tri @ Ro.T + o
        R, p = T[link]
        return tri @ R.T + p   # (n, 3, 3) triangulos no frame do base_link


def ler_stl(f):
    with open(f, 'rb') as h:
        h.read(80)
        n = struct.unpack('<I', h.read(4))[0]
        dados = np.frombuffer(h.read(n * 50), dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    return dados['v'].reshape(n, 3, 3).astype(float)


def dentro(pontos, tri):
    """Paridade de raio (+z) de cada ponto contra a malha fechada tri (n,3,3)."""
    v0, e1, e2 = tri[:, 0], tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]
    d = np.array([0.0137, 0.0071, 1.0])   # levemente inclinado: evita arestas
    d /= np.linalg.norm(d)
    h = np.cross(d, e2)
    a = np.einsum('ij,ij->i', e1, h)
    ok = np.abs(a) > 1e-12
    v0, e1, e2, h, a = v0[ok], e1[ok], e2[ok], h[ok], a[ok]
    res = np.zeros(len(pontos), bool)
    for k, p in enumerate(pontos):
        s = p - v0
        u = np.einsum('ij,ij->i', s, h) / a
        qv = np.cross(s, e1)
        v = (qv @ d) / a
        t = np.einsum('ij,ij->i', e2, qv) / a
        res[k] = np.count_nonzero((u >= 0) & (v >= 0) & (u + v <= 1) & (t > 1e-9)) % 2 == 1
    return res


# --- posturas ------------------------------------------------------------------
def q_bracos(roll, cot, pit, swing_l=0.0, swing_r=0.0):
    """Angulos de junta (rad) com a convencao do gerador (OmnidirectionalWalkZMP.cpp)."""
    return {'l_sho_roll_link': -roll * D2R, 'r_sho_roll_link': roll * D2R,
            'l_sho_pitch_link': -pit * D2R + swing_l, 'r_sho_pitch_link': -pit * D2R + swing_r,
            'l_el_link': -cot * D2R, 'r_el_link': -cot * D2R}


def pose_pernas(juntas_csv):
    """Media das juntas das pernas (process_value) com t > 3 s: postura agachada."""
    acc = {}
    for x in csv.DictReader(open(juntas_csv)):
        if float(x['t']) > 3.0 and x['junta'][2:] in PERNAS:
            acc.setdefault(x['junta'] + '_link', []).append(float(x['process_value']))
    return {k: float(np.mean(v)) for k, v in acc.items()}


def folga(mod, q, lados=('l', 'r')):
    """Menor distancia braco <-> corpo [mm] e no. de vertices do braco dentro do corpo."""
    T = mod.fk(q)
    corpo_tri = {n: mod.vertices(n, T) for n in CORPO}
    corpo_pts = np.concatenate([t.reshape(-1, 3) for t in corpo_tri.values()])
    arv = cKDTree(corpo_pts)
    dmin, dentro_n = 1e9, 0
    for lado in lados:
        for n in BRACO[lado][1:]:          # o shoulder_1 e o suporte do servo, encaixado no tronco
            pts = np.unique(mod.vertices(n, T).reshape(-1, 3).round(6), axis=0)
            d, _ = arv.query(pts)
            dmin = min(dmin, float(d.min()))
            cand = pts[d < 0.03]
            if len(cand):
                cand = cand[:: max(1, len(cand) // 300)]
                for cn, tri in corpo_tri.items():
                    flat = tri.reshape(-1, 3)
                    lo, hi = flat.min(0), flat.max(0)
                    sel = cand[np.all((cand >= lo) & (cand <= hi), axis=1)]
                    if len(sel):
                        dentro_n += int(dentro(sel, tri).sum())
    return 1000 * dmin, dentro_n


# --- momento angular -------------------------------------------------------------
def trajetorias(juntas_csv, t0=5.0, t1=29.0):
    por_t = {}
    for x in csv.DictReader(open(juntas_csv)):
        t = float(x['t'])
        if t0 <= t <= t1:
            por_t.setdefault(round(t, 3), {})[x['junta'] + '_link'] = float(x['process_value'])
    ts = sorted(k for k, v in por_t.items() if len(v) >= 20)
    return np.array(ts), [por_t[k] for k in ts]


def momento_z(mod, ts, qs, braco_q):
    """H centroidal (frame do tronco, tronco parado) por diferenca finita de FK.

    Devolve (H_pernas, H_total) em kg m^2/s: H_pernas usa so os links das pernas
    (e a massa do resto parada), H_total soma os bracos com as juntas de braco_q(k)."""
    Ts = []
    for k, q in enumerate(qs):
        qq = {j: v for j, v in q.items() if not j.startswith(('l_sho', 'r_sho', 'l_el', 'r_el', 'head'))}
        qq.update(braco_q(k, q))
        Ts.append(mod.fk(qq))
    nomes = [n for n in Ts[0] if mod.links[n]['m'] > 0]
    pernas = [n for n in nomes if n not in BRACO['l'] + BRACO['r'] + ['base_link', 'neck_1', 'head_1']]
    bracos = BRACO['l'] + BRACO['r']
    Hp, Hb = [], []
    for k in range(1, len(Ts) - 1):
        dt = ts[k + 1] - ts[k - 1]
        _, c = mod.com(Ts[k])
        _, c0 = mod.com(Ts[k - 1])
        _, c1 = mod.com(Ts[k + 1])
        vc = (c1 - c0) / dt
        h = {'p': np.zeros(3), 'b': np.zeros(3)}
        for grupo, lst in (('p', pernas), ('b', bracos)):
            for n in lst:
                L = mod.links[n]
                R0, p0 = Ts[k - 1][n]
                R1, p1 = Ts[k + 1][n]
                R, p = Ts[k][n]
                r = p + R @ L['c']
                v = ((p1 + R1 @ L['c']) - (p0 + R0 @ L['c'])) / dt
                W = (R1 - R0) / dt @ R.T                  # [w]x
                w = np.array([W[2, 1] - W[1, 2], W[0, 2] - W[2, 0], W[1, 0] - W[0, 1]]) * 0.5
                h[grupo] += L['m'] * np.cross(r - c, v - vc) + R @ L['I'] @ R.T @ w
        Hp.append(h['p'])
        Hb.append(h['b'])
    return np.array(Hp), np.array(Hb)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--massa', default='0.5')
    ap.add_argument('--saida', default=os.path.join(AQUI, 'b0'))
    ap.add_argument('--rotulo', default='M33')
    a = ap.parse_args()
    os.makedirs(a.saida, exist_ok=True)
    mod = Modelo(gerar_urdf(a.massa, os.path.join(a.saida, 'marta_tms%s.urdf' % a.massa)))
    dados = os.path.join(EXP, 'omni', 'dados')
    jref = os.path.join(dados, 'E7_%s_kd06_reta0.10_pD_r1_juntas.csv' % a.rotulo)
    pernas = pose_pernas(jref)
    mtot = sum(L['m'] for L in mod.links.values())
    mbr = sum(mod.links[n]['m'] for n in BRACO['l'] + BRACO['r'])
    print('massa total %.3f kg; bracos %.3f kg (%.1f%%)' % (mtot, mbr, 100 * mbr / mtot))

    # 1. sinais
    print('\n1. SINAIS (posicao da mao l_hand_1 no frame do tronco, mm; x frente, y esquerda, z cima)')
    for rot, q in (('tudo 0', q_bracos(0, 0, 0)), ('cotovelo +45 (flexao)', q_bracos(0, 45, 0)),
                   ('ombro pitch +20 (frente)', q_bracos(0, 0, 20)), ('ombro roll +30 (abre)', q_bracos(30, 0, 0))):
        T = mod.fk(dict(pernas, **q))
        _, cm = mod.com(T, ['l_hand_1'])
        _, cr = mod.com(T, ['r_hand_1'])
        print('  %-26s esq (%+6.1f %+6.1f %+6.1f)  dir (%+6.1f %+6.1f %+6.1f)' % ((rot,) + tuple(1000 * cm) + tuple(1000 * cr)))

    # 2-3. posturas
    print('\n2-3. POSTURAS (pernas na postura media da reta; balanco nos extremos +-0,28 rad = s 2)')
    T0 = mod.fk(dict(pernas, **q_bracos(50, 0, 0)))
    _, c_hist = mod.com(T0)
    linhas = []
    cab = ('roll', 'cot', 'pit', 'folga_mm', 'dentro', 'folga_sw_mm', 'dentro_sw', 'dcx_mm', 'dcy_mm', 'dcz_mm',
           'Ixx', 'Iyy', 'Izz')
    print('  ' + ' '.join('%9s' % c for c in cab))
    for roll in (0, 5, 10, 15, 20, 30, 50):
        for cot in (0, 45, 90):
            q = dict(pernas, **q_bracos(roll, cot, 0))
            fo, de = folga(mod, q)
            fs, ds = 1e9, 0
            for sw in (-0.28, 0.28):
                f2, d2 = folga(mod, dict(pernas, **q_bracos(roll, cot, 0, sw, -sw)))
                fs, ds = min(fs, f2), ds + d2
            T = mod.fk(q)
            _, c = mod.com(T)
            I = mod.inercia(T, c)
            lin = (roll, cot, 0, fo, de, fs, ds) + tuple(1000 * (c - c_hist)) + (I[0, 0], I[1, 1], I[2, 2])
            linhas.append(lin)
            print('  ' + ' '.join(('%9.4f' % v) if isinstance(v, float) and abs(v) < 0.1 and v != 0 else
                                  ('%9.1f' % v if isinstance(v, float) else '%9d' % v) for v in lin))
    with open(os.path.join(a.saida, 'posturas_%s.csv' % a.rotulo), 'w') as f:
        w = csv.writer(f)
        w.writerow(cab)
        w.writerows(linhas)

    # 2b. viabilidade DINAMICA: pernas e balanco gravados (reta, giro, lateral),
    # 12 instantes por corrida ao longo de 2 passadas, balanco s = 2 (pior caso).
    # Criterio (pre-registrado em docs/bracos.md): viavel se nenhum vertice do
    # braco entra no corpo e a folga minima fica >= 5 mm em todos os instantes.
    print('\n2b. VIABILIDADE DINAMICA (pior instante; balanco s = 2)')
    amostras = []
    for mov in ('reta0.10', 'giro+0.175', 'lat+0.05'):
        jc = os.path.join(dados, 'E7_%s_kd06_%s_pD_r1_juntas.csv' % (a.rotulo, mov))
        ts, qs = trajetorias(jc, 10.0, 12.4)
        amostras += [(mov, q) for q in qs[:: max(1, len(qs) // 12)]]
    din = []
    for roll, cot in ((50, 0), (0, 0), (5, 0), (10, 0), (15, 0), (0, 90), (5, 90), (10, 90),
                      (15, 90), (10, 45), (15, 45)):
        pior, dentro_tot, onde = 1e9, 0, ''
        for mov, q in amostras:
            ql = {j: v for j, v in q.items() if not j.startswith(('l_sho', 'r_sho', 'l_el', 'r_el', 'head'))}
            sw_l, sw_r = 2 * q['l_sho_pitch_link'], 2 * q['r_sho_pitch_link']
            fo, de = folga(mod, dict(ql, **q_bracos(roll, cot, 0, sw_l, sw_r)))
            dentro_tot += de
            if fo < pior:
                pior, onde = fo, mov
        viavel = dentro_tot == 0 and pior >= 5.0
        din.append((roll, cot, pior, onde, dentro_tot, int(viavel)))
        print('  roll %2d cot %2d: folga minima %5.1f mm (%s), vertices dentro %4d -> %s'
              % (roll, cot, pior, onde, dentro_tot, 'VIAVEL' if viavel else 'INVIAVEL'))
    with open(os.path.join(a.saida, 'viabilidade_%s.csv' % a.rotulo), 'w') as f:
        w = csv.writer(f)
        w.writerow(['roll', 'cot', 'folga_min_mm', 'mov_pior', 'vertices_dentro', 'viavel'])
        w.writerows(din)

    # 4. momento angular em yaw
    print('\n4. MOMENTO ANGULAR CENTROIDAL EM z (tronco parado), RMS [1e-4 kg m^2/s]')
    res = []
    for mov in ('reta0.10', 'curva+0.175', 'giro+0.175', 'lat+0.05'):
        jc = os.path.join(dados, 'E7_%s_kd06_%s_pD_r1_juntas.csv' % (a.rotulo, mov))
        if not os.path.exists(jc):
            continue
        ts, qs = trajetorias(jc)
        sw_l = np.array([q['l_sho_pitch_link'] for q in qs])
        sw_r = np.array([q['r_sho_pitch_link'] for q in qs])
        sw_l, sw_r = sw_l - sw_l.mean(), sw_r - sw_r.mean()   # balanco gravado (s = 1)
        for roll, cot in ((50, 0), (15, 0), (15, 45), (15, 90), (5, 90)):
            for s in (0, 1, 2):
                bq = (lambda k, q, roll=roll, cot=cot, s=s:
                      q_bracos(roll, cot, 0, s * sw_l[k], s * sw_r[k]))
                Hp, Hb = momento_z(mod, ts, qs, bq)
                hz_p = math.sqrt(np.mean(Hp[:, 2] ** 2))
                hz_t = math.sqrt(np.mean((Hp[:, 2] + Hb[:, 2]) ** 2))
                hx_t = math.sqrt(np.mean((Hp[:, 0] + Hb[:, 0]) ** 2))
                res.append((mov, roll, cot, s, 1e4 * hz_p, 1e4 * hz_t, 1 - hz_t / hz_p, 1e4 * hx_t,
                            math.degrees(sw_l.std() * math.sqrt(2))))
                print('  %-12s roll %2d cot %2d s %d: pernas %6.2f  total %6.2f  cancelado %+5.1f%%  (H_x total %6.2f; balanco +-%.1f graus)'
                      % (mov, roll, cot, s, 1e4 * hz_p, 1e4 * hz_t, 100 * (1 - hz_t / hz_p), 1e4 * hx_t,
                         s * math.degrees(sw_l.std() * math.sqrt(2))))
    with open(os.path.join(a.saida, 'momento_%s.csv' % a.rotulo), 'w') as f:
        w = csv.writer(f)
        w.writerow(['mov', 'roll', 'cot', 's', 'Hz_pernas_rms_1e4', 'Hz_total_rms_1e4', 'cancelado',
                    'Hx_total_rms_1e4', 'amp_balanco_graus'])
        w.writerows(res)


if __name__ == '__main__':
    main()
