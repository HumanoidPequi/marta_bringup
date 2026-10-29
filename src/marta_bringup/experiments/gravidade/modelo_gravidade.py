#!/usr/bin/env python3
"""Torque de gravidade por junta a partir do URDF (referencia em Python).

Mesmo modelo do GravityCompensator (marta_control), escrito de forma
independente para validar fisica e sinais (plano_fase3_roll.md, portao E1):

  - cinematica direta no frame do tronco (base_link), tronco suposto
    vertical: g = (0, 0, -9,81) nesse frame (como em Maximo, doutorado §4.3);
  - perna de APOIO (pe fixo no chao): a junta k sustenta tudo o que NAO esta
    entre ela e o pe: tau_k = [sum_{i fora da subarvore do filho de k}
    (r_i - p_k) x m_i g] . e_k;
  - perna em BALANCO (pendurada): a junta k sustenta a subarvore do filho:
    tau_k = -[sum_{i na subarvore} (r_i - p_k) x m_i g] . e_k.
tau e o torque que o atuador aplica no link filho em torno de +e_k, a mesma
convencao do comando do effort_controllers (servo: tau = Kp*(alvo - q)).
"""
import math
import xml.etree.ElementTree as ET

import numpy as np

G = np.array([0.0, 0.0, -9.81])


def rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_mat(a, q):
    a = np.asarray(a, float) / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K


def vec(e, attr, default):
    return np.array([float(v) for v in (e.get(attr, default) if e is not None else default).split()])


class Modelo:
    def __init__(self, urdf_path, raiz='base_link'):
        r = ET.parse(urdf_path).getroot()
        self.raiz = raiz
        self.massa, self.com = {}, {}
        for l in r.findall('link'):
            i = l.find('inertial')
            if i is not None:
                self.massa[l.get('name')] = float(i.find('mass').get('value'))
                self.com[l.get('name')] = vec(i.find('origin'), 'xyz', '0 0 0')
        self.juntas = {}
        self.filhos = {}
        for j in r.findall('joint'):
            o = j.find('origin')
            self.juntas[j.get('name')] = dict(
                pai=j.find('parent').get('link'), filho=j.find('child').get('link'),
                xyz=vec(o, 'xyz', '0 0 0'), R=rpy_mat(*vec(o, 'rpy', '0 0 0')),
                eixo=vec(j.find('axis'), 'xyz', '1 0 0'), tipo=j.get('type'))
            self.filhos.setdefault(j.find('parent').get('link'), []).append(j.get('name'))
        self.junta_do_link = {v['filho']: k for k, v in self.juntas.items()}
        self.M = sum(self.massa.values())

    def subarvore(self, link):
        out = [link]
        for j in self.filhos.get(link, []):
            out += self.subarvore(self.juntas[j]['filho'])
        return out

    def cadeia(self, pe):
        """Juntas da raiz ate o link do pe, na ordem raiz -> pe."""
        c, l = [], pe
        while l != self.raiz:
            j = self.junta_do_link[l]
            c.append(j)
            l = self.juntas[j]['pai']
        return c[::-1]

    def fk(self, q):
        """q: dict junta -> angulo. Devolve poses dos links e (p, e) das juntas."""
        T = {self.raiz: (np.eye(3), np.zeros(3))}
        J = {}
        pilha = [self.raiz]
        while pilha:
            l = pilha.pop()
            R, p = T[l]
            for jn in self.filhos.get(l, []):
                j = self.juntas[jn]
                Rj, pj = R @ j['R'], p + R @ j['xyz']
                J[jn] = (pj, Rj @ (j['eixo'] / np.linalg.norm(j['eixo'])))
                ang = q.get(jn, 0.0) if j['tipo'] in ('revolute', 'continuous') else 0.0
                T[j['filho']] = (Rj @ axis_mat(j['eixo'], ang), pj)
                pilha.append(j['filho'])
        return T, J

    def torques_grf(self, q, pe, cop_local=(0.0, 0.0, -0.03705)):
        """Perna de apoio pela forca de reacao do chao (LIPM).

        A reacao age no ZMP planejado (cop_local, no frame do link do pe:
        sob o tornozelo, na sola) e aponta do ZMP para o CoM, com componente
        vertical M*g: F = M*g*(r_com - r_zmp)/(z_com - z_zmp). Cada junta k da
        perna de apoio sustenta essa forca e o peso dos links entre ela e o pe:
        tau_k = -[(r_zmp - p_k) x F + sum_sub (r_i - p_k) x m_i g] . e_k.
        No caso estatico (CoM sobre o ZMP) coincide com torques(apoio=True).
        """
        T, J = self.fk(q)
        r = {l: T[l][1] + T[l][0] @ self.com[l] for l in self.massa}
        com = sum(self.massa[l] * r[l] for l in self.massa) / self.M
        Rf, pf = T[pe]
        zmp = pf + Rf @ np.asarray(cop_local)
        F = -self.M * G[2] * (com - zmp) / (com[2] - zmp[2])
        out = {}
        for jn in self.cadeia(pe):
            p, e = J[jn]
            sub = [l for l in self.subarvore(self.juntas[jn]['filho']) if l in self.massa]
            G_sub = sum((np.cross(r[l] - p, self.massa[l] * G) for l in sub), np.zeros(3))
            out[jn] = -float((np.cross(zmp - p, F) + G_sub) @ e)
        return out

    def torques(self, q, pe, apoio):
        T, J = self.fk(q)
        r = {l: T[l][1] + T[l][0] @ self.com[l] for l in self.massa}
        S_all = sum(self.massa[l] * r[l] for l in self.massa)
        out = {}
        for jn in self.cadeia(pe):
            p, e = J[jn]
            sub = [l for l in self.subarvore(self.juntas[jn]['filho']) if l in self.massa]
            M_sub = sum(self.massa[l] for l in sub)
            S_sub = sum((self.massa[l] * r[l] for l in sub), np.zeros(3))
            if apoio:
                mom = np.cross(S_all - S_sub - (self.M - M_sub) * p, G)
                out[jn] = float(mom @ e)
            else:
                mom = np.cross(S_sub - M_sub * p, G)
                out[jn] = -float(mom @ e)
        return out
