#!/usr/bin/env python3
"""Portao E1 (plano_fase3_roll.md §5): valida o modelo de gravidade com dados.

Para cada amostra no MIOLO do apoio simples (25-75% do trecho), calcula o
torque de gravidade previsto (modelo_gravidade.py, angulos REAIS das juntas,
pe de apoio fixo) e compara com o torque aplicado pelo PID (`command`) nas
juntas da perna de apoio. Corridas com tau = 1000 (sem saturacao).
Portao: inclinacao em [0,7; 1,3], R2 > 0,6 e mesmo sinal, por junta.

  python3 validar_e1.py <urdf> <padrao_dos_prefixos_de_dados>
"""
import bisect
import csv
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from modelo_gravidade import Modelo  # noqa: E402

JUNTAS_APOIO = ('hip_roll', 'hip_pitch', 'knee', 'ank_pitch', 'ank_roll')


def amostras(prefixo, modelo, t_fim):
    sup = [(float(r['t']), int(float(r['support']))) for r in csv.DictReader(open(prefixo + '_estabilizador.csv'))]
    # miolo de cada trecho de apoio simples
    alvos, i = [], 0
    while i < len(sup):
        s = sup[i][1]
        j = i
        while j < len(sup) and sup[j][1] == s:
            j += 1
        if s in (1, 2) and j - i >= 8:
            for k in range(i + (j - i) // 4, i + 3 * (j - i) // 4):
                if 2.0 <= sup[k][0] <= t_fim:
                    alvos.append(sup[k])
        i = j
    rows = sorted(csv.DictReader(open(prefixo + '_juntas.csv')), key=lambda r: float(r['t']))
    estado, out, ri = {}, [], 0
    for t, s in alvos:
        while ri < len(rows) and float(rows[ri]['t']) <= t:
            r = rows[ri]
            estado[r['junta']] = (float(r['process_value']), float(r['command']), float(r['set_point']))
            ri += 1
        if len(estado) < 20:
            continue
        lado = 'l' if s == 1 else 'r'
        q = {j + '_link': v[0] for j, v in estado.items()}
        prev = (modelo.torques_grf(q, '%s_feet_1' % lado) if os.environ.get('GRF')
                else modelo.torques(q, '%s_feet_1' % lado, apoio=True))
        for jt in JUNTAS_APOIO:
            nome = '%s_%s' % (lado, jt)
            out.append((jt, prev[nome + '_link'], estado[nome][1]))
    return out


def main():
    modelo = Modelo(sys.argv[1])
    print('massa total do URDF: %.3f kg' % modelo.M)
    dados = {}
    for f in sorted(glob.glob(sys.argv[2] + '_estabilizador.csv')):
        pre = f[:-len('_estabilizador.csv')]
        for jt, p, m in amostras(pre, modelo, 28.0):
            dados.setdefault(jt, []).append((p, m))
    print('%-10s %6s %10s %10s %8s %6s %s' % ('junta', 'n', 'prev med', 'medido', 'inclin.', 'R2', 'portao'))
    for jt in JUNTAS_APOIO:
        a = np.array(dados.get(jt, []))
        if len(a) < 20:
            continue
        x, y = a[:, 0], a[:, 1]
        k = float(x @ y / (x @ x))
        r2 = 1 - ((y - k * x) ** 2).sum() / ((y - y.mean()) ** 2).sum()
        ok = 0.7 <= k <= 1.3 and r2 > 0.6 and np.sign(x.mean()) == np.sign(y.mean())
        print('%-10s %6d %+10.2f %+10.2f %8.2f %6.2f %s' % (jt, len(a), x.mean(), y.mean(), k, r2, 'OK' if ok else 'FALHA'))


if __name__ == '__main__':
    main()
