#!/usr/bin/env python3
"""Validacao do teleop: por trecho da sequencia (sequencia_cmd.py), vx e w
realizados contra os comandados, roll RMS verdadeiro e quedas.

    python3 analisar_teleop.py [dados]

t0 da sequencia = primeiro instante em que o pe de apoio troca (inicio da
marcha), menos MARGEM; cada trecho descarta os primeiros DESC s (o comando
entra no passo seguinte, ate 0,6 s, mais o transitorio).
"""
import csv, glob, math, os, sys

SEQ = [(0, 'reta', 1, 0), (8, 'curva+', 1, 1), (16, 'reta', 1, 0), (22, 'giro+', 0, 1),
       (30, 'giro-', 0, -1), (36, 'no lugar', 0, 0), (42, 'curva-', 1, -1), (50, 'giro+', 0, 1),
       (56, 'reta', 1, 0), (62, 'no lugar', 0, 0), (68, 'PARADO', 0, 0), (75, 'reta', 1, 0),
       (82, 'curva+', 1, 1), (90, 'fim', 0, 0)]
DESC = 1.5
MARGEM = 0.3


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dados')
    for f in sorted(glob.glob(os.path.join(d, 'T_*_telemetria.csv'))):
        tag = os.path.basename(f)[:-len('_telemetria.csv')]
        w = float(tag.split('_w')[1].split('_')[0])
        R = list(csv.DictReader(open(f)))
        t = [float(r['t']) for r in R]
        sup = [r['support'] for r in R]
        i0 = next(i for i in range(1, len(R)) if sup[i] != sup[i - 1])
        t0 = t[i0] - MARGEM
        print('\n== %s  (w %.3f; t0 = %.1f s da gravacao; fim %.1f s) ==' % (tag, w, t0, t[-1] - t0))
        print('%-9s %6s %6s %7s %7s %6s %6s' % ('trecho', 'vx_cmd', 'vx', 'w_cmd', 'w', 'w/cmd', 'roll'))
        for k in range(len(SEQ) - 1):
            a, rot, fx, fw = SEQ[k]
            b = SEQ[k + 1][0]
            idx = [i for i in range(len(R)) if a + DESC <= t[i] - t0 < b]
            if len(idx) < 20:
                print('%-9s (sem dados: caiu antes?)' % rot)
                continue
            yaw = [float(R[i]['yaw_true']) for i in idx]
            for j in range(1, len(yaw)):      # desenrola
                while yaw[j] - yaw[j - 1] > math.pi: yaw[j] -= 2 * math.pi
                while yaw[j] - yaw[j - 1] < -math.pi: yaw[j] += 2 * math.pi
            dt = t[idx[-1]] - t[idx[0]]
            wr = (yaw[-1] - yaw[0]) / dt
            vx = 0.0
            for j in range(1, len(idx)):
                dx = float(R[idx[j]]['com_x']) - float(R[idx[j - 1]]['com_x'])
                dy = float(R[idx[j]]['com_y']) - float(R[idx[j - 1]]['com_y'])
                vx += dx * math.cos(yaw[j]) + dy * math.sin(yaw[j])
            vx /= dt
            roll = math.sqrt(sum(math.degrees(float(R[i]['roll_true'])) ** 2 for i in idx) / len(idx))
            wc = fw * w
            print('%-9s %6.2f %6.3f %+7.3f %+7.3f %6s %6.2f' % (
                rot, 0.1 * fx, vx, wc, wr, '%.2f' % (wr / wc) if wc else '-', roll))


if __name__ == '__main__':
    main()
