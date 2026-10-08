# Resultados da campanha — estabilizador de pitch

> Parte de [README.md](README.md). Contexto de projeto: [estabilizadores.md](estabilizadores.md).

78 corridas, nenhuma descartada por divergência numérica. Marcha de 20 s a
0,1 m/s no `world.world`. Todas as métricas de atitude são **verdade de solo do
Gazebo**, não a estimativa da IMU.

Reproduzir: `bash src/marta_bringup/experiments/rodar_campanha.sh` e depois
`python3 src/marta_bringup/experiments/analisar.py`.

## Tabela geral

| τ [N·m] | cond | n | quedas | RMS pitch real | RMS roll | dist [m] | viés do estimador |
|---|---|---|---|---|---|---|---|
| 1000 | sem | 5 | 0/5 | 2,60 ± 0,27 | 3,82 | 1,644 | +0,6° |
| 1000 | com | 5 | 0/5 | 2,31 ± 0,41 | 3,86 | 1,721 | +1,3° |
| 12 | sem | 5 | 0/5 | 2,71 ± 0,36 | 4,05 | 1,628 | +4,2° |
| 12 | com | 5 | 0/5 | 2,27 ± 0,53 | 3,81 | 1,701 | +1,6° |
| 8 | sem | 8 | 0/8 | 2,27 ± 0,17 | 4,67 | 1,329 | +1,9° |
| 8 | com | 8 | 0/8 | 2,45 ± 0,32 | 4,66 | 1,304 | +5,2° |
| **6** | sem | 8 | 0/8 | **5,54 ± 0,41** | 6,58 | 0,943 | +21,3° |
| **6** | com | 8 | 0/8 | **3,18 ± 0,55** | 6,35 | 0,943 | +13,6° |
| **5** | sem | 8 | **8/8** | — | — | 0,152 | +13,5° |
| **5** | com | 8 | **6/8** | — | — | 0,374 | +21,2° |
| 4 | sem | 5 | 5/5 | — | — | 0,117 | −0,6° |
| 4 | com | 5 | 5/5 | — | — | 0,194 | −0,2° |

RMS calculado só sobre corridas que completaram os 20 s: numa corrida que caiu,
o RMS é dominado pelos 45° da queda e mede outra coisa.

## Os quatro regimes

**τ ≥ 8 N·m — o controlador é desnecessário.** Sem diferença detectável
(p = 0,23 / 0,16 / 0,19). Também não atrapalha, que é o resultado que se quer.

**τ = 6 N·m — ganho de qualidade.** RMS 5,54 → 3,18, **−43%**,
t = 9,71, **p = 2,6 × 10⁻⁷**, d de Cohen = 4,9. Nenhuma queda em nenhuma das
16 corridas. Custo: Δθ de pico 6,99°, **0% do tempo em saturação**, distância
idêntica (0,943 m).

**τ = 5 N·m — ganho de sobrevivência.** 8/8 quedas contra 6/8. Mediana do tempo
até a queda **4,6 s → 10,6 s**; log-rank χ² = 5,12, **p = 0,024**. Distância
percorrida 2,5× maior.

**τ = 4 N·m — nada salva.** 5/5 nas duas condições.

## O mecanismo: postura, não amortecimento

Decompondo RMS² = média² + variância a 6 N·m:

| | sem | com | |
|---|---|---|---|
| postura (\|média\|) | 4,23° | 0,92° | −78% |
| oscilação (desvio) | 3,57° | 2,98° | −16% |

**82% da queda do RMS vem de postura, 18% de oscilação.** A saturação de torque
faz o robô ceder para trás ~4°; o estabilizador detecta e devolve o tronco à
vertical com uma correção de quadril quase constante (visível na fig. 6).

Isso casa exatamente com o modo de falha medido antes da campanha: as quedas
reais são **sagitais e para trás**, com o pitch cruzando 20° em ~1 s e chegando
a −45 a −55°, enquanto o roll só reage depois. O estabilizador ataca a postura
que precede a queda.

## Um defeito do estimador que a campanha expôs

O viés (verdade − estimativa) acompanha a dificuldade: ~1° nos regimes fáceis,
**+21° a 6 N·m**. Nesse regime a estimativa chega a cruzar 45° com o robô de pé
e o tronco na altura normal.

Duas consequências já corrigidas: a métrica passou a ser verdade de solo, e a
detecção de queda também (antes disparava por `|pitch estimado| > 45°`, gerando
quedas fantasma). A causa provável é o portão de confiança testar só a **norma**
da aceleração — durante saturação ela fica perto de g com a **direção** errada, e
com τ = 5 s do filtro complementar o erro leva segundos para ser lavado.

## Limitações

- n de 5 a 8 por célula. Suficiente para o efeito a 6 N·m (d = 4,9); apertado
  para taxas de queda, que exigiriam ~100 corridas por condição.
- Ganhos sintonizados no Gazebo, com juntas controladas por esforço, não
  transferem direto para os servos de posição do robô real.
- O plano frontal segue em malha aberta. O roll é o maior resíduo (6,4°
  contra 3,2° de pitch a 6 N·m), mas **não é o modo de falha**.
- Peso adaptativo no acelerômetro é o próximo item, antes da Fase 3.
