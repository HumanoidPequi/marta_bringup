# Consulta rápida: que parâmetros usar num robô com estas especificações?

> Tabela de consulta da bateria do panorama
> ([panorama_parametros.md](panorama_parametros.md): 354 corridas,
> servo MX-28, marcha a 0,1 m/s, simulação).
>
> A mesma lógica, como calculadora:
> `python3 src/marta_bringup/experiments/consulta_parametros.py`. Ela pergunta
> cada valor; Enter mantém o da Marta atual. Ou passe flags:
> `--massa 3.5 --coxa 95 ... -n`.

## Em 5 passos

### 1. Índice de carga: anda ou não anda?

**I = m · g · L / stall**
- m = massa total;
- L = coxa + canela;
- stall = torque de stall do servo do joelho na tensão de uso. O MX-28 dá
  2,5 N·m a 12 V e 2,31 N·m a 11,1 V; o stall é proporcional à tensão.

| I | combinações que andaram (de 16) | classe |
|---|---|---|
| 2,19 | 13 | janela ampla |
| 2,53 | 11 | janela ampla |
| 2,73 | 10 (8 com ganho 0,5) | janela média |
| 2,92 | 2 | borda |
| 3,10 | 1 | não anda |
| ≥ 3,29 | 0 | não anda |

**Mire I ≤ 2,6.** Acima de 3,0, nenhuma postura segura o 1º apoio: o joelho
de apoio satura e o robô cai para trás.

**Para baixar I:** servo mais forte, tensão maior, menos massa ou perna mais
curta. A massa máxima para I = 2,6 é m = 2,6 · stall / (g · L).

### 2. Postura: que flexão de joelho?

Joelho-alvo parado: combinações de `hip_offset_x` que andaram, de 4, no
nível de massa testado.

| I (massa da Marta) | 82° | 73° | 63° | 53° | de onde vem |
|---|---|---|---|---|---|
| 2,19 (2,85 kg) | **4** | **4** | 3 | 2 | grade com ganho 0,5 |
| 2,53 (3,30 kg) | **4** | 3 | 2 | 2 | grade com ganho 0,5 |
| 2,73 (3,56 kg) | **3** | 2 | **3** | 2 | grade com ganho 0,3 |
| 2,92 (3,80 kg) | 1 (3/3 com ganho 0,3) | 0 | 1 | 0 | grade com 0,5 e fase B |
| 3,10 (4,04 kg) | 0 | 1 | 0 | 1 | grades com 0,3 e 0,5 |
| ≥ 3,29 | 0 | 0 | 0 | 0 | – |

**Leitura:**
- **Joelho entre 73 e 82° é o mais robusto.** A postura agachada cai menos de
  lado, e enquanto não houver estabilizador de roll o lado é o que mais
  derruba.
- **Esticar a perna poupa torque**, mas só compensa perto do limite de carga.

**Torque previsto no joelho de apoio:** τ ≈ m · g · b, com b = (b/L) · L.
Mantenha τ ≤ ~0,9 · stall.

| joelho-alvo | 82° | 73° | 63° | 53° |
|---|---|---|---|---|
| b/L (medido) | 0,302 | 0,271 | 0,240 | 0,204 |

### 3. Converter o joelho em `z_com` com a geometria do robô

Pela IK do `marta_control`, com a = coxa, b = canela:

```
θ_ik   = θ + 1,73 + 0,0457·(θ − 53,17)              (correção empírica, ver abaixo)
c²     = a² + b² + 2·a·b·cos(θ_ik)                  (distância quadril-tornozelo²)
y      = y_separation − (tronco→quadril, lateral)
x      = tronco→quadril em x
z_com  = √(c² − y² − x²) + (tronco→quadril, vertical) + (tornozelo→sola)
```

**Na Marta atual**, a conta dá 0,239 / 0,250 / 0,260 / 0,269 para 82 / 73 /
63 / 53°. São os níveis testados, com erro de até 0,3 mm.

**Correção empírica:** a IK estática superestima o joelho parado da Marta em
1,7 a 3,1°, por causa do estado inicial do gerador. Num robô novo, confira o
joelho real (`l_knee` no log ou na telemetria) e ajuste o `z_com`.

### 4. `hip_offset_x`: o CoM à frente do pé de apoio

- **Alvo:** CoM − pé de apoio entre 0 e +10 mm, em apoio simples
  (telemetria `sup_x`).
- **Com mais massa, o alvo sobe**, porque a cedência empurra o tronco para
  trás.
- **Fora da faixa:**
  - CoM atrás do pé cai para trás;
  - CoM muito à frente (> +15 mm) tende a cair de lado: 44 das 52 quedas
    laterais foram com x ≥ 9 mm.
- **Na prática:** comece no x do URDF (tronco → quadril) e ajuste pela
  telemetria.

| massa (I) | CoM − pé nas corridas que andaram, mediana | `hip_offset_x` que andaram na Marta (joelho 82°) |
|---|---|---|
| 2,85 kg (2,19) | +2,8 mm | −4; 3,6; 9; 13 mm |
| 3,30 kg (2,53) | +1,6 mm | −4; 3,6; 9; 13 mm |
| 3,56 kg (2,73) | +1,7 mm | 3,6; 9; 13 mm |
| 3,80 kg (2,92) | +8,8 mm | 9 mm |
| 4,04 kg (3,10) | +12 mm | 9 mm (com joelho 73° ou 53°) |

### 5. Compensação e tronco

| parâmetro | valor | por quê |
|---|---|---|
| `gravity_comp_gain` | 0,3 (I > 2,6); 0,3–0,5 (I ≤ 2,6) | 0,3 e 0,5 empatam em sobrevivência; 0,3 deixa o tronco mais ereto; 0,7 derruba para a frente |
| `gravity_comp_joints` | `hip_roll,knee` | máscara validada |
| `hip_pitch_offset_deg` | −5 | a compensação só entra no 1º passo; sem o −5°, o robô começa inclinado para trás e cai (19/24 contra 6/24) |
| estabilizador de pitch | ligado | não muda a postura parada, mas sem ele 2 de 5 condições que andavam caem |

## Exemplo: a Marta com 3,80 kg

```
python3 consulta_parametros.py -n --massa 3.80
  I = 2,92 → BORDA
  joelho 82° → z_com 0,239; hip_offset_x 0,009; ganho 0,3; hip_pitch_offset −5
  (na bateria: 3/3 com ganho 0,3)
```

## O que vale e o que não vale

- **Vale para:** robôs parecidos com a Marta, isto é, tronco concentrando a
  maior parte da massa e coxa ≈ canela, com servo do tipo MX-28 e esta marcha
  (período 0,6 s, apoio duplo 0,3, `y_sep` 0,06, 0,1 m/s).
- **É ponto de partida, não garantia**, quando a geometria muda. A bateria
  variou só a massa do tronco de uma geometria.
- **Não foi medido:**
  - giro, marcha lateral ou outras velocidades;
  - robustez a empurrões;
  - postura com estabilizador de roll, que ainda não existe e deve abrir as
    posturas esticadas.
- **Simulação:** o servo real tem folga, aquecimento e queda de tensão.
