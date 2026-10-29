# Ajustes de parâmetros da marcha (outubro de 2026)

> Parte de [README.md](README.md). A análise que motivou estes ajustes está em
> [plano_comparacao_darwin.md](plano_comparacao_darwin.md) §2.

## Contexto

A Marta usa o servo MX-28AT (stall de 2,5 N·m), mas na simulação só andava com
o limite de torque em 6 N·m. Ao investigar, encontramos dois tipos de problema:

1. **A geometria da cinemática inversa (IK) não batia com o robô (URDF).** Por
   isso a Marta andava muito agachada e com o pé de apoio atrás do centro de
   massa (CoM).
2. **Os parâmetros da marcha sobrecarregavam a perna em balanço.** Ela tinha
   pouco tempo, os pés ficavam abertos demais e o pé subia pouco. Na campanha
   de 72 corridas, essa perna foi o maior pedido de torque.

## O que mudou

**Geometria da IK** (`marta_control/src/RobotPhysicalParameters.cpp`; vale para
todo uso, requer recompilar):

| parâmetro | antes | depois | por quê |
|---|---|---|---|
| `leftAnkleToLeftFoot.z` (tornozelo → sola) | −0,13305 m | −0,03705 m | A IK supunha 13,3 cm entre o tornozelo e a sola; no URDF são ~3,7 cm. Para manter a altura do tronco, ela dobrava o joelho a ~110° (o Darwin-OP anda a ~55°). |
| `torsoToLeftHip.x` (tronco → quadril, eixo x) | 0,027349 m | 0,013349 m | Medido em 12 corridas: o CoM ficava **9 a 19 mm à frente** do pé de apoio, e por isso o robô pendia para frente. A IK punha o quadril 27 mm à frente do tronco; no URDF são 3,6 mm. Recuamos 14 mm, o deslocamento medido. |

**Marcha** (args do `marta_bringup/launch/stabilizer_test.launch`, que valem
sobre o `.cpp`):

| parâmetro | antes | depois | por quê |
|---|---|---|---|
| `z_com` (altura do tronco na IK) | 0,2975 m | **0,252 m** (default do launch e do `.cpp`) | Com a geometria nova, 0,2975 fica fora do alcance da perna. Com 0,252, a flexão prevista do joelho é de ~70°. |
| `double_support_ratio` | 0,5 | 0,3 (default do launch) | Com 0,5, o apoio simples durava só 0,2 s, e a perna em balanço errava a trajetória em 6 a 13°. Com 0,3 ele passa a 0,28 s (Darwin: 0,27 s). |
| `y_separation` | 0,08 m | 0,06 m (default do launch) | Com 0,08, os pés ficavam a 16 cm um do outro, para quadris a 8 cm. A perna em balanço abria ~44° em roll. |
| `z_step` (altura do pé) | 0,06 m | 0,08 m (default do launch) | O pé subia ~2 cm dos 6 planejados e raspava no chão. |
| `step_period` (duração de um passo) | 0,4 s | 0,6 s | O joelho não acompanhava o balanço de 0,28 s. Com 0,6, o atraso no pouso cai de 15° para 5°, e as raspadas e os pousos de ponta despencam (Experimento 1). |

**Sem efeito na marcha** (só ferramentas):
- `z_com`, `z_step`, `step_period`, `double_support_ratio`, `y_separation` e
  `torso_mass_scale` viraram args do launch;
- o `walk_test.py` passou a medir o fator de tempo real só até a queda
  (README, achado H).

## Resultado

**Observado (visual):** com a geometria corrigida, a marcha ficou "bem mais
normal". O robô deixou de pender para frente e anda menos agachado.

**Medido:** **ainda não.** As mudanças foram feitas juntas, e falta a medida
no protocolo da campanha:
- 6 corridas a τ = 6 e 6 a τ = 1000;
- pé em caixa, estabilizadores ligados, verdade de solo.

O que comparar com a linha de base antiga (mesmo protocolo, n = 6):

| métrica | linha de base | nova |
|---|---|---|
| quedas com τ = 6 / τ = 4 | 0/6 / 6/6 | a medir |
| torque do joelho, RMS (τ = 1000) | 6,3 N·m | a medir |
| tempo com o joelho acima do stall (2,5 N·m) | 73% | a medir |
| CoM − pé de apoio, eixo x | +14 mm | a medir |
| flexão do joelho na marcha | ~114° | ~70° previsto |

## Investigação: o pé raspa e pousa de ponta (2026-10-20/21)

Configuração atual, pé planejado a 80 mm. Scripts:
`experiments/rodar_pe.sh`, `analisar_pe.py` e `registrar_pe.py`; dados em
`experiments/pe/` e `pe_juntas/`.

- **O pé sobe menos que o planejado.** Pico de 62–68 mm com τ = 1000 e de
  42–51 mm com τ = 6. Com τ = 6, o joelho da perna em balanço fica
  **saturado ~75% do tempo** e erra o alvo em ~18°. Mesmo sem limite de
  torque, o erro é de ~13°.
- **O atrito alto não é a causa.** Com μ = 1,0 o pé raspou mais (55–68% dos
  passos, contra 36–39%) e houve mais quedas. É tendência, com n = 4.
- **O pouso de ponta (63–88% dos passos) vem do atraso do joelho.** Juntas e
  pé foram gravados no mesmo instante, e o pitch do pé é explicado por
  0,89 × tronco + 0,88 × (soma dos ângulos reais de quadril, joelho e
  tornozelo), com R² = 0,96.
  - O **joelho** fica 14° **aquém** do alvo no meio do balanço, e por isso o
    pé sobe pouco.
  - No fim do balanço, ele fica 15° **ainda dobrado**: a ponta desce, cerca de
    +8,6° no pé.
  - O tornozelo segue o alvo dele (erro de −2°), mas não compensa o atraso do
    joelho, porque o alvo dele assume o joelho no lugar.
  - O plano em si deixa o pé 4,5° de ponta para cima. É o offset fixo de −5°
    no pitch do quadril (`OmnidirectionalWalkZMP.cpp:225`), que o tornozelo
    não compensa e que hoje atenua o pouso de ponta.

**Conclusão:** um só mecanismo, o joelho que não acompanha a trajetória do
balanço, explica os dois sintomas. Remédios possíveis: dar mais tempo ao
balanço, um modelo de servo realista e compensação de gravidade ou
feedforward no joelho.

## Experimento 1: mais tempo de balanço (2026-10-21)

**Configuração:** a atual, τ = 6, tronco pela metade, n = 4 por condição.
Scripts: `experiments/rodar_balanco.sh` e `analisar_balanco.py`; dados em
`experiments/balanco/`.

| `step_period` | apoio duplo | apoio simples | quedas | pico do pé | raspou | ponta | joelho no pouso (real − alvo) |
|---|---|---|---|---|---|---|---|
| 0,4 (atual) | 0,3 | 0,28 s | 2/4 | 48 mm | 50% | 71% | +15,5° |
| 0,5 | 0,3 | 0,35 s | 1/4 | 56 mm | 7% | 61% | +11,0° |
| **0,6** | **0,3** | **0,42 s** | **1/4** | **61 mm** | **10%** | **17%** | **+5,4°** |
| 0,4 | 0,2 | 0,32 s | 4/4 | 45 mm | 67% | 27% | +14,9° |
| 0,5 | 0,2 | 0,40 s | 4/4 | 47 mm | 50% | 0% | +8,7° |
| 0,6 | 0,2 | 0,48 s | 2/4 | 45 mm | 29% | 15% | +5,7° |

- **Mais tempo de passo reduz o atraso do joelho no pouso.** Com ele, caem
  as raspadas e os pousos de ponta, e o pé sobe mais.
- **Encurtar o apoio duplo para 0,2 piora as quedas** (2 a 4 de 4).
- **Com o tronco de massa normal**, `step_period` 0,6 com apoio duplo 0,3
  teve **2/4 quedas, em 13 e 21 s**. A configuração anterior tinha 4/4 quedas
  em 3–4 s.
- O passo fica mais longo: a 0,1 m/s, 6 cm em vez de 4.
- **O joelho continua saturado ~60% do balanço** com τ = 6.
- **Ressalva:** n = 4, então as diferenças de quedas não têm significância
  estatística. As métricas do pé são consistentes entre os três períodos.

**Marcha usada nos experimentos 2 e 3:** `step_period` 0,6, apoio duplo
0,3, tronco de massa normal.

## Experimento 2: modelo de servo MX-28 na Marta (2026-10-21)

**Modelo** (plano_comparacao_darwin.md §3), ligado por args do
`stabilizer_test.launch`:
- `gains:=…/controller_mx28.yaml`: p = 12,74 N·m/rad, i = d = 0;
- `implicit_damping:=true joint_damping:=0.43`;
- `joint_velocity:=100`;
- limite de 2,5 N·m.

**Marcha:** a do experimento 1. Script: `experiments/rodar_servo.sh`.

**Decomposição** (τ = 6, n = 2):

| servo | tronco | resultado |
|---|---|---|
| só o ganho do MX-28 (p = 12,74) | normal | cai **antes da marcha**, no 1º passo após o agachamento (2/2) |
| só o amortecimento (p = 50 + 0,43 implícito) | normal | anda, cai de lado em 8–9 s (2/2) |
| MX-28 completo | normal | cai antes da marcha (2/2), também com τ = 1000 |
| MX-28 completo | metade | anda ~30 s com τ = 1000 e com **τ = 2,5** (n = 1 cada) |

- **O que derruba com o servo realista é a rigidez, não o torque.** No 1º
  apoio simples, o joelho de apoio cede **20°** (alvo 91°, real 111°). Um
  servo de posição cede ~τ/Kp: 3,5 N·m ÷ 12,74 N·m/rad ≈ 16°. O tronco tomba
  para trás.
- **O amortecimento implícito é numericamente estável.** O achado D do
  README ("amortecimento instável") vale para o explícito.
- **`walk_test.py`:** nas quedas antes da marcha ele reporta "CAIU em 2,5 s"
  com RTF 0,000. O laço termina na hora, e a queda é do pós-agachamento.

**E3, servo atual × MX-28** (tronco pela metade, n = 4). Quedas com t > 30 s
aconteceram **na parada**, depois da janela de marcha, e contam como
"completou".

| servo | τ | quedas na marcha | completou 30 s | pico do pé | joelho no pouso | joelho saturado |
|---|---|---|---|---|---|---|
| atual (p = 50, vel. 0,01) | 2,5 | 4/4 (3 antes da marcha) | 0/4 | 33 mm | +19° | 73% |
| atual | 4 | 0/4 | 4/4 | 51 mm | +9° | 84% |
| atual | 6 | 1/4 | 3/4 | 59 mm | +6° | 61% |
| **MX-28** | **2,5** | **1/4** (26,5 s) | **3/4** | 61 mm | +13° | 43% |
| MX-28 | 4 | 2/4 | 2/4 | 60 mm | +13° | 6% |
| MX-28 | 6 | 2/4 | 2/4 | 60 mm | +12° | 0% |
| MX-28 | 1000 | 1/4 | 3/4 | 62 mm | +13° | 0% |

- **Com o servo realista e o stall real de 2,5 N·m, a Marta (tronco pela
  metade) completa a marcha em 3 de 4 corridas.** Com o servo atual e o mesmo
  limite, ela cai sempre.
- Com o MX-28 **não há tendência com τ** (as quedas são 1–2 de 4 em qualquer
  limite). O fator limitante deixa de ser o torque.
- O joelho do MX-28 erra menos no meio do balanço (−7° a −11°, contra −20° a
  −26°), mas chega ao pouso mais dobrado (+12° a +13°). Mesmo assim, **não
  houve pouso de ponta** (0%).
- O critério "raspou" sai alto com o MX-28 (≈ 90%) porque o pé **descola
  tarde** no início do balanço. No meio do balanço a folga é igual à do servo
  atual (calcanhar a 30–34 mm).

**Limiar de massa** (MX-28, τ = 2,5): com o tronco a **75%** (4,04 kg no
total), **4/4 quedas em 3,5 s**. O limiar fica entre 3,30 e 4,04 kg de massa
total; a Marta real tem 4,78 kg.

**Limite de velocidade das juntas** (servo atual, τ = 6, tronco pela metade):
o URDF declara 0,01 rad/s, e o Gazebo zera o esforço que empurra a junta
acima disso. Removido o limite (100 rad/s), a marcha cai **antes de
começar** em 4/4; com 0,01, cai 2/4 em 20–26 s. **O servo atual só é estável
porque esse limite artificial funciona como um freio.** O MX-28 com
amortecimento físico de 0,43 dispensa o limite.

## Experimento 3: Darwin-OP com o mesmo modelo de servo (2026-10-21)

**Configuração:** a simulação do Darwin em `~/Documents/ic/op2_ws`, com a
marcha da ROBOTIS e o passo máximo. O modelo MX-28 é idêntico ao da Marta:
p = 12,74, amortecimento implícito de 0,43 e limite τ em todas as juntas.
n = 8 por condição, 30 s. Script: `op2_sim_bringup/scripts/rodar_darwin_mx28.sh`;
análise em `experiments/analisar_darwin.py`; dados em
`experiments/darwin_mx28/`.

| Darwin | quedas | avanço em 30 s | joelho: comando RMS / p99 | > 2,5 N·m | erro do joelho |
|---|---|---|---|---|---|
| MX-28, τ = 2,31 (bateria 11,1 V) | **0/8** | 4,30 m | 1,69 / 3,01 N·m | 12% | 7,6° |
| MX-28, τ = 2,5 (12 V) | **0/8** | 4,18 m | 1,65 / 2,91 | 11% | 7,4° |
| MX-28, τ = 3, 4, 6 | 0/8 cada | 4,11 m | 1,63 / 2,79 | 9% | 7,3° |
| servo anterior (p = 30, limite 10) | 0/8 | 4,08 m | 2,48 / 5,40 | 33% | 4,7° |

**Marta × Darwin no mesmo servo MX-28** (joelho, comando antes da saturação,
toda a marcha):

| | massa total | quedas a 2,5 N·m | joelho RMS / p99 | > 2,5 N·m | erro |
|---|---|---|---|---|---|
| Darwin | 2,93 kg | 0/8 | 1,65 / 2,91 N·m | 11% | 7,4° |
| Marta, tronco pela metade (τ = 1000) | 3,30 kg | 1/4 | 1,73 / 3,78 N·m | 13% | 7,8° |
| Marta, tronco 75% | 4,04 kg | 4/4, em 3,5 s | — | — | — |
| Marta, tronco normal | 4,78 kg | cai antes da marcha | — | — | — |

- **O Darwin anda com o stall real do MX-28**, até a 11,1 V, sem nenhuma
  queda em 40 corridas.
- **Com massa parecida (3,30 contra 2,93 kg), a Marta pede quase o mesmo do
  joelho** (RMS 1,73 contra 1,65 N·m), com picos ~30% maiores. A diferença
  que sobra é de **massa**.
- O "comando" aqui é Kp × erro e **não inclui** o torque do amortecimento
  (0,43 × velocidade). Os números com p = 30 e com p = 12,74 não se comparam
  diretamente.

## Resumo dos três experimentos

1. **Mais tempo de balanço** (`step_period` 0,6, apoio duplo 0,3) reduz o
   atraso do joelho no pouso de 15° para 5°. Com isso, o pé sobe mais, raspa
   menos e quase não pousa de ponta. Com o servo atual e o tronco normal,
   passa a andar em 2 de 4 corridas.
2. **O servo simulado atual não é físico.** Ele só é estável por causa do
   limite artificial de 0,01 rad/s nas juntas. O modelo MX-28, com
   amortecimento físico, anda com o stall real (2,5 N·m) **quando o tronco é
   mais leve**. Com a massa real, o joelho de apoio cede ~20° (τ/Kp) e a Marta
   cai no primeiro passo.
3. **No mesmo servo, o Darwin e a Marta leve pedem o mesmo torque do
   joelho.** O que impede a Marta de andar com MX-28s é a massa: o limiar fica
   entre 3,3 e 4,0 kg, e ela tem 4,78. Mais exatamente, é a rigidez do servo
   diante dessa massa. Isso aponta para a compensação de gravidade
   (feedforward Δθ = τ/Kp, plano_fase3_roll.md), que ataca exatamente a
   cedência do joelho de apoio, ou para reduzir a massa do tronco.

## Compensação de gravidade (implementada em 2026-10-22, desligada por padrão)

**O que é:** um feedforward nas pernas, somado ao alvo da IK antes do
estabilizador de pitch:

    Δθ = clamp(τ_gravidade × ganho, ±limite de torque) / Kp

- **τ** vem do modelo de corpos rígidos do `robot_description`: pé de apoio
  fixo, tronco vertical (Maximo, doutorado §4.3).
- **Kp** é lido do controlador que de fato roda (`/marta/<junta>_position/pid/p`).
- **Na troca de apoio**, os pesos de carga são interpolados como em Maximo
  (eq. 4.23–4.24) e somam 1.
- **No robô real (`isReal`)**, fica desligada.

**Código** (submódulo `marta_control`):
- `GravityCompensator.h/.cpp`: o núcleo, sem ROS;
- `RobotModelLoader.h`: monta o modelo a partir do URDF;
- `LegGravityCompensation.h/.cpp`: a camada ROS;
- `OmnidirectionalWalkZMP::getLoadWeights`;
- ligação no `controller.cpp`.

**Teste offline:** `test/test_gravity_compensator.cpp`, com 19 verificações,
todas passando:
- pêndulo de 2 links analítico;
- massa do URDF (4,781 kg);
- torques iguais aos do modelo Python independente
  (`experiments/gravidade/modelo_gravidade.py`);
- pesos de carga.

Os testes antigos (simetria da marcha, estabilizador, atitude) continuam
passando.

**Args do `stabilizer_test.launch`:**

| arg | default | função |
|---|---|---|
| `gravity_comp_enabled` | false | liga a compensação |
| `gravity_comp_joints` | `hip_roll,knee` | máscara de juntas; `all` ou uma lista |
| `gravity_comp_gain` | 1,0 | multiplica o torque previsto |
| `max_gravity_comp` | 0,35 rad | teto da correção |
| `gravity_comp_swing` | false | compensa também a perna em balanço |
| `hip_offset_x` | 0,013349 | x do quadril na IK, sem recompilar |

O diagnóstico sai em `/marta/gravity_comp`.

**Validação E1 com dados** (`experiments/gravidade/validar_e1.py`, τ = 1000, 8
corridas):
- **`hip_roll` passa** (inclinação 1,17, R² 0,76).
- **No joelho**, o sinal confere e o modelo prevê ~57% do torque médio. O
  R² ≈ 0, porque a dinâmica da perna em balanço domina passo a passo.
- **Quadril e tornozelo pitch não passam.**
- A versão pela força de reação do chão (LIPM) não melhorou.

**Em simulação** (servo MX-28, τ = 2,5; `experiments/rodar_gravidade.sh`,
`analisar_gravidade.py`):

| condição | tronco | quedas na marcha | cedência do joelho de apoio |
|---|---|---|---|
| sem compensação | metade | 0/4 (3 caem na parada) | +7,1° |
| `hip_roll,knee`, ganho 1,0 / 1,75 | metade | 4/4, antes da marcha | −4,6° / −8,1° (passa do ponto) |
| todas as juntas, ganho 0,5 | metade | 3/3 em 4,2 s | +2,1° |
| **todas as juntas, ganho 0,5 + `hip_offset_x` 0,0036 (URDF)** | **metade** | **1/3** (2 completaram 32 s) | **+1,4°**; pé a 77 mm, 6% de raspadas |
| **só `hip_roll,knee`, ganho 0,5 + 0,0036** | metade | **1/3** (2 completaram 32 s) | +2,9°; pé a 68 mm, 3% de raspadas |
| todas as juntas, ganho 0,5 + 0,0036 | **normal** | 2/2 antes da marcha (também com τ = 1000) | +1,1° |

1. **O mecanismo funciona.** A cedência do joelho de apoio cai de +17,6°
   para ~+1° (massa real, τ = 1000).
2. **A máscara de juntas não faz diferença.** Com ganho 0,5 e x do URDF,
   "só `hip_roll` e joelho" (a escolha do Maximo) e "todas" dão o mesmo
   resultado (1/3 quedas cada). Os torques previstos para quadril e tornozelo
   pitch são pequenos. **Corrige uma conclusão anterior** desta página
   ("precisa ser a cadeia toda"), que confundia o efeito da máscara com o do
   ganho.
3. **O ganho que funciona é ~0,5**, e não 1,0. Com 1,0 a compensação passa do
   ponto.
4. **O `hip_offset_x` 0,013 foi ajustado com o joelho cedendo.** Com a
   cedência compensada, o CoM fica 19 mm à frente do pé, e o valor geométrico
   do URDF (0,0036) volta a ser o certo (CoM a +8 mm).
5. **Com a massa real, a compensação não basta.** Com o servo MX-28, a Marta
   cai nos passos iniciais **mesmo com torque ilimitado e o joelho na IK**.
   O modelo estático já pede 2,74 N·m médios no joelho de apoio, acima do
   stall de 2,5. E a marcha e o estabilizador foram sintonizados para o servo
   de p = 50.

6. **As quedas com a compensação são laterais.** Foram 2 de 2 (roll, em 6 e
   23 s), enquanto sem a compensação nenhuma corrida caiu durante a marcha. A
   compensação reduz a inclinação estática em roll no apoio simples (−2,2° →
   −1,2°), mas não amortece a oscilação (roll RMS de 4,0–6,3°). É o papel do
   estabilizador de roll em realimentação (Maximo §4.4.2, P+V no quadril e no
   tornozelo), ainda não implementado (plano_fase3_roll.md §4.3). Com n = 3, é
   uma indicação, não uma prova.

**Configuração recomendada para usar com o servo MX-28:**
`gravity_comp_enabled:=true gravity_comp_joints:=hip_roll,knee gravity_comp_gain:=0.5 hip_offset_x:=0.0036`.
Os defaults continuam como antes: compensação desligada e x = 0,013349.

## Atenção

- **Defaults alinhados (2026-10-21):** o `OmnidirectionalWalkZMP.cpp` e o
  `stabilizer_test.launch` usam os valores desta página. Com isso, `z_com`
  0,252 e `step_period` 0,6 valem sem precisar passar argumento.
- **`torso_mass_scale:=0.5` é um teste, não um ajuste.** Ele corta a massa do
  tronco pela metade (4,78 → 3,30 kg) e não pertence à configuração da Marta.
  **Medido em 2026-10-20 (`experiments/rodar_pe.sh`):** com os ajustes acima,
  o τ = 6 e o tronco de massa normal, a Marta **cai em 4 de 4 corridas, entre
  3,0 e 3,9 s**. Com o tronco pela metade, anda (0/4 quedas, atrito padrão).
  Ou seja, a marcha "bem mais normal" foi observada com o tronco leve, e os
  ajustes sozinhos não bastam com a massa real.
- **Os efeitos individuais não foram separados.** Se alguma métrica piorar,
  testar uma mudança por vez.
