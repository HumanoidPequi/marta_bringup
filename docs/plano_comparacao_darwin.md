# Comparação Marta × Darwin-OP: por que a Marta precisa de 6 N·m (plano, não implementado)

> Parte de [README.md](README.md). Leia o mapa central primeiro.
> Documentos irmãos: [estabilizadores.md](estabilizadores.md),
> [telemetria_marcha.md](telemetria_marcha.md),
> [plano_fase3_roll.md](plano_fase3_roll.md).

**Estado (2026-10-15): plano.** Nenhuma corrida nova foi feita. Só a análise de dados que já existiam (E0c parcial, §2.1), que mudou a prioridade dos experimentos.

**Pergunta.** A Marta usa o servo MX-28AT, o mesmo do Darwin-OP/OP2. Na
simulação, porém, ela só anda com o limite de torque de junta em 6 N·m, e o
stall do servo é de 2,5 N·m. Queremos saber o que a marcha de cada robô
exige e onde a Marta difere: massa, geometria, parâmetros de marcha, modelo de
servo na simulação, controle ou contato.

**Ponto de retomada (2026-10-15, pausa a pedido do usuário):**

**Feito:**
- E0 e E1c (§2.3);
- correção do RTF no `walk_test.py`;
- parâmetros do gerador expostos no launch.

**Próximo, nesta ordem (estimativa total de ~11 h sem o E4, ou 16–20 h com
ele):**
1. **E3:** modelo comum de MX-28 na Marta (§3), começando pela estabilidade
   do amortecimento implícito.
2. **Tempo de balanço:** `step_period` × `double_support_ratio`.
3. **E2:** o Darwin no mesmo servo.
4. **E5, E7, E1a/b.**
5. **E4:** o nosso gerador no Darwin, decidido depois do E2.

**Atualização (2026-10-21):** os passos 1 (tempo de balanço), 2 (E3) e 3
(E2) foram executados. Resultados em [ajustes_marcha.md](ajustes_marcha.md)
§Experimentos 1–3 e §Resumo. O próximo candidato natural é a compensação de
gravidade, porque com o servo realista o limite é a cedência do joelho de
apoio diante da massa da Marta.

**Nada foi commitado.** Uma decisão pendente: pôr
`experiments/postura/dados/` no `.gitignore`?

**Princípio.** Diferenças entre os dois robôs só são interpretáveis se o
resto for igual: o mesmo motor de física, o mesmo modelo de servo, as mesmas
métricas e o mesmo protocolo. Por isso o plano começa fixando um **modelo
comum de MX-28** (§3) e **métricas idênticas** (§4). Só depois vêm as
hipóteses (§5) e os experimentos cruzados (§6).

**Convenção de evidência.** Todo número traz uma marca:

| marca | significado |
|---|---|
| **[F]** | fato com fonte (página ou URL) |
| **[M]** | medido por nós, com a forma de reproduzir |
| **[I]** | inferência, a confirmar por experimento |
| **[?]** | falta o dado |

**Referências por sigla:**

| sigla | obra |
|---|---|
| **Maximo-D** | Maximo, *Automatic Walking Step Duration through Model Predictive Control*, tese de doutorado, ITA, 2017. Lida no PDF; o número impresso coincide com a página do PDF. |
| **Maximo-M** | Maximo, *Omnidirectional ZMP-Based Walking for a Humanoid Robot*, dissertação de mestrado, ITA, 2015. Lida no PDF (`~/Downloads/Mestrado Marcos Maximo.pdf`); o número impresso coincide com a página do PDF. |

---

## 1. Premissas verificadas e como elas mudam o plano

### 1.1 Servo

**[F] O MX-28T, o MX-28AT e o MX-28AR têm o mesmo desempenho.** A tabela de
especificações do e-Manual é única para os quatro modelos
(https://emanual.robotis.com/docs/en/dxl/mx/mx-28/):

| especificação | valor |
|---|---|
| stall a 11,1 V | 2,3 N·m |
| stall a 12 V | 2,5 N·m |
| stall a 14,8 V | 3,1 N·m |
| redução | 193:1 |
| velocidade em vazio a 12 V | 55 rpm |
| diferença do T para o AT | só a carcaça (TTL × RS-485 e carcaça metálica; "same features and perform equally") |

A tese confirma:
- Maximo-D p. 56: "the only upgrade MX-28AT has is a metallic case… no
  difference in terms of dynamic response is expected".
- Maximo-D p. 51: o OP2 usa um MX-28T em cada junta.

**[F] Os "~2 N·m" não aparecem no datasheet.** O valor mais próximo é
2,3 N·m a 11,1 V, a tensão nominal da bateria do Darwin (e-Manual do
Darwin-OP). **[I]** É provável que os "~2 N·m" venham daí.

O plano usa as duas tensões como bracketing:
- **12 V → 2,496 N·m**, a tensão em que Maximo validou o servo
  (Maximo-D p. 65);
- **11,1 V → 2,309 N·m**, a bateria do robô.

### 1.2 Massa

**[F] OP2: 3,0 kg e 454,5 mm de altura** (Maximo-D p. 51).
**[M] O modelo de CAD tem 2,925 kg** (soma das massas do `Darwin-op.proto` do
Webots; ver §2).
**Darwin-OP: 2,8–2,9 kg, só por fontes secundárias** (fichas RoboCup:
Falconbots 2015, 2,9 kg; Manitoba 2014, 2,8 kg). A figura "Dimensions and
Weight" do e-Manual **não foi lida**: a imagem não carregou nesta etapa. **[?]**
Conferir a figura.

**Marta: 4,781 kg no URDF** [M]. São **+1,86 kg (+64%)** em relação ao CAD do
Darwin, e não os "~0,5 kg" da conversa inicial.

**[F] A frase do [README.md](README.md) §7, item 3 está errada em duas
coisas:**
- O OP3 usa o servo **XM430-W350**, de 4,1 N·m, e não o MX-28AT
  (https://emanual.robotis.com/docs/en/platform/op3/introduction/).
- O OP3 pesa ~3,5 kg; daí +1,28 kg, e não ~0,5 kg.

A referência certa para a Marta é o **Darwin-OP/OP2**. O README ainda não foi
corrigido; isso espera decisão do usuário.

### 1.3 De onde vem o nosso gerador: Bioloid, não OP2

**[F] O mestrado, que é a origem do nosso `OmnidirectionalWalkZMP`, não foi
feito para o OP2.**
- O robô real é um **Bioloid modificado com servos AX-12+** (Maximo-M §5.2,
  p. 102, e Apêndice A).
- O stall do AX-12+ é **12 kgf·cm a 7 V e 16,5 kgf·cm a 10 V** (Maximo-M,
  Tabela A.1, p. 111), ou seja, **1,18 e 1,62 N·m**.
- A simulação usa o modelo do Bioloid no **V-REP, com ODE** (Maximo-M §5.1 e
  Apêndice B, p. 115–118).
- **Os parâmetros da Tabela 5.1 (p. 85) são desse robô.**

**[F] O doutorado usa o OP2 com MX-28** (Maximo-D p. 51 e 56), mas com outros
geradores:
- preview control do ZMP no cap. 5;
- duração de passo por MPC nos cap. 6 e 7.

A Tabela 5.1 do doutorado (p. 96) é do preview control, com parâmetros
"motivated by the DARwIn-OP robot".

**Consequência [I]:**
- **Não existe publicação do nosso gerador rodando no Darwin.** O E4 é a
  primeira vez que isso aconteceria.
- Os dois conjuntos do Maximo servem só como referência de ordem de grandeza:
  um é o nosso gerador num robô diferente, o outro é o robô certo com um
  gerador diferente.
- A premissa do prompt, de que o gerador "veio da dissertação feita para o
  OP2", está corrigida aqui.

**[F] Definições do mestrado** (Tabela 3.1, p. 72), que tiram a ambiguidade
dos nomes:

| símbolo | definição no mestrado | equivalente no nosso código |
|---|---|---|
| T | "Period of a step": duração de **um** passo | `period` (confere: `OmnidirectionalWalkZMP.cpp:114`) |
| zCoM | "Height of the center of mass", com o CoM aproximado por um ponto fixo no tronco (p. 50: "We approximate the CoM by a fixed position in the torso") | `zCom`, usado como altura do tronco |
| zstep | altura máxima do pé em balanço | `zStep` |
| rds | fração de apoio duplo | `doubleSupportRatio` |
| ysep | "**Half** of the separation between the feet" | `ySeparation` |
| s | fator da coordenada x do pé oposto aplicado ao braço | `armsCompensationFactor` (`OmnidirectionalWalkZMP.cpp:210`) |

**Implicações:**
- **[I]** Pela definição do mestrado, usar `zCom` como altura do tronco é o
  projeto original, uma aproximação, e não um erro de implementação. O
  achado G continua válido: a física do LIPM pede a altura real do CoM.
- **[I]** Com `ySeparation = 0,08`, a separação nominal entre os pés é de
  **0,16 m**, o dobro da distância entre os quadris no URDF (~0,08 m). A
  largura real na marcha precisa ser medida (E0b).

### 1.4 A simulação do Darwin

**[M] A simulação do Darwin está em `~/Documents/ic/op2_ws`**, na imagem
`op2_sim:noetic` derivada da `marta_bringup:1.0`, que não foi alterada.

**Componentes:**
- o modelo da HumaRobotics (`darwin_description`, `darwin_control`);
- o port do Walking da ROBOTIS (`robotis_op_simulation_walking`);
- correções nossas, documentadas em
  `op2_ws/src/op2_sim_bringup/README.md` e na docstring de
  `scripts/fix_inertia.py`.

**As correções foram:**
- massa, CoM e inércia dos 21 corpos tirados do CAD da ROBOTIS, via o PROTO do
  Webots;
- sola do pé em caixa;
- IMU no frame do tronco;
- amortecimento implícito de 0,43 N·m·s/rad;
- patch no nó de marcha.

**Resultado com o passo máximo de 20 mm:** 8/8 corridas de 30 s sem queda,
4,08 ± 0,08 m em 30 s (5 corridas) [M].

**[F] A tese aponta os mesmos defeitos que corrigimos no modelo da
HumaRobotics:** "incorrect mass parameters, lack of joint damping and
inadequate actuator dynamics" (Maximo-D p. 54).

**[I] A simulação do Darwin ainda não serve de referência de torque.** Hoje ela
tem:
- PID `p = 30`, `i = d = 0`;
- amortecimento de 0,43;
- limite de effort das pernas de **10 N·m**.

Com isso, o pico medido no joelho é 5,9 N·m e o RMS é 2,5 N·m [M]. O **E2**
(§6) põe o Darwin no modelo comum de servo antes de qualquer comparação.

### 1.5 O limiar de 6 N·m da Marta foi medido com o pé em malha

**[F] O limiar está documentado com a colisão do pé em malha.**
- As campanhas de torque (τ = 4, 5 e 6) são anteriores a 2026-10-07, quando o
  padrão passou a ser `foot_collision:=box` (README, achado I).
- Os números vêm de [resultados_campanha.md](resultados_campanha.md), com o
  estabilizador ligado:

| limite τ | quedas |
|---|---|
| 5 N·m | 6/8 corridas |
| 4 N·m | 5/5 corridas |

- A linha de base com o pé em caixa para τ = 5 e τ = 4 está listada como
  pendente (README §7, item 5).

**[I] O pé em caixa mudou a marcha:** o avanço em 30 s foi de 1,05 m para
2,47 m. O limiar pode ter mudado junto. **O experimento E0 refaz o limiar
antes de tudo.** O problema que este plano ataca precisa ser medido de novo
nas condições atuais.

### 1.6 O que a tese do Maximo dá e o que não dá

**Dá** [F]:
- o modelo físico do MX-28 em Gazebo/ODE (§3);
- a arquitetura de marcha (Fig. 4.1, p. 73–74);
- a compensação de gravidade (§4.3, p. 78–81);
- os parâmetros dos geradores (Tabelas 5.1, 5.2, 6.1 e 7.1).

**Não dá** [F]:
- **torque de junta medido na marcha** (o texto não traz nenhum);
- os ganhos do compensador de tronco (p. 85: "os valores não são
  publicados").

**Consequência:** a referência de torque do Darwin tem de ser **gerada por nós**.

---

## 2. Tabela estrutural Marta × Darwin-OP/OP2

O frame de cada linha é o do respectivo modelo. "Perna" vai do eixo de pitch do
quadril até a sola.

| grandeza | Marta | Darwin-OP/OP2 | marca / fonte |
|---|---|---|---|
| **massa total** | 4,781 kg | 2,925 kg (CAD); 3,0 kg (OP2); 2,8–2,9 kg (Darwin-OP, fontes secundárias) | Marta [M] URDF; Darwin [M] PROTO, [F] Maximo-D p. 51, §1.2 |
| tronco | 2,966 kg (62%) | 0,976 kg (33%) | [M] URDF / PROTO |
| cada perna | 0,542 kg | 0,630 kg | [M] (Darwin: 0,027 + 0,167 + 0,119 + 0,070 + 0,167 + 0,079) |
| cada braço | 0,280 kg | 0,254 kg | [M] |
| cabeça + pescoço | 0,170 kg | 0,182 kg | [M] |
| **coxa** (pitch do quadril → joelho) | 0,101 m no URDF; **0,0982 m na IK** | 0,093 m | Marta [M] `marta_joints.xacro`, `RobotPhysicalParameters.cpp`; Darwin [F] `THIGH_LENGTH` do Walking.cpp e âncoras do PROTO |
| **canela** (joelho → pitch do tornozelo) | 0,100 m vertical no URDF (com 0,0415 m de desvio lateral); **0,0974 m na IK** | 0,093 m | idem |
| tornozelo → sola | ~0,037 m no URDF; **0,133 m na IK** (`leftAnkleToLeftFoot.z`) | 0,0335 m | Marta [M] `marta.xacro` (fundo da caixa da sola) e `RobotPhysicalParameters.cpp`; Darwin [F] `ANKLE_LENGTH` |
| **flexão do joelho na marcha** | **109° planejada; 114–119° medida** (média no apoio, 10 corridas por condição) | **55° planejada; 56° medida** | [M] §2.1 |
| **perna L** | ~0,238 m | 0,2195 m | Marta [I] soma; Darwin [F] `LEG_LENGTH = 219.5` |
| eixos roll/pitch do quadril | **não coincidem** (desvio de 21 mm em x e em y) | coincidem | [M] URDF / PROTO; [M] a IK **não** modela o desvio: `hipRoll = atan2(y, z)`, com um único deslocamento tronco→quadril (`InverseKinematics.cpp`) |
| **m·g·L** | 11,16 N·m | 6,30 N·m | [I] |
| **stall / (m·g·L)** | 0,224 (com 2,5 N·m); 0,538 (com 6 N·m) | 0,397 (com 2,5 N·m) | [I] |
| altura do CoM em pé | **[?]** | 0,222 m (pernas retas, modelo do CAD) | Darwin [M] FK do `darwin_fixed.urdf` |
| altura do CoM na marcha | **depende do limite de torque:** 0,2197 ± 0,0002 m (τ = 1000); 0,2124 ± 0,0008 (τ = 6); 0,2158 ± 0,0004 (τ = 6 com estabilizador) | **[?]** medir (E2); Maximo usa h = 0,24 m (LIPM) e 0,20 m (corpo inteiro) | Marta [M] `z_com_meas`, t ∈ [5, 28] s, n = 10 por condição, `experiments/yaw/dados/*caixa*_telemetria.csv`; Darwin [F] Maximo-D p. 96, 98 e 113 |
| largura entre os pés (centro a centro) | **[?]** na marcha; ~0,079 m na pose zero do URDF | 0,074 m na pose zero; a marcha da ROBOTIS soma `Y_OFFSET` = 5 mm | [M] URDF / PROTO; **medir na marcha** (E0b) |
| **sola** | caixa 0,140 × 0,070 m | 0,104 × 0,066 m (caixa do CAD, na nossa simulação) | Marta [M] `marta.xacro`; Darwin [F] Maximo-D p. 115 ("0.1 m and 0.066 m") |
| atrito / contato do pé | μ = 1e10, kp = 1e6, kd = 1e4 | μ = 9000, kp = 1e6, kd = 10, `maxContacts` = 10 | [M] xacro / URDF |
| **servo real** | MX-28AT | MX-28T (OP2), MX-28 (OP) | [F] §1.1 |
| stall real | 2,5 N·m a 12 V; 2,3 N·m a 11,1 V | idem | [F] |
| **servo na simulação: controlador** | `effort_controllers/JointPositionController` | idem | [M] |
| ganhos (pernas) | p = 50 N·m/rad, d = 0,05, i ≈ 0 | p = 30, i = d = 0 | [M] `controller.yaml` / `gains.yaml` |
| amortecimento de junta | 0 | 0,43 N·m·s/rad (implícito) | [M] |
| inércia refletida do rotor | não | não | [M] |
| limite de effort | 1000 por default; 6 no regime de operação | 10 (pernas); 2,8 (resto) | [M] |
| física | ODE, 1 ms, 1000 Hz, `worlds/empty.world` | ODE, 1 ms, 50 iterações SOR, 1000 Hz, `worlds/empty.world` | Darwin [M] `get_physics_properties`; Marta [I] mesmo world (**conferir** com o mesmo serviço) |
| **gerador de marcha** | OmnidirectionalWalkZMP (LIPM analítico) | Walking da ROBOTIS (senoidal, sem LIPM) | [F] código |
| duração de **um passo** | 0,4 s (`period`) | 0,3 s (`PERIOD_TIME` = 600 ms é o ciclo de **dois** passos) | [F] `OmnidirectionalWalkZMP.cpp:114`; `Walking.cpp` (apoios simples em 1/4 e 3/4 do período) |
| fração de apoio duplo | 0,5 | 0,1 | [F] |
| altura do pé no balanço | `zStep` = 0,06 m (sobe ~2 cm na prática) | `foot_height` = 40 mm | [F] / [M] desvio_de_direcao.md |
| altura do "CoM" no gerador | `zCom` = 0,2975 m: **distância tronco–pé, não altura do CoM** (achado G) | o gerador não usa CoM; `Z_OFFSET` = 20 mm de flexão | [F] |
| velocidade | 0,1 m/s | 0,136 m/s medido (passo de 20 mm) | [M] |
| **parâmetros do Maximo-D** (Darwin) | — | Tstep = 0,5 s; h = 0,24 m (LIPM) e 0,20 m (corpo inteiro); ysep = 0,1 m (0,06 na Tab. 7.1); fds = 0,2; sola usada no controlador 0,06 × 0,03 m ("conservative"); v = 0,1–0,2 m/s | [F] Tab. 5.1 p. 96, 5.2 p. 98, 6.1 p. 113, 7.1 p. 152; §6.5 p. 113–115 |
| **parâmetros do Maximo-M** (Tabela 5.1, p. 85) | — | T = 0,2 s; zCoM = 0,23 m; zstep = 0,04 m; rds = 0,2; ysep = 0,04 m; s = 0,8 **, do Bioloid com AX-12+ no V-REP, não do Darwin** | [F] Maximo-M p. 85; robô: §1.3 |

**Cinco pontos da tabela que mudam o plano:**

1. **Normalização [I].** Com o mesmo stall, a Marta tem 0,224 de stall por
   m·g·L, contra 0,397 do Darwin. Se massa e comprimento de perna fossem a
   única diferença, a Marta precisaria de ~0,397 × 11,16 ≈ **4,4 N·m** para ter
   a mesma folga relativa do Darwin a 2,5 N·m. **Mas isso pressupõe que o
   Darwin anda com 2,5 N·m, o que ainda não sabemos (E2).** Se ele andar e a
   Marta precisar mesmo de 6 N·m, sobra um fator de ~1,4 para as outras
   hipóteses.
2. **A massa está em lugares diferentes [M].** A perna do Darwin é **mais
   pesada** que a da Marta (0,630 contra 0,542 kg). O excesso da Marta está
   **todo no tronco** (2,97 contra 0,98 kg). Para o joelho e o quadril em apoio
   simples, o que pesa é a massa acima deles. Por isso a hipótese (a) separa
   **massa total** de **distribuição**.
3. **Os nomes dos parâmetros não significam a mesma coisa [F].**
   - `period` da Marta é a duração de um passo. O `PERIOD_TIME` da ROBOTIS é o
     ciclo de dois passos.
   - `zCom` da Marta é a altura do **tronco**, por projeto (o mestrado aproxima
     o CoM por um ponto fixo no tronco, §1.3). Não é a altura real do CoM.
   - `ysep` é **metade** da separação no mestrado (Tabela 3.1) e no texto do
     doutorado (p. 96: "lateral distance of 2·ysep"). A Tabela 5.1 do
     doutorado, porém, o descreve como "Minimum lateral distance between
     feet", e a Fig. 5.1 mostra 0,1 m com ysep = 0,1. Há ambiguidade dentro do
     próprio doutorado.
   - **Regra do plano: comparar grandezas medidas** (duração do passo, altura
     do CoM, largura entre os pés), **nunca nomes de parâmetros.**
4. **A IK e o URDF não concordam [M], com consequência [I].** A IK da Marta usa
   coxa e canela de 98,2 e 97,4 mm. O URDF tem 101 e 100 mm. Os eixos de roll e
   pitch do quadril não coincidem, e não se sabe se a IK compensa isso. Se não
   compensar, a postura real difere da planejada, inclusive a flexão do
   joelho, que é o que decide o torque no joelho (achado E). O **E0c** mede
   isso nos bags que já existem.
5. **A altura do CoM da Marta não é um número só [M].** Ela varia com o limite
   de torque: 0,212 a 0,220 m, porque o joelho cede mais com τ = 6
   (flexão de 119° contra 114,5°, §2.1). Os 0,2134 m do README e os 0,2184 m
   da telemetria_marcha.md eram medidas certas, em condições diferentes.
   Toda normalização usa a altura **medida na própria corrida**.

### 2.1 Primeira medida (E0c parcial, só com dados existentes): a Marta anda muito agachada

**[M] Flexão do joelho na marcha**, média de \|ângulo\| de `l_knee` e `r_knee`
em t ∈ [5, 28] s. Nos dois modelos, joelho = 0 é perna reta.

| condição | n | planejado | medido | τ_cmd RMS do joelho |
|---|---|---|---|---|
| Marta, τ = 1000, pé em caixa | 10 | 109,0° | 114,5 ± 0,1° | 6,6 N·m |
| Marta, τ = 6, pé em caixa | 10 | 109,0° | 119,0 ± 0,5° | 10,5 N·m |
| Marta, τ = 6, pé em caixa, estabilizador ligado | 10 | 109,0° | 117,1 ± 0,3° | 8,8 N·m |
| Darwin (marcha da ROBOTIS, `track_final`) | 1 | 55° | 56° | 2,5 N·m (p = 30) |

Fontes:
- Marta: `src/marta_bringup/experiments/yaw/dados/tau*_caixa*_juntas.csv`;
- Darwin: corrida de 2026-10-15 na simulação corrigida.

Os τ_cmd usam servos diferentes (p = 50 na Marta, p = 30 no Darwin) e **não**
são comparáveis entre si. Estão aqui só como contexto.

**[M] Origem da flexão:** a IK da Marta (`RobotPhysicalParameters.cpp`) usa:
- tronco → quadril: z = −0,0595 m;
- tornozelo → pé: z = **−0,133 m**.

No URDF, a sola fica a ~0,037 m do eixo do tornozelo (fundo da caixa do pé em
`marta.xacro`). Com `zCom = 0,2975`, sobram para quadril → tornozelo
0,2975 − 0,0595 − 0,133 ≈ **0,105 m**, contra uma perna de 0,196 m. **[I]** Só
dá para fechar a conta agachando muito: ~115° em pé parado, ~109° com o
balanço da marcha.

**[I] Hipótese sobre o 0,133 m:** pode ser herança de outro robô, já que o
arquivo traz `Author: itandroids`, ou uma escolha deliberada. **[?]** Perguntar
a quem portou o gerador.

Consequência: o `zCom` "nominal" não é realizado. A altura real do tronco é
definida pela IK agachada.

**[I] Torque estático do joelho em apoio simples** (modelo grosseiro):
- hipóteses: quadril na vertical do tornozelo, coxa e canela iguais, regime
  quase estático;
- conta: τ ≈ m_acima·g·a·sen(flexão/2).

| robô | massa acima do joelho | braço | τ estático |
|---|---|---|---|
| Marta | 4,53 kg | 80 mm | **3,55 N·m (142% do stall de 2,5)** |
| Darwin | 2,61 kg | 43 mm | **1,10 N·m (44% do stall)** |

A razão é **3,2×**: massa (1,74×) × braço (1,86×).

**Leitura [I], a confirmar por experimento:**
- Nesta postura, a Marta **não sustenta nem o apoio simples parado** com o
  stall de um MX-28 real, qualquer que seja o controle.
- O Darwin tem folga de ~2×.
- Isso põe a **postura (hipótese b/b')** em pé de igualdade com a massa e
  explica uma parte grande do "precisa de 6 N·m".
- O modelo ignora dinâmica, perna em balanço e deslocamento horizontal do
  CoM. Os números servem como ordem de grandeza, e não como predição.

**[I] O que muda no plano:** o **E1c** (varrer `zCom`, ou o deslocamento
tornozelo → pé da IK, para reduzir a flexão) passa a ser o experimento mais
barato e mais informativo, junto com o E0.

### 2.2 Pilotos de postura (2026-10-15): a estimativa estática só vale para a perna de apoio

**Ferramenta.** `z_com`, `z_step`, `step_period`, `double_support_ratio` e
`y_separation` passaram a ser args do `stabilizer_test.launch` (rosparams do
`controller_node`), com default igual ao gerador. Comportamento padrão
inalterado.

**[M] Geometria da IK.** A flexão do joelho prevista pelos comprimentos da IK
reproduz a medida no `zCom` atual (110° contra 109° planejados).

| `zCom` | flexão prevista | resultado (uma corrida-piloto) |
|---|---|---|
| 0,2975 | 110° | anda |
| 0,32 | 93° | anda |
| 0,345 | 72° | anda |
| 0,36 | 56° (a do Darwin) | **cai em 2,5 s** |

O alvo de 0,369 que constava antes estava errado; o certo era ~0,36.

**[M] Torque pedido** (τ = 1000, uma corrida por nível, t ∈ [5, 28] s), RMS do
`command` por fase:

| `zCom` (flexão medida) | joelho de **apoio** | joelho em **balanço** | quadril pitch em balanço |
|---|---|---|---|
| 0,2975 (114°) | 5,40 N·m | 5,32 | 4,11 |
| 0,32 (99°) | 4,55 | 10,14 | 8,37 |
| 0,345 (78°) | 3,94 | 10,34 | 7,80 |

**[I] Leitura (n = 1 por nível, a confirmar na campanha):**
- **A estimativa estática acerta a perna de apoio.** O joelho de apoio
  alivia 27% de 0,2975 para 0,345.
- **A perna em balanço dobra de torque.** Com a perna quase reta, levantar o
  pé `zStep` = 6 cm dentro de um apoio simples de 0,2 s
  (0,4 s × (1 − 0,5)) exige uma variação angular muito maior no joelho e no
  quadril. O PID com p = 50 persegue isso com 10–12° de erro.
- **O torque não é só postura: é postura × exigência do balanço.** A marcha
  do Darwin levanta o pé 4 cm, num apoio simples de 0,27 s.

### 2.3 Campanha de postura × altura do pé (2026-10-15)

**Desenho:**
- `zCom` {0,2975; 0,345} × `zStep` {0,06; 0,04} × τ {1000, 6, 4};
- n = 6 por célula, 72 corridas, intercaladas;
- regime de operação (pé em caixa, estabilizador de pitch e malha de rumo
  ligados), 30 s a 0,1 m/s;
- scripts: `experiments/rodar_postura.sh` e `experiments/analisar_postura.py`;
  dados em `experiments/postura/`.

**[M] RTF.** Todas as 72 corridas são válidas. O RTF que o `walk_test.py`
reportava incluía o tempo **depois** da queda. Com o robô deitado, a física fica
~2× mais lenta, e toda queda cedo saía com RTF ~0,5. Com um monitor de
tempo simulado × tempo real, o RTF era 1,00 até o instante da queda; as 29
corridas sem queda deram ≥ 0,999. O `walk_test.py` foi corrigido para fechar a
janela na queda. Verificação: a mesma condição passou a medir 0,999.
**Isso também põe em dúvida a leitura do achado H do README.**

**[M] Quedas** (Fisher contra a linha de base no mesmo τ; tempo até a queda
contado a partir do início da marcha):

| `zCom` | `zStep` | τ = 1000 | τ = 6 | τ = 4 |
|---|---|---|---|---|
| 0,2975 | 0,06 (**linha de base, E0**) | 0/6 | **0/6** | **6/6** (2,5–5 s) |
| 0,2975 | 0,04 | 1/6 | 3/6 (p = 0,18) | 6/6 |
| 0,345 | 0,06 | 4/6 (p = 0,06) | 6/6 (p = 0,002) | 6/6 |
| 0,345 | 0,04 | 6/6 (p = 0,002) | 6/6 (p = 0,002) | 6/6 |

**[M] Torque pedido** (τ = 1000, `command` antes da saturação, RMS médio entre
corridas):

| `zCom`, `zStep` (n efetivo) | joelho de apoio | joelho em balanço | joelho, todas as fases | fração > 2,5 N·m |
|---|---|---|---|---|
| 0,2975, 0,06 (6) | 5,51 | 5,55 | 6,29 (p99 12,7) | 73% |
| 0,2975, 0,04 (6) | 5,35 | 5,82 | 5,96 | 76% |
| 0,345, 0,06 (5) | 3,03 (−45%, p = 0,009) | 10,92 (+97%, p = 0,001) | 7,07 (+12%, p = 0,03) | 64% |
| 0,345, 0,04 (3) | 2,22 | 8,29 | 5,09 | 47% |

Nas duas células de `zCom` 0,345, as corridas que caíram contribuem só até a
queda, e as que caíram antes de 5 s ficam fora. **Há viés de sobrevivência**:
a coluna (0,345; 0,04) tem n = 3.

**[I] Leitura:**
1. **Nova linha de base (E0).** Com o pé em caixa, o limiar continua entre 4 e
   6 N·m: 0/6 quedas a 6 N·m, 6/6 a 4 N·m, todas nos primeiros 5 s. O
   resultado antigo, com o pé em malha (τ = 4: 5/5), se mantém.
2. **No regime atual, a marcha pede 3–5 vezes o stall de um MX-28.** O joelho
   fica acima de 2,5 N·m 73% do tempo. **Mesmo o joelho de apoio fica acima do
   stall 98% do tempo.** Com 2,5 N·m, a Marta não anda nesta configuração.
3. **A hipótese "a postura agachada é a causa principal" não se sustenta na
   forma simples.**
   - Esticar a perna alivia o apoio (−45%), mas o balanço quase dobra.
   - O torque total do joelho não cai (+12%).
   - Com a mesma marcha e o mesmo estabilizador, a postura alta **cai até com
     torque ilimitado**.
   - A marcha atual está acoplada à postura agachada: o `tZMP` muda com
     `zCom`, e o estabilizador e a marcha foram ajustados agachados.
4. **O gargalo dominante é a perna em balanço.** Joelho e quadril pitch pedem
   5–11 N·m RMS no balanço, com 6–13° de erro de rastreamento. É um servo de
   posição com p = 50, sem amortecimento, perseguindo uma trajetória rápida:
   apoio simples de 0,2 s. **O que mais pesa passa a ser o modelo de servo
   (hipótese d) e o tempo de balanço (hipótese c: `period` e
   `doubleSupportRatio`),** mais do que a massa ou a postura.
5. **Baixar o pé de 6 para 4 cm**, como na marcha do Darwin, não reduz o
   torque na postura atual (−5%, p = 0,09) e tende a aumentar as quedas a
   τ = 6 (3/6, n.s.).

**Próximo passo derivado:** medir quanto do torque do balanço é dinâmica e
quanto é artefato do servo simulado. Experimentos:
- **E3** (modelo comum de MX-28, com amortecimento);
- variação do tempo de apoio simples (`step_period`, `double_support_ratio`);
- **E2** no Darwin, para saber quanto a perna em balanço do Darwin pede com o
  mesmo servo.

---

## 3. Modelo comum de MX-28 na simulação (proposta)

**Objetivo.** Os dois robôs simulam o **mesmo motor**, de modo que a diferença
de torque seja do robô e não do modelo. A base é o modelo físico do Maximo-D,
§3.3.2 a §3.3.4, validado em bancada contra um MX-28AT real até ~32 rad/s
(p. 65–70).

**Parâmetros do motor e da caixa** (Maximo-D, Tabela 3.1, p. 62, e
p. 58–59) [F]:

| parâmetro | valor |
|---|---|
| R | 8,3 Ω |
| L | 2,03 mH (desprezada: polo elétrico em −4,01e4) |
| N | 193 |
| η | 0,836 |
| Kt | 0,0107 N·m/A |
| Kω | 93,1 rad/V (unidade como impressa na tabela) |
| Jm | 8,68e−8 kg·m² |
| bm | 8,87e−8 N·m·s |
| ganho padrão | P = 32 (Kp^c = P/8 = 4); I = D = 0 |

### 3.1 As quatro peças do modelo

| peça | expressão | valor | marca |
|---|---|---|---|
| amortecimento equivalente da força contraeletromotriz | B = bm + N²·Kt²·η/R (eq. 3.34, p. 62) | **0,4295 N·m·s/rad** | [F] fórmula; [I] conta |
| saturação | τ_stall = Vs·N·Kt·η/R (eq. 3.35, p. 63) | **2,496 N·m** (12 V); **2,309 N·m** (11,1 V) | [F] / [I] conta |
| ganho P no Gazebo | Kp,gz = (4096/2π)·(N·Kt·η/R)·(P/8)·(Vs/511) (eq. 3.37, p. 63) | **12,74 N·m/rad** (12 V); **11,78** (11,1 V); Ki = Kd = 0 | [F] / [I] conta |
| inércia refletida | Jm·N²·η, somada ao corpo do rotor (§3.3.4, p. 64–65) | **2,70e−3 kg·m²** | [F] / [I] conta |

**Verificação de consistência [I].** Com B e τ_stall acima, a velocidade em
vazio do modelo é τ_stall/B = 5,81 rad/s. O datasheet dá 55 rpm = 5,76 rad/s.

### 3.2 Por que o amortecimento é parte do servo

**[F]** No Maximo-D, eq. 3.36 (p. 63), o torque do motor é
**τ_motor = k·u − B·θ̇**, com |u| ≤ Vs. O motor real perde torque com a
velocidade.

Um limite de effort puro, como o `joint_effort` da Marta hoje, entrega o stall
**em qualquer velocidade**. **[I]** Ele é otimista em torque e não tem a
dissipação do motor.

### 3.3 Inércia refletida: aplicar com cuidado

**[F] O que a tese diz:**
- sem a inércia refletida, a simulação "explodes" com passo de 1 ms (p. 64);
- somá-la ao corpo do rotor "tends to greatly overestimate the inertia,
  especially for joints at the beginning of the kinematic chain" (p. 65).

**[M] O que observamos:** no Darwin, o amortecimento **implícito** (opção
`implicitSpringDamper` do ODE) deu estabilidade **sem** a inércia refletida,
com p = 30 a 1 ms.

**[I] Proposta:**
- modelo comum **sem** a inércia refletida (variante S0);
- variante S0+J **com** ela, como teste de sensibilidade;
- 2,70e−3 kg·m² é ~25 vezes a inércia de um link de perna do Darwin
  (~1e−4 kg·m²), por isso o efeito tem de ser medido, e não assumido.

### 3.4 Variantes do modelo de servo

| variante | p | d | B (implícito) | limite | uso |
|---|---|---|---|---|---|
| **S-atual** | como está hoje em cada robô | | | | ponto de partida |
| **S0 (MX-28, 12 V)** | 12,74 | 0 | 0,4295 | 2,50 | **referência principal** |
| S0-11V | 11,78 | 0 | 0,4295 | 2,31 | bateria real |
| S0+J | igual a S0 + inércia refletida | | | | sensibilidade (3.3) |
| S0-τ | igual a S0, com limite ∈ {3, 4, 5, 6, 1000} | | | | curva de limiar |

### 3.5 Pendências de implementação (nada disso foi feito)

**Na Marta:**
- **[F]** `joint_damping` existe no xacro, mas o `marta.gazebo` **não** liga
  `implicitSpringDamper`.
- **[F]** O README (achado D) registra que o amortecimento de junta foi
  "numericamente instável no ODE e inutilizável". Nenhum documento detalha o
  teste.
- **[I]** A instabilidade é a do amortecimento explícito, e o implícito a
  evitaria, como no Darwin. **Esse é o primeiro ponto a verificar no E3.**

**No Darwin:** o limite de effort sai do URDF. É preciso uma opção no
`fix_inertia.py` para fixá-lo.

### 3.6 Definição de torque para as métricas

| símbolo | origem | significado |
|---|---|---|
| **τ_cmd** | `.../state.command` | saída do PID **antes** da saturação |
| **τ_app** | `joint_states.effort` (`GetForce` do gazebo_ros_control, depois do `EffortJointSaturationInterface`) | torque **aplicado** pelo "motor"; **não** inclui o amortecimento da junta |
| **τ_motor** | τ_app − B·θ̇ | torque líquido na saída; só existe no modelo com B |

**"Perto do stall"** se mede em **τ_cmd** contra o limite, porque é τ_cmd que
satura (o análogo da tensão).

---

## 4. Métricas idênticas nos dois robôs

**Janela de análise:** a mesma nos dois robôs. Fica de fora o primeiro 1,0 s
depois do início da marcha (transitório de partida; cf. desvio_de_direcao.md)
e o último passo antes do pedido de parada.

**Regras do projeto** [F] (README, achados B e H):
- métricas de atitude e queda sempre pela **verdade de solo**;
- descartar corridas com RTF < 0,85.

| # | métrica | definição | Marta: fonte | Darwin: fonte |
|---|---|---|---|---|
| 1 | torque por junta | τ_cmd e τ_app: pico de \|τ\|, RMS e percentis 50/95/99, por junta e separados em apoio simples e duplo | `/marta/<j>_position/state` (100 Hz), `/marta/joint_states` | `/darwin/<j>_position_controller/state` (100 Hz) [M], `/darwin/joint_states` (50 Hz) [M] |
| 2 | fração perto do stall | fração do tempo com \|τ_cmd\| ≥ 0,9·τ_lim e com \|τ_cmd\| > τ_lim | idem | idem |
| 3 | erro de rastreamento | `state.error`: RMS e máximo por junta | idem | idem |
| 4 | torque normalizado | τ / (m·g·L), com L da §2 | derivada | derivada |
| 5 | potência mecânica | P_j = τ_app·q̇: média de \|P\|, trabalho positivo por passo e custo de transporte Σ\|P\| / (m·g·v) | `joint_states` (velocidade) | idem |
| 6 | altura e balanço do CoM | CoM pela soma de massa × posição dos links: altura média, amplitude lateral, frequência do balanço | `walk_telemetry_node` (`com_measured`) | **a adaptar** (abaixo) |
| 7 | atitude do tronco | roll e pitch verdadeiros: RMS e máximo, por fase de apoio | `/gazebo/model_states` | idem |
| 8 | velocidade real × comandada | deslocamento do tronco na janela ÷ tempo, dividido pela velocidade comandada | `walk_test.py` | idem |
| 9 | quedas | \|roll\| ou \|pitch\| verdadeiro > 45° (`FALL_THRESHOLD_DEG` do `walk_test.py`); taxa e tempo até a queda | `walk_test.py` | idem |
| 10 | geometria efetiva | flexão média do joelho em apoio simples, largura real entre os pés, duração medida do passo | juntas, `left/right_foot_contact` | juntas e posição dos pés por `link_states` |

### 4.1 O que reaproveitar, e o que cada script precisaria (sem implementar agora)

**`walk_test.py`:**
- Usa tópicos e nomes de junta fixos (`/marta/...`, lista `JUNTAS`).
- Precisaria de três parâmetros: namespace, lista de juntas e o comando de
  marcha (`/darwin/enable_walking` + `/darwin/cmd_vel` no Darwin).
- A detecção de queda e o filtro de RTF ficam iguais.

**`bag_para_csv.py`:**
- Os mesmos nomes fixos.
- Precisaria de um mapa de nomes Darwin → Marta:

| Darwin | Marta |
|---|---|
| `pelvis` | `hip_yaw` |
| `thigh1` | `hip_roll` |
| `thigh2` | `hip_pitch` |
| `tibia` | `knee` |
| `ankle1` | `ank_pitch` |
| `ankle2` | `ank_roll` |
| `shoulder` | `sho_pitch` |
| `high_arm` | `sho_roll` |
| `low_arm` | `el` |
| `pan`, `tilt` | `head_pan`, `head_tilt` |

  Esse mapa já foi conferido por cinemática nesta etapa.
- **[I] Atenção aos sinais.** Os eixos das juntas diferem entre os modelos. As
  métricas usam \|τ\|, mas qualquer métrica com sinal precisa de uma tabela de
  sinais.

**`walk_telemetry_node`:**
- Tem parâmetro `model_name`, mas os links `base_link`, `l_feet_1` e
  `r_feet_1` estão fixos no código.
- Precisaria de parâmetros para os nomes do tronco e dos pés (no Darwin:
  `MP_BODY`, `MP_ANKLE2_L/R`) e para o deslocamento até a sola.
- **[I]** Com a marcha da ROBOTIS não existe `/marta/walk/reference`. Os campos
  do LIPM (energia orbital, DCM) ficam vazios no Darwin, mas CoM, contatos e
  yaw verdadeiro funcionam.
- Com o **nosso** gerador no Darwin (E4), tudo funciona.

**Figuras:** `figuras_marcha.py`, com `LIMITE_TORQUE` = τ_lim da variante.

---

## 5. Hipóteses concorrentes

Para cada hipótese: o enunciado, a predição que a distingue das outras, o
experimento e o custo.

| | hipótese | predição | experimento | custo |
|---|---|---|---|---|
| **(0)** | **O limiar de 6 N·m mudou com o pé em caixa** | com o pé em caixa, a taxa de quedas a 5 e a 4 N·m é diferente da medida com malha (6/8 e 5/5) | **E0** | baixo: só corridas |
| **(a1)** | **massa total** | o torque dos joelhos e quadris escala com m: Darwin com massa ×1,635 pede ~1,6× | **E5a** (Darwin com massas ×1,635) | médio |
| **(a2)** | **distribuição de massa** (tronco pesado, 62%) | com a massa total igual, pôr o excesso no tronco aumenta o torque de joelho e quadril em apoio simples **mais** que distribuí-lo; ou seja, E5b > E5a | **E5b** (Darwin com +1,86 kg no tronco) | médio |
| **(b)** | **geometria e postura**: joelho mais dobrado e braço de alavanca maior | a flexão do joelho em apoio simples da Marta é maior que a do Darwin; com o mesmo gerador, reduzir a flexão (mais `zCom`) reduz o torque do joelho de forma monótona | **E0c** (mede flexão nos bags existentes) → **E1c** (varre `zCom`) | E0c zero; E1c baixo |
| **(b')** | **IK × URDF divergentes** (98,2/97,4 contra 101/100 mm; eixos do quadril desalinhados) | a pose real dos pés difere da planejada de forma sistemática; corrigir os comprimentos muda a flexão e o torque do joelho | **E0c** (planejado × medido nos bags) | zero |
| **(c)** | **parâmetros de marcha**: os nossos (passo 0,4 s, apoio duplo 0,5, `zStep` 0,06, `ySeparation` 0,08) contra os do Maximo | com os parâmetros do Maximo (em grandezas medidas equivalentes), o torque e a taxa de quedas a 5 e 4 N·m caem | **E1** (Marta com o conjunto do Maximo) | **baixo: só parâmetros do nosso gerador** |
| **(d)** | **modelo de servo**: p = 50, sem amortecimento e sem inércia refletida, contra o modelo da §3 | no S0, a Marta e o Darwin mudam de limiar na **mesma direção**; a razão de torque Marta/Darwin fica estável entre S-atual e S0 se o servo não for a causa | **E2 + E3** | médio |
| **(e)** | **limite de effort ≠ stall** (sem a curva torque × velocidade) | com B (S0), o torque disponível cai com a velocidade da junta: as juntas rápidas (joelho no balanço) saturam mais | **E3**, métrica 2 por fase | junto com E3 |
| **(f)** | **contato do pé** (sola maior, μ = 1e10, kd = 1e4) | trocar a sola e o contato da Marta pelos do Darwin não muda o limiar; o achado D já sugere que o atrito não importa | **E7** | baixo |
| **(g)** | **o gerador**, e não o robô: o OmnidirectionalWalkZMP pede mais do que a marcha da ROBOTIS | o Darwin com o **nosso** gerador pede mais torque que com a marcha da ROBOTIS, com o mesmo servo | **E4** | **alto: porte da IK** |

### 5.1 Comentários de projeto

**Sobre (c): não confundir o nome do parâmetro com o que ele faz.**
- O conjunto do Maximo-D (Darwin) é: passo de 0,5 s, h = 0,20 m no corpo
  inteiro, fds = 0,2, ysep = 0,1 (ou 0,06) e v = 0,1–0,2 m/s.
- O conjunto do Maximo-M é T = 0,2 s, zCoM = 0,23 m, zstep = 0,04 m,
  rds = 0,2, ysep = 0,04 m e s = 0,8 [F] (p. 85). **É do Bioloid** (§1.3), um
  robô menor e com servo mais fraco.
- **[I] O zCoM do mestrado não pode ser aplicado literalmente na Marta.** Com a
  IK atual, `zCom = 0,23` deixaria 0,23 − 0,0595 − 0,133 ≈ 0,037 m de quadril
  a tornozelo, para uma perna de 0,196 m. A perna dobraria quase por inteiro.
  O E1a usa só T, rds, zstep e ysep, com a altura vinda do E1c.
- **Duas versões do E1:**
  - **E1a, literal:** só os parâmetros com o mesmo significado no nosso
    gerador. São a duração do passo, a fração de apoio duplo e a altura do pé,
    depois de conferir que `zStep` e `zstep` são a mesma coisa.
  - **E1b, por semelhança dinâmica:** escalar por L_Marta / L_Darwin ≈ 1,08 as
    distâncias e por √(L ratio) os tempos (número de Froude). **[I]** É a
    maneira justa de transferir parâmetros de um robô para outro.
- O `zCom` da Marta **não** recebe o "zCoM" do Maximo diretamente (achado G).
  Ele é ajustado para que a **altura medida** do CoM fique na proporção
  h/L do Darwin (0,20 / 0,2195 = 0,91).

**Sobre (d) e (e): dois efeitos possíveis.**
- **[I]** Passar para o S0 pode **piorar** a Marta: rigidez quatro vezes menor
  (12,7 contra 50), menos torque disponível em velocidade.
- **[I]** Também pode **melhorar**: o amortecimento implícito estabiliza.
- O experimento decide. O resultado que interessa não é o valor absoluto, e
  sim a **razão Marta/Darwin no mesmo servo**.

**Sobre (g):** é o experimento que isola o robô de verdade (o mesmo gerador, o
mesmo servo, robôs diferentes). É também o mais caro (§6, E4).

---

## 6. Experimentos

**Notação:**
- τ = limite de effort das juntas;
- n = corridas por condição;
- **todas as corridas** duram 30 s de marcha a 0,1 m/s (salvo quando dito o
  contrário), com o pé em caixa e a verdade de solo.

| id | robô | o que muda | condições | n | precisa implementar |
|---|---|---|---|---|---|
| **E0** | Marta | nada; nova linha de base com a caixa | τ ∈ {4, 5, 6, 1000}, estabilizador ligado e desligado | 8 | nada |
| **E0b** | Marta | nada; métricas da §4 em E0 | — | — | só análise (CSV) |
| **E0c** | Marta | nada; flexão do joelho e pose planejada × medida nos bags | bags de E0 e de `experiments/yaw/dados/` | — | só análise |
| **E1a / E1b** | Marta | parâmetros do Maximo, literal e escalado | τ ∈ {4, 5, 6} | 8 | só parâmetros (rosparam / launch) |
| **E1c** | Marta | varredura de `zCom` (flexão do joelho) | 3 a 4 níveis × τ = 6 | 5 | só parâmetro |
| **E2** | Darwin | modelo de servo S0 e varredura do limite | S0 com τ ∈ {2,31; 2,5; 3; 4; 6}; S-atual | 8 | opção de limite no `fix_inertia.py`; `gains.yaml` S0 |
| **E2J** | Darwin | S0+J (inércia refletida) | τ = 2,5 | 5 | idem + inércia no corpo do rotor |
| **E3** | Marta | modelo de servo S0 | primeiro **estabilidade parado** com amortecimento implícito; depois τ ∈ {2,5; 4; 6; 1000} | 8 | `implicitSpringDamper` no `marta.gazebo`; ganhos S0 |
| **E4** | Darwin | **nosso gerador** no Darwin | servo S0; parâmetros do Maximo-D (passo 0,5 s, h = 0,20 m); τ ∈ {2,5; 4; 6} | 8 | porte do `controller_node` (§6.1) |
| **E5a / E5b** | Darwin | massa da Marta, uniforme e no tronco | marcha da ROBOTIS e S0; τ ∈ {2,5; 4; 6} | 8 | variantes de URDF (`fix_inertia.py`) |
| **E7** | Marta | sola e contato do Darwin | τ = 6 e τ = 5 | 8 | args do xacro (já existem: `foot_mu`, `contact_kp/kd`; a sola precisa de arg) |

### 6.1 E4: o nosso gerador no Darwin

O que o porte envolve [I], sem implementar agora:
- comprimentos da IK do Darwin: coxa 0,093, canela 0,093, tornozelo 0,0335 m;
  quadril com eixos coincidentes;
- mapa de nomes e **sinais** das juntas (tabela da §4.1), conferido por
  cinemática direta, como fizemos com a marcha da ROBOTIS;
- `hip_yaw_sign` de acordo com o eixo da junta no Darwin (achado L);
- a telemetria ganha os links do Darwin (§4.1).

**Ordem de validação:**
1. pose estática;
2. marcha no lugar;
3. só depois, marcha para frente.

**[I] Riscos:**
- O `controller_node` presume a agachada `Agacha()` da Marta, com cerca de
  10 s de rampa.
- O estabilizador de pitch tem ganhos sintonizados para a Marta. No E4 ele
  roda **desligado**, para comparar com a marcha da ROBOTIS, que só tem o
  balanço por giroscópio.

### 6.2 Ordem de execução (do mais barato e informativo ao mais caro)

1. **E0 + E0b + E0c.** Refazem o problema nas condições atuais e respondem
   (0), (b) e (b') **sem uma linha de código**. Cerca de 64 corridas, mais
   análise dos bags existentes. **Comece aqui.** O E0c já foi feito em parte
   (§2.1).
2. **E1c, redesenhado depois dos pilotos (§2.2): fatorial postura × altura
   do pé**, **feito (§2.3)**. Ele mostrou que o gargalo é a perna em balanço,
   e isso reordena o que vem a seguir: E3 e tempo de balanço primeiro.
   - **Predição original [I], refutada no piloto para a perna em balanço:**
     "o τ_cmd do joelho cai de forma monótona com a altura".
   - **Risco que continua valendo:** o `tZMP` muda junto (achado G).
3. **E1a / E1b.** Parâmetros do Maximo. É a suspeita do usuário e testa (c).
   Rodar **já com a melhor altura do E1c**, para não confundir parâmetros com
   postura. Cerca de 60 corridas.
4. **E2.** Dá a **referência**: o Darwin anda com o MX-28 de verdade? Sem isso,
   "6 N·m é muito" não tem com o que comparar. Precisa de duas mudanças
   pequenas, fora do repositório da Marta.
5. **E3.** A Marta no mesmo servo. Primeiro checar a estabilidade com
   amortecimento implícito; se não ficar estável, testar S0+J antes.
6. **E5a / E5b.** Massa, no Darwin.
7. **E7.** Contato, por desencargo (o achado D já sugere que não importa).
8. **E4.** O mais caro e o mais conclusivo sobre "robô × gerador". Só faz
   sentido depois de E2 e E3.

**Regras em todas as etapas** [F] (regras do projeto):
- condições **intercaladas** (r1 de cada condição, depois r2…);
- **uma simulação por vez**;
- RTF registrado por corrida e corridas com RTF < 0,85 descartadas e repostas.

### 6.3 Estatística e critérios de decisão

**Testes:**

| tipo de resposta | teste |
|---|---|
| quedas (binária) | Fisher exato |
| tempo até a queda | log-rank, como em resultados_campanha.md |
| métricas contínuas | Welch t, ou Mann-Whitney se a distribuição for assimétrica; d de Cohen como tamanho de efeito |

Com n = 8, Fisher detecta 0/8 contra 6/8 (p ≈ 0,007) e não detecta 2/8 contra
4/8. **[I]** Diferenças sutis de taxa de queda exigem n maior.

**Critérios por hipótese** (definidos **antes** de rodar):

| hipótese | aceita se |
|---|---|
| (0) | a taxa de quedas com caixa a 5 N·m diferir de 6/8 com p < 0,05 |
| (c) | o E1 reduzir o τ_cmd RMS do joelho em ≥ 20% (p < 0,05) **ou** reduzir as quedas a 5 N·m (p < 0,05) |
| (b) | E0c mostrar flexão do joelho da Marta ≥ 10° maior que a do Darwin (**primeira parte já atendida: 109° × 55°, §2.1**) **e** E1c mostrar torque monótono com a flexão |
| (d) | a razão de torque do joelho Marta/Darwin mudar > 20% entre S-atual e S0 |
| (a1) / (a2) | o torque do joelho do Darwin em E5a/E5b subir em proporção ≥ 0,8 da razão de massa (a1); e E5b > E5a com p < 0,05 (a2) |
| (g) | o τ_cmd RMS do joelho do Darwin com o nosso gerador for ≥ 20% maior que com a marcha da ROBOTIS, no mesmo servo |

**Leitura final:** decompor o fator Marta/Darwin (torque normalizado do joelho
em S0) em contribuições de massa, geometria, parâmetros e gerador.

**[I]** A decomposição é aproximada, porque os efeitos interagem. Reportá-la
como ordem de grandeza, e não como soma exata.

---

## 7. O que o usuário terá ao fim

**Tabelas:**
1. A tabela da §2 com todos os **[?]** resolvidos (CoM medido, largura entre os
   pés, flexão do joelho, duração do passo).
2. Por condição: as métricas 1 a 10 da §4, com média ± desvio e n; o RTF de
   cada corrida.
3. Taxa de quedas × τ, por robô e por variante de servo, com os p-valores.
4. Para cada hipótese: aceita, rejeitada ou inconclusiva, com o número que
   decidiu.

**Figuras:**
1. **Curva de limiar:** taxa de quedas × τ, Marta e Darwin no S0, no mesmo
   gráfico; τ também normalizado por m·g·L.
2. Barras do torque por junta (RMS e p99), Marta × Darwin, normalizado.
3. Mapa de calor da fração do tempo perto do stall, junta × fase de apoio.
4. Flexão do joelho × torque do joelho (E0c, E1c, E2).
5. Decomposição do fator Marta/Darwin (§6.3).
6. Para o E4: torque do joelho do Darwin com a marcha da ROBOTIS × o nosso
   gerador.

---

## 8. Pendências antes de começar

1. **Resolvido:** a Tabela 5.1 do mestrado foi conferida; ela é do Bioloid
   (§1.3).
2. **Resolvido:** a altura do CoM depende do limite de torque (§2, ponto 5).
   O README e a telemetria_marcha.md passaram a dizer a condição de cada
   número.
3. **Resolvido:** o README §7, item 3 foi corrigido (Darwin-OP/OP2, MX-28,
   +1,86 kg).
4. **A origem do achado D** ("amortecimento de junta instável"): qual teste, com
   que valor, se era explícito. Ninguém documentou; o E3 refaz.
5. **Resolvido:** a IK **não** modela o desvio entre os eixos de roll e pitch do
   quadril (§2). **Continua [?]:** de onde vem o deslocamento tornozelo → pé
   de 0,133 m da IK (§2.1)? O usuário não sabe, e o mestrado não traz as
   dimensões do Bioloid. **[I]** Uma forma de investigar é o histórico do
   `RobotPhysicalParameters.cpp` no git do submódulo `marta_control`.
6. **[?] Confirmar** `worlds/empty.world` da Marta com
   `get_physics_properties` (§2).

---

## 9. Fontes

**Teses do Maximo:**
- Maximo-D (PDF do usuário), p. 51, 54, 56–65, 73–74, 78–81, 84–85, 94–98,
  113–115, 152.
- Maximo-M (PDF do usuário), p. 50, 72, 85, 102, 111 e 115–118.

**e-Manual da ROBOTIS:**
- MX-28: https://emanual.robotis.com/docs/en/dxl/mx/mx-28/
- Darwin-OP: https://emanual.robotis.com/docs/en/platform/op/getting_started/
- OP2: https://emanual.robotis.com/docs/en/platform/op2/getting_started/
- OP3: https://emanual.robotis.com/docs/en/platform/op3/introduction/

**Código e modelos:**
- Walking da ROBOTIS (via Webots):
  https://github.com/cyberbotics/webots/blob/master/projects/robots/robotis/darwin-op/libraries/robotis-op2/robotis/Framework/src/motion/modules/Walking.cpp
- PROTO do Darwin-OP com massas do CAD:
  https://github.com/cyberbotics/webots/blob/master/projects/robots/robotis/darwin-op/protos/Darwin-op.proto
- Simulação do Darwin usada aqui: `~/Documents/ic/op2_ws`, com o README do
  pacote `op2_sim_bringup`.
- gazebo_ros_control (semântica do effort):
  https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_ros_control/src/default_robot_hw_sim.cpp
- effort_controllers (`state.command` antes da saturação):
  https://github.com/ros-controls/ros_controllers/blob/noetic-devel/effort_controllers/src/joint_position_controller.cpp
