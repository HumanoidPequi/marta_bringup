# Marta — estabilização de marcha: mapa do projeto

**Este é o documento central.** Se você é um agente entrando agora no projeto,
leia este arquivo inteiro antes de qualquer outra coisa; ele dá o contexto e
aponta para os dois documentos de detalhe.

| documento | conteúdo |
|---|---|
| **este arquivo** | contexto, objetivos, estado, achados, como rodar |
| [estabilizadores.md](estabilizadores.md) | o desenvolvimento dos controladores: teoria, implementação, resultados |
| [telemetria_marcha.md](telemetria_marcha.md) | a camada de medição: tópicos, mensagens, pipeline, experimento |
| [como_ler_as_figuras.md](como_ler_as_figuras.md) | interpretação de cada figura gerada |
| [resultados_campanha.md](resultados_campanha.md) | a campanha de 78 corridas do estabilizador de pitch |
| [desvio_de_direcao.md](desvio_de_direcao.md) | a investigação do giro em yaw: instrumentação, hipóteses, resultados |
| [../src/marta_bringup/docs/estabilizador.md](../src/marta_bringup/docs/estabilizador.md) | runbook operacional (comandos do dia a dia) |

---

## 1. O sistema

**Marta**, humanoide KidSize da equipe HumanoidPequi (UFG). Simulação em
Gazebo classic sob ROS 1 Noetic, dentro da imagem docker `marta_bringup:1.0`.

| | |
|---|---|
| massa total | 4,781 kg (21 links dinâmicos) |
| juntas | 20, todas `EffortJointInterface` + `effort_controllers/JointPositionController` |
| altura do tronco em pé | ~0,27 m |
| altura real do CoM | **0,2134 ± 0,0034 m** acima do contato |
| laço de controle | 100 Hz (`controller_node`) |
| física do Gazebo | 1 kHz |

Workspace catkin com submódulos: `marta_control` (controle), `marta_description`
(URDF), `marta_gazebo` (mundo e launch), `marta_msgs` (mensagens),
`marta_bringup` (launch, scripts, experimentos), `ball_tracking` (visão).

### Como a marcha funciona hoje

`OmnidirectionalWalkZMP` gera a trajetória em **malha aberta**: LIPM analítico
com ZMP pré-planejado para o CoM, splines trigonométricas (*raised cosine*) para
os pés, cinemática inversa geométrica fechada. Parâmetros:
`period = 0,4 s`, `doubleSupportRatio = 0,5`, `zCom = 0,297509`,
`zStep = 0,06`, `ySeparation = 0,08`, `comSwingFactor = 1,0`.

Sobre isso, uma camada de **realimentação** (o que este projeto construiu)
corrige a atitude do tronco somando um desvio angular após a IK.

---

## 2. Referências

As três primeiras são a base teórica; a quarta é diretamente relevante para a
questão em aberto do desvio de direção.

1. **Kajita, S. et al., "The 3D Linear Inverted Pendulum Mode: A simple modeling
   for a biped walking pattern generation", IROS 2001.**
   É o modelo que o gerador implementa. Dele vêm as equações usadas como métrica:
   - eq. (20)(21): `ÿ = (g/z_c)·y`, `ẍ = (g/z_c)·x`
   - eq. (28)(29): energia orbital `E = −(g/2z_c)·x² + ½·ẋ²`, **invariante no tempo**
   - eq. (40): órbita hiperbólica `ẋ² − (g/z_c)·x² = 2E`
   - eq. (41): transição de passo com `C_T = cosh(Ts/Tc)`, `Tc = sqrt(z_c/g)`
   - figuras 4 e 5: as que reproduzimos com dados medidos sobrepostos
   - **§5, p. 245**: o artigo relata que o robô *não fechou o círculo planejado*
     por escorregamento do pé, inclusive **em torno do eixo z**, "which diverted
     the robot from the planned walking direction" (μ = 0,5). É o precedente
     publicado do problema que queremos investigar.

2. **Kajita, S. et al., *Introduction to Humanoid Robotics*, §4.5 / 4.5.1.**
   A separação que organiza a arquitetura: gerador de marcha em malha aberta
   produz a referência nominal; malhas rápidas de realimentação corrigem o
   desvio. É por isso que o estabilizador é camada separada, e não está dentro
   do gerador.

3. **Raibert, M., *Legged Robots That Balance*, cap. 2 (2D) e 3 (3D).**
   Controle de atitude do corpo pela perna de apoio:
   `τ = −k_p(φ − φ_d) − k_v·φ̇`. Em 3D, pitch e roll como malhas independentes —
   motivo da divisão em fases deste projeto.

4. **Fujimoto, Y. e Kawamura, A., "Robust Control of Biped Walking Robot with
   Yaw Moment Compensation by Arm Motion", ASCC'97, vol. 3, pp. 327–330.**
   Citada por Kajita 2001 como a solução proposta para justamente a divergência
   de direção. Ainda **não foi estudada** por nós.

---

## 3. Objetivos traçados

O plano original tinha quatro fases, definidas com o usuário:

| fase | objetivo | estado |
|---|---|---|
| **1** | Tratamento de sensores e leitura de pitch do tronco | **concluída** |
| **2** | Estabilizador de pitch do tronco (quadril sagital) | **concluída** |
| **3** | Estabilizador de roll do tronco (quadril lateral) | **não iniciada** |
| **4** | Ajuste fino na marcha completa e transição de pernas | parcial |

Objetivo transversal que surgiu depois e foi cumprido: **metrificar** o efeito
do controlador de forma defensável, em vez de avaliar visualmente.

**Questão do desvio de direção — PARCIALMENTE resolvida (2026-10-07):** o giro
grande para a esquerda (~+146° em 30 s) vinha da colisão do pé em malha STL
(achado **I**) e sumiu com o pé em caixa. **Mas o robô ainda faz curva para a
DIREITA:** no regime de operação (τ = 6, estabilizador ligado), −48 ± 15° em 30 s,
em 10/10 corridas, ~−28° deles nos primeiros 5 s. O teste do espelho mostrou que ela vem da
**iniciação da marcha**: partindo pelo outro pé (`start_left_swing:=true`), a
curva troca de lado (+57° × −32°, p = 0,001). Uma partida suave
(`start_ramp_steps`) remove esse viés, **mas o robô continua curvando ~50° em
30 s, agora para um lado sorteado**. O que resta é deriva própria de cada
corrida, e a **malha de rumo** (ligada por default no `stabilizer_test.launch`
desde 2026-10-08; `heading_enabled:=false` desliga) a corrige: desvio
lateral de −0,69 ± 0,62 m para +0,03 ± 0,15 m em 30 s (p = 2e−4), ao custo de
+25% no pitch RMS. Ver [desvio_de_direcao.md](desvio_de_direcao.md).

---

## 4. Achados que todo agente precisa conhecer

Estes custaram caro e evitam repetir erros.

**A.** `real_to_sim_topics.py` cria um `rospy.Publisher` dentro de cada callback.
Sob carga isso derruba o **RTF do Gazebo pela metade** (0,526 contra 0,999) e o
robô cai sozinho. A causa original era `isReal` lido do namespace errado, com
default `true`. **Em simulação, `isReal` tem que ser `false`.** Ver a nota de
memória do projeto.

**B.** **A métrica não pode ser o sinal que o controlador regula.** Avaliar o
estabilizador pelo pitch *estimado* superestima o ganho, porque a malha empurra
justamente essa estimativa para zero. Todas as métricas de atitude usam **verdade
de solo do Gazebo**. A estimativa fica ao lado, como diagnóstico do estimador.

**C.** O **viés do estimador** de atitude cresce com a dificuldade: ~1° no regime
fácil, **+21° com torque saturado**, a ponto de cruzar o limiar de queda com o
robô de pé. O portão de confiança testa só a *norma* da aceleração e é cego à
*direção*; com `α = 0,998` (τ ≈ 5 s) um erro sustentado leva segundos para ser
lavado. **Correção pendente: peso adaptativo no acelerômetro.**

**D.** **Torque é o único gargalo real** dos quatro fatores de realismo testados.
Atrito do pé não é (nenhuma queda de μ=1e10 até 0,3); damping de junta é
numericamente instável no ODE e inutilizável. O regime informativo é
`joint_effort:=6` N·m — degradado mas de pé, e realista para servo KidSize.

**E.** **Os joelhos são o gargalo mecânico**: pedem ~22 N·m contra 6 disponíveis,
com ~26° de desvio de rastreamento.

**F.** As quedas reais são **sagitais e para trás**: o pitch cruza 20° em ~1 s e
chega a −45/−55°, o roll só reage depois. O roll é o *maior resíduo* em regime
(6,4° contra 3,2° de pitch), mas **não é o modo de falha**.

**G.** `zCom = 0,2975` **não é a altura do CoM** (0,2134 m): 36% de erro em
`g/z_c`, o único parâmetro do LIPM. Mas corrigir só o `comSwingFactor` **piora**
a aderência ao modelo (+15%, p=0,043) — os parâmetros estão co-sintonizados.
`comSwingFactor` permanece em 1,0.

**H.** Corridas com **RTF < 0,85 são inválidas** (limitadas por CPU, não por
física) e o filtro é obrigatório nas análises. Numa campanha, as 4 corridas com
RTF entre 0,52 e 0,66 foram as únicas 4 que caíram; as 12 acima de 0,85 não
tiveram nenhuma. Um teste controlado mostrou que a telemetria **não** causa isso
(RTF 0,995 com ela, 0,998 sem).

**I.** **O giro em yaw era artefato da colisão do pé em malha.** Com o pé em
malha STL, o pé de apoio gira 1–2° no chão a cada passo e o robô deriva ~4 °/s
para a esquerda (17/17 corridas), com qualquer torque. O modelo é simétrico; a
assimetria está na resolução numérica do contato. Com `foot_collision:=box`
(mesma sola), o pé para de girar, a deriva cai para ~1 °/s sem lado preferido e
o avanço em 30 s vai de 1,05 m para 2,47 m (τ = 1000). Com τ = 6 a deriva em
regime fica indistinguível de zero, e sobra um giro de ~25° para a direita nos
primeiros 5 s. O estabilizador de pitch **se mantém** com a caixa (pitch RMS real
6,43° → 2,28°, −65%, p = 2e−4). **Desde 2026-10-07 o padrão é
`box`. Todas as campanhas anteriores (incluindo as 78 corridas de pitch e a de
`comSwingFactor`) rodaram com `mesh`.** As conclusões delas são de pitch e
torque e provavelmente se mantêm, mas números de distância e de quedas não são
comparáveis com os novos sem uma nova linha de base.

**J.** `restart()` do gerador comandava **17° de yaw nas hip_yaw no primeiro
passo** de toda marcha (`torsoBegin.z = 0.297509`, o valor de `zCom` no campo de
yaw). Corrigido e travado por `marta_control/test/test_walk_symmetry.cpp`.

**L.** **O sinal do yaw na IK estava invertido para a simulação.** As
`hip_yaw` do URDF têm eixo −z, mas a IK fazia `hipYaw = theta`: pedir rotação
para a esquerda girava o robô para a direita. Ninguém via porque a marcha nunca
era comandada a girar. Corrigido por parâmetro `hip_yaw_sign` (default +1; o
`stabilizer_test.launch` passa −1). **No robô real, verificar o sentido dos
servos antes de comandar rotação.** No mesmo trabalho, o gerador tratava
rotações negativas como linha reta (`velocity.z < DBL_EPSILON`), também
corrigido.

**K.** **Taxas angulares a 100 Hz não são integráveis** no impacto do pé
(aliasing dos picos de ±190°/s da física de 1 kHz). Para giro acumulado, use
sempre diferença de ângulo.

---

## 5. Como rodar

Sempre pela imagem docker. Os comandos completos do dia a dia estão no
[runbook](../src/marta_bringup/docs/estabilizador.md); o essencial:

```bash
cd ~/Documents/ic/marta_bringup

# compilar (o run.sh monta so src/, entao a imagem pode estar dessincronizada)
docker run --rm -v "$PWD":/ws -w /ws marta_bringup:1.0 \
  bash -lc 'source /opt/ros/noetic/setup.bash && catkin_make'

# terminal 1: simulacao (sobe PAUSADA de proposito)
xhost +local:root
docker run --rm -it --name marta_stab --network host --gpus all --device /dev/dri \
  -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 \
  -e __GLX_VENDOR_LIBRARY_NAME=nvidia -e __NV_PRIME_RENDER_OFFLOAD=1 \
  -v "$PWD":/ws -v /tmp/.X11-unix:/tmp/.X11-unix -w /ws marta_bringup:1.0 \
  bash -c 'source /opt/ros/noetic/setup.bash && source devel/setup.bash && \
           roslaunch marta_bringup stabilizer_test.launch gui:=false joint_effort:=6.0'

# terminal 2: despausa, anda, mede e grava bag
docker exec -it marta_stab bash -c \
  'source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash && \
   rosrun marta_bringup walk_test.py _duration:=30'

# terminal 3 (opcional): PlotJuggler ao vivo
docker exec -it marta_stab -e LIBGL_ALWAYS_SOFTWARE=1 -e MESA_LOADER_DRIVER_OVERRIDE= \
  -e __GLX_VENDOR_LIBRARY_NAME= -e GALLIUM_DRIVER= bash -c \
  'source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash && \
   rosrun plotjuggler plotjuggler'
```

**Armadilhas de operação:**
- A simulação sobe **pausada**; sem despausar nada acontece. O `walk_test.py`
  despausa sozinho.
- O `controller_node` passa **~10 s na rampa do `Agacha()`** antes de entrar no
  laço principal. Nada de telemetria de marcha sai antes disso.
- O PlotJuggler precisa de GL por software no `docker exec` (variáveis acima),
  senão abre em tela preta.
- A imagem força driver NVIDIA por ENV, para o Gazebo. **O notebook tem gráfico
  híbrido** (a tela é da GPU integrada): sem `-e __NV_PRIME_RENDER_OFFLOAD=1`, o
  OpenGL da NVIDIA falha com `X Error ... BadValue (NV-GLX)` e o `gzserver` morre
  (a câmera simulada precisa de GL). **Não contornar com
  `LIBGL_ALWAYS_SOFTWARE=1`:** isso funciona, mas renderiza na CPU (llvmpipe).
  Conferir com `glxinfo -B | grep renderer` dentro do container: tem que
  aparecer a RTX 3050.

### Knobs de realismo (todos com default = modelo original)

`foot_mu`, `joint_effort`, `joint_damping`, `joint_friction`, `contact_kp/kd`,
`imu_gyro_noise`, `imu_accel_noise`, `imu_gyro_bias`, `foot_collision` — passados
ao `stabilizer_test.launch`, que repassa ao xacro. Ver cabeçalho de
`marta_description/urdf/marta.xacro`.

`foot_collision` é a única exceção à regra "default = modelo original": vale
`box` desde 2026-10-07 (achado I). Para reproduzir uma campanha antiga, passe
`foot_collision:=mesh`.

---

## 6. Estado do repositório

**Nada está commitado.** As mudanças estão espalhadas por três repositórios
(submódulos `marta_control`, `marta_description` e o raiz `marta_bringup`), e os
submódulos precisam de commit próprio antes do ponteiro na raiz.

### Defeitos conhecidos, não corrigidos

| onde | o quê |
|---|---|
| `marta_gazebo/scripts/real_to_sim_topics.py` | `Publisher` por callback; derruba o RTF quando usada |
| `marta_bringup/include/joint_to_tf_publisher.cpp` | assina `/marta/<membro>/state` enquanto o controlador publica em `/command` — **nó efetivamente morto** |
| `marta_description/urdf/marta.gazebo` | referencia `r_tight_1`/`l_tight_1`; os links são `r_thigh_1`/`l_thigh_1`, então dois blocos de atrito não têm efeito |
| `marta_control/src/torso_attitude_node.cpp` | portão de confiança cego à direção da aceleração (achado C) |

---

## 7. Próximos passos, em ordem de valor

1. **Linha de base com o pé em caixa** (achado I): o ganho do estabilizador já
   foi reconfirmado com τ = 6; faltam as taxas de queda com τ = 5 e τ = 4 e o
   transiente de yaw na partida.
2. **Peso adaptativo no acelerômetro** — destrava usar a estimativa como métrica.
3. **Fase 3, estabilizador de roll** — maior resíduo, mas não é o modo de falha;
   atenção ao fato de a IK manter `ankle_roll = −hip_roll`, de modo que corrigir
   só no quadril inclina o pé lateralmente.
4. **Re-sintonia conjunta** dos parâmetros do gerador (achado G).
5. **Malha de rumo** (ligada por default na simulação, kp = 1, sat = 0,3):
   ganhos maiores derrubam o robô com τ = 6; falta testar kp = 2 com sat = 0,3
   e entender o evento de giro dos primeiros passos — ver §5 de
   [desvio_de_direcao.md](desvio_de_direcao.md).
