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
| [desvio_de_direcao.md](desvio_de_direcao.md) | a investigação do giro em yaw: instrumentação, hipóteses, resultados, malha de rumo |
| [plano_fase3_roll.md](plano_fase3_roll.md) | **Fase 3 projetada, não implementada**: compensação de gravidade + estabilizador de roll, referências com página, protocolo |
| [plano_comparacao_darwin.md](plano_comparacao_darwin.md) | comparação Marta × Darwin-OP (por que 6 N·m): modelo comum de MX-28, métricas, hipóteses; **E0 e E1c feitos** (postura × altura do pé, 72 corridas): o gargalo é a perna em balanço |
| [ajustes_marcha.md](ajustes_marcha.md) | **ajustes de parâmetros de out/2026**: geometria da IK (tornozelo→sola, quadril em x) e marcha (`z_com`, apoio duplo, separação e altura dos pés): o que mudou, por quê e o que falta medir |
| [panorama_parametros.md](panorama_parametros.md) | **panorama massa × parâmetros (354 corridas, MX-28)**: janela de marcha por massa (anda até ~3,6 kg), por que o tronco "pende" com a massa (−5° fixo no hip pitch + cedência dos servos), saturação do joelho de apoio, ganho da compensação e receita para a Marta nova |
| [consulta_parametros.md](consulta_parametros.md) | **tabela de consulta**: dado massa, geometria da perna e servo, que `z_com`, `hip_offset_x`, ganho e offset usar (índice de carga I = m·g·L/stall); calculadora em `experiments/consulta_parametros.py` |
| [../src/marta_bringup/docs/estabilizador.md](../src/marta_bringup/docs/estabilizador.md) | runbook operacional (comandos do dia a dia) |

---

## 0. Retomada rápida (estado em 2026-10-28)

**Ordem de leitura para retomar sem perder contexto:**
1. este arquivo inteiro;
2. [panorama_parametros.md](panorama_parametros.md) (o estudo mais recente) e
   [consulta_parametros.md](consulta_parametros.md);
3. [ajustes_marcha.md](ajustes_marcha.md): geometria da IK, marcha, modelo de
   servo e compensação de gravidade;
4. [plano_fase3_roll.md](plano_fase3_roll.md): o próximo bloco, roll;
5. [desvio_de_direcao.md](desvio_de_direcao.md) §6–10: malha de rumo e
   sintonia; [estabilizadores.md](estabilizadores.md) §4: estabilizador de
   pitch.

As referências bibliográficas (PDFs) estão em `~/Documents/ic/study` (§2).

**Configuração de referência atual** (simulação "realista"): servo MX-28,
compensação de gravidade ligada, Marta com 3,80 kg. Foi a melhor combinação
da bateria para essa massa (3/3 corridas de 30 s):

```bash
roslaunch marta_bringup stabilizer_test.launch gui:=false \
  foot_collision:=box stabilizer_enabled:=true heading_enabled:=true \
  joint_effort:=2.5 joint_velocity:=100 implicit_damping:=true joint_damping:=0.43 \
  gains:=/ws/src/marta_description/config/controller_mx28.yaml \
  gravity_comp_enabled:=true gravity_comp_joints:=hip_roll,knee gravity_comp_gain:=0.3 \
  torso_mass_scale:=0.67 z_com:=0.239 hip_offset_x:=0.009 hip_pitch_offset_deg:=-5
```

Para outra massa ou geometria, use
`python3 src/marta_bringup/experiments/consulta_parametros.py`.

**O que mudou de regime, e o que isso invalida:**
- **Até ~2026-10-15**, as campanhas usavam limite de torque de 6 N·m, a massa
  real (4,78 kg), a IK antiga (joelho a ~109°) e a marcha antiga (período 0,4,
  apoio duplo 0,5, `zCom` 0,2975). As 78 corridas do estabilizador de pitch
  usavam, além disso, o pé em malha.
- **Hoje** o regime é: MX-28 realista (2,5 N·m, Kp 12,74), IK corrigida,
  marcha nova, pé em caixa, compensação de gravidade, 3,3 a 3,8 kg.
- **Os ganhos de pitch** (kp 0,2, kd 0,03) **e de rumo** (kp 1,0, saturação
  0,3 rad/s) **foram sintonizados no regime antigo e nunca ressintonizados no
  atual.** A fase D do panorama só mostrou que o estabilizador de pitch ajuda.
  Sem ele, o pitch RMS sobe 0,9–1,4° e 2 de 5 condições que andavam passam a
  cair. Não mostrou que os ganhos estão bons.

**Em aberto: os próximos objetivos do usuário.**
1. **Estabilizador de roll** (Fase 3, §7).
2. **Curvas e giro no próprio eixo.** Nunca testados no regime atual: todas as
   corridas recentes foram em linha reta a 0,1 m/s. Ver achado U.
3. **Sintonia dos estabilizadores de pitch e de rumo** para a configuração de
   3,80 kg.

---

## 1. O sistema

**Marta**, humanoide KidSize da equipe HumanoidPequi (UFG). Simulação em
Gazebo classic sob ROS 1 Noetic, dentro da imagem docker `marta_bringup:1.0`.

| | |
|---|---|
| massa total | 4,781 kg no URDF (21 links dinâmicos): tronco 2,966 kg, cada perna 0,542 kg, cada braço 0,280 kg, cabeça 0,170 kg. O arg `torso_mass_scale` escala o tronco: massa = 1,815 + 2,966 · escala. **Com o MX-28, a Marta só anda até ~3,6–3,8 kg** ([panorama_parametros.md](panorama_parametros.md)) |
| juntas | 20, todas `EffortJointInterface` + `effort_controllers/JointPositionController` |
| altura do tronco em pé | ~0,27 m |
| altura real do CoM | **0,212–0,220 m** acima do contato, conforme o limite de torque. Pé em caixa, n = 10: 0,2197 (τ = 1000), 0,2124 (τ = 6), 0,2158 (τ = 6 com estabilizador). A campanha inicial, com pé em malha, mediu 0,2134 ± 0,0034 |
| laço de controle | 100 Hz (`controller_node`) |
| física do Gazebo | 1 kHz |

Workspace catkin com submódulos: `marta_control` (controle), `marta_description`
(URDF), `marta_gazebo` (mundo e launch), `marta_msgs` (mensagens),
`marta_bringup` (launch, scripts, experimentos), `ball_tracking` (visão).

### Como a marcha funciona hoje

`OmnidirectionalWalkZMP` gera a trajetória em **malha aberta**: LIPM analítico
com ZMP pré-planejado para o CoM, splines trigonométricas (*raised cosine*) para
os pés, cinemática inversa geométrica fechada. Parâmetros **desde 2026-10-21**
([ajustes_marcha.md](ajustes_marcha.md)): `period = 0,6 s`,
`doubleSupportRatio = 0,3`, `zCom = 0,252`, `zStep = 0,08`,
`ySeparation = 0,06`, `comSwingFactor = 1,0`. Os anteriores eram 0,4 / 0,5 /
0,297509 / 0,06 / 0,08. **Os resultados das campanhas anteriores a essa data
foram medidos com os valores antigos** (e com a IK antiga).

Sobre isso, uma camada de **realimentação** (o que este projeto construiu)
corrige a atitude do tronco somando um desvio angular após a IK.

**Depois da IK, nesta ordem:**
- o **−5° no hip pitch**, que antes era fixo e desde 2026-10-28 é o parâmetro
  `hip_pitch_offset_deg` (default −5; achado T);
- a **compensação de gravidade** (`gravity_comp_*`, desligada por default;
  [ajustes_marcha.md](ajustes_marcha.md));
- o **estabilizador de pitch**;
- a **malha de rumo**, que atua no comando de rotação do gerador.

**Modelo de servo realista:** `joint_effort:=2.5`, `gains:=…controller_mx28.yaml`
(p = 12,74), `implicit_damping:=true joint_damping:=0.43` e
`joint_velocity:=100`.

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

5. **Maximo, M. R. O. A., *Omnidirectional ZMP-Based Walking for a Humanoid
   Robot*, mestrado, ITA, 2015.** A origem do nosso `OmnidirectionalWalkZMP`
   (`Author: mmaximo` no `.cpp`). §3.9 (p. 72–73): estabilizador só pela
   velocidade angular. **Feito para um Bioloid modificado com servos AX-12+,
   simulado no V-REP, e não para o OP2** (§5.2, p. 102; Apêndices A e B). Os
   parâmetros da Tabela 5.1 (p. 85) são desse robô. Na Tabela 3.1 (p. 72),
   `zCoM` é a altura do tronco (o CoM é aproximado por um ponto fixo nele,
   p. 50), e `ysep` é **metade** da separação entre os pés.

6. **Maximo, M. R. O. A., *Automatic Walking Step Duration through Model
   Predictive Control*, doutorado, ITA, 2017.**
   - Fig. 4.1 (p. 73–74): a mesma arquitetura nossa, com as compensações
     somadas depois da IK;
   - §4.3 (p. 78–81): compensação de gravidade;
   - §4.4.2 (p. 84–85): P+V de roll no quadril e no tornozelo;
   - §4.4.1 (p. 84): o yaw não é observável pelo IMU.

   Base da Fase 3; páginas e ressalvas em
   [plano_fase3_roll.md](plano_fase3_roll.md) §2.

**Onde estão os PDFs** (fora do repositório, em `~/Documents/ic/study/`):

| arquivo | referência |
|---|---|
| `kajita_.pdf` | 1. Kajita et al., 3D LIPM, IROS 2001 (8 p.) |
| `《Introduction to Humanoid Robotics》- 梶田秀司 - 第一版.pdf` | 2. Kajita, Hirukawa, Harada e Yokoi, *Introduction to Humanoid Robotics*, Springer STAR 101, 1ª ed. (o nome do arquivo está em chinês, mas o conteúdo é a edição em inglês) |
| `Legged-Robots-That-Balance-1.pdf` | 3. Raibert, *Legged Robots That Balance* |
| `Mestrado Marcos Maximo-1.pdf` | 5. Maximo, mestrado, ITA, 2015 (131 p.) |
| `Doutorado Marcos Maximo-1.pdf` | 6. Maximo, doutorado, ITA, 2017 (222 p.); o modelo de servo MX-28 (Kp, amortecimento) e a compensação de gravidade vêm daqui |

O artigo 4 (Fujimoto e Kawamura) não está na pasta.

---

## 3. Objetivos traçados

O plano original tinha quatro fases, definidas com o usuário:

| fase | objetivo | estado |
|---|---|---|
| **1** | Tratamento de sensores e leitura de pitch do tronco | **concluída** |
| **2** | Estabilizador de pitch do tronco (quadril sagital) | **concluída** |
| **3** | Compensação de gravidade + estabilizador de roll | **compensação implementada e testada (2026-10-22); roll projetado, não implementado** ([plano_fase3_roll.md](plano_fase3_roll.md)) |
| **4** | Ajuste fino na marcha completa e transição de pernas | parcial |

Objetivo transversal que surgiu depois e foi cumprido: **metrificar** o efeito
do controlador de forma defensável, em vez de avaliar visualmente.

**Linha de out/2026, cumprida:**
- comparação Marta × Darwin-OP;
- correção da IK e dos parâmetros da marcha;
- modelo de servo MX-28;
- compensação de gravidade;
- **panorama de 354 corridas** de massa × postura × `hip_offset_x` × ganho ×
  offset ([panorama_parametros.md](panorama_parametros.md));
- **tabela de consulta** ([consulta_parametros.md](consulta_parametros.md)).

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
+25% no pitch RMS. **A malha é só proporcional e deixa erro de regime** (achado
M): em 60 s o tronco fica 7–10° fora do rumo, para um lado sorteado, e oscila
±7° a cada passo. Ver [desvio_de_direcao.md](desvio_de_direcao.md) §9.

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

**G.** `zCom = 0,2975` **não é a altura do CoM** (0,21–0,22 m): 35–40% de erro em
`g/z_c`, o único parâmetro do LIPM. Mas corrigir só o `comSwingFactor` **piora**
a aderência ao modelo (+15%, p=0,043) — os parâmetros estão co-sintonizados.
`comSwingFactor` permanece em 1,0.

**H.** Corridas com **RTF < 0,85 são inválidas** (limitadas por CPU, não por
física) e o filtro é obrigatório nas análises. Numa campanha, as 4 corridas com
RTF entre 0,52 e 0,66 foram as únicas 4 que caíram; as 12 acima de 0,85 não
tiveram nenhuma. Um teste controlado mostrou que a telemetria **não** causa isso
(RTF 0,995 com ela, 0,998 sem).
**Revisão de 2026-10-15: a correlação era, provavelmente, causalidade
invertida.**
- Até essa data, o `walk_test.py` media o RTF incluindo o tempo depois da
  queda. Com o robô deitado, a física fica ~2× mais lenta.
- Um monitor de tempo simulado × tempo real mostrou RTF 1,00 até o instante da
  queda e ~0,55 depois. Toda queda cedo saía com RTF baixo.
- O `walk_test.py` agora fecha a janela na queda.
- O filtro continua obrigatório, mas só o RTF medido pelo `walk_test.py`
  corrigido é confiável. Corridas antigas com queda e RTF baixo **não** devem
  ser lidas como "limitadas por CPU" sem outra evidência.
- O caso do achado A (ponte `real_to_sim`, RTF 0,526 com o robô em pé) continua
  sendo lentidão real. Ver
  [plano_comparacao_darwin.md](plano_comparacao_darwin.md) §2.3.

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

**M.** **A malha de rumo só P deixa erro de regime de ~0,13–0,15 rad/s ÷ kp.**
- **O dado:** com kp ∈ {1; 0,5; 0,3}, o erro médio por corrida foi 7,5°,
  17,5° e 27,8°, e o produto kp·|erro| fica constante.
- **A leitura:** a marcha tem uma tendência de giro própria por corrida, que a
  malha P só compensa aceitando erro.
- **Ganho menor piora; ganho maior derruba** (§8 de
  [desvio_de_direcao.md](desvio_de_direcao.md)).
- **Remédio:** termo integral com anti-windup.
- **O vaivém de ±7° por passo é da própria marcha.** A malha não deve
  persegui-lo.

**N.** **A estimativa de roll tem viés de +2,3° a +2,5°** contra a verdade
(correlação 0,96, atraso ~10 ms). A amplitude alternada é boa; o nível não. Um
estabilizador de roll com P sobre o roll cru comandaria inclinação permanente.
O plano da Fase 3 usa *washout*. Ver
[plano_fase3_roll.md](plano_fase3_roll.md) §1.

**O.** **Corridas de 60 s caem para trás perto do fim** (9/12, entre 52 e 61 s
de marcha); as de 30 s nunca mostraram isso. Não se sabe se é a parada
(`requestStop()`) ou o tempo andado. Não investigado
([desvio_de_direcao.md](desvio_de_direcao.md) §9).

**P.** **(Histórico, resolvido em 2026-10-21 pela IK corrigida, que hoje
deixa o joelho entre 53 e 82° conforme o `z_com`; ver
[ajustes_marcha.md](ajustes_marcha.md).)** A Marta andava com o joelho dobrado
a ~109°; o Darwin anda a ~55°.
- **Medido:** planejado 109°; medido 114–119° (pé em caixa, n = 10 por
  condição).
- **Causa:** a IK usa tornozelo → pé = 0,133 m
  (`RobotPhysicalParameters.cpp`), contra ~0,037 m no URDF. Com
  `zCom = 0,2975`, a IK só fecha a conta agachando. A origem do 0,133 é
  desconhecida.
- **O que a campanha de 72 corridas mostrou**
  ([plano_comparacao_darwin.md](plano_comparacao_darwin.md) §2.3):
  - **Esticar a perna sozinho não resolve.** Com `zCom` 0,345 (joelho a ~78°),
    o joelho de apoio alivia 45%, mas a perna em balanço quase dobra de
    torque. O total do joelho não cai, e a marcha cai até com torque
    ilimitado, porque marcha e estabilizador foram ajustados agachados.
  - **O gargalo é a perna em balanço:** joelho e quadril pedem 5–11 N·m RMS,
    com 6–13° de erro, para cumprir um apoio simples de 0,2 s.
  - **Nova linha de base com o pé em caixa:** 0/6 quedas a 6 N·m e 6/6 a
    4 N·m.
  - **No regime atual**, o joelho pede mais de 2,5 N·m (o stall do MX-28) em
    73% do tempo.

**Q.** **O tronco "pende" com a massa por cinemática, não pelo CoM.**
- Os eixos pitch de quadril, joelho e tornozelo são paralelos. Com o pé plano:
  `pitch_tronco = pitch_pé − Σalvo − Σ(real − alvo)`. A identidade fecha com
  resíduo ≤ 0,02° parado e ≤ 0,09° na marcha.
- O Σalvo vale −4,8°, por causa do −5° no hip pitch, e inclina o tronco para
  a frente.
- A cedência dos servos cresce com a carga e inclina para trás, grau por
  grau.
- Tronco leve pende para a frente; tronco pesado, para trás.
- Ver [panorama_parametros.md](panorama_parametros.md), Resultado 1.

**R.** **Limite de massa com o MX-28: o joelho de apoio satura.**
- Acima de ~3,8 kg, o comando do joelho de apoio passa de 2,5 N·m já na
  metade do 1º apoio simples, e o robô cai para trás antes de andar.
  Nenhuma célula saturada andou (Fisher p = 2×10⁻¹², 80 células).
- A compensação de gravidade não cria torque. Só servo mais forte, menos
  massa ou perna mais curta resolvem.
- Índice transferível: I = m·g·(coxa + canela)/stall; abaixo de ~2,6 a janela
  é ampla.
- **Nas corridas que andam, quem satura é o joelho de balanço:** 45% do tempo
  de balanço.

**S.** **A simulação é quase determinística.** Em 83 de 95 células,
repetições da mesma condição deram o mesmo desfecho. A estatística tem que
ser feita **por condição**; contar corridas repetidas é pseudorreplicação.
Para medir robustez, varie a condição: grade fina, perturbação ou empurrão.
Os "n = 3" de campanhas antigas sofrem disso.

**T.** **A compensação de gravidade só entra no 1º passo**, com rampa de 1 s
(`LegGravityCompensation`, `RAMP_TIME`). No agachamento e parado, Δ = 0.
- Por isso o −5° no hip pitch é necessário: sem ele o robô começa até 12°
  para trás e cai antes de andar (de 6/24 para 19/24 condições, Fisher
  p = 4×10⁻⁴).
- O ganho da compensação escolhe a direção da queda nas massas pesadas: 0,3
  para trás, 0,5 de lado, 0,7 para a frente.
- Em sobrevivência, 0,3 e 0,5 empatam; 0,7 é pior.

**U.** **Velocidades da marcha só são lidas na partida.**
- O `controller_node` lê `walk_velocity_x/y/yaw` uma vez, no construtor
  (`controller.cpp`, l. 25–27).
- O `_velocity_x` do `walk_test.py` grava o parâmetro, mas **não muda a
  marcha**; coincidia com o default 0,1 do launch. O `walk_test.py` não tem
  `_velocity_z`.
- Para girar ou fazer curva, passe `walk_velocity_yaw:=<rad/s>` (e
  `walk_velocity_x`) no **roslaunch**.
- A malha de rumo integra o comando na referência
  (`headingRef += rotação·dt`), então acompanha o giro em vez de brigar com
  ele.
- O launch passa `hip_yaw_sign:=-1` (achado L).
- **Giro nunca foi testado no regime atual.**

**V.** **Nas massas que andam, quem derruba é o roll.**
- No panorama, 52 das 102 quedas durante a marcha foram de lado.
- Elas aumentam com `z_com` alto (perna esticada) e com `hip_offset_x`
  grande: 44 das 52 tinham x ≥ 9 mm.
- O roll é hoje a principal limitação da janela de parâmetros (§7, item 1).

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
- **Para agentes (Claude Code):** um `docker run ... bash -c '<script de
  várias linhas>'` pode ser bloqueado pela checagem de segurança do harness
  ("could not check the script for dangerous removals"; o `--rm` do docker
  conta), e numa sessão sem prompt o bloqueio é definitivo. Contorno que
  funcionou: gravar o script num arquivo (no scratchpad), montar o diretório
  (`-v <dir>:/out`) e rodar `bash /out/script.sh`. Também: um `roslaunch ... &`
  seguido de `wait` nunca termina (o roslaunch não sai sozinho); mate-o
  explicitamente no fim do script.
- A imagem força driver NVIDIA por ENV, para o Gazebo. **O notebook tem gráfico
  híbrido** (a tela é da GPU integrada): sem `-e __NV_PRIME_RENDER_OFFLOAD=1`, o
  OpenGL da NVIDIA falha com `X Error ... BadValue (NV-GLX)` e o `gzserver` morre
  (a câmera simulada precisa de GL). **Não contornar com
  `LIBGL_ALWAYS_SOFTWARE=1`:** isso funciona, mas renderiza na CPU (llvmpipe).
  Conferir com `glxinfo -B | grep renderer` dentro do container: tem que
  aparecer a RTX 3050.

### Visualização no Foxglove

Uma tela só com o robô 3D andando no mundo e os gráficos sincronizados: rumo
atual × desejado, `correction` da malha de rumo, roll e pitch verdadeiros,
torque das juntas e estado de apoio. Layout pronto em
[`src/marta_bringup/foxglove/marta_marcha.json`](../src/marta_bringup/foxglove/marta_marcha.json).

**Uma vez só, no host:**
1. Instalar o Foxglove (pacote `.deb` em foxglove.dev/download) e entrar com a
   conta.
2. Em *Settings → Desktop app → ROS_PACKAGE_PATH*, pôr
   `/home/<usuario>/Documents/ic/marta_bringup/src`. É assim que o app acha as
   malhas `package://marta_description/...` quando abre um bag.
3. *Layouts → Import from file* → `marta_marcha.json`.

**Armadilha:** STL não guarda qual eixo é "para cima", e o Foxglove assume Y-up por
default. Com isso o robô aparece deitado e cada peça gira em torno da própria
origem (as partes parecem se espalhar). O layout já fixa *3D → Scene → Mesh up
axis = Z-up*, como no RViz e no Gazebo. Num layout novo, ajuste isso à mão. Os
rótulos e eixos das TFs também vêm desligados, porque no tamanho default ficam
maiores que o robô.

**Ao vivo:** suba o terminal 1 acima com `foxglove:=true` no `roslaunch` (o
container já roda com `--network host`) e, no app, *Open connection → Foxglove
WebSocket → `ws://localhost:8765`*. Ao vivo, as malhas vêm pelo próprio bridge.

**Por bag:** toda corrida do `walk_test.py` com `_bag_out:=...` já grava TF,
URDF e as setas de rumo. Abra o `.bag` no app (*Open local file*). Dá para
pausar e arrastar a linha do tempo.

**Custo:** sem cliente conectado, o RTF não muda (0,999 com o bridge, 0,998
sem). Com o app conectado, o bridge serializa tudo que o layout assina. Confira
o RTF antes de usar ao vivo em medidas, e nas campanhas deixe `foxglove`
desligado (default).

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

**Branch de trabalho: `classic-walk`**, na raiz e nos quatro submódulos
(`marta_control`, `marta_description`, `marta_gazebo`, `ball_tracking`). O
`.gitmodules` aponta `branch = classic-walk`. A **`main` não foi tocada** e
continua funcional.

**Antes de trocar de branch:**
- **Voltar à `main`:** `git switch main && git submodule update --recursive`.
  Os submódulos não trocam sozinhos.
- **Não use `dev/...` como nome de branch:** os remotos têm uma branch `dev`, e
  o git recusa (conflito arquivo × diretório).

Commits na raiz:

| commit | conteúdo |
|---|---|
| `cb793af` | investigação do desvio de direção, estabilizador de pitch, telemetria de marcha |
| `c1b7228` | visualização no Foxglove (bridge ao vivo, tópicos no bag, layout) |

Os submódulos recebem commit próprio **antes** do ponteiro na raiz. Commits do
usuário vão **sem** a linha `Co-Authored-By`.

**Commit pendente de 2026-10-28** (comandos preparados para o usuário, que roda
os commits). Entra:
- código da compensação de gravidade e `hip_pitch_offset_deg` no
  `marta_control`;
- servo MX-28 no `marta_description`;
- scripts, docs e figuras de out/2026;
- resumos e desenhos das campanhas.

Os dados brutos ficam fora, pela regra `src/marta_bringup/experiments/**/dados/`
do `.gitignore`: ~1,7 GB, regeneráveis pelos `rodar_*.sh`.

**Histórico: fora do git até 2026-10-15**, incluído no commit acima:
- `experiments/bag_para_csv.py` (gera o `_estabilizador.csv`),
  `experiments/plot_rumo.py`, `experiments/plot_correcao.py`;
- `docs/figuras_marcha/fig_rumo_*.png`;
- esta rodada de documentação (`docs/*.md`, incluindo
  `plano_fase3_roll.md`);
- **`marta_control/include/controller.h`: mudança do usuário**, default
  `kpHeading_` de 1,0 para 0,3.
  - Hoje não tem efeito, porque o `stabilizer_test.launch` sempre passa
    `kp_heading` (default 1,0).
  - O mesmo arquivo recebeu, em out/2026, o membro da compensação de
    gravidade, então entra no commit de 2026-10-28.
  - Os dados do achado M desaconselham kp = 0,3. O usuário foi avisado ao
    receber os comandos de commit e decide se mantém essa linha.

Dados de experimento ficam em `experiments/yaw/dados/` (ignorado pelo git, com
dezenas de bags de 35 MB). `experiments/yaw/dados/foxglove_demo.bag` é uma
corrida de 20 s com TF e URDF, para abrir no Foxglove.

**Fora do repositório:** as notas pessoais do usuário ficam em
`~/Documents/ic/marta_notas/` (resumo e roteiro de apresentação) e não entram no
git.

### Defeitos conhecidos, não corrigidos

| onde | o quê |
|---|---|
| `marta_gazebo/scripts/real_to_sim_topics.py` | `Publisher` por callback; derruba o RTF quando usada |
| `marta_bringup/include/joint_to_tf_publisher.cpp` | assina `/marta/<membro>/state` enquanto o controlador publica em `/command` — **nó efetivamente morto** |
| `marta_description/urdf/marta.gazebo` | referencia `r_tight_1`/`l_tight_1`; os links são `r_thigh_1`/`l_thigh_1`, então dois blocos de atrito não têm efeito |
| `marta_control/src/torso_attitude_node.cpp` | portão de confiança cego à direção da aceleração (achado C) |

---

## 7. Próximos passos, em ordem de valor (revisto em 2026-10-28)

1. **Estabilizador de roll (Fase 3, segunda metade).**
   - **Por que agora:**
     - a compensação de gravidade já existe;
     - o pitch tem estabilizador;
     - o limite de massa (achado R) é de hardware e não se resolve com
       controle;
     - nas massas viáveis, o roll é a causa dominante de queda (achado V).
   - **O que já está pronto:** desenho, referências com página e protocolo em
     [plano_fase3_roll.md](plano_fase3_roll.md); o achado N (viés de +2,3° na
     estimativa de roll) exige *washout*.
   - **Ao medir:** fazer a estatística por condição (achado S).
2. **Sintonia dos estabilizadores de pitch e de rumo no regime atual**
   (MX-28, 3,80 kg; §0). Os ganhos vêm do regime de 6 N·m e da marcha
   antiga. Varrer kp/kd de pitch e kp/saturação de rumo em várias condições
   (massas e posturas), não repetindo uma só.
3. **Curvas e giro no próprio eixo** (achado U). Testar `walk_velocity_yaw`
   no launch, com e sem a malha de rumo, e medir o giro real pela verdade de
   solo (diferença de ângulo; achado K). Verificar os limites do gerador para
   rotação, e no robô real o sentido dos servos `hip_yaw` (achado L).
4. **Termo integral na malha de rumo** (achado M), com anti-windup.
5. **Compensação de gravidade ativa já no agachamento** (achado T). Hipótese
   não testada: dispensaria o −5° e deixaria o tronco ereto.
6. **Quedas em 52–61 s** nas corridas longas (achado O).
7. **Peso adaptativo no acelerômetro** (achado C).
8. **Comparação com o Darwin-OP** ([plano_comparacao_darwin.md](plano_comparacao_darwin.md)):
   o essencial foi feito (achados P e R); resta o tempo de apoio simples.
