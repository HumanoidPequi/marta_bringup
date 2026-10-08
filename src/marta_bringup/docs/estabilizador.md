# Estabilizador de atitude do tronco — como rodar

Guia de operação da malha de estabilização da Marta em simulação: o robô parado
sob perturbação e o robô caminhando.

## O que está no ar

```
/marta/imu  ──►  torso_attitude_node  ──►  /marta/torso_attitude
 (Gazebo,          filtro complementar        [pitch, pitch_rate,
  100 Hz)          + Butterworth no giro       roll, roll_rate]
                                                      │
                                                      ▼
                        controller_node  ──►  OmnidirectionalWalkZMP (malha aberta)
                                          ──►  InverseKinematics
                                          ──►  TorsoStabilizer  ── Δθ no hip pitch
                                          ──►  tópicos de junta
                                          ──►  /marta/stabilizer_debug
```

O gerador de marcha continua em malha aberta. O estabilizador é uma camada
separada que soma uma correção angular depois da cinemática inversa, como em
Kajita 4.5.1.

**Convenção de sinais:** `pitch > 0` = tronco tombando para **frente**;
`roll > 0` = tombando para a **direita**.

---

## Compilar

O workspace precisa estar compilado com o código atual. Dois caminhos:

### Caminho A — repositório inteiro montado (usado nos testes deste guia)

```bash
cd ~/Documents/ic/marta_bringup
docker run --rm -v "$PWD":/ws -w /ws marta_bringup:1.0 \
  bash -lc 'source /opt/ros/noetic/setup.bash && catkin_make'
```

O `build/` e o `devel/` ficam no host e são reaproveitados entre execuções.

### Caminho B — pelo `run.sh`

O `run.sh` monta **apenas o `src/`** dentro de uma imagem cujos binários foram
compilados no `docker build`. Então, a cada container novo:

```bash
cd ~/Documents/ic/marta_bringup
./docker/run.sh bash
# dentro do container:
catkin_make && source devel/setup.bash
```

Para não repetir isso sempre, regere a imagem uma vez:

```bash
docker build -t marta_bringup:1.0 -f docker/Dockerfile .
```

---

## Subir a bancada

Um único launch serve para os dois modos de teste. Ele sobe o Gazebo **pausado**
de propósito: o `controller_spawner` leva alguns segundos e, sem comando de
junta, o robô desaba antes de qualquer controle assumir. Quem despausa é o
script de teste (o botão *play* da interface também serve).

```bash
cd ~/Documents/ic/marta_bringup
xhost +local:root

docker run --rm -it --name marta_stab --network host --gpus all --device /dev/dri \
  -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 \
  -e __GLX_VENDOR_LIBRARY_NAME=nvidia -e __NV_PRIME_RENDER_OFFLOAD=1 \
  -v "$PWD":/ws -v /tmp/.X11-unix:/tmp/.X11-unix \
  -w /ws marta_bringup:1.0 \
  bash -c 'source /opt/ros/noetic/setup.bash && source devel/setup.bash && \
           roslaunch marta_bringup stabilizer_test.launch'
```

O `-v "$PWD":/ws` tem que ser exatamente esse: o `devel/` foi gerado com
caminhos `/ws`.

Terminais adicionais no mesmo container:

```bash
docker exec -it marta_stab bash -c \
  'source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash && bash'
```

### Argumentos do launch

| argumento | padrão | efeito |
|---|---|---|
| `gui` | `true` | `false` roda sem janela (bem mais rápido) |
| `world_name` | `world.world` | o campo. `worlds/empty.world` para rodada leve |
| `stabilizer_enabled` | `true` | liga/desliga a malha PD na partida |
| `kp_pitch` | `0.2` | ganho proporcional |
| `kd_pitch` | `0.03` | ganho derivativo |
| `max_pitch_correction` | `0.1745` | saturação (10°) |
| `walk_enabled` | `false` | começar já caminhando (normalmente o script liga) |
| `walk_velocity_x` | `0.1` | velocidade de avanço [m/s] |

> O `world.world` não é só cenário: ele traz ajustes próprios de contato
> (`contact_max_correcting_vel = 0`, `contact_surface_layer = 1e-4`) que mudam
> como o pé toca o chão. Medir nele é medir no ambiente real de uso.

---

## Modo A — robô parado sob perturbação

Aplica um pulso de força no tronco e mede a resposta do pitch.

```bash
rosrun marta_bringup push_test.py
```

Opções:

```bash
rosrun marta_bringup push_test.py _force:=8 _duration:=0.15   # empurrão mais forte
rosrun marta_bringup push_test.py _force:=-5                  # empurrão por trás
rosrun marta_bringup push_test.py _settle:=5.0                # janela de observação
rosrun marta_bringup push_test.py _pre_wait:=8                # espera antes do pulso
```

O script não fica esperando o robô "assentar": ele espera o laço de controle
entrar em regime — a primeira mensagem em `/marta/stabilizer_debug`, que só sai
depois da rampa do `Agacha()` — e daí conta `pre_wait` segundos fixos (5 s por
padrão) antes do pulso.

Imprime pico, tempo de acomodação e o envelope da oscilação em três fatias.

### O que esperar

Sequência visual: o robô agacha lentamente por ~10 s (é o `Agacha()` rampando a
escala de 0 a 1), fica em pé e passa a oscilar **lateralmente** de forma
visível — o plano frontal ainda não tem realimentação.

Medido no campo, pulso de 5 N × 0,1 s (0,5 N·s):

| | sem estabilizador | com estabilizador |
|---|---|---|
| pico de pitch | +6,08° | **+3,09°** |
| tempo de acomodação | 5,59 s (não acomodou) | **0,82 s** |
| envelope do pitch | 5,8 → 6,1 → 6,1° (não decai) | 3,1 → 0,5 → 0,8° (amortecido) |
| correção máx. no quadril | — | 0,99° |

> Impulsos acima de ~1 N·s derrubam o robô com ou sem estabilizador: jogam o CoM
> para fora da região de captura, e nenhuma estratégia de quadril recupera isso.
> Para calibrar a intuição, `_force:=20` derruba nos dois casos.

---

## Modo B — durante a caminhada

Despausa, espera o laço entrar em regime, liga a marcha em tempo de execução,
mede e manda parar.

```bash
rosrun marta_bringup walk_test.py _duration:=30 _velocity_x:=0.1
```

Reporta: se caiu e quando, distância percorrida, RMS e pico de pitch e roll,
correção de pico no quadril, tempo em saturação e número de trocas de apoio.

### O que esperar

Medido no campo, marcha a 0,1 m/s por 30 s, três repetições por condição:

| repetição | pitch RMS sem | pitch RMS com | distância sem | distância com |
|---|---|---|---|---|
| 1 | 5,50° | 3,61° | 2,095 m | 2,477 m |
| 2 | 5,90° | 5,21° | 1,769 m | 2,529 m |
| 3 | 4,68° | 2,12° | 2,348 m | 2,408 m |
| **média** | **5,36°** | **3,65°** | **2,071 m** | **2,471 m** |

RMS de pitch 32% menor e distância 19% maior, com a mesma direção nos três
pares. Nenhuma queda em nenhuma das seis corridas. Saturação em 0% do tempo,
correção de pico na casa de 3°.

**A variância entre execuções é grande.** Um único par de corridas não conclui
nada: em uma delas as duas condições empataram. Compare sempre com pelo menos
três repetições.

O RMS de roll fica em 4–6° nas duas condições — a estabilização de pitch não
melhora o plano frontal de forma mensurável.

---

## Comparar com e sem estabilizador

Depois de cair o robô não volta a uma condição limpa, então **suba a simulação
duas vezes** em vez de alternar no meio:

```bash
roslaunch marta_bringup stabilizer_test.launch gui:=false stabilizer_enabled:=false
roslaunch marta_bringup stabilizer_test.launch gui:=false stabilizer_enabled:=true
```

### Demonstração ao vivo (a mais convincente)

Com o robô **caminhando**, em outro terminal:

```bash
rosparam set /controller_node/stabilizer_enabled false   # desliga no meio da marcha
rosparam set /controller_node/stabilizer_enabled true    # religa
```

O tronco passa a balançar visivelmente mais assim que você desliga. Ligar e
desligar não dá solavanco: os pesos das pernas decaem em 0,05 s.

---

## Roteiro de apresentação

Duas corridas em sequência: primeiro a marcha crua, depois a mesma marcha com a
malha fechada. O clímax é o terceiro ato, desligar ao vivo.

Orçamento de tempo: cada ato leva ~1 min 20 s (25 s para o Gazebo subir, ~9 s de
rampa do `Agacha()`, 5 s de acomodação, 30 s de marcha medida, alguns segundos de
relatório). Com fala no meio, a apresentação inteira fica em torno de 8 minutos.

### Antes da plateia entrar

Compile e faça uma corrida completa de ensaio. A primeira execução do dia é a que
descobre que faltou `catkin_make` ou que o `xhost` não foi liberado.

```bash
cd ~/Documents/ic/marta_bringup
xhost +local:root
docker run --rm -v "$PWD":/ws -w /ws marta_bringup:1.0 \
  bash -lc 'source /opt/ros/noetic/setup.bash && catkin_make'
```

Deixe três terminais prontos e posicionados na tela: **T1** para o launch, **T2**
para o script de teste, **T3** para o `rosparam`. A janela do Gazebo precisa estar
visível junto com T2 — é a comparação entre o que se vê e o que o número diz que
convence.

### Ato 1 — a marcha sem nenhuma estabilização

**T1:**
```bash
docker run --rm -it --name marta_stab --network host --gpus all --device /dev/dri \
  -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 \
  -e __GLX_VENDOR_LIBRARY_NAME=nvidia -e __NV_PRIME_RENDER_OFFLOAD=1 \
  -v "$PWD":/ws -v /tmp/.X11-unix:/tmp/.X11-unix \
  -w /ws marta_bringup:1.0 \
  bash -c 'source /opt/ros/noetic/setup.bash && source devel/setup.bash && \
           roslaunch marta_bringup stabilizer_test.launch stabilizer_enabled:=false'
```

**T2**, assim que o Gazebo abrir:
```bash
docker exec -it marta_stab bash -c \
  'source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash && \
   rosrun marta_bringup walk_test.py _duration:=30'
```

O que apontar, na ordem em que acontece:

1. **O Gazebo abre parado.** É de propósito: sem comando de junta o robô desaba
   antes de os controladores subirem. O script despausa.
2. **O robô agacha por ~10 s.** É a rampa que leva a postura de pernas retas até a
   postura de marcha.
3. **Em pé, já balança sozinho** — e pende para a esquerda. Vale dizer que o plano
   frontal não tem realimentação nenhuma: isso é a marcha nua.
4. **Andando**, chame a atenção para o tronco cabeceando para frente e para trás.
   É o pitch em malha aberta.
5. **O relatório**: pitch RMS na casa de 5°, pico acima de 11°, distância ~2,1 m.

> Não prometa que o robô vai cair sem o estabilizador. Em sete corridas ele caiu
> uma vez só, e não se reproduziu. O argumento aqui é a qualidade da marcha, não a
> queda.

Encerre o Ato 1 com `Ctrl+C` no T1.

### Ato 2 — a mesma marcha com a malha fechada

Mesmos dois comandos, trocando uma palavra no launch:

**T1:**
```bash
docker run --rm -it --name marta_stab --network host --gpus all --device /dev/dri \
  -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 \
  -e __GLX_VENDOR_LIBRARY_NAME=nvidia -e __NV_PRIME_RENDER_OFFLOAD=1 \
  -v "$PWD":/ws -v /tmp/.X11-unix:/tmp/.X11-unix \
  -w /ws marta_bringup:1.0 \
  bash -c 'source /opt/ros/noetic/setup.bash && source devel/setup.bash && \
           roslaunch marta_bringup stabilizer_test.launch stabilizer_enabled:=true'
```

**T2:** o mesmo `walk_test.py _duration:=30`.

O que apontar:

1. **O cabeceio some quase todo.** O tronco fica visivelmente mais quieto no plano
   sagital, enquanto o balanço lateral continua igual — o que separa bem o que a
   Fase 2 resolve do que a Fase 3 ainda deve.
2. **O relatório lado a lado com o do Ato 1**: pitch RMS ~3,6° contra ~5,4°,
   distância maior.
3. **`tempo em saturação: 0,0%`** — a correção nunca bate no limite de segurança de
   10°; o pico fica na casa de 3°. São 3° de quadril comprando 30% de RMS.

Para o Ato 3, deixe esta simulação rodando.

### Ato 3 — desligar ao vivo, com o robô andando

É o momento mais convincente, porque elimina a dúvida de "as duas corridas eram
diferentes por acaso".

**T3:**
```bash
docker exec -it marta_stab bash -c \
  'source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash && bash'

# põe o robô para andar continuamente
rosparam set /controller_node/walk_enabled true

# e então, com a plateia olhando o Gazebo:
rosparam set /controller_node/stabilizer_enabled false
rosparam set /controller_node/stabilizer_enabled true
```

O nó relê os parâmetros a cada 0,5 s, então o efeito aparece em menos de um
segundo. O tronco volta a cabecear ao desligar e se aquieta ao religar. Ligar e
desligar não dá solavanco: os pesos das pernas decaem em 0,05 s.

Se houver um segundo monitor, deixe o gráfico rodando durante este ato — a curva
mudando de amplitude ao vivo é mais legível que o robô:

```bash
rqt_plot /marta/torso_attitude/data[0] /marta/stabilizer_debug/data[0]
```

### Fechamento

Três coisas que valem ser ditas antes que alguém pergunte:

- **A comparação de uma corrida só não prova nada.** A simulação tem variância
  alta; os números apresentados são a média de três repetições por condição, e num
  dos pares as duas condições empataram.
- **O plano frontal ainda está em malha aberta.** É o modo dominante hoje, e é a
  Fase 3.
- **Ganhos do Gazebo não transferem para o robô real.** Aqui as juntas são
  controladas por esforço com PID; no robô são servos de posição.

### Plano B

| se acontecer | faça |
|---|---|
| o robô cai durante a demo | não tente recuperar; `Ctrl+C` no T1 e suba de novo — depois de cair ele não volta a uma condição limpa |
| o Gazebo abre e nada se move | é o estado pausado normal; o script despausa. Se o script não estiver rodando, aperte *play* |
| `walk_test.py` reclama de `/marta/stabilizer_debug` | o `controller_node` não subiu; veja o T1 |
| a simulação está lenta demais na projeção | rode com `gui:=false` e apresente só os números e o `rqt_plot` |
| esqueceu de compilar | `Ctrl+C`, rode o `catkin_make` do começo deste roteiro |

---

## Gráficos

### rqt_plot ao vivo

```bash
# pitch, roll e a correção aplicada no quadril
rqt_plot /marta/torso_attitude/data[0] /marta/torso_attitude/data[2] \
         /marta/stabilizer_debug/data[0]

# só o pitch e sua velocidade angular
rqt_plot /marta/torso_attitude/data[0] /marta/torso_attitude/data[1]

# correção por perna, para ver a migração na troca de apoio
rqt_plot /marta/stabilizer_debug/data[1] /marta/stabilizer_debug/data[2]
```

### Gravar para análise depois

```bash
rosbag record -O marcha.bag /marta/torso_attitude /marta/stabilizer_debug \
                            /marta/imu /marta/joint_states

# depois
rqt_bag marcha.bag
rostopic echo -b marcha.bag -p /marta/torso_attitude > atitude.csv
```

### Conteúdo dos tópicos

`/marta/torso_attitude` (`std_msgs/Float64MultiArray`), rad e rad/s:

| índice | campo |
|---|---|
| `data[0]` | pitch — positivo tombando para frente |
| `data[1]` | pitch_rate — do giroscópio filtrado |
| `data[2]` | roll — positivo tombando para a direita |
| `data[3]` | roll_rate |

`/marta/stabilizer_debug` (`std_msgs/Float64MultiArray`):

| índice | campo |
|---|---|
| `data[0]` | Δθ calculado, já saturado [rad] |
| `data[1]` | Δθ somado ao hip pitch **esquerdo** [rad] |
| `data[2]` | Δθ somado ao hip pitch **direito** [rad] |
| `data[3]` | pitch usado no cálculo [rad] |
| `data[4]` | pitch_rate usado no cálculo [rad/s] |
| `data[5]` | apoio: 0 = duplo, 1 = esquerdo, 2 = direito |
| `data[6]` | 1 se a saturação está ativa |

---

## Parâmetros de controle

Todos são relidos a cada 0,5 s (`getParamCached`): dá para sintonizar com o robô
em pé ou caminhando, **sem reiniciar nada**.

### Estabilizador — `/controller_node`

| parâmetro | padrão | o que faz |
|---|---|---|
| `stabilizer_enabled` | `true` | liga/desliga a malha PD |
| `kp_pitch` | `0.2` | rad de correção por rad de inclinação |
| `kd_pitch` | `0.03` | rad de correção por rad/s |
| `pitch_ref` | `0.0` | referência de pitch [rad] |
| `max_pitch_correction` | `0.1745` | saturação rígida [rad] = 10° |
| `support_blend_time` | `0.05` | transição de peso entre as pernas [s] |
| `walk_enabled` | conforme launch | liga/desliga a marcha em tempo real |
| `walk_velocity_x/y/yaw` | `0.1 / 0 / 0` | velocidade comandada |

```bash
rosparam set /controller_node/kp_pitch 0.35
rosparam set /controller_node/kd_pitch 0.05
rosparam set /controller_node/max_pitch_correction 0.12
rosparam set /controller_node/walk_enabled true
rosparam set /controller_node/walk_velocity_x 0.05

rosparam get /controller_node/kp_pitch      # conferir
```

### Estimador de atitude — `/torso_attitude_node`

Lidos **na partida**; mudanças exigem reiniciar o nó.

| parâmetro | padrão | o que faz |
|---|---|---|
| `alpha` | `0.998` | peso do giro no filtro complementar (τ ≈ 5 s) |
| `gyro_cutoff_hz` | `15.0` | corte do Butterworth de 2ª ordem |
| `accel_trust_band` | `2.0` | faixa de aceite do acelerômetro [m/s²] |
| `imu_mount_rpy` | `[0, π, 0]` | rotação sensor → corpo |
| `expected_rate_hz` | `100.0` | taxa nominal da IMU |

---

## Como sintonizar

Ordem sugerida, sempre com o robô **parado** primeiro (Modo A) e só depois em
marcha (Modo B):

1. Comece em `kp_pitch = 0.2`, `kd_pitch = 0.03`.
2. Suba `kp` até o pico de pitch parar de melhorar. Se a oscilação começar a
   **crescer** (o `push_test.py` avisa com `CRESCENDO`), você passou do ponto.
3. Suba `kd` para amortecer o que sobrou. `kd/kp ≈ 0,17 s` coloca o zero do PD
   sobre a dinâmica do pêndulo invertido (ω = √(g/z) ≈ 5,7 rad/s, 0,91 Hz) — é
   o teto natural para começar a procurar.
4. Repita em marcha. Ganhos que funcionam parado podem excitar ressonância no
   regime dinâmico.

### Sinais de que algo está errado

| sintoma | causa provável |
|---|---|
| `[Stabilizer] correcao saturada` durante marcha normal | `kp` alto demais, ou o robô está mesmo caindo |
| oscilação cresce após o empurrão | `kp` alto, `kd` baixo |
| o robô mal reage ao empurrão | `kp` baixo, ou `stabilizer_enabled=false` |
| pitch derivando devagar sem motivo | bias do giroscópio (erro de regime = bias × τ) |
| `acelerometro fora da faixa de confianca` constante | `accel_trust_band` apertado demais |
| `[Controller] sem dados em /marta/torso_attitude` | o `torso_attitude_node` não está no ar |

---

## Testes offline (sem ROS)

Codificam as convenções de sinal e a resposta dos filtros. Rodam em segundos, a
partir da raiz do `marta_control`:

```bash
g++ -std=c++14 -Iinclude -I/usr/include/eigen3 \
    test/test_torso_attitude.cpp src/TorsoAttitudeEstimator.cpp \
    -o /tmp/t1 && /tmp/t1

g++ -std=c++14 -Iinclude -I/usr/include/eigen3 \
    test/test_torso_stabilizer.cpp src/TorsoStabilizer.cpp src/Joints.cpp \
    -o /tmp/t2 && /tmp/t2
```

Além de passar/falhar, imprimem as tabelas usadas para escolher `alpha` e o
corte do giroscópio.

---

## Limitações conhecidas

- **O plano frontal (roll) não tem realimentação.** É o modo dominante hoje:
  4–6° de RMS e picos de 10–16° durante a marcha. É a Fase 3.
- **Parado, o robô pende ~10° para a esquerda** e oscila em torno desse valor,
  não em torno de zero. Vale investigar antes da Fase 3: se for assimetria real
  da postura nominal, o `roll_ref` do estabilizador frontal não deve ser zero.
- **A simulação tem variância alta entre execuções.** Compare sempre com pelo
  menos três repetições por condição.
- **Rigidez das juntas em simulação é o teto prático.** Os controladores de
  posição em esforço usam `p = 50`; testado com `p = 500`, a física diverge para
  NaN. No robô real são servos de posição, com dinâmica diferente — ganhos
  sintonizados no Gazebo não transferem diretamente.
- **`imu_mount_rpy` vale para a simulação.** No robô real, medir a montagem
  antes de confiar no sinal: com o robô parado e vertical, os quatro campos de
  `/marta/torso_attitude` devem ficar em ~0; inclinando para frente à mão, o
  pitch tem que subir positivo.
