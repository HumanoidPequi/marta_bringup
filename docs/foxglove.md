# Visualização da marcha no Foxglove: como foi feita e como reproduzir

Estado em 2026-11-16, branch `classic-walk`. Fatos conferidos no código e nos
commits; o que não foi verificado está marcado como tal.

**Resumo.** Assistimos à simulação da Marta no Foxglove de dois jeitos:

- **ao vivo**, enquanto o Gazebo roda, pelo `foxglove_bridge`;
- **depois**, abrindo um rosbag gravado pelo `walk_test.py`.

Nos dois casos aparecem o robô 3D andando no mundo e gráficos
sincronizados com ele. Há dois layouts prontos:

| layout | arquivo | para quê |
|---|---|---|
| `marta_marcha` | `src/marta_bringup/foxglove/marta_marcha.json` | o primeiro (out/2026): rumo, malha de rumo, atitude, torque e apoio, numa tela |
| `marta_omni` | `src/marta_bringup/foxglove/marta_omni.json` | o atual (nov/2026): 3D mais 5 abas (torque/limite, erro das juntas, controladores, LIPM, marcha) |

---

## 1. O problema que cada peça resolve

O Foxglove só desenha o que chega a ele **como tópico**. A simulação, sozinha,
não entregava quatro coisas.

| faltava | por quê | solução | onde |
|---|---|---|---|
| **o robô andar no mundo** | o `robot_state_publisher` só publica as juntas em relação ao tronco: o robô "anda" parado no lugar | a TF `odom → base_link` passa a vir da pose verdadeira do tronco no Gazebo (`/gazebo/link_states`) | `walk_telemetry_node.cpp`, `publishViz()` |
| **o modelo 3D dentro do bag** | o URDF fica no *parâmetro* `/robot_description`, e parâmetro não entra em bag | o mesmo nó republica o URDF como **tópico** latched `/robot_description` (`std_msgs/String`) | `walk_telemetry_node.cpp` |
| **ver o rumo** | o rumo desejado da malha de rumo não existia como sinal | setas no 3D (`/marta/viz/rumo`) e série em graus (`/marta/viz/rumo_graus`). O painel Plot do Foxglove não subtrai séries, por isso o erro já vem calculado | `walk_telemetry_node.cpp` |
| **grandezas derivadas** (torque/limite, erro por junta, LIPM no frame do pé) | os tópicos brutos têm 20 controladores, e o LIPM da telemetria usa os eixos do mundo | o nó `viz_marcha.py` lê, calcula e publica três vetores prontos para plotar. Ele não toca no controle | `src/marta_bringup/scripts/viz_marcha.py` |

Além disso:

- o `foxglove_bridge` foi instalado na imagem docker;
- ganhou um *switch* no launch;
- o `walk_test.py` passou a gravar esses tópicos no bag.

---

## 2. Fluxo dos dados

```
Gazebo ──/gazebo/link_states──► walk_telemetry_node ──► /tf (odom→base_link)
   │                                  │                  /robot_description (latched)
   │                                  │                  /marta/viz/rumo, /marta/viz/rumo_graus
   │                                  └───────────────► /marta/walk/telemetry
   ├─/marta/joint_states──► robot_state_publisher ────► /tf (juntas, 50 Hz)
   ├─/marta/<junta>_position/state ─┐
   │                                ├──► viz_marcha.py ► /marta/viz/torque_pernas
controller_node ─/marta/walk/reference┘                  /marta/viz/erro_pernas
   │                                                     /marta/viz/lipm
   └──► /marta/stabilizer_debug, /marta/roll_stabilizer, /marta/gravity_comp

            ┌── foxglove_bridge (ws://localhost:8765) ──► app Foxglove (ao vivo)
tópicos ────┤
            └── walk_test.py _bag_out:=… (rosbag) ──► .bag ──► app Foxglove (offline)
                                                             + layout .json
```

---

## 3. Fluxo de desenvolvimento, em ordem

### Etapa 1 (2026-10-08): ponte ao vivo, TF do mundo, URDF no bag, layout `marta_marcha`

**Commits:**
- `2d32dfd`, no submódulo `marta_control`;
- `c1b7228`, na raiz.

1. **Imagem docker** (`docker/Dockerfile`): uma camada nova com
   `ros-noetic-foxglove-bridge`, posta depois das camadas pesadas para não
   invalidar o cache delas. A imagem atual `marta_bringup:1.0` tem a versão
   0.8.4.
2. **Launch** (`stabilizer_test.launch`):
   - **`foxglove` (default `false`):** inclui o `foxglove_bridge.launch` na
     porta 8765. O container roda com `--network host`, então o app no host
     conecta direto.
   - **`tf_rate` (default 50):** `publish_frequency` do
     `robot_state_publisher`.
3. **`walk_telemetry_node.cpp`:**
   - **Parâmetros novos:**
     - `~world_frame` (default `odom`);
     - `~publish_world_tf` (default `true`).
   - **Tópicos novos:** `/tf odom→base_link`, `/robot_description` latched,
     `/marta/viz/rumo` e `/marta/viz/rumo_graus`.
   - **Dependências** (`CMakeLists.txt` e `package.xml`): `tf2_ros` e
     `visualization_msgs`.
   - **Detalhe 1, TF repetida:** com `use_sim_time`, o timer pode disparar
     duas vezes no mesmo instante simulado, e o tf2 reclama de
     `TF_REPEATED_DATA`. Por isso a TF só é publicada se o carimbo de tempo
     avançou (`last_tf_stamp_`).
   - **Detalhe 2, rumo desejado:**
     - É calculado como `yaw_verdadeiro − heading_error`. A malha guarda a
       referência no yaw estimado, e o erro vale o mesmo nos dois frames
       enquanto o estimador não deriva. É a mesma conta do
       `experiments/plot_rumo.py`.
     - Parado, as duas setas coincidem.
     - O valor é levado para perto do atual, para não dar salto de 360°.
4. **`walk_test.py`:** com `_bag_out:=…`, grava também `/tf`, `/tf_static`,
   `/robot_description`, `/marta/viz/rumo` e `/marta/viz/rumo_graus`. O bag
   fica ~30% maior. `_bag_viz:=false` deixa esses tópicos de fora.
5. **Layout `marta_marcha.json`.** O 3D fica à esquerda; à direita, de cima
   para baixo:
   - rumo atual × desejado;
   - `correction` e erro da malha de rumo;
   - atitude (roll e pitch, verdadeiros e estimados);
   - torque `command` das juntas de roll e dos joelhos;
   - estado de apoio.

   As duas configurações do 3D que estavam erradas por default (tópico 4)
   foram corrigidas no próprio JSON.
6. **Custo medido:** sem cliente conectado, o RTF não muda (0,999 com o
   bridge, 0,998 sem). **Não foi medido** com o app conectado.

### Etapa 2 (2026-11-12): `viz_marcha.py` e layout `marta_omni` (demo da marcha omnidirecional)

**Commit:** `c6246e0`, na raiz.

1. **`viz_marcha.py`**, ligado por `viz_marcha:=true` no launch. Recebe
   `effort` = `joint_effort` do launch. Publica três vetores
   `Float64MultiArray`, com os rótulos no `layout.dim[0].label`:
   - **torque exigido / limite:** pega o `command` de cada controlador, que é
     o torque pedido **antes** de o Gazebo cortar no limite, e divide pelo
     `joint_effort`. **Acima de 1, a junta está saturada.**
   - **erro alvo − real**, em graus.
   - **3D-LIPM de Kajita (2001) no frame do pé de apoio, com os eixos girados
     pelo yaw desse pé.** A telemetria usa os eixos do mundo. Na reta dá no
     mesmo, mas no giro e na curva x e y misturam avanço e lateral, e a
     energia orbital e o DCM deixam de valer.
2. **`walk_test.py`:** passa a gravar também os três tópicos do
   `viz_marcha.py`, mais `/marta/gravity_comp` e `/marta/roll_stabilizer`.
3. **Layout `marta_omni.json`:** 3D à esquerda (40%), um painel de abas à
   direita e 14 gráficos (tabela na seção 5).
   - **Títulos:** cada painel leva o título em `title` e em
     `foxglovePanelTitle`, porque versões diferentes do app leem chaves
     diferentes.
   - **Legendas:** cada série tem `label` e cor fixos.
4. **Bags de demonstração:**
   - 9 corridas definidas em `experiments/omni/desenho_demo.csv`, todas com
     `viz_marcha:=true`;
   - gravadas pelo `rodar_panorama.sh` com `BAG_DIR`;
   - ficam em `experiments/omni/demo/`, fora do git pelo `.gitignore`, com
     10–28 MB cada.
5. **Conferência nos bags:**
   - `rosbag info` mostra todos os tópicos do layout;
   - a fração do tempo com torque ≥ 1,0 no joelho bate com a análise
     independente (`joelho_sat`): 8,5% contra 8,9% no giro M33 com roll, e
     12,6% contra 11,9% na reta M38.

---

## 4. Configurações do painel 3D que não podem faltar

Num layout novo, as duas primeiras vêm erradas por default.

| configuração | valor | o que acontece sem ela |
|---|---|---|
| *Scene → Mesh up axis* (`scene.meshUpAxis`) | **`z_up`** | o STL não diz qual eixo é "para cima" e o Foxglove assume Y-up: o robô aparece deitado, e cada peça gira em torno da própria origem (as partes parecem espalhadas) |
| *Transforms*: rótulos e eixos (`showLabel: false`, `axisScale: 0.05`, `frame:*: visible false`) | desligados | no tamanho default, os eixos e nomes das 23 frames ficam maiores que o robô |
| camada URDF (`foxglove.Urdf`) | `sourceType: topic`, `topic: /robot_description`, `displayMode: visual` | o 3D fica vazio no bag |
| *Follow* | `followTf: base_link`, `followMode: follow-position` | o robô sai da tela |
| camada de grade (`foxglove.Grid`) | `frameId: odom`, 10 m, 20 divisões | sem referência de chão |
| tópico `/marta/viz/rumo` | visível | sem as setas de rumo (azul = atual, laranja = desejado, 0,3 m, acima da cabeça) |

**No app (uma vez só):** em *Settings → Desktop app → ROS_PACKAGE_PATH*, pôr
`/home/<usuario>/Documents/ic/marta_bringup/src`. O URDF referencia as malhas
como `package://marta_description/...`. Ao abrir um bag, o app resolve esse
caminho no disco do host; sem isso, o robô não aparece. Ao vivo, as malhas
vêm pelo próprio bridge.

---

## 5. Referência de tópicos e índices

Os gráficos usam caminhos como `/topico.data[i]`. O sufixo `.@degrees`
converte de rad para graus.

### Publicados para a visualização

| tópico | tipo | quem publica | conteúdo |
|---|---|---|---|
| `/tf` `odom→base_link` | TF | `walk_telemetry_node` | pose verdadeira do tronco (verdade de solo) |
| `/tf` das juntas | TF | `robot_state_publisher` | 50 Hz (`tf_rate`) |
| `/robot_description` | `std_msgs/String`, latched | `walk_telemetry_node` | cópia do URDF |
| `/marta/viz/rumo` | `visualization_msgs/MarkerArray` | `walk_telemetry_node` | 2 setas, `ns` `rumo_atual` e `rumo_desejado` |
| `/marta/viz/rumo_graus` | `Float64MultiArray` | `walk_telemetry_node` | `[0]` yaw verdadeiro, `[1]` rumo desejado, `[2]` erro, em graus |
| `/marta/viz/torque_pernas` | `Float64MultiArray` (13) | `viz_marcha.py`, 50 Hz | `[0–11]` \|command\|/effort; `[12]` = 1,0 (linha do limite) |
| `/marta/viz/erro_pernas` | `Float64MultiArray` (12) | `viz_marcha.py`, 50 Hz | alvo − real [°] |
| `/marta/viz/lipm` | `Float64MultiArray` (11) | `viz_marcha.py`, a cada telemetria | `[0,1]` x, y do CoM − pé de apoio [m]; `[2,3]` referência do gerador; `[4,5]` vx, vy; `[6,7]` energia orbital (eq. 28–29); `[8,9]` DCM; `[10]` yaw do frame [°] |

**Ordem das juntas** em `torque_pernas` e `erro_pernas`: 0–5 = perna
esquerda, 6–11 = perna direita. Em cada perna: `hip_yaw`, `hip_roll`,
`hip_pitch`, `knee`, `ank_pitch`, `ank_roll`.

### Já existentes, usados pelos layouts

| caminho | significado |
|---|---|
| `/marta/walk/telemetry.{roll,pitch}_true`, `.{roll,pitch}`, `.yaw_rate_true`, `.support_state` | atitude verdadeira e estimada; taxa de giro; apoio (0 duplo, 1 esq, 2 dir) |
| `/marta/stabilizer_debug.data[0]` | correção de pitch (`delta_pitch`) |
| `/marta/stabilizer_debug.data[7]`, `[8]` | erro e correção da malha de rumo |
| `/marta/roll_stabilizer.data[3]`, `[4]`, `[5]` | erro pós-washout (φ − φ̄), u do quadril, u do tornozelo |
| `/marta/gravity_comp.data[3..14]` | Δ da compensação de gravidade por junta, na mesma ordem acima (os índices 0–2 são `w_left`, `w_right` e a rampa). O layout usa `[4]` hip_roll esq, `[6]` joelho esq, `[10]` hip_roll dir, `[12]` joelho dir |
| `/marta/<junta>_position/state.command` | torque pedido pelo PID (`marta_marcha`) |

### Abas do `marta_omni`

| aba | gráficos |
|---|---|
| Torque/limite | `torque_pernas` [0–5, 12] (esq) e [6–11, 12] (dir); y de 0 a 1,6 |
| Erro das juntas | `erro_pernas` [0–5] e [6–11] |
| Controladores | pitch; roll; rumo (`rumo_graus[0,1]` e `stabilizer_debug[8]` em °/s); gravidade |
| LIPM (Kajita) | x medido × planejado (`lipm[0,2]`); y (`lipm[1,3]`); energia (`lipm[6,7]`); plano de fase (x = `lipm[0]`, y = `lipm[4]`) |
| Marcha | apoio (StateTransitions); taxa de giro; DCM (`lipm[8,9]`) |

Todos os gráficos de tempo mostram uma janela de 10 s (`followingViewWidth`)
e são sincronizados com o 3D (`isSynced`).

---

## 6. Como reproduzir

### 6.1 Preparação (uma vez)

1. **Imagem:**
   ```bash
   docker run --rm marta_bringup:1.0 bash -c 'source /opt/ros/noetic/setup.bash && rospack find foxglove_bridge'
   ```
   Tem que imprimir `/opt/ros/noetic/share/foxglove_bridge`. Se não
   imprimir, reconstrua a imagem com o `docker/Dockerfile`.
2. **Compilar o workspace** (por causa do `walk_telemetry_node`, no
   submódulo):
   ```bash
   docker run --rm -v "$PWD":/ws -w /ws marta_bringup:1.0 bash -c 'source /opt/ros/noetic/setup.bash && catkin_make'
   ```
3. **App no host:** Foxglove Studio, pacote `.deb` (aqui está a versão
   3.5.0). Depois:
   - **ROS_PACKAGE_PATH** como na seção 4;
   - **layout:** no seletor do canto superior direito, *Import from file* →
     `src/marta_bringup/foxglove/marta_omni.json`.

### 6.2 Ao vivo

**Terminal 1:** simulação com o bridge e o `viz_marcha`. Esta é a
configuração atual: 3,3 kg, roll, giro 0,175.

```bash
cd ~/Documents/ic/marta_bringup
xhost +local:root
docker run --rm -it --name marta_stab --network host --gpus all --device /dev/dri \
  -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 \
  -e __GLX_VENDOR_LIBRARY_NAME=nvidia -e __NV_PRIME_RENDER_OFFLOAD=1 \
  -v "$PWD":/ws -v /tmp/.X11-unix:/tmp/.X11-unix -w /ws marta_bringup:1.0 \
  bash -c 'source /opt/ros/noetic/setup.bash && source devel/setup.bash && \
    roslaunch marta_bringup stabilizer_test.launch gui:=true foxglove:=true viz_marcha:=true \
      foot_collision:=box stabilizer_enabled:=true heading_enabled:=true \
      joint_effort:=2.5 joint_velocity:=100 implicit_damping:=true joint_damping:=0.43 \
      gains:=/ws/src/marta_description/config/controller_mx28.yaml \
      gravity_comp_enabled:=true gravity_comp_joints:=hip_roll,knee gravity_comp_gain:=0.3 \
      torso_mass_scale:=0.5 z_com:=0.250 hip_offset_x:=0.0036 hip_pitch_offset_deg:=-5 \
      roll_stabilizer_enabled:=true kp_roll_hip:=0.5 kd_roll_hip:=0.075 \
      kp_roll_ankle:=0.5 kd_roll_ankle:=0.075 kd_pitch:=0.06 \
      walk_velocity_x:=0 walk_velocity_y:=0 walk_velocity_yaw:=0.175'
```

**No app:** *Open connection → Foxglove WebSocket → `ws://localhost:8765`*,
com o layout `marta_omni`.

**Terminal 2:** despausa e anda 30 s. Acrescente `_bag_out:=/ws/<nome>.bag`
para gravar.

```bash
docker exec -it marta_stab bash -c 'source /opt/ros/noetic/setup.bash && \
  source /ws/devel/setup.bash && rosrun marta_bringup walk_test.py _duration:=30'
```

- **Sem `viz_marcha:=true`**, as abas Torque, Erro e LIPM ficam vazias. O
  3D, o rumo e os controladores continuam.
- **Mudar o movimento:** a velocidade vem dos `walk_velocity_*` do
  `roslaunch`, lidos só na partida. O `_velocity_x` do `walk_test.py` não
  muda nada.
- **Com o teleop** (`teleop:=true`, README §5), basta acrescentar
  `foxglove:=true viz_marcha:=true`. Nesse caso, não rode o
  `walk_test.py`.

### 6.3 Por bag (offline)

1. **Gravar:** use o comando de 6.2 com `_bag_out:=…` no `walk_test.py`. Para
   regenerar os 9 bags da demo:
   1. apague as linhas `D_*` de `experiments/omni/campanha_panorama.csv`;
   2. rode:
      ```bash
      BAG_DIR=src/marta_bringup/experiments/omni/demo \
        bash src/marta_bringup/experiments/rodar_panorama.sh \
        src/marta_bringup/experiments/omni/desenho_demo.csv 20261121 omni
      ```
   Essa campanha roda sem GUI. Os bags da demo foram gravados em 2026-11-12,
   com kd de pitch 0,03 e a postura de braço histórica.
2. **Conferir os tópicos:**
   `rosbag info <bag>` tem que listar `/tf`, `/robot_description` e os
   `/marta/viz/*`.
3. **Abrir:** no app, *Open local file*, **um bag por vez** (abertos juntos,
   os dados se misturam). O layout tem que estar como `marta_omni`. Aperte
   play ou arraste a linha do tempo.

---

## 7. Como saber se deu certo

- **3D:**
  - o robô em pé, sobre a grade, em Z-up;
  - ele se desloca no mundo;
  - duas setas acima da cabeça, que coincidem quando o robô está parado.
- **À direita, cinco abas.** Na aba Torque/limite há uma linha horizontal em
  1,0.
- **Valores de referência** do bag `D_M33_roll_giro+0.175_pD_r1`:
  - **roll** entre −2° e +4°;
  - **taxa de giro** ~10°/s (0,175 rad/s);
  - **joelho de apoio** encosta em 1,0 em ~8% do tempo.

---

## 8. Problemas que já apareceram e a solução

| sintoma | causa | solução |
|---|---|---|
| robô deitado ou com peças espalhadas | Mesh up axis = Y-up (default) | 3D → Scene → Mesh up axis = Z-up (já no JSON) |
| 3D vazio ao abrir um bag | app sem `ROS_PACKAGE_PATH`, ou bag sem `/robot_description` | seção 4; regravar sem `_bag_viz:=false` |
| robô anda "parado no lugar" | sem a TF `odom→base_link` | `publish_world_tf` do `walk_telemetry_node` (default `true`) |
| abas ou gráficos diferentes dos descritos | o layout selecionado é o `marta_marcha` | seletor → `marta_omni` (ou *Import from file*) |
| títulos não aparecem | layout importado antes da correção dos títulos | importar o JSON de novo |
| Torque, Erro e LIPM vazios | `viz_marcha:=false` | `viz_marcha:=true` |
| app não conecta | `foxglove:=false`, ou container sem `--network host` | conferir os dois; a porta é 8765 |
| janela do Gazebo não abre (`gazebo_gui` exit 134) | faltam `xhost +local:root`, `DISPLAY` e o socket X11 | comando de 6.2 completo |
| RTF baixo ao vivo | o bridge serializa tudo que o layout assina | conferir o RTF antes de medir; nas campanhas, deixar `foxglove:=false` (default) |

---

## 9. Pendências conhecidas

- **Plano de fase (aba LIPM):** o JSON grava o eixo x como
  `xAxisVal: custom` + `xAxisPath`, que é o formato do Foxglove Studio
  antigo. **Não confirmei** que a versão 3.5.0 lê essas chaves.
  - **Se o painel abrir como série no tempo**, ajuste à mão:
    - *X-axis value* = Expression, `/marta/viz/lipm.data[0]`;
    - série y = `/marta/viz/lipm.data[4]`;
    - *Time range* = All.
  - Depois do ajuste, exporte o layout de novo.
- **RTF com o app conectado:** não foi medido.
- **Robô real:** nada disto foi testado nele. A TF do mundo vem da verdade
  de solo do Gazebo, que não existe no robô.

---

## 10. Arquivos envolvidos

| arquivo | papel |
|---|---|
| `docker/Dockerfile` | instala o `ros-noetic-foxglove-bridge` |
| `src/marta_bringup/launch/stabilizer_test.launch` | args `foxglove`, `tf_rate` e `viz_marcha` |
| `src/marta_control/src/walk_telemetry_node.cpp` (+ `CMakeLists.txt`, `package.xml`) | TF do mundo, URDF latched e rumo |
| `src/marta_bringup/scripts/viz_marcha.py` | torque/limite, erro e LIPM no frame do pé |
| `src/marta_bringup/scripts/walk_test.py` | lista de tópicos do bag (`_bag_out`, `_bag_viz`) |
| `src/marta_bringup/foxglove/marta_marcha.json`, `marta_omni.json` | layouts |
| `src/marta_bringup/experiments/omni/desenho_demo.csv`, `rodar_panorama.sh` (`BAG_DIR`) | bags da demo |

Documentos relacionados:
- README §5, "Visualização no Foxglove";
- `telemetria_marcha.md` (tópicos da telemetria);
- `marcha_omnidirecional.md`, "Demonstração no Foxglove" (roteiro dos
  bags);
- `roteiro_reuniao.md` §2.
