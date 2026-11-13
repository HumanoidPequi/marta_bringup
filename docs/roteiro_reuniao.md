# Roteiro da reunião: marcha omnidirecional com roll

Tudo roda a partir de `~/Documents/ic/marta_bringup`. Todos os comandos usam o
campo completo, com gol e bola.

## 0. Antes de começar (uma vez por sessão)

```bash
cd ~/Documents/ic/marta_bringup
xhost +local:root
```

Sem o `xhost`, a GUI do Gazebo e a janela do teleop não abrem: aparece o erro
`Authorization required`.

---

## 1. Giro no próprio eixo, com roll (3,3 kg)

**Terminal 1:** sobe a simulação com GUI. Ela sobe pausada.

```bash
docker run --rm -it --name marta_stab --network host --gpus all --device /dev/dri \
  -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 \
  -e __GLX_VENDOR_LIBRARY_NAME=nvidia -e __NV_PRIME_RENDER_OFFLOAD=1 \
  -v "$PWD":/ws -v /tmp/.X11-unix:/tmp/.X11-unix -w /ws marta_bringup:1.0 \
  bash -c 'source /opt/ros/noetic/setup.bash && source devel/setup.bash && \
    roslaunch marta_bringup stabilizer_test.launch gui:=true \
      foot_collision:=box stabilizer_enabled:=true heading_enabled:=true \
      joint_effort:=2.5 joint_velocity:=100 implicit_damping:=true joint_damping:=0.43 \
      gains:=/ws/src/marta_description/config/controller_mx28.yaml \
      gravity_comp_enabled:=true gravity_comp_joints:=hip_roll,knee gravity_comp_gain:=0.3 \
      torso_mass_scale:=0.5 z_com:=0.250 hip_offset_x:=0.0036 hip_pitch_offset_deg:=-5 \
      roll_stabilizer_enabled:=true kp_roll_hip:=0.5 kd_roll_hip:=0.075 \
      kp_roll_ankle:=0.5 kd_roll_ankle:=0.075 kd_pitch:=0.06 \
      walk_velocity_x:=0 walk_velocity_y:=0 walk_velocity_yaw:=0.175'
```

**Terminal 2:** despausa, espera o robô agachar (~15 s) e gira por 30 s.

```bash
docker exec -it marta_stab bash -c 'source /opt/ros/noetic/setup.bash && \
  source /ws/devel/setup.bash && rosrun marta_bringup walk_test.py _duration:=30'
```

- **O que esperar:** gira ~300° em 30 s (0,175 rad/s = 10°/s) sem cair. No
  fim, o terminal 2 imprime um resumo: resultado, giro do corpo, pitch e roll
  RMS, fator de tempo real (≈ 1,0).
- **Para comparar sem roll:** no terminal 1, troque para
  `roll_stabilizer_enabled:=false`. O robô cai, em geral entre 7 e 18 s:
  o instante varia entre corridas, a queda não.
- **Para encerrar:** Ctrl-C no terminal 1.

---

## 2. Foxglove: os 3 bags gravados

**Configuração (uma vez só, no app Foxglove):**
1. *Settings → Desktop app → ROS_PACKAGE_PATH* =
   `/home/lucasolives/Documents/ic/marta_bringup/src`. Sem isso, o robô 3D
   não aparece.
2. No seletor de layout (canto superior direito), use *Import from file* →
   `src/marta_bringup/foxglove/marta_omni.json`.
3. **Confira se o seletor mostra `marta_omni`,** não `marta_marcha`.

**Para abrir:** use *Open local file*, um bag por vez. Os bags estão em
`src/marta_bringup/experiments/omni/demo/`.

| # | bag | o que mostrar |
|---|---|---|
| 1 | `D_M33_roll_giro+0.175_pD_r1.bag` | **Giro com roll, 3,3 kg.** Gira os 30 s; ω realizado = 1,002 do comandado. Aba **Controladores:** o roll fica em −2 a +4°, e dá para ver a correção no quadril e no tornozelo. Aba **Torque/limite:** o joelho passa de 1,0 (saturado) em ~8% do tempo. |
| 2 | `D_M33_roll_curva+0.175_pE_r1.bag` | **Curva com roll, 3,3 kg** (0,1 m/s com 0,175 rad/s). Completa os 30 s. Aba **LIPM:** o CoM segue o planejado a ~5–7 mm. |
| 3 | `D_M38_roll_reta0.10_pD_r1.bag` | **Reta com roll, 3,8 kg** (a única reta com roll gravada). Aba **LIPM:** o CoM segue o planejado a ~5 mm, o melhor de todos os movimentos. Passa pela bola aos ~19 s. |
| extra | `D_M33_off_giro+0.175_pD_r1.bag` | **O mesmo giro do bag 1, sem roll.** A oscilação de roll cresce de ±5° para ±9° e ±13°, e o robô cai aos 11,6 s. É o melhor contraste com o bag 1. |

**Abas do `marta_omni`:**

| aba | conteúdo |
|---|---|
| Torque/limite | > 1,0 = junta saturada; em cima a perna esquerda, embaixo a direita |
| Erro das juntas | alvo − real, em graus |
| Controladores | pitch, roll (verdadeiro, estimado, correções), rumo, compensação de gravidade |
| LIPM (Kajita) | CoM medido × planejado, energia orbital, plano de fase |
| Marcha | estado de apoio, taxa de giro, DCM |

**Números da campanha** (para citar):
- com roll, **42 de 44** corridas completaram 30 s; sem roll, **13 de 44**;
- no giro no lugar: **16/16** com roll contra **0/16** sem.

---

## 3. Teleop pelo teclado (3,4 kg, com roll)

Um terminal só. A janela "Marta - teleop" despausa a simulação sozinha. **Não
rode o `walk_test.py` junto.**

```bash
docker run --rm -it --name marta_stab --network host --gpus all --device /dev/dri \
  -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 \
  -e __GLX_VENDOR_LIBRARY_NAME=nvidia -e __NV_PRIME_RENDER_OFFLOAD=1 \
  -v "$PWD":/ws -v /tmp/.X11-unix:/tmp/.X11-unix -w /ws marta_bringup:1.0 \
  bash -c 'source /opt/ros/noetic/setup.bash && source devel/setup.bash && \
    roslaunch marta_bringup stabilizer_test.launch gui:=true teleop:=true \
      foot_collision:=box stabilizer_enabled:=true heading_enabled:=true \
      joint_effort:=2.5 joint_velocity:=100 implicit_damping:=true joint_damping:=0.43 \
      gains:=/ws/src/marta_description/config/controller_mx28.yaml \
      gravity_comp_enabled:=true gravity_comp_joints:=hip_roll,knee gravity_comp_gain:=0.3 \
      torso_mass_scale:=0.534 z_com:=0.239 hip_offset_x:=0.0036 hip_pitch_offset_deg:=-5 \
      roll_stabilizer_enabled:=true kp_roll_hip:=0.5 kd_roll_hip:=0.075 \
      kp_roll_ankle:=0.5 kd_roll_ankle:=0.075 kd_pitch:=0.06'
```

Espere a janela mostrar **"em pe - Enter para andar"** (~15 s). Clique nela e
segure as teclas:

| tecla | movimento |
|---|---|
| ↑ ou W | reta (0,1 m/s); com o robô em pé, já dá a partida |
| ← → ou A D | giro no próprio eixo |
| ↑ + ← / → | curva |
| nenhuma | marcha no lugar |
| ↓ ou S | para |
| `c` | ↑ + lado passa a girar no lugar em vez de curvar |
| `+` / `-` | giro mais rápido / normal |

A janela mostra a última tecla recebida. Se ela não muda, clique de novo na
janela.

**Validado** em 4 sequências de 90 s com todas as trocas de comando, com
kd 0,03 e com kd 0,06: nenhuma queda. No campo completo, bater na bola ou no
gol derruba o robô.

Os comandos usam **kd de pitch 0,06** (E7). Os bags da seção 2 foram gravados
antes, com kd 0,03.

---

## Se algo der errado

| sintoma | solução |
|---|---|
| `Authorization required` / Gazebo ou janela não abre | `xhost +local:root` e rode de novo |
| `container name "/marta_stab" is already in use` | `docker rm -f marta_stab` |
| fator de tempo real bem abaixo de 1 | outra simulação rodando na máquina; feche-a |
| robô 3D não aparece no Foxglove | confira o `ROS_PACKAGE_PATH` (seção 2, passo 1) |
| gráficos diferentes dos descritos | o layout selecionado não é o `marta_omni` |
