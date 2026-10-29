# Telemetria de marcha fundamentada no 3D-LIPM

> Parte de [README.md](README.md) — leia o mapa central primeiro.
> Documento irmão: [estabilizadores.md](estabilizadores.md).
> Interpretação das figuras: [como_ler_as_figuras.md](como_ler_as_figuras.md).

Camada de telemetria tipada que substitui o `stabilizer_debug`, ligada às
grandezas de Kajita et al., *The 3D Linear Inverted Pendulum Mode* (IROS 2001).

## Tópicos

| tópico | tipo | taxa | conteúdo |
|---|---|---|---|
| `/marta/walk/reference` | `marta_msgs/WalkReference` | 100 Hz | o **plano**: CoM, pés, fase, apoio, parâmetros do LIPM |
| `/marta/walk/telemetry` | `marta_msgs/WalkTelemetry` | 100 Hz | o **medido**: CoM real, energia orbital, DCM, erros |
| `/marta/<junta>_position/state` × 20 | `control_msgs/JointControllerState` | 100 Hz | torque e desvio por junta — **já existiam** |

Desde 2026-10-07, a `WalkTelemetry` também traz yaw de verdade de solo (`yaw_true`,
`left/right_foot_yaw` e as taxas) e o ponto de contato de cada pé
(`left/right_foot_contact`). As taxas **não** devem ser integradas (achado K do
README). Uso em [desvio_de_direcao.md](desvio_de_direcao.md).

`/marta/torso_attitude` passou de 4 para 6 campos (`yaw`, `yaw_rate` no fim) e
`/marta/stabilizer_debug` de 7 para 9 (`heading_error`, `heading_correction` no
fim). Os índices antigos não mudaram.

Desde 2026-10-08, o `walk_telemetry_node` também publica para visualização
(Foxglove; ver o README, §5):

| tópico | tipo | conteúdo |
|---|---|---|
| `/tf`: `odom → base_link` | TF | pose do tronco no mundo, verdade de solo (desliga com `~publish_world_tf:=false`) |
| `/robot_description` | `std_msgs/String`, latched | cópia do URDF do parâmetro, para entrar no bag |
| `/marta/viz/rumo` | `visualization_msgs/MarkerArray` | setas do rumo atual (azul) e do rumo desejado (laranja) |
| `/marta/viz/rumo_graus` | `std_msgs/Float64MultiArray` | `[yaw_verdadeiro, rumo_desejado, erro]` em graus |

O rumo desejado no frame da verdade é `yaw_true − heading_error` (a malha guarda a
referência no yaw estimado; mesma conta do `experiments/plot_rumo.py`). O
`walk_test.py` grava esses tópicos e `/tf_static`; o bag fica ~30% maior.
`_bag_viz:=false` deixa tudo de fora.

O CoM real é soma massa×posição sobre os 21 links dinâmicos, com massas lidas de
`/robot_description` e poses/twists de `/gazebo/link_states` — velocidade sem
derivada numérica. Massa somada: **4,7811 kg**, idêntica ao URDF.

## Torque e desvio saem de graça

As 20 transmissões usam `EffortJointInterface` e cada controlador publica:

| campo | significado |
|---|---|
| `set_point` | ângulo comandado |
| `process_value` | ângulo medido |
| `error` | **desvio de rastreamento** |
| `command` | **torque pedido pelo PID, antes da saturação** |

O torque *aplicado* (já saturado) está em `/marta/joint_states.effort`. O par
define saturação exatamente: com `joint_effort:=6`, o joelho **pede 22,6 N·m e
recebe 6,00** — 3,8× acima do disponível.

## Pipeline

```
walk_test.py _bag_out:=...        ~0,6 MB/s sem os topicos de visualizacao,
   ↓                              ~0,8 MB/s com eles (_bag_viz:=true, default)
bag_para_csv.py <bag> <dir>       roda no container (precisa de rosbag)
   ↓  <nome>_telemetria.csv + <nome>_juntas.csv + <nome>_estabilizador.csv
figuras_marcha.py <prefixo> <dir> roda em qualquer lugar (só matplotlib)
plot_rumo.py / plot_correcao.py   yaw em torno do rumo e correcao da malha
                                  (desvio_de_direcao.md §9)
```

O `_estabilizador.csv` (desde 2026-10-08) traz o `/marta/stabilizer_debug` com
as colunas `t, delta_pitch, pitch, pitch_rate, support, saturated,
heading_error, heading_correction`. São o que o **controlador** enxerga: o erro
de rumo vem do yaw estimado, não da verdade de solo. Bags anteriores à malha de
rumo dão NaN nas duas últimas colunas.

**Bag sem índice.** Se o `rosbag record` for morto sem SIGINT, o bag fica sem
índice (`ROSBagUnindexedException`). `rosbag reindex` às vezes recupera; na
corrida `rumo_kp0.5_r1` não recuperou ("Bag contains no message").

`/gazebo/link_states` fica **fora** do bag de propósito: são ~700 KB/s e o nó de
telemetria já destila dele tudo que importa.

Variáveis de ambiente: `LIMITE_TORQUE` (marca o limite nas figuras de junta) e
`JANELA_S` (largura da janela temporal, padrão 4 s — o artigo mostra ~4 s; com
20 s de marcha seriam 50 passos ilegíveis).

## Figuras

| arquivo | conteúdo |
|---|---|
| `fig_paper5_com_tempo` | figura 5 do artigo, medido sobre planejado |
| `fig_paper4_plano_xy` | figura 4: traço do CoM no plano com as pegadas |
| `fig_energia_orbital` | eq. (28)(29) com as faixas de apoio simples |
| `fig_plano_fase` | órbita `(x, ẋ)` com a hipérbole prevista, eq. (40) |
| `fig_torque_juntas` | mapa de calor torque × junta × tempo |
| `fig_desvio_juntas` | idem para o erro de rastreamento |
| `fig_ranking_juntas` | quem pede mais torque × quem menos rastreia |

## Dois achados da validação

**1. O `zCom` do gerador não é a altura do CoM.** Ele assume **0,2975 m**, mas o
CoM real fica a **0,2184 m** do ponto de contato — **36% de erro em `g/zc`**, que
é o único parâmetro do 3D-LIPM. O `zCom` é a distância torso–pé usada pela
cinemática inversa, não a altura do centro de massa.

> **Nota (2026-10-15).** A altura do CoM varia com o limite de torque: de 0,212
> a 0,220 m com o pé em caixa (README, tabela do §1). Os 0,2184 m acima e os
> 0,2134 m do experimento abaixo são medidas de condições diferentes, e não
> uma contradição.

Consequência: a constante de tempo do gerador é `tZMP = 0,174 s` quando a física
pede `sqrt(0,2184/9,81) = 0,149 s`. O parâmetro `comSwingFactor` existe
exatamente para corrigir isso — `comSwingFactor = 0,734` alinharia os dois. Não
foi alterado: é mudança de controle, não de instrumentação.

Por isso `WalkTelemetry` publica `z_com_measured` ao lado do `z_com` do plano.

**2. Os joelhos são o gargalo.** No ranking, `l_knee` e `r_knee` pedem ~22 N·m
contra 6 disponíveis e acumulam ~26° de desvio — os dois primeiros lugares nas
duas metades da figura, o que liga causa e efeito diretamente. Dez das doze
juntas de perna passam do limite; só os `ank_roll` ficam abaixo.

## Ressalvas honestas

- **Não há sensor de força nos pés**, então o ZMP não é mensurável. O
  `zmp_est_*` publicado vem de `x − (zc/g)·ẍ`, o que ignora momento angular e
  precisa derivar a velocidade. Medir ZMP de verdade exigiria
  `gazebo_ros_ft_sensor` nos tornozelos (~15 linhas de URDF).
- A energia orbital **salta nas trocas de apoio** porque a origem do frame muda
  de pé — é o mesmo efeito que produz os saltos na figura 5 do artigo, não um
  defeito. A deriva a observar é a de *dentro* de cada faixa sombreada.
- A telemetria só começa após a rampa do `Agacha()` (~10 s), porque a referência
  é publicada de dentro do laço principal. O `walk_test.py` já espera por isso.

## Custo

Fator de tempo real com o nó de telemetria no ar: **0,998**. Sem impacto
mensurável — o nó usa `queue_size=1`, timer de 100 Hz e `AsyncSpinner`.

> O `AsyncSpinner` não é decoração: com `ros::spin()` de thread única, a
> enxurrada de `/gazebo/link_states` (1 kHz × 4,9 KB) satura a fila e os demais
> callbacks nunca rodam.

---

# Primeiro experimento com a telemetria nova

## Pergunta

A telemetria revelou que o `zCom` do gerador (0,2975 m, distância torso–pé da
IK) não é a altura real do CoM (medida: **0,2134 ± 0,0034 m**) — 36% de erro em
`g/z_c`, o único parâmetro do 3D-LIPM. O `comSwingFactor` entra em exatamente um
lugar, `tZMP = sqrt(zCom·csf/g)`, então `csf = 0,734` casaria os dois.

E a figura de energia mostrou algo direcional: **E_x cresce sistematicamente
dentro de cada apoio simples**, o que é o sintoma esperado de um `tZMP` grande
demais. Predição: corrigi-lo deve reduzir a deriva.

## Desenho

2×2: torque ∈ {1000, 6} N·m × `csf` ∈ {1,0; 0,734}, n=4, estabilizador
**desligado** (mede o gerador, sem a realimentação mascarando). Métrica
primária: amplitude de `E_x` por fase de apoio simples, bordas aparadas em 20%.

## A predição foi refutada

Nas 12 corridas válidas (RTF > 0,85, zero quedas):

| τ | csf | deriva (z_c medido) | deriva (z_c gerador) | pitch RMS real |
|---|---|---|---|---|
| 1000 | 1,000 | 0,0148 ± 0,0014 | 0,0147 ± 0,0008 | 2,48° |
| 1000 | 0,734 | 0,0191 ± 0,0067 | 0,0198 ± 0,0078 | 2,89° |
| **6** | **1,000** | **0,0202 ± 0,0018** | **0,0160 ± 0,0007** | **6,18°** |
| **6** | **0,734** | **0,0232 ± 0,0015** | **0,0181 ± 0,0008** | **6,48°** |

Em τ=6: deriva **+15,2% (p=0,043)** com a régua física e **+13,7% (p=0,006)**
com a do gerador. Casar o `tZMP` com a altura real do CoM **piora** a aderência
ao modelo. O pitch RMS também piora, mas sem significância (+4,9%, p=0,434).

Leitura: os parâmetros do gerador estão **co-sintonizados**. Passo, separação
lateral, altura de torso da IK e `tZMP` formam um conjunto ajustado em volta do
valor "errado"; mexer em um só desarranja o resto. Corrigir o `zCom` de verdade
exigiria re-sintonizar o conjunto, não trocar um número.

**O `comSwingFactor` fica em 1,0.** A descoberta permanece registrada e agora
mensurável — é hipótese para uma re-sintonia futura, não um bug a corrigir.

## Duas verificações de método que o resultado exigiu

**A régua.** A primeira rodada calculou a energia com o `z_c` do gerador nas duas
condições. Isso é questionável: se o `z_c` físico é outro, a grandeza não é
invariante nem para um robô LIPM perfeito, e a régua embutiria o próprio erro em
investigação. Refiz guardando o estado bruto (x, ẋ, altura medida) e calculando
com as duas réguas. **Elas concordam** — a conclusão é robusta à escolha.

**A instrumentação perturba?** Quatro corridas caíram, todas com RTF entre 0,52
e 0,66, contra 0/5 numa campanha anterior sem telemetria. A suspeita óbvia era
que assinar `/gazebo/link_states` (37 links a 1 kHz) estivesse estrangulando o
gzserver — a mesma falha do `real_to_sim_topics.py`. Um teste controlado em três
configurações, três repetições cada, refutou:

| configuração | RTF |
|---|---|
| sem telemetria | 0,998 / 0,998 / 0,994 |
| telemetria sem `link_states` | 0,992 / 0,996 / 0,997 |
| telemetria completa | 0,995 / 0,997 / 0,993 |

Sem diferença mensurável. Era contenção de CPU na máquina. O guarda de RTF fez o
trabalho: as 12 corridas acima de 0,85 não tiveram nenhuma queda, e o filtro
passou a ser **obrigatório** em `analisar.py` e `analisar_csf.py`.
