# Marcha omnidirecional e estabilizador de roll (Marta, MX-28, 3,3 e 3,8 kg)

> Parte de [README.md](README.md).
>
> - Dados: `src/marta_bringup/experiments/omni/`.
> - Análise: `analisar_omni.py`.
> - Explicação para reunião:
>   [marcha_omnidirecional_explicacao.md](marcha_omnidirecional_explicacao.md).

**Estado (2026-11-12):** E0 a E6env concluídas (~520 corridas), mais a
demonstração no Foxglove.
- **Resposta curta:** com o estabilizador de roll (quatorn K1), a Marta gira
  no próprio eixo e anda na direção comandada nas duas massas. Sem ele, o
  giro não sobrevive (E1 e E5).
- **Achado de método** que corrige a E1: o campo simulado tem um gol e uma
  bola no caminho da marcha em linha reta. Ver
  [Obstáculos no campo simulado](#obstáculos-no-campo-simulado-achado-de-método).
- **Decisões do usuário** no fim: [Decisões](#decisões). A E7 (2026-11-13)
  fechou a sintonia com roll: **kd de pitch 0,06** e `hip_offset_x` base.

## Perguntas

1. **Mínimo:** a Marta consegue **girar no próprio eixo** e **andar na direção
   comandada**?
2. **Depois:** curva, passo lateral, diagonal e combinados (a marcha
   omnidirecional completa).
3. **O estabilizador de roll ajuda?** Em geral e, principalmente, no giro.
4. **Os ganhos de pitch e de rumo estão bem sintonizados** em toda a faixa
   testada (3,3 e 3,8 kg, todos os movimentos)?

## O que já existia

**O gerador já é omnidirecional.** O `OmnidirectionalWalkZMP` vem do mestrado
do Maximo (*Omnidirectional ZMP-Based Walking for a Humanoid Robot*, ITA,
2015; `~/Documents/ic/study/Mestrado Marcos Maximo-1.pdf`):
- ele recebe vx, vy e vψ;
- rotação e lateral avançam em passos alternados, abrindo e fechando as
  pernas (§3, p. 56);
- o `limitTorso` impede que as pernas se cruzem.

**O Maximo testou esse gerador** (Tabela 5.4, p. 86; resultados nas p. 89–93):

| experimento | vx (m/s) | vy (m/s) | vψ (rad/s) |
|---|---|---|---|
| lateral | 0 | 0,05 | 0 |
| giro no lugar | 0 | 0 | π/18 ≈ 0,175 |
| diagonal | 0,2 | 0,05 | 0 |
| curva | 0,15 | 0 | π/18 |

No giro no lugar, ele chegou a 80° em 8 s (p. 90). **Mas** os testes foram num
Bioloid simulado no V-REP, **com o estabilizador desligado** (p. 86) e sem
medir quedas. O que este estudo acrescenta:
- a Marta, com o servo MX-28 realista;
- os estabilizadores de pitch, de roll e de rumo;
- um mapa de sobrevivência.

**As velocidades só são lidas na partida** (achado U do README): cada
condição é um `roslaunch` próprio, com `walk_velocity_x/y/yaw`.

## Configurações

Comum a todas as corridas:
- servo MX-28 (2,5 N·m, Kp 12,74, amortecimento implícito 0,43);
- pé em caixa;
- compensação de gravidade `hip_roll,knee` com ganho 0,3;
- `hip_pitch_offset_deg` −5;
- estabilizador de pitch e malha de rumo ligados, salvo quando são o fator
  testado;
- `gui:=false`;
- 30 s.

| nome | massa | `torso_mass_scale` | `z_com` | `hip_offset_x` |
|---|---|---|---|---|
| M33 | 3,30 kg | 0,5 | 0,250 | 0,0036 |
| M38 | 3,80 kg | 0,67 | 0,239 | 0,009 |

São as melhores combinações de cada massa no panorama
([panorama_parametros.md](panorama_parametros.md)): ambas andaram 3 de 3 em
reta.

## Método

### Replicação e estatística

A simulação é **quase** determinística (achado S). Repetir a mesma condição
na mesma carga costuma dar o mesmo resultado. **Mas a temporização das
mensagens ROS muda o desfecho em condições sensíveis:**
- na verificação de regressão, a reta M33 caiu 1 vez em 5, depois de mudar a
  carga (mais tópicos gravados);
- as séries partem iguais (0,03° de diferença) e se separam até a queda.

É o mesmo fenômeno que fazia corridas do usuário "às vezes andarem, às vezes
caírem".

**Como replicamos:** com **fatores reais**, não repetindo a mesma condição:
- o pé que inicia a marcha;
- o sentido do movimento (giro e lateral para os dois lados).

**Estatística:** por condição. Sobrevivência por teste do sinal/McNemar;
contínuas por Wilcoxon pareado, com Holm nos contrastes primários.

### Métricas novas (`analisar_omni.py`)

As velocidades comandadas e a configuração são lidas do próprio `_launch.log`
(rosparam do `controller_node`), o que também confere os argumentos.

| métrica | definição |
|---|---|
| `w_real`, `razao_w` | inclinação do yaw verdadeiro desenrolado (3 s até o fim ou a queda); realizado/comandado |
| `vx_b`, `vy_b`, `razao_v` | deslocamento do CoM por passada completa (2 passos, cancela o balanço lateral), girado para o frame do corpo |
| `erro_dir` | direção realizada − comandada no frame do corpo [°] |
| `deriva_m` | no giro no lugar, distância entre o CoM médio da primeira e da última passada |
| `escorr_yaw` | \|taxa de yaw do pé de apoio\| média em apoio simples [°/s] |
| `sep_min_mm` | menor separação lateral entre os pés no frame do corpo. Não há colisão entre as pernas (o URDF não tem `self_collide`); < 0 significa pés cruzados |
| `a_roll` | roll verdadeiro no miolo do apoio simples, + = para o lado da perna em balanço ([plano_fase3_roll.md](plano_fase3_roll.md) §5) |
| `sat_hip_roll/ank_roll/hip_yaw` | % do tempo com \|comando\| ≥ limite |

**Validação** numa corrida em reta conhecida do panorama:
- v_x realizada 0,1006 m/s, contra 2,96 m / 30 s ≈ 0,099 do `walk_test`;
- ω ≈ 0 e v_y ≈ 0;
- roll RMS igual ao do panorama.

### LIPM no frame do pé de apoio (correção)

A telemetria (`WalkTelemetry`) põe a origem do LIPM no pé de apoio, mas
mantém **os eixos do mundo**. Em linha reta dá no mesmo. No giro e na curva,
x e y misturam avanço e lateral, e três coisas deixam de valer:
- a energia orbital (Kajita 2001, eq. 28–29);
- o DCM;
- a comparação com a referência do gerador, que vem no frame do pé.

**A correção gira essas grandezas pelo yaw do pé de apoio**, sem mudar o C++:
- ao vivo, no `viz_marcha.py`, tópico `/marta/viz/lipm`;
- nas figuras, no `figuras_marcha.py` (`FRAME=mundo` reproduz as antigas).

**Validação:**
- o g/z_c recuperado da telemetria dá z_c = 0,2500 m, exatamente o do
  gerador;
- em linha reta, a diferença entre os dois frames é de milímetros (máximo de
  6,8 mm em `sup_x`).

### Obstáculos no campo simulado (achado de método)

O mundo padrão (`world.world`) é o campo SPL da RoboCup 2009. Ele tem **dois
objetos com colisão no caminho da marcha**:
- **O gol em x = +3,0 m.** Traves em y = ±0,7 m (raio 5 cm, 0,8 m de altura),
  travessão e rede (`robocup09_spl_field/model.sdf`).
- **Uma bola grande e leve:** 26 g, raio de colisão 0,12 m.
  - O `world.world` a põe em (0; −0,2), mas o link tem `<pose>2 0 0` no
    `robocup_3Dsim_ball/model.sdf`. **A bola fica de fato em (2,0; −0,2)**,
    com o centro a 0,13 m de altura.
  - O `/gazebo/model_states` informa a pose do modelo, (0; −0,2), e não a do
    link. Por isso ela parecia estar na origem.

A 0,1 m/s, o robô passa por x = 2 m aos ~19 s e chega a x ≈ 3 m, exatamente
no gol, aos 30 s.

**Fatos (telemetria das campanhas):**
- **Todas as corridas de reta e diagonal tombam ao chegar a x ≈ 2,6–3,0 m.**
  Isso inclui as que contam como "completou", porque o tombo vem depois dos
  30 s. Curva, giro e lateral ficam em |x| < 0,6 m e não tombam depois da
  parada.
- **As 8 quedas da reta 0,15** (E1 e E6) acontecem aos 22–24 s, com o CoM em
  x = 2,72–2,83 m.
- **Quatro quedas da diagonal aos 29,7–30,0 s** acontecem com o CoM em
  x = 2,58–2,69 m:
  - E1: M33 e M38 +0,03 pE;
  - E5: M33 sem roll +0,03 pE e M38 com roll +0,03 pD.
- **A bola:** 60 corridas de reta e diagonal passam com o CoM a menos de
  0,3 m do centro dela. Todas completaram, exceto as quedas no gol acima e
  uma: E5, M38 com roll, diagonal −0,03 pE, aos 22,7 s.
- **Nessa corrida**, a marcha era periódica até 21 s (roll ±2–3°). Aos 21,2 s,
  o pé esquerdo desceu e parou 2,6 cm acima do chão, em (2,15; −0,55), e o
  robô tombou de lado a partir daí.

**Diagnóstico** (12 corridas; `experiments/omni/diag/`, `rodar_diag.sh`). São
as mesmas linhas de roslaunch, em dois mundos:
- **W** = `world.world`, com registro da pose dos modelos (`log_modelos.py`);
- **L** = `campo_livre.world`: mesma física, mesmo plano de contato e mesmo
  atrito do campo, sem gol e sem bola.

| condição | campanha original | W (réplica) | L (sem gol nem bola) |
|---|---|---|---|
| M38 roll, diagonal −0,03 pE | caiu 22,7 | caiu 29,7 (x = 2,6: gol) | **ok** |
| M38 roll, diagonal +0,03 pD | caiu 29,7 | – | **ok** |
| M33 sem roll, reta 0,15 pD (E1) | caiu 23,6 | – | **ok** (v_x 0,147) |
| M38 sem roll, reta 0,15 pD (E1) | caiu 22,5 | – | **ok** (v_x 0,154) |
| M33 roll, reta 0,15 pD (E6) | caiu 23,5 | – | **ok** |
| M38 roll, reta 0,15 pD (E6) | caiu 23,0 | – | **ok** |
| M33 roll, giro +0,175 pD (controle) | ok | – | ok (ω 1,002 do comandado) |
| M33 sem roll, giro +0,175 pD | caiu 10,3 | caiu 9,1 | caiu 14,0 |
| M33 sem roll, lateral −0,05 pD | caiu 18,9 | caiu 11,8 | caiu 19,3 |

**Na réplica W, a bola é empurrada** quando o robô passa a ~0,24 m do centro
dela, aos 19,3 s. Ela é lançada a vários m/s e quica pelo campo até 1,9 m de
altura. É um comportamento não físico para uma bola de 26 g com 24 cm de
diâmetro. Nessa réplica, ela não voltou a tocar o robô.

**Leitura:**
- **Fato:** sem gol e sem bola, a reta 0,15 anda os 30 s nas duas massas, com
  e sem roll. **A "janela de velocidade" da reta (E1, fato 2) era o gol, e
  foi retirada.**
- **Inferência forte:** as quedas da diagonal perto dos 30 s são o gol (trave
  em y = ±0,7 m, rede entre elas).
- **Inferência:** a queda da E5 aos 22,7 s é a bola, empurrada e depois
  pisada. A réplica no mesmo mundo não repetiu a queda, e a condição anda sem
  a bola.
- **O que não muda:**
  - giro, curva e lateral não chegam perto do gol nem da bola;
  - as quedas sem roll se repetem sem eles (giro 14,0 s, lateral 19,3 s);
  - a E2 e a E4 não têm queda perto de nenhum dos dois (a reta 0,10 completou
    em todas).

  Os resultados de giro, curva e lateral e o efeito do roll ficam de pé.
- **Panorama (2026-10):** só reta a 0,1 m/s, e quedas depois de 30 s não
  contam. Das 234 quedas antes de 30 s, uma acontece na faixa da bola
  (x 1,6–2,4 m), e com o CoM a 0,56 m do centro dela. O panorama não é
  afetado.
- **Ressalva:** a passagem pela bola aos ~19 s pode perturbar a reta e a
  diagonal (métricas de pitch e roll). Isso não foi medido.

**Recomendação (decisão do usuário):** rodar as campanhas futuras de marcha
no `campo_livre.world` (ou com o robô longe do gol) e corrigir a pose do link
da bola no submódulo `marta_gazebo`. **O mundo padrão não foi alterado.**

## Estabilizador de roll (implementação)

**Arquivos (`marta_control`):** `include/RollStabilizer.h`,
`src/RollStabilizer.cpp`, `test/test_roll_stabilizer.cpp`. Integração no
`controller.cpp`, **depois** do estabilizador de pitch.

**Desenho:** o do [plano_fase3_roll.md](plano_fase3_roll.md) §3 e §4.3:
- Raibert, *Legged Robots That Balance*, cap. 3: pitch e roll como malhas
  independentes;
- Maximo, doutorado, §4.4.2, eq. 4.34–4.35, p. 85: P+D no hip roll e no ankle
  roll, com ganhos separados.

```
φ̄ ← φ̄ + (dt/τ_w)(φ − φ̄)                      washout
u_h = kp_h(φ − φ̄) + kd_h·φ̇,   u_a = kp_a(φ − φ̄) + kd_a·φ̇
hipRoll_X += w_X·sat(u_h),    ankleRoll_X += w_X·sat(u_a)
```

- **Pesos:** w_X é o peso de carga do gerador (`getLoadWeights`): 1 na perna
  de apoio, ~0 na de balanço. A colocação do pé não muda.
- **Sinal:** +Δ restaura nas duas juntas com ganhos positivos.
- **Desligado** por default. Forçado a desligado com `isReal` (os ganhos são
  da simulação).

**Parâmetros** (`stabilizer_test.launch`): `roll_stabilizer_enabled`,
`kp/kd_roll_hip`, `kp/kd_roll_ankle`, `roll_washout_tau` (2 s) e
`max_roll_correction` (0,14 rad).

**Debug:** `/marta/roll_stabilizer`, com os campos enabled, roll, φ̄, erro,
u_h, u_a, w_L, w_R e saturado. Vai para o `_roll.csv` no `bag_para_csv.py` e
para o `registrar_pe.py`.

**Washout:** mantido em 2 s, depois de remedir o viés da estimativa de roll
no regime atual:

| condição | viés | correlação com a verdade |
|---|---|---|
| reta | −0,1 a −0,3° | ≥ 0,985 |
| antes (regime antigo) | +2,3° | – |
| curva | −1,2° | – |

**Testes offline** (`test_roll_stabilizer`): 14 verificações, todas OK:
- sinal;
- termo D;
- a perna em balanço fica intocada;
- divisão no apoio duplo;
- saturação;
- o washout remove o viés constante e preserva 99,3% da alternância a
  0,83 Hz;
- desligado, não toca nas juntas.

**Regressão:** com o roll desligado, o binário novo reproduz o piloto. Houve
uma queda isolada por ruído de temporização, e a mesma condição completou
3/3 em seguida.

## Resultados

### Piloto (E0)

| movimento | M33 (3,3 kg) | M38 (3,8 kg) |
|---|---|---|
| reta 0,1 m/s | completou; v_x 0,103 | completou; v_x 0,103 |
| curva (0,1 m/s; 0,1 rad/s) | completou; ω 0,100 | completou; ω 0,099 |
| giro no lugar 0,175 rad/s | **caiu aos 12,6 s** (para trás) | **caiu aos 4,2 s** |
| lateral 0,05 m/s | **caiu aos 16,3 s** (de lado); v_y realizada 58%, erro de direção −24° | **caiu aos 4,1 s** |

- A curva com 3,4 kg, sem GUI, completou 3/3, com resultados quase
  idênticos.
- **Mecanismo da queda no giro (M33):** a oscilação de roll cresce a cada
  passo, de ±3–4° (3–5 s) para ±7–8° (6–8 s), ~14° (9 s) e 28–44° antes da
  queda (11,5–12 s). Em reta, o roll fica em ~±2°.

  É uma instabilidade lateral que se amplifica no giro: o alvo natural do
  estabilizador de roll. O classificador de direção marca "para trás"
  porque olha o sentido final do tombo; o eixo que passa de 15° primeiro é
  o roll (confirmado nas 20 quedas no giro do M33 da E1).

### E1: linha de base omnidirecional (sem roll, ganhos atuais)

128 corridas, todas com RTF ≥ 0,99:
- 25 movimentos × 2 pés iniciais × 2 massas, com malha de rumo (h1);
- giro e curva sem malha de rumo (h0), só com pé direito, nas 2 massas.

Dados em `experiments/omni/` (tags `E1_*`), métricas em `resumo_omni.csv`.

**Sobrevivência com malha de rumo.** ✓ = andou 30 s; número = queda, em s.
pD = o pé direito dá o primeiro passo (default); pE = o esquerdo.

| movimento | M33 pD | M33 pE | M38 pD | M38 pE |
|---|---|---|---|---|
| reta 0,05 | ✓ | ✓ | 7,1 | 16,3 |
| reta 0,10 | ✓ | ✓ | ✓ | ✓ |
| reta 0,15 | 23,6 (gol) | 23,6 (gol) | 22,5 (gol) | 22,7 (gol) |
| giro +0,10 | ✓ | 13,2 | 4,3 | 5,1 |
| giro −0,10 | 10,9 | 10,8 | 4,8 | 6,5 |
| giro +0,175 | 12,8 | 9,8 | 4,2 | 5,9 |
| giro −0,175 | 8,5 | 18,8 | 4,6 | 6,0 |
| giro ±0,25 | 9,9 / 8,5 | 11,0 / 9,1 | 5,2 / 4,6 | 4,7 / 6,1 |
| giro ±0,35 | 7,9 / 6,4 | 7,7 / 6,8 | 4,1 / 4,7 | 5,6 / 4,7 |
| curva (0,1; +0,10) | ✓ | ✓ | 4,7 | ✓ |
| curva (0,1; −0,10) | ✓ | 6,3 | ✓ | ✓ |
| curva (0,1; +0,175) | 10,3 | ✓ | 4,7 | ✓ |
| curva (0,1; −0,175) | ✓ | 5,7 | ✓ | 4,8 |
| curva (0,1; +0,25) | ✓ | 12,5 | 4,4 | ✓ |
| curva (0,1; −0,25) | ✓ | 6,1 | 7,8 | 4,7 |
| lateral +0,025 | ✓ | ✓ | 6,4 | 7,4 |
| lateral −0,025 | ✓ | ✓ | 7,8 | ✓ |
| lateral ±0,05 | 10,2 / 6,1 | 16,1 / 15,2 | 4,1 / 4,9 | 4,7 / 4,0 |
| diagonal (0,1; +0,03) | ✓ | 29,9 (gol) | ✓ | 29,7 (gol) |
| diagonal (0,1; −0,03) | ✓ | ✓ | 5,4 | ✓ |
| combinado (0,1; +0,03; +0,175) | ✓ | ✓ | ✓ | ✓ |
| combinado (0,1; +0,03; −0,175) | 8,5 | 5,2 | 4,7 | 4,7 |

Figuras:
- [`fig_omni_sobrevivencia_E1.png`](figuras_marcha/fig_omni_sobrevivencia_E1.png):
  a mesma tabela, mais a coluna h0;
- [`fig_omni_omega_E1.png`](figuras_marcha/fig_omni_omega_E1.png): ω
  realizado × comandado.

As duas são geradas por `experiments/fig_omni.py`.

**Fatos**

1. **O mínimo pedido ainda não é atingido.**
   - **Giro no lugar:** 1 de 32 corridas completou (M33, +0,10, pD). A
     sobrevida mediana é de 9,8 s no M33 e 4,7 s no M38.
   - **Andar na direção comandada:** no M33, completam nos dois pés a reta
     0,05 e 0,10, o lateral 0,025 e a diagonal (3 de 4; a 4ª bateu no gol
     aos 29,9 s). No M38, completam nos dois pés só a reta 0,10, a curva
     (0,1; −0,10) e o combinado +0,175 (e a diagonal +0,03, descontado o
     gol).
2. **Reta: ~~janela de velocidade~~ (retirado em 2026-11-12).**
   - As 4 quedas a 0,15 m/s (22–24 s) são colisões com o gol do campo, em
     x ≈ 2,8 m. Sem gol, a reta 0,15 anda os 30 s nas duas massas (ver
     [Obstáculos no campo simulado](#obstáculos-no-campo-simulado-achado-de-método)).
     A v_x realizada é 0,147–0,154.
   - **Continua de pé:** no M38, 0,05 m/s cai (7,1 e 16,3 s, em x < 1 m).
3. **Quando anda, segue o comando.** Medianas das corridas completas:

   | movimento | ω real / comandado | v real / comandada |
   |---|---|---|
   | curva | 1,00–1,01 | 1,02–1,03 |
   | combinado | 1,00–1,01 | 1,04–1,05 |
   | reta | – | 1,03–1,07 |

   **O lateral é a exceção:**
   - a 0,025 m/s, v_y = 0,020 (80%), com um avanço espúrio de v_x ≈ 0,011
     m/s;
   - a direção realizada fica a **~30° da comandada** (|erro_dir| 26–36°).

   A diagonal tem erro de ~5°, pelo mesmo avanço.
4. **No M38, as quedas são na partida.**
   - 43 das 47 quedas do M38 acontecem antes de 8 s; no M33, 11 de 37.
   - O pico de roll nos primeiros passos (1–3 s) é de ~12° no M38 e de ~6,5°
     no M33, inclusive nas corridas que completam.
   - Esse pico **não separa**, sozinho, as corridas que caem das que
     completam: as medianas são 12,0° e 11,7° no M38.
5. **Mecanismo do giro (M33).** Em todas as 20 quedas no giro, o roll
   verdadeiro passa de 15° antes do pitch (critério aplicado depois de 3,5 s).
   - O "para trás" do classificador de direção é o sentido final do tombo.
   - Isso confirma a leitura do piloto.
   - No M38, o primeiro eixo é o roll em 14 quedas e o pitch em 10: mistura
     o transitório de partida e o mecanismo de roll.
6. **Pé inicial × sentido da curva** (análise exploratória, não
   pré-registrada). Na curva, o desfecho depende de qual pé dá o primeiro
   passo em relação ao sentido do giro.
   - **Concordante:** o pé do lado para onde se gira começa (pE com ω > 0, pD
     com ω < 0).
   - Em 12 pares (massa × sentido × velocidade), só o concordante completou em
     8, e só o discordante em 1. Teste do sinal: p = 0,039.
   - A sobrevida é maior no concordante em 9 de 10 pares que diferem (p =
     0,021).
   - No giro, no lateral e no combinado, não há efeito (p ≥ 0,55).
   - **Leitura (inferência):** depois da partida, a marcha é periódica e
     igual nos dois casos. A diferença fica, portanto, nos primeiros passos:
     o primeiro passo do giro "abre" ou "fecha" o quadril, conforme o pé.
7. **A malha de rumo ajuda a curva; no giro, o efeito é inconclusivo.**
   - Na curva do M33, ela leva ω de 0,79–0,87 do comandado (h0) a 0,99–1,01
     (h1, corridas completas), e completou 5/6 contra 3/6 (pD; 3 pares
     diferem, os 3 a favor de h1, p = 0,25). No M38 a curva completa 3/6 sem a
     malha e 2/6 com ela (pD; 2 pares diferem, 1 para cada lado), com ω
     0,88–1,01 sem a malha.
   - **Giro: o efeito não é distinguível do ruído.** Na E1, o M33 sem a malha
     sobreviveu mais nos 7 pares que diferem (p = 0,016 pelo sinal): 14,4
     contra 12,8 s e 13,1 contra 8,5 s.
     - As réplicas exatas da E2 mostram que essa diferença está dentro do
       ruído entre corridas idênticas. O giro +0,175 do M33, com a malha e os
       ganhos atuais, durou 6,8; 6,9; 12,6; 12,8 e 17,6 s em cinco corridas
       (piloto, regressão, E1, E2p, E2h).
     - Com todas as sintonias da malha da E2h, o giro ±0,175 durou
       9,8–25,8 s: uma faixa que contém os valores sem malha (13,1 e
       14,4 s).
     - **Retirado como achado.**
8. **Margens que não limitam.**
   - Saturação de hip_roll, ank_roll e hip_yaw: 0% em todas.
   - Os pés nunca se cruzaram: separação lateral mínima ≥ 38 mm, ~100 mm nas
     corridas completas.
   - O escorregamento de yaw do pé de apoio é de 0,1–3 °/s nas completas, e
     chega a 5–12 °/s nas que caem (giro e lateral).

### E1b: partida suave e `hip_offset_x` (exploratória, fora do plano)

Os fatos 4 e 6 da E1 apontam para a partida. O lento que cai para trás (reta
0,05 no M38) aponta para o equilíbrio sagital sem avanço.

**Desenho:** 44 corridas, nas 11 condições que caíram (7 no M38, 4 no M33):
- partida suave já existente (`start_ramp_steps` 4 e 8; ver
  [desvio_de_direcao.md](desvio_de_direcao.md) §5);
- `hip_offset_x` +5 mm (M33: 8,6 mm; M38: 14 mm);
- as duas coisas juntas.

| condição | E1 (base) | rampa 4 | rampa 8 | x +5 mm | rampa 4 + x +5 |
|---|---|---|---|---|---|
| M38 giro +0,175 | 4,2 | inválida | 5,6 | 7,4 | inválida |
| M38 giro −0,175 | 4,6 | inválida | 5,6 | **15,6** | inválida |
| M38 giro +0,10 | 4,3 | inválida | 5,6 | **ok** | inválida |
| M38 lateral +0,025 | 6,4 | inválida | 5,7 | **ok** | inválida |
| M38 lateral −0,025 | 7,8 | inválida | 5,5 | **ok** | inválida |
| M38 reta 0,05 | 7,1 | inválida | 5,5 | 6,8 | inválida |
| M38 curva +0,175 (pD) | 4,7 | inválida | 5,5 | 4,6 | inválida |
| M33 giro +0,175 | 12,8 | inválida | 5,7 | 10,7 | inválida |
| M33 giro −0,175 | 8,5 | inválida | 5,7 | 12,2 | inválida |
| M33 giro +0,10 (pE) | 13,2 | inválida | 5,7 | 13,4 | 4,3 |
| M33 lateral +0,05 | 10,2 | inválida | 5,6 | **ok** | inválida |

**A partida suave é prejudicial neste regime.**
- **Rampa de 4 passos:** o robô cai antes de a marcha começar, nos passos no
  lugar que o `requestStop()` dá depois do agachamento. O pitch já está em
  −75° no início da gravação, e o RTF registrado é 0. As corridas são
  inválidas, não quedas na marcha.
- **Rampa de 8 passos:** as 11 condições caem aos 5,5–5,7 s, nas duas
  massas. Os primeiros passos são mais suaves que na E1 (roll ±1–7° contra
  ±6–16°). Depois o roll cresce até 20–40° durante um apoio simples, com o
  CoM a 0,10–0,14 m do pé de apoio, e o robô tomba para o lado da perna em
  balanço.
- **Mecanismo (inferência pelo código):** a rampa escala pelo mesmo fator o
  balanço lateral do CoM e a altura do passo
  (`OmnidirectionalWalkZMP.cpp`, `startRamp`). No meio da rampa, o pé sai do
  chão com o CoM ainda longe do pé de apoio. Com a massa atual, isso tomba o
  robô. No estudo do desvio de direção (2026-10) a rampa não derrubava.
- **Ficou fora** da E4 e da E5.

**`hip_offset_x` +5 mm ajuda o que cai na partida.**
- Passaram a completar 4 das 11 condições: M38 giro +0,10, lateral ±0,025 e
  M33 lateral +0,05. Nenhuma condição piorou de desfecho.
- **No M38, as quedas da E1 se repetem com precisão** (4,1–5,3 s em todas as
  réplicas da E2). Por isso o ganho de sobrevida no giro é real: de 4,6 para
  15,6 s no −0,175, e de 4,2 para 7,4 s no +0,175.
- **No M33, o giro continua caindo**, com a sobrevida dentro do ruído.
- **Ressalva:** no panorama, `hip_offset_x` grande derrubava de lado em reta
  (44 de 52 quedas laterais com x ≥ 9 mm). A reta com x +5 mm não foi
  testada aqui.
- **Candidato a confirmar:** a grade da E1 com x +5 mm.

### E2: sintonia de pitch e de rumo

132 corridas, pé direito, sem roll. Análise em `experiments/analisar_e2.py`.

**Desenho:**
- **Pitch:** kp ∈ {0,1; 0,2; 0,35} × kd ∈ {0,015; 0,03; 0,06}, em 8 condições:
  reta 0,10, giro +0,175, curva (0,1; +0,175) e lateral +0,05, × 2 massas.
- **Rumo:** kp_heading ∈ {0,5; 1,0; 2,0} × saturação ∈ {0,3; 0,5} rad/s, em 10
  condições: reta 0,10, giro ±0,175, curva +0,175 e combinado +0,175, × 2
  massas.

**Ruído entre corridas idênticas (fato de método).** E1 pD h1, E2p atual e
E2h atual são a mesma configuração (conferido no `_launch.log`).

| condição | réplicas |
|---|---|
| reta 0,10, combinado +0,175 (M33 e M38) | todas completam |
| M38 giro, curva +0,175 e lateral | todas caem aos 4,1–5,3 s |
| M33 giro +0,175 | 6,8 / 6,9 / 12,6 / 12,8 / 17,6 s |
| M33 giro −0,175 | 8,5 / 20,8 s |
| M33 curva +0,175 | 10,3 / 6,2 / 5,9 s |
| M33 lateral +0,05 | 10,2 / 17,5 s |

- **O desfecho binário se repete.** Os extremos são "anda" e "cai na
  partida".
- **O instante da queda nas condições de borda não se repete.** Varia por um
  fator de ~2–3.
- **Consequência:** sobrevida de **uma** corrida nessas condições não serve
  para comparar braços. Só servem diferenças de desfecho, ou padrões que se
  repetem em várias condições.

**Veredito pelo critério pré-registrado: nenhuma combinação supera a atual.**
Nas duas famílias, o Holm deu ≥ 0,6 na sobrevivência e na métrica
secundária.

| pitch (kp, kd) | completou (de 8) | melhor / pior que a atual (pares) |
|---|---|---|
| **0,2; 0,03 (atual)** | **2** | – |
| 0,1; 0,015 | 2 | 4 / 1 |
| 0,1; 0,03 | 3 | 2 / 3 |
| 0,1; 0,06 | 3 | 2 / 2 |
| 0,2; 0,015 | 2 | 2 / 2 |
| 0,2; 0,06 | 4 | 3 / 0 |
| 0,35; 0,015 | 2 | 2 / 1 |
| 0,35; 0,03 | 3 | 2 / 1 |
| 0,35; 0,06 | 5 | 4 / 1 |

O teste é fraco. Com 8 condições e só 2 pares em que as duas completam (as
retas), a métrica contínua não tem poder, e a sobrevida é ruidosa, como
mostrado acima.

**Padrão exploratório (não pré-registrado; candidato a confirmar): kd = 0,06
ajuda fora da reta.**
- Fora da reta, completaram:
  - kd 0,06: 6 de 18;
  - kd 0,015 e 0,03: 2 de 36.
  - Fisher: p = 0,012.
- O efeito está no M33:
  - lateral +0,05: 3/3 com kd 0,06, 0/6 com os outros;
  - curva +0,175: 2/3 contra 0/6.
- **No giro, nenhum ganho de pitch ajuda:** 0/18 nas duas massas.
- Na reta, kp 0,35 / kd 0,06 tem o menor pitch RMS (1,61° e 1,34°, contra
  1,80° e 2,15° da atual). São só 2 pares.
- **Leitura (inferência):** mais amortecimento de pitch segura os movimentos
  laterais. Não resolve o giro, que cai por roll (E1, fato 5).

**Rumo: os ganhos atuais estão bons para a faixa testada.**
- Nenhuma combinação completou mais que a atual: 4/10 em todas.
- kp 0,5 com saturação 0,5 foi pior em 4 pares e melhor em nenhum (p =
  0,125).
- O erro de rumo verdadeiro (RMS, de 3 s ao fim) nas corridas completas fica
  em 1,2–8,6° em todas as combinações, sem diferença (Wilcoxon, Holm ≥ 0,6).
- **O termo integral condicional não é acionado.** O erro de regime não
  passa do limiar de ~10°.

**Decisão do usuário:**
- **Rumo:** manter kp_heading 1,0 e saturação 0,3.
- **Pitch:** manter 0,2 / 0,03, como manda o critério pré-registrado, ou
  adotar kd 0,06 (com kp 0,2 ou 0,35), pelo padrão exploratório.

A E4 e a E5 rodam com os ganhos atuais. Se der tempo, a E2c confirma o kd 0,06
na grade da E1.
### E4: piloto do roll

**E4a:** M33, pé direito, 40 corridas (`desenho_E4a.csv`, análise em
`experiments/analisar_e4.py`). Cada configuração tem ganho total K e
kd = 0,15·K por junta:
- **cint** (cintura, Raibert): só quadril, kp_h = K;
- **quatorn** (Maximo): quadril e tornozelo, K/2 em cada;
- **torn:** só tornozelo, kp_a = K;
- **soD:** só derivativo, kd = 0,05 nas duas juntas.

O braço desligado ("off") é a E1 na mesma condição.

Cada célula: desfecho (ok = 30 s, ou o instante da queda) / roll RMS / pitch
RMS, em graus. Nas que caíram, as métricas vão até a queda.

| configuração | giro +0,175 | lateral +0,05 | lateral −0,05 | reta 0,10 | completou |
|---|---|---|---|---|---|
| **off (E1)** | 12,8 / 8,3 / 7,8 | 10,2 / 13,2 / 7,3 | 6,1 / – / 15,3 | ok / 2,1 / 2,2 | **1/4** |
| cint K0,25 | ok / 2,5 / 3,7 | ok / 2,0 / 3,5 | ok / 2,1 / 3,5 | ok / 1,6 / 2,4 | 4/4 |
| cint K0,5 | ok / 2,3 / 3,8 | 6,6 / 10,4 / 10,5 | ok / 1,7 / 3,8 | ok / 1,3 / 2,3 | 3/4 |
| cint K1 | ok / 1,6 / 3,5 | ok / 1,4 / 3,9 | ok / 1,4 / 3,9 | ok / 1,3 / 2,7 | 4/4 |
| quatorn K0,25 | ok / 2,1 / 3,9 | ok / 1,8 / 3,5 | ok / 1,8 / 3,5 | ok / 1,4 / 2,4 | 4/4 |
| quatorn K0,5 | ok / 1,8 / 3,8 | ok / 1,5 / 3,8 | ok / 1,6 / 3,5 | ok / 1,5 / 2,3 | 4/4 |
| quatorn K1 | ok / 1,4 / 3,7 | ok / 1,4 / 3,7 | ok / 1,5 / 3,6 | ok / 1,3 / 2,4 | 4/4 |
| torn K0,25 | ok / 1,9 / 4,0 | ok / 1,6 / 3,7 | ok / 1,8 / 3,6 | ok / 1,8 / 1,9 | 4/4 |
| torn K0,5 | ok / 1,7 / 3,7 | ok / 1,8 / 3,3 | ok / 2,0 / 3,4 | ok / 1,4 / 2,0 | 4/4 |
| torn K1 | ok / 1,4 / 3,5 | ok / 2,2 / 2,9 | ok / 1,8 / 2,8 | ok / 1,1 / 2,0 | 4/4 |
| soD kd 0,05 | 18,4 / 5,3 / 8,0 | 5,5 / – / – | ok / 4,2 / 3,3 | ok / 1,5 / 2,4 | 2/4 |

**Fatos**
1. **Com termo proporcional, o roll resolve o giro e o lateral no M33.**
   - Das 9 configurações P+D, 35 de 36 corridas completaram. Sem roll, 1 de
     4. A exceção é cint K0,5 no lateral +0,05 (6,6 s).
   - **Giro:** ω realizado entre 0,997 e 1,004 do comandado nas 9
     configurações. O escorregamento do pé de apoio fica em 0,15–0,95 °/s,
     contra 2,9 °/s sem roll.
   - **Lateral ±0,05:** v_y entre 91% e 114% do comando. O erro de direção
     fica em ±11–15°, contra −36° sem roll.
   - **O avanço espúrio de ~0,011 m/s continua no lateral** (E1, fato 3).
     Ele não vem do roll.
2. **O roll RMS cai:**
   - giro: de 8,3° para 1,4–2,5° (−70 a −83%);
   - reta: de 2,1° para 1,1–1,8°.
   - A_roll em reta já era 0,3° sem roll: não há o que reduzir ali.
3. **Só derivativo não basta:** 2/4. O que segura é o termo proporcional
   sobre φ − φ̄.
4. **Margens:**
   - saturação da correção (0,14 rad): 0% em todas;
   - saturação de torque: ank_roll ≤ 0,2% e hip_roll 0%.
5. **Pitch RMS na reta** (portão ≤ +10% sobre 2,2°):
   - passam quatorn (2,3–2,4°), torn (1,9–2,0°), cint K0,25 e cint K0,5;
   - cint K1 tem +23% (2,7°).
6. **Pé plano: não medido diretamente.** A telemetria não tem o roll do pé.
   Indício indireto: o ank_roll não satura.

**As 3 melhores para o M38** passam no portão de pitch e têm o menor roll RMS
médio nas 4 condições:
- quatorn K1: 1,40°;
- quatorn K0,5: 1,60°;
- torn K1: 1,63°.

**E4b:** essas 3 configurações no M38, nos mesmos 4 movimentos (12 corridas).

| configuração (M38) | giro +0,175 | lateral +0,05 | lateral −0,05 | reta 0,10 | completou |
|---|---|---|---|---|---|
| **off (E1)** | 4,2 | 4,1 | 4,9 | ok / 2,7 / 2,1 | **1/4** |
| quatorn K0,5 | ok / 2,1 / 3,4 | ok / 2,0 / 3,3 | ok / 2,1 / 3,0 | ok / 1,3 / 1,9 | 4/4 |
| quatorn K1 | ok / 1,5 / 3,4 | ok / 1,5 / 2,9 | ok / 2,0 / 3,2 | ok / 1,0 / 1,9 | 4/4 |
| torn K1 | ok / 1,5 / 3,0 | ok / 1,6 / 1,6 | ok / 2,0 / 2,2 | ok / 1,1 / 2,0 | 4/4 |

- **No M38, o roll elimina a queda na partida nesses movimentos:** 12/12
  contra 1/4. As quedas sem roll se repetem com precisão (4,1–5,3 s em
  todas as réplicas da E1 e da E2), então a diferença não é ruído.
- **Na reta, o pitch RMS não piora** (1,9–2,0° contra 2,1°), e o roll RMS
  cai de 2,7° para 1,0–1,3°.

**Configuração escolhida para a E5: quatorn K1.** São kp 0,5 e kd 0,075 no
quadril e no tornozelo, ou seja, K/2 em cada junta, o desenho do Maximo.
- **Por quê:** é a de menor roll RMS médio nas 8 condições das duas massas
  (1,45°). torn K1 tem 1,59° e quatorn K0,5 tem 1,74°.
- **Portão:** passa no de pitch nas duas massas (M33 +9%, M38 −10%).
- **torn K1 tem o pitch um pouco menor:** é a alternativa, se o pitch pesar
  mais.

Comando: `roll_stabilizer_enabled:=true kp_roll_hip:=0.5 kd_roll_hip:=0.075
kp_roll_ankle:=0.5 kd_roll_ankle:=0.075`.

### E5: roll na marcha omnidirecional (confirmatória)

**Desenho:** 88 corridas (`desenho_E5.csv`, semente 20261119). Análise em
`experiments/analisar_e5.py`.
- 11 movimentos: giro ±0,175 e ±0,35; curva (0,1; ±0,175); reta 0,10;
  lateral ±0,05; diagonal (0,1; ±0,03).
- × 2 pés iniciais × 2 massas × {sem roll, quatorn K1}.
- **Os dois braços foram intercalados** na mesma campanha, em ordem
  embaralhada.
- Ganhos de pitch e de rumo atuais. RTF 0,999 em todas.

**Par** = (massa, movimento, pé inicial): são 44 pares.

Figuras:
- [`fig_omni_sobrevivencia_E5.png`](figuras_marcha/fig_omni_sobrevivencia_E5.png);
- [`fig_omni_roll_E5.png`](figuras_marcha/fig_omni_roll_E5.png): roll RMS
  sem × com roll, um ponto por par.

**Completaram** (de 4 por célula; reta, de 2):

| movimento | M33 sem roll | M33 com roll | M38 sem roll | M38 com roll |
|---|---|---|---|---|
| reta 0,10 | 2/2 | 2/2 | 2/2 | 2/2 |
| giro ±0,175 | 0/4 | **4/4** | 0/4 | **4/4** |
| giro ±0,35 | 0/4 | **4/4** | 0/4 | **4/4** |
| curva (0,1; ±0,175) | 2/4 | **4/4** | 2/4 | **4/4** |
| lateral ±0,05 | 0/4 | **4/4** | 0/4 | **4/4** |
| diagonal (0,1; ±0,03) | 3/4 (a 4ª: gol) | 4/4 | 2/4 | 2/4 (gol; bola) |
| **total** | **7/22** | **22/22** | **6/22** | **20/22** |

**Contrastes primários (pré-registrados; Holm sobre os 3):**

| contraste | resultado | p | Holm |
|---|---|---|---|
| 1. sobrevivência (sinal sobre os pares discordantes) | só com roll: 31; só sem: 2; ambos: 11; nenhum: 0 | 1,3·10⁻⁷ | 3,9·10⁻⁷ |
| 2. A_roll (Wilcoxon, 24 pares com a métrica nos dois braços) | mediana com − sem: −0,74° | 0,037 | 0,037 |
| 3. seguimento: \|ω/ω_cmd − 1\| (10 pares) e \|erro_dir\| (18 pares) | −0,027 e −1,18° | 0,0039 e 0,00053 (pior: 0,0039) | 0,0078 |

Secundária: o roll RMS cai em mediana 1,99° (24 pares, p = 1,6·10⁻⁵).

**Os três contrastes passam.** O A_roll tem efeito pequeno porque, nos pares
em que a métrica existe nos dois braços, o braço sem roll sobreviveu pelo
menos 3 s. Isso concentra os pares em reta, curva e diagonal, onde o A_roll
sem roll já era baixo. O roll RMS mostra melhor a diferença:
- sem roll, 1,7–18,0°;
- com roll, 1,0–2,3° nas corridas completas.

**Sensibilidade aos obstáculos.** Os 2 pares "só sem roll" são as duas
diagonais do M38 cujas quedas com roll são o gol (29,7 s) e, por inferência,
a bola (22,7 s); as duas andam sem gol nem bola (diagnóstico). Sem os 8
pares de diagonal, ficam 28 pares só com roll e 0 só sem roll (p = 7·10⁻⁹).

**Fatos, braço com roll:**
1. **Giro no lugar** (±0,175 e ±0,35; 16 corridas, todas completas):
   - ω realizado entre 0,993 e 1,010 do comandado;
   - escorregamento do pé de apoio de 0,02–0,97 °/s;
   - deriva do tronco de 0,08–0,15 m em 30 s.
2. **A deriva no giro vem do avanço espúrio do gerador** (inferência
   numérica). No giro, v_x no frame do corpo é 0,012–0,017 m/s. Um avanço v
   constante no frame do corpo, girando a ω, descreve um círculo de diâmetro
   2v/ω:
   - a 0,175 rad/s, ~14 cm;
   - a 0,35 rad/s, ~8 cm.

   As derivas medidas são 0,12–0,15 m e 0,08–0,10 m, respectivamente.
3. **Curva:** ω de 0,990 a 1,010 do comandado; v de 1,04 a 1,06.
4. **Lateral ±0,05:**
   - v_y de 96–101% do comando no M33 e de 92–93% no M38;
   - erro de direção de ±12–14° (M33) e ±18–19° (M38), pelo avanço espúrio
     de 0,010–0,015 m/s.
5. **Diagonal:** direção realizada a ±2–4° da comandada (16,7°). **Reta:** v
   de 1,03 a 1,06 do comando, erro de direção ≤ 0,5°.
6. **Margens:**
   - saturação de hip_roll ≤ 0,24% do tempo; ank_roll e hip_yaw, 0%;
   - separação mínima entre os pés ≥ 99 mm nas completas.
7. **Pitch RMS na reta 0,10** (portão da E4: ≤ +10%):
   - M33: 2,31 e 2,38° com roll, contra 2,10 e 2,00° sem (+10% e +19%);
   - M38: 1,99 e 1,88° contra 1,96 e 1,79° (+2% e +5%).

   **O M33 passa do portão em um dos pés.** Na E4, torn K1 teve 2,0° na
   mesma reta (uma corrida).
8. **O roll não corrige o avanço espúrio do lateral.** Ele já estava na E1 e
   vem do gerador (ver E6env).

### E6env: envelope com roll (56 corridas)

**Desenho:** quatorn K1 nas velocidades da grade da E1 que a E5 não cobriu,
× 2 pés × 2 massas (`desenho_E6env.csv`, semente 20261120):
- reta 0,05 e 0,15;
- giro ±0,10 e ±0,25;
- curva (0,1; ±0,10) e (0,1; ±0,25);
- lateral ±0,025;
- combinado (0,1; 0,03; ±0,175).

O braço sem roll é a E1.

| movimento | M33 com roll | M38 com roll | E1 (sem roll), M33 / M38 |
|---|---|---|---|
| reta 0,05 | 2/2 | 2/2 | 2/2 / 0/2 |
| reta 0,15 | 0/2 (gol) | 0/2 (gol) | 0/2 / 0/2 (gol). **Sem gol (pD): ok com e sem roll, nas duas massas** |
| giro ±0,10 | 4/4 | 4/4 | 1/4 / 0/4 |
| giro ±0,25 | 4/4 | 4/4 | 0/4 / 0/4 |
| curva (0,1; ±0,10) | 4/4 | 4/4 | 3/4 / 3/4 |
| curva (0,1; ±0,25) | 4/4 | 4/4 | 2/4 / 1/4 |
| lateral ±0,025 | 4/4 | 4/4 | 4/4 / 1/4 |
| combinado (0,1; 0,03; ±0,175) | 4/4 | 3/4 | 2/4 / 2/4 |

**Fatos:**
- **Fora do gol, completou 51 de 52.** A exceção é o M38, combinado −0,175
  pE: queda na partida, aos 4,8 s. Na E1, esse combinado caiu nas 4
  corridas, nas duas massas.
- **ω realizado:**
  - giro, 0,995–1,016 do comandado;
  - curva, 0,988–1,022;
  - combinado, 0,984–1,018.
- **Lateral ±0,025:** anda, mas na direção errada. O erro de direção é de
  26–34°, como na E1, porque o avanço espúrio de v_x (0,010–0,014 m/s) é
  metade do v_y comandado (0,021 m/s). **O estabilizador não muda isso.**
  Corrigir é trabalho no gerador.
- **Reta 0,05:** v_x de 0,057–0,059 m/s, 14–19% acima do comando.
- **Saturação de hip_roll e ank_roll:** 0% em todas.

**Envelope com roll quatorn K1** (andou 30 s nos dois pés iniciais, juntando
E4, E5, E6env e o diagnóstico sem gol):

| movimento | faixa testada | M33 | M38 |
|---|---|---|---|
| reta vx [m/s] | 0,05–0,15 | 0,05–0,10; 0,15 ok sem gol (só pD) | 0,05–0,10; 0,15 ok sem gol (só pD) |
| giro ω [rad/s] | ±0,10 a ±0,35 | toda | toda |
| curva (0,1; ω) | ±0,10 a ±0,25 | toda | toda |
| lateral vy [m/s] | ±0,025 e ±0,05 | toda (erro de direção 12–29°) | toda (erro 18–34°) |
| diagonal (0,1; vy) | ±0,03 | toda | toda, descontados gol e bola |
| combinado (0,1; 0,03; ω) | ±0,175 | toda | +0,175; −0,175 1 de 2 |

A tabela de consulta está em
[consulta_parametros.md](consulta_parametros.md) §"Que velocidades usar".

### E7: kd de pitch e `hip_offset_x` com roll (fatorial, campo livre)

**Estado: concluída em 2026-11-13.** Veredito: **adotar kd 0,06** (kp
0,2) e **manter o `hip_offset_x` base.** Resultados no fim desta seção.

O pré-registro foi escrito em 2026-11-12, antes da primeira corrida.

**Execução:**
- Pausada às 11h40, com 23 de 176 corridas, para recompilar o controlador
  com o teleop; retomada às 11h57. Com o teleop desligado, o binário novo
  reproduz duas corridas da E7 (ver "Teleop", regressão).
- Pausada de novo a pedido do usuário, com 154 corridas, e concluída em
  2026-11-13.
- **Uma corrida refeita:** `E7_M33_kd06x5_lat+0.05_pD` levou 48 min de
  relógio para 32 s de simulação (RTF 0,011, marcada como limitada por CPU).
  - A linha original está em `omni/E7_descartadas.csv` e os dados em
    `omni/dados/descartadas/`.
  - A repetição deu o mesmo desfecho, com RTF 0,999.
  - As outras 175 têm RTF ≥ 0,92.

**Motivo.** Decisões do usuário em 2026-11-12:
- roll **quatorn K1** adotado;
- pitch: "manter o que ficou melhor";
- `hip_offset_x` +5 mm: "então teste".

O kd 0,06 só tinha um padrão exploratório da E2, medido **sem** roll. O x
+5 mm só foi visto sem roll (E1b). Com o roll adotado, os dois precisam ser
medidos **com** roll.

**Desenho:** `omni/desenho_E7.csv`, 176 corridas intercaladas (semente
20261124).
- Grade e pés iniciais iguais aos da E5: 11 movimentos × 2 pés × 2 massas = 44
  pares.
- Roll quatorn K1 em todas.
- **Mundo:** `campo_livre.world`, sem gol nem bola.
- **Braços:**

  | braço | kd de pitch | `hip_offset_x` |
  |---|---|---|
  | `ref` | 0,03 | M33 3,6 mm; M38 9 mm |
  | `kd06` | 0,06 | base |
  | `x5` | 0,03 | M33 8,6 mm; M38 14 mm |
  | `kd06x5` | 0,06 | +5 mm |

**Critério pré-registrado.** Cada braço é comparado com `ref`, pareado por
(massa, movimento, pé).
- **Primárias, com Holm sobre 3 braços × 2 métricas = 6 testes:**
  1. sobrevivência: teste do sinal nos pares discordantes;
  2. `pitch_rms` (pitch verdadeiro, RMS em torno de zero, a métrica do custo
     do roll na E5): Wilcoxon nos pares em que os dois completam.
- **Vetos**, descritivos, por mediana das diferenças pareadas:
  - roll RMS mais de 10% acima de `ref`;
  - |erro de direção| pior em mais de 2°;
  - |ω/ω_cmd − 1| pior em mais de 0,02;
  - separação mínima dos pés abaixo de 90 mm em alguma corrida.
- **Secundária:** a oscilação de pitch (desvio-padrão, sem a média), porque
  o x +5 mm pode deslocar só a média.
- **Regra de adoção:** um braço é adotado se cumprir as três condições:
  - não completa menos corridas que `ref`;
  - melhora uma primária com Holm p < 0,05;
  - não aciona veto.

  Se mais de um passar, fica o de menor `pitch_rms`. **Se nenhum passar,
  fica `ref`** (kd 0,03, x base): é o que "ficou melhor" pelo critério.

**Resultados** (`analisar_e7.py`):

| braço | completou | pitch RMS: mediana (braço − ref) | Holm | pares melhores que ref | vetos | veredito |
|---|---|---|---|---|---|---|
| `ref` (kd 0,03, x base) | 44/44 | – (ref: 3,06°) | – | – | – | – |
| **`kd06`** | 44/44 | **−0,31°** (−10%) | 4·10⁻¹⁰ | **41/44** | nenhum | **passa** |
| `x5` | 44/44 | +1,31° | 7·10⁻¹³ (pior) | 0/44 | nenhum | não passa |
| `kd06x5` | 44/44 | +0,96° | 1·10⁻⁶ (pior) | 8/44 | nenhum | não passa |

**Fatos:**
- **A sobrevivência fica no teto:** 44/44 em todos os braços, no campo livre
  e com roll. Nenhum braço muda o desfecho.
- **kd 0,06:**
  - reduz o pitch RMS em todos os grupos de massa × movimento (mediana por
    grupo de −0,07° a −0,89°);
  - o ganho é maior no giro (M33 −0,72°, M38 −0,51°) e no lateral (M33
    −0,89°, M38 −0,58°);
  - na reta, o ganho é menor: M33 2,26/2,03 → 2,10/1,98°; M38 1,91/1,82 →
    1,57/1,87°;
  - a oscilação de pitch cai 0,34° (p = 1·10⁻¹³);
  - a média do pitch não muda (+0,10°);
  - o roll RMS cai 3%;
  - a fração de saturação do estabilizador de pitch é 0 nos dois braços.
- **x +5 mm:**
  - piora o pitch RMS em todos os 44 pares, porque **inclina o tronco +1,57°
    para frente** (mediana do pitch médio; ref 1,24°);
  - a oscilação não muda (−0,02°, p = 0,29);
  - o benefício visto sem roll na E1b (menos quedas na partida) não tem o
    que melhorar com roll, já que a sobrevivência está no teto.
- **kd06x5:** o kd reduz a oscilação (−0,42°), mas a inclinação do x domina
  o RMS.

**Leitura (inferência):**
- com o roll ligado, o amortecimento de pitch maior compensa parte do custo
  de pitch que o roll trouxe (E5: +10–19% na reta M33);
- o ganho é maior onde o roll mais trabalha, no giro e no lateral.

**Decisão** (regra do usuário, "manter o que ficou melhor", aplicada pelo
critério):
- **kd de pitch 0,06**, com kp 0,2;
- `hip_offset_x` base (M33 3,6 mm; M38 9 mm).

**Implementação da decisão:**
- O default do launch continua kd 0,03, para que as campanhas E0–E7 sigam
  reprodutíveis pelos comandos registrados.
- Os comandos recomendados passam a levar `kd_pitch:=0.06`: README §0,
  `consulta_parametros`, roteiro da reunião e teleop.

### LIPM (Kajita) por movimento

As quatro figuras do `figuras_marcha.py`, no frame do pé de apoio, para as
corridas da demo, estão em `figuras_marcha/lipm_omni/<movimento>/`:
- `fig_paper5_com_tempo`: fig. 5 do artigo, CoM medido × planejado;
- `fig_paper4_plano_xy`: fig. 4, traço do CoM e as pegadas;
- `fig_energia_orbital`: eq. 28–29;
- `fig_plano_fase`: eq. 40.

Os movimentos são reta (M38), giro com e sem roll (M33), lateral (M33) e
curva (M33).

**Medidas no apoio simples**, de 5 s ao fim (ou até 0,5 s antes da queda):
- erro do CoM em relação à referência do gerador (RMS);
- deriva da energia orbital dentro de cada fase (média do pico a pico). Sob
  um LIPM perfeito, seria zero.

| corrida | fases | erro x [mm] | erro y [mm] | deriva E_x [m²/s²] | deriva E_y [m²/s²] |
|---|---|---|---|---|---|
| reta 0,10, M38, roll | 41 | 5,1 | 4,2 | 0,0125 | 0,020 |
| curva +0,175, M33, roll | 41 | 7,2 | 4,6 | 0,0115 | 0,020 |
| diagonal +0,03, M33, roll | 41 | 7,2 | 4,5 | 0,0088 | 0,034 |
| giro +0,175, M33, roll | 41 | 12,3 | 7,0 | 0,0129 | 0,025 |
| giro +0,175, M38, roll | 41 | 10,0 | 8,1 | 0,0114 | 0,031 |
| lateral +0,05, M33, roll | 41 | 11,7 | 9,9 | 0,0117 | 0,062 |
| lateral +0,05, M38, roll | 41 | 8,8 | 8,6 | 0,0090 | 0,059 |
| giro +0,175, M33, **sem roll** (até a queda) | 10 | 25,2 | 42,1 | 0,0147 | 0,085 |

**Fatos:**
- **Reta, curva e diagonal seguem o LIPM do gerador mais de perto:** o CoM
  fica a ~4–7 mm da referência.
- **No giro com roll, o erro dobra** (7–12 mm). Na fig. 5, o x medido oscila
  de −2 a +1 cm onde o plano é ~0.
- **No lateral, a deriva de E_y é ~3× a da reta.**
- **Sem roll, o giro se afasta do modelo:** 25 e 42 mm de erro, e deriva de
  E_y ~4× a da reta, antes de cair.
- **A deriva de E_x é parecida em todos** (0,009–0,015).

**Leitura (inferência):**
- o modelo continua descrevendo o CoM nos movimentos novos, com erro maior
  onde há rotação ou passo lateral;
- o roll é o que mantém o giro perto do modelo.

O RMS do erro é uma medida desta análise, não do artigo.

## Teleop (2026-11-12)

**Pedido do usuário:** dirigir a Marta pelo teclado em M34 (3,4 kg), com
roll. O comando, as teclas e os limites estão no README §5, "Teleop pelo
teclado".

**Implementação:**
- **`controller_node`:** com `cmd_vel_enabled` (desligado por padrão), a
  velocidade vem de `/marta/cmd_vel` e não dos `walk_velocity_*`.
  - Watchdog: sem mensagem por 0,5 s, o comando vai a zero.
  - Limites: |vx| ≤ 0,15, |vy| ≤ 0,05, |ω| ≤ 0,35.
  - O gerador continua lendo a velocidade a cada passo (0,6 s).
- **`scripts/teleop_marta.py`:** janela Tk que registra quando cada tecla é
  apertada e solta. Um filtro de 80 ms descarta os pares falsos do
  auto-repeat do X11. Só gera comandos do envelope testado.

**M34 é interpolação.** A postura vem da calculadora:
- `torso_mass_scale` 0,534;
- `z_com` 0,239;
- `hip_offset_x` 0,0036.

Antes desta validação, o nível nunca tinha sido testado com roll.

**Validação** (`experiments/omni/teleop/`, campo livre):
- `sequencia_cmd.py` reproduz em `/marta/cmd_vel` uma sequência de 90 s com
  as trocas que o teclado pode gerar: reta → curva+ → reta → giro+ → giro−
  (inversão direta) → no lugar → curva− → giro+ → reta → no lugar → parar →
  andar → reta → curva+.
- ω ∈ {0,175; 0,35} × 2 pés iniciais = 4 corridas.
- O `walk_test.py` mede 92 s; `analisar_teleop.py` faz a análise por
  trecho.
- Cada trecho descarta 1,5 s depois da troca.

**Fatos:**
- **As 4 andaram os 94 s sem cair**, inclusive depois de parar e voltar a
  andar.
- **vx realizado:** 0,101–0,118 m/s nos trechos com vx 0,1, e 0,002–0,023
  m/s nos trechos sem avanço. Este é o avanço espúrio do gerador (achado
  AA).
- **ω/ω_cmd:** 0,95–1,28 nos trechos de 4,5 s. O desvio maior vem logo
  depois da inversão giro+ → giro− (1,11–1,28). Nas corridas de comando
  constante da E5, a faixa é 0,99–1,01.
- **Roll RMS por trecho:**
  - ω 0,175: 0,8–1,8°;
  - ω 0,35: 1,0–3,1°.
- **Pico de pitch:** 7,7° com ω 0,175 e 12,6–13,4° com ω 0,35.
- **Leitura (inferência):** o excesso de ω depois da inversão é a malha de
  rumo. Ela integra o ω comandado na referência e recupera o atraso da
  troca, com a correção limitada a 0,3 rad/s.

**Recomendação:** ω 0,175 como padrão. O 0,35 andou nas duas corridas, mas
com o dobro do roll e ~70% a mais de pico de pitch.

**Regressão do binário com o teleop desligado.** Duas corridas da E7 foram
repetidas com o binário novo (`omni/regressao_teleop/`): M33 reta 0,10 pD e
M38 giro +0,175 pE.

| corrida | binário | desfecho | trocas de apoio | pitch RMS [°] | distância [m] |
|---|---|---|---|---|---|
| M33 reta 0,10 pD | antigo | completou | 102 | 2,31 | 3,084 |
| M33 reta 0,10 pD | novo | completou | 102 | 2,36 | 3,076 |
| M38 giro +0,175 pE | antigo | completou | 102 | 3,41 | 0,122 |
| M38 giro +0,175 pE | novo | completou | 102 | 3,41 | 0,129 |

As diferenças são da ordem do ruído de temporização já descrito em
"Replicação e estatística": as séries partem iguais e se separam aos poucos.

**Revalidação com kd de pitch 0,06** (2026-11-13, depois da E7;
`omni/teleop/desenho_teleop_kd06.csv`): mesma sequência, 4 corridas, nenhuma
queda.

| ω, pé | pitch RMS kd 0,03 → 0,06 [°] | pico de pitch [°] | roll RMS [°] |
|---|---|---|---|
| 0,175 pD | 2,14 → 1,82 | 7,7 → 6,4 | 1,31 → 1,31 |
| 0,175 pE | 2,35 → 1,89 | 7,7 → 6,5 | 1,39 → 1,46 |
| 0,35 pD | 3,20 → 2,23 | 13,4 → 7,1 | 1,70 → 1,51 |
| 0,35 pE | 3,03 → 2,43 | 12,6 → 9,7 | 1,90 → 1,63 |

O comando do teleop (README §5) passou a usar `kd_pitch:=0.06`.

**Correção de 2026-11-13: queda para trás na partida.**
- **Sintoma** (relato do usuário): muitas vezes o robô já começava caindo
  para trás ao subir o teleop.
- **Causa:** o `teleop_marta.py` sobe junto com o launch e despausava o
  Gazebo ~0,5 s depois, **antes** de os controladores de junta serem
  carregados. As juntas ficavam soltas até o spawner terminar.
- **Reprodução** (`omni/teleop/partida/`, M34 + roll, sem GUI):

  | carga da máquina | despausa | resultado |
  |---|---|---|
  | 1 simulação extra em paralelo | antes dos controladores (0 de 20) | 4/4 em pé |
  | 1 simulação extra em paralelo | depois dos 20 | 4/4 em pé, trajetória igual |
  | 3 simulações simultâneas | antes dos controladores | **3/3 caem de costas** (pitch −90°) |
  | 3 simulações simultâneas | depois dos 20 | 3/3 em pé |

  Com a máquina ocupada (GUI, Foxglove, outra simulação), o carregamento
  atrasa, e o robô desaba. A validação anterior não pegou isso porque usava
  o `walk_test.py` com `sleep 24`, e não a partida do teleop.
- **Correção:** `esperar_controladores()` no `teleop_marta.py` e no
  `walk_test.py`. A despausa espera os tópicos `<junta>_position/state` de
  todos os controladores declarados no servidor de parâmetros. O
  `list_controllers` não serve, porque fica bloqueado pelo spawner enquanto a
  simulação está pausada.
- **Conferido sob a mesma carga:**
  - a função nova, 3/3 em pé;
  - o `walk_test.py` lançado junto com o launch, 3/3 completaram 10 s de
    giro;
  - o teste de teclas (`omni/teleop/teste_teclas.py`), 16/16.
- **Nas campanhas, nada muda:** elas esperam 24 s antes do `walk_test.py`,
  e a checagem passa na hora, com a simulação ainda pausada.
- **Fato à parte:** em todas as partidas, o agachamento inicial (`Agacha()`,
  10 s, sem estabilizador) inclina o tronco até −8,5° (para trás) antes de
  voltar a +2,9°.

**Limitações:**
- uma massa (M34);
- uma sequência fixa;
- uma corrida por ω e por pé;
- ninguém dirigiu com as mãos ainda, e o padrão de troca de uma pessoa
  pode ser mais rápido que o da sequência (trechos de 6–8 s);
- o robô real não foi testado.

## Demonstração no Foxglove

**Layout:** [`foxglove/marta_omni.json`](../src/marta_bringup/foxglove/marta_omni.json).
O robô 3D fica à esquerda, e há cinco abas à direita:

| aba | o que mostra |
|---|---|
| Torque/limite | \|torque pedido\| / limite das 6 juntas de cada perna. **Acima de 1,0 a junta está saturada** (o Gazebo corta no limite) |
| Erro das juntas | alvo − real [°] das 6 juntas de cada perna |
| Controladores | pitch (verdadeiro, estimado, correção); roll (verdadeiro, estimado, erro pós-washout, u quadril, u tornozelo); rumo (yaw × referência e correção); compensação de gravidade (Δ de hip_roll e joelho) |
| LIPM (Kajita) | CoM medido × planejado em x e y no frame do pé de apoio (fig. 5 do artigo); energia orbital (eq. 28–29); plano de fase (x, ẋ) (eq. 40) |
| Marcha | estado de apoio, taxa de giro real e DCM |

**Fonte dos dados:** o nó `scripts/viz_marcha.py`, ligado com
`viz_marcha:=true`. Os índices de cada tópico estão no cabeçalho do nó.

**A conferir na primeira abertura: o plano de fase.** O layout grava o eixo x
como `xAxisVal: custom` + `xAxisPath`, que é o formato do Foxglove Studio
antigo. A documentação atual do painel Plot chama esse modo de
**Expression**, mas não mostra as chaves do JSON salvo, e não consegui
confirmá-las.
- **Se o painel abrir como série no tempo**, ajuste à mão nas configurações
  do painel:
  - *X-axis value type* = **Expression**;
  - *Default value* (x) = `/marta/viz/lipm.data[0]`;
  - série y = `/marta/viz/lipm.data[4]`;
  - *Time range* = All;
  - *Axis scales*: livre, porque as unidades são m e m/s.
- **Depois do ajuste**, exporte o layout. O JSON exportado passa a ter as
  chaves certas da sua versão.
- **Sem o painel**, o plano de fase continua na figura estática
  (`fig_plano_fase`).

**Conferido nos bags:**
- todos os tópicos do layout estão gravados (`rosbag info`);
- **a fração do tempo com torque ≥ 1,0 no joelho bate com a `joelho_sat`
  da análise:** 8,5% contra 8,9% no giro M33 com roll, e 12,6% contra
  11,9% na reta M38;
- o normalizado chega a 1,2–2,3: o `command` é o pedido antes do corte, e
  **> 1 é saturação**.

#### Roteiro

**Bags** em `experiments/omni/demo/`, fora do git (~28 MB cada; os sem roll
são curtos porque a gravação para na queda):

| bag | o que mostra |
|---|---|
| `D_M33_off_giro+0.175_pD_r1` | giro sem roll: a oscilação de roll cresce passo a passo; cai aos 11,6 s |
| `D_M33_roll_giro+0.175_pD_r1` | o mesmo giro com roll: 30 s, ω 1,002 do comandado |
| `D_M38_off_giro+0.175_pD_r1` | 3,8 kg sem roll: cai na partida (4,2 s) |
| `D_M38_roll_giro+0.175_pD_r1` | 3,8 kg com roll: gira 30 s |
| `D_M33_roll_curva+0.175_pE_r1` | curva (0,1; 0,175) |
| `D_M33_roll_lat+0.05_pD_r1`, `D_M38_roll_lat+0.05_pD_r1` | lateral: anda de lado, com o avanço espúrio do gerador (erro de 13° e 19°) |
| `D_M33_roll_diag+0.03_pD_r1` | diagonal |
| `D_M38_roll_reta0.10_pD_r1` | reta de referência a 3,8 kg (passa pela bola aos ~19 s) |

**Para a reunião (~10 min, por bag: dá para pausar e voltar):**
1. **`D_M33_off_giro`**, aba **Controladores**: o roll verdadeiro sai de ±5°
   (2–4 s) e cresce para ±9° (6–8 s) e ±13° (8–10 s) antes da queda. Na aba **Marcha**, a taxa de giro fica
   abaixo de 0,175. No 3D, o tombo.
2. **`D_M33_roll_giro`**, mesmas abas:
   - o roll fica entre −2° e +4° do começo ao fim;
   - u_quadril e u_tornozelo alternam com a perna de apoio (pesos w_L e
     w_R);
   - a taxa de giro fica constante em 0,175.

   Na aba **Torque/limite**, o joelho de apoio encosta em 1,0 (~8% do
   tempo), e as juntas de roll ficam longe do limite.
3. **`D_M38_off_giro`** contra **`D_M38_roll_giro`**: a queda da partida
   some com o roll.
4. **`D_M33_roll_lat`**, no 3D: o rastro anda de lado e um pouco à frente.
   É o erro de direção que o roll não corrige.
5. **Aba LIPM (Kajita)** em `D_M38_roll_reta0.10` e depois em
   `D_M33_roll_giro`:
   - na reta, o CoM segue a referência de perto (~5 mm);
   - no giro, segue com o dobro do erro, e o x oscila de −2 a +1 cm onde o
     plano é ~0;
   - a energia orbital deriva pouco dentro de cada apoio simples, nos dois
     (tabela em "LIPM por movimento").
6. **Aba Erro das juntas:** a cedência dos servos, maior no joelho de apoio.

**Ao vivo** (o RTF cai com o app conectado; confira antes de tirar
conclusões):

```bash
# terminal 1 (README §5, com foxglove e viz_marcha; aqui M33 + roll, giro 0,175)
# campo completo (world.world, o default): gol em x = 3 m e bola em (2; -0,2)
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
# no app: Open connection -> Foxglove WebSocket -> ws://localhost:8765; layout marta_omni.json
# terminal 2: despausa e anda 30 s
docker exec -it marta_stab bash -c 'source /opt/ros/noetic/setup.bash && \
  source /ws/devel/setup.bash && rosrun marta_bringup walk_test.py _duration:=30'
```

- **As velocidades vêm do `roslaunch`** (`walk_velocity_*`, lidas só na
  partida). O `_velocity_x` do `walk_test.py` escreve o parâmetro, mas o
  controlador não o relê.
- **Sem roll:** troque para `roll_stabilizer_enabled:=false`.
- **Para gravar:** `_bag_out:=/ws/<arquivo>.bag`.
- **GUI:** sem `DISPLAY`, o socket X11 e o `xhost`, o `gzclient` aborta
  (`Aborted (core dumped)`, `gazebo_gui` exit 134) e o resto sobe sem
  janela. Com eles, o giro acima andou 31 s com a GUI aberta e RTF 0,999
  (conferido em 2026-11-12). Sem janela: `gui:=false`.
- **Campo completo:** a reta e a diagonal a 0,1 m/s passam pela bola
  (~19 s) e chegam ao gol (~30 s). O giro, a curva e o lateral não chegam
  perto deles.

**Regenerar os bags:** apague as linhas `D_*` de
`omni/campanha_panorama.csv` e rode
`BAG_DIR=src/marta_bringup/experiments/omni/demo bash src/marta_bringup/experiments/rodar_panorama.sh src/marta_bringup/experiments/omni/desenho_demo.csv 20261121 omni`.

## Decisões

Tomadas pelo usuário em 2026-11-12. As alternativas apresentadas estão
registradas abaixo.

1. **Configuração do roll: quatorn K1, adotado.**
   - **quatorn K1** (quadril + tornozelo, kp 0,5 / kd 0,075 em cada) foi a
     configuração confirmada na E5. Tem o menor roll RMS.
   - Na reta 0,10 do M33, ela sobe o pitch RMS em +10–19%.
   - A alternativa era **torn K1** (só tornozelo, kp 1,0 / kd 0,15). Teve
     menos pitch na E4, com uma corrida por condição, e não foi confirmada.
2. **Ganhos de pitch:** "manter o que ficou melhor".
   - Medido com roll na **E7**.
   - **kd 0,06 adotado** (kp 0,2): −10% de pitch RMS, melhor em 41 de 44
     pares, Holm 4·10⁻¹⁰, sem veto.
3. **`hip_offset_x` +5 mm:** "então teste".
   - Testado na **E7**: **não adotado**.
   - Inclina o tronco +1,6° para frente e piora o pitch RMS em 44 de 44
     pares, sem mudar a sobrevivência, que já estava no teto.
4. **Mundo:**
   - **campanhas automáticas** no `campo_livre.world`
     (`experiments/omni/diag/`);
   - **todo comando para rodar à mão** no campo completo (`world.world`, o
     default).
5. **Avanço espúrio do lateral e do giro** (v_x de 0,010–0,017 m/s). Ele vem
   do gerador, e nenhum estabilizador o corrige. Investigar no
   `OmnidirectionalWalkZMP` é um próximo passo possível.

## Limitações

- **Simulação:** o servo real tem folga, aquecimento e queda de tensão. Os
  ganhos de roll são de simulação, e o código força o roll a desligado no
  robô real (`isReal`).
- **Obstáculos no campo:** reta e diagonal passam pela bola (~19 s) e chegam
  ao gol (~30 s a 0,1 m/s). Ver a seção do método.
- **Uma corrida por condição e por pé:** desfechos na borda variam entre
  corridas idênticas (E2). As conclusões se apoiam em desfechos que se
  repetem em várias condições.
- **Pé plano não medido:** a telemetria não tem o roll do pé.
- **Sem colisão entre as pernas:** no giro e no passo lateral, os pés podem
  se atravessar. `sep_min_mm` mede isso (≥ 99 mm com roll).
- **Velocidades da tabela do Maximo** foram usadas como referência; o
  Bioloid é outro robô.

## Reprodução

```bash
bash src/marta_bringup/experiments/rodar_panorama.sh \
  src/marta_bringup/experiments/omni/desenho_E1.csv 20261114 omni
cd src/marta_bringup/experiments && python3 analisar_omni.py omni
```

**Desenhos:** `omni/desenho_*.csv`. **Logs:** `omni/log_*.txt`.

**Resumo por corrida:** `omni/resumo_omni.csv`.

**Diagnóstico de gol e bola:** `bash src/marta_bringup/experiments/omni/diag/rodar_diag.sh src/marta_bringup/experiments/omni/diag`
(`desenho_diag.csv`, `campo_livre.world`, `log_modelos.py`; resultados em
`diag/resumo_omni.csv`, dados em `diag/dados/`).

**E5:** `python3 analisar_e5.py omni`. **Figuras:**
`python3 fig_omni.py omni/resumo_omni.csv E5_ off_pD,quatornK1_pD,off_pE,quatornK1_pE`.

**Dados brutos** em `omni/dados/`, fora do git:
- `_pe.csv`, `_telemetria.csv`, `_juntas.csv`, `_estabilizador.csv`;
- `_roll.csv`;
- `_launch.log`.
