# Investigação do desvio de direção

> Parte de [README.md](README.md) — leia o mapa central primeiro.

**Pergunta:** por que o robô muda de direção andando, comandado a andar reto
(`rotationZ_ = 0`)?

**Resposta (2026-10-07), parcial:** o giro grande e sistemático para a ESQUERDA
era um **artefato da colisão do pé em malha STL no ODE**. Com a colisão em caixa
(hoje o padrão, `foot_collision:=box`), ele some. **Mas sobra uma curva para a
DIREITA, que não é desprezível:** com τ = 6 e o estabilizador ligado (o regime
de operação), −48 ± 15° em 30 s, 10/10 corridas, ~−28° deles nos primeiros 5 s;
visível a olho nu. Essa curva **não está explicada**.

---

## 1. Instrumentação criada

Antes, o fenômeno não era medido: não havia yaw na telemetria, e a distância das
campanhas era `hypot(dx, dy)`, que mistura avanço com deriva.

| onde | o quê |
|---|---|
| `WalkTelemetry.msg` / `walk_telemetry_node` | `yaw_true`, yaw dos dois pés (ângulo e taxa), ponto de contato de cada pé — verdade de solo do `/gazebo/link_states` |
| `walk_test.py` | colunas `psi_final`, `psi_pico`, `psi_taxa`, `avanco`, `lateral` (frame do rumo inicial), `caranguejo` (rumo da trajetória − yaw, por passada) |
| `analisar_yaw.py` | uma corrida: giro do corpo e dos pés por fase planejada; giro do pé de apoio por janela de apoio simples |
| `analisar_yaw_campanha.py` | campanha: filtro de RTF/queda, teste do sinal, ψ(t), saturação, caixa × malha |
| `rodar_yaw.sh` | campanhas E2 (padrão) e E3 (`caixa`) |
| `marta_control/test/test_walk_symmetry.cpp` | simetria do gerador, offline |

**Armadilhas de medida:**
- As **taxas** de yaw dos pés não são integráveis a 100 Hz: os picos de impacto
  (±190°/s) sofrem aliasing. Integrar a do pé esquerdo deu −269° contra +89° do
  ângulo. Use sempre diferença de **ângulo**.
- A decomposição chão × perna por **detecção de contato** falha com τ = 6,
  porque o pé em balanço sobe só ~2 cm (zStep = 6 cm) e se arrasta. A régua
  confiável é a contabilidade **por fase planejada**.
- Em bags antigos, sem yaw, o `/marta/imu.orientation` serve como verdade de
  solo. O eixo z do sensor está invertido pela montagem rpy=(π,0,π), então o
  giro z integrado tem sinal trocado.

## 2. Hipóteses e o que decidiu cada uma

| | hipótese | veredito | evidência |
|---|---|---|---|
| H0 | não é giro, é andar de lado | **refutada** | caranguejo ~3°; o rumo da trajetória acompanha o yaw |
| H1 | o pé de apoio gira no chão | **confirmada como caminho** | com malha e τ = 1000, o pé de apoio gira +1°/+2° (esq/dir) por apoio, ≈ toda a taxa medida |
| H2 | cedência/saturação da hip_yaw | **refutada como causa** | τ = 6 e τ = 1000 giram igual (p = 0,42); o desvio da junta não se correlaciona com o giro |
| H3 | assimetria física do modelo | **refutada** | pernas, braços, CoM (0,5 mm) e malhas dos pés (0,4 mm) espelham |
| H4 | assimetria do gerador | **refutada em regime**, com bug na partida | simétrico até 1e−15; mas `restart()` comandava 17° de yaw (ver §3) |
| H5/H6 | momento da perna em balanço / pé arrastando | **abertas**, para o resíduo | — |
| — | **contato de malha no ODE** | **causa** | caixa × malha: −0,9 × +3,6 °/s, p = 1e−4; o pé para de girar |

"Ausência de realimentação em yaw" não é causa: é o motivo de o erro
acumular. É verdade por construção e não se testa.

## 3. Bug corrigido no gerador

`OmnidirectionalWalkZMP::restart()` fazia `torsoBegin.z = 0.297509`. Nesse
`Vector3`, `.z` é o **yaw** do torso, não a altura: era o valor de `zCom` no campo
errado. Como o `controller_node` chama `restart()` toda vez que a marcha liga,
toda corrida começava comandando ~17° nas duas hip_yaw, e daí o pico de 16° de
desvio medido. Corrigido para `0.0` e travado por `test_walk_symmetry.cpp`.

Um A/B das quedas com τ = 1000 (3/10 com a correção × 1/10 sem, Fisher
p = 0,58) não mostrou efeito da correção nas quedas.

## 4. Resultados

Estabilizador desligado, 30 s a 0,1 m/s, RTF > 0,85, sem quedas. Dados em
`src/marta_bringup/experiments/yaw/`.

| condição | n | taxa de giro | para a esquerda | pé de apoio por apoio (esq/dir) | avanço em 30 s |
|---|---|---|---|---|---|
| malha, τ = 1000 | 7 | +3,6 ± 1,2 °/s | 7/7 | +1,0° / +2,0° | 1,05 m |
| malha, τ = 6 | 10 | +4,4 ± 1,5 °/s | 10/10 | −2,7° / +3,8° | 0,33 m* |
| malha, τ = 6, μ = 0,5 | 3 | +2,4 ± 1,1 °/s | 3/3 | −4,1° / +4,6° | 1,38 m |
| **caixa, τ = 1000** | 10 | **−0,9 ± 1,0 °/s** | 2/10 | **~0° / ~0°** | **2,47 m** |
| **caixa, τ = 6** | 10 | **−0,7 ± 1,0 °/s** | 1/10 | −1,5° / +1,8° | 1,28 m |
| caixa, τ = 6, estabilizador ligado | 10 | −1,5 ± 0,5 °/s | 0/10 | −0,6° / +0,3° | 1,11 m |

\* No frame do rumo inicial; o robô girou ~146°, então quase não "avançou".

**Transiente de partida × deriva em regime.** A taxa de 30 s mistura duas
coisas. Separando o giro até 5 s da inclinação de ψ(t) entre 5 e 30 s:

| condição | giro 0–5 s | deriva 5–30 s | deriva ≠ 0? |
|---|---|---|---|
| malha, τ = 1000 | +18 ± 5° | +3,5 ± 1,3 °/s | 7/7 esq |
| malha, τ = 6 | +17 ± 18° | +4,1 ± 1,6 °/s | 10/10 esq |
| caixa, τ = 1000 | +1 ± 7° | −0,9 ± 1,1 °/s | Wilcoxon p = 0,049 |
| caixa, τ = 6 | **−24 ± 12°** | −0,3 ± 0,9 °/s | p = 0,49 (não) |
| caixa, τ = 6, estabilizador ligado | **−28 ± 13°** | −1,0 ± 0,7 °/s | estabilizador × sem: p = 0,21 |

Com a caixa e τ = 6, **a deriva em regime é indistinguível de zero**. O que
sobra é um **giro de ~25° para a direita entre 2,5 e 5 s**, nos primeiros
passos, que não aparece com τ = 1000. É o próximo alvo: um transiente de
partida sob torque limitado.

**O estabilizador de pitch se mantém com a caixa:** o pitch RMS real com τ = 6
vai de 6,43° para 2,28° (−65%, p = 2e−4), contra −43% com a malha.

Outros achados:
- Com a malha e τ = 6, o tronco torce ~±6° a cada apoio, e o giro líquido é a
  pequena sobra de movimentos alternados grandes (63% dele no apoio duplo).
- O robô já gira 3–22° **parado**, durante a rampa e o assentamento (com malha).
- Com τ = 1000 e malha, 3/10 corridas caíram **para frente** nos primeiros 5 s.
  Com caixa, 0/10.

## 5. A curva para a direita com o pé em caixa (τ = 6)

Análise das 20 corridas com caixa e τ = 6 (10 sem e 10 com estabilizador),
sem simulação nova. Validação de ambiente: 5 corridas no setup headless e 5 com
GL por software, `DISPLAY` e rede host (o setup da visualização) dão o mesmo
giro típico (−43° × −48° em 30 s). A câmera só existe com GL, mas não muda o
resultado. A dispersão entre corridas tem caudas longas: há corridas com +55°
e −130°.

**Quando acontece.** Giro do corpo por passada (0,8 s), média de 10 corridas,
contando do primeiro passo:

| passada | sem estabilizador | com estabilizador |
|---|---|---|
| 0 | +5,4° | +4,7° |
| **1** | **−10,9°** | **−16,0°** (10/10 negativas) |
| 2–4 | −3 a −4,5° cada | −3,5 a −4,6° cada |
| 12–35 (regime) | +0,1°/passada | −0,5°/passada |

A curva é sobretudo um **transiente dos primeiros ~4 s de marcha**.

**Onde, dentro do passo.** O gerador sempre começa com o pé **esquerdo** de
apoio (o direito balança primeiro). Passo a passo:
- Nos apoios simples **esquerdos**, o corpo gira −5 a −7° e o pé direito, em
  balanço, gira −8 a −13°. Nos **direitos**, o corpo gira só +3 a +4° e o pé
  esquerdo +4 a +6°. A torção alternada é **maior no lado esquerdo**.
- O maior evento isolado é o **apoio duplo logo depois do 2º apoio esquerdo**
  (quando o pé direito pousa): o corpo gira −13° e **os dois pés giram ~−10°
  juntos**, ou seja, o robô inteiro pivota no chão. Ele se repete menor (−6°)
  a cada apoio duplo depois de um apoio esquerdo, contra +3 a +4° depois de um
  direito.
- O pé de apoio quase não gira durante o apoio simples (−2,5° no pior caso).
  Quase todo o giro entra no apoio duplo e no pé em balanço.

**Hipóteses descartadas por essa análise:** a fase do balanço lateral na
partida, porque com a caixa o robô parado quase não balança (roll real
±0,2°) e o roll inicial não se correlaciona com o giro (r = −0,04).

### Teste do espelho (2026-10-07): a curva acompanha o pé de partida

Parâmetro novo `start_left_swing` (gerador, controller e
`stabilizer_test.launch`; default `false` = histórico). A partida espelhada é o
espelho exato da padrão, tick a tick (`test_walk_symmetry.cpp`, seção 4). Pé em
caixa, τ = 6, estabilizador ligado, 10 + 10 corridas intercaladas, todas
válidas.

| partida | giro final (30 s) | giro 0–4 s | deriva em regime (> 6 s) |
|---|---|---|---|
| direito balança primeiro (padrão) | −32 ± 32° (9/10 direita) | −15,8 ± 18,5° | −0,51 ± 1,33 °/s |
| esquerdo balança primeiro | **+57 ± 27° (10/10 esquerda)** | **+16,2 ± 15,5°** | **+1,12 ± 0,77 °/s** |
| p (Mann–Whitney) | 0,0013 | 0,0017 | 0,0058 |
| soma (0 se for espelho perfeito) | +24,7° | **+0,5°** | +0,6 °/s |

Por passada (0 a 4), a partida pela direita dá +6,4 −12,1 −4,6 −3,8 −1,7 e a
partida pela esquerda dá −5,2 +13,3 +2,8 +3,0 +2,3: espelho.

**Conclusão:** a curva que sobrou com o pé em caixa é causada pela **iniciação
da marcha**. O gerador parte do repouso direto para o ciclo periódico, sempre
pelo mesmo pé. O transiente dos primeiros 4 s é espelho perfeito (a soma dá
+0,5°). **E a deriva em regime também troca de sinal:** a partida escolhe em
qual de dois ciclos-limite espelhados a marcha se assenta, e cada um curva
para o seu lado. Sobra um viés comum pequeno, de ~+0,3 °/s para a esquerda
(metade da soma), que não depende da partida.

Pela física, isso faz sentido: o LIPM do gerador planeja o primeiro passo como
se o CoM já estivesse oscilando, mas o robô parte parado e centrado. O primeiro
apoio recebe uma transferência lateral de peso brusca e assimétrica, e o
contato converte isso em giro em z.

### Partida suave (2026-10-07): tira o viés, mas o robô não passa a andar reto

Parâmetro novo `start_ramp_steps` (gerador, controller, launch; default 0 =
desligado). Nos primeiros N passos, um fator smoothstep de 0 a 1 escala o
balanço lateral do CoM em torno do ponto médio entre os pés, a altura do passo e
a velocidade. Offline (`test_walk_symmetry.cpp`, seção 5): a velocidade lateral
comandada no início cai de 0,259 para 0,001 m/s; depois da rampa a marcha é
idêntica à sem rampa (1e−17 m); o espelho continua exato.

Mesmo desenho do espelho, com N = 4. Giro final de cada corrida, ordenado:

| partida | sem rampa | com rampa de 4 passos |
|---|---|---|
| direita | −50 −50 −48 −44 −44 −40 −38 −32 −29 +55 | −71 −47 −45 −44 −25 −24 +44 +51 +87 +91 |
| esquerda | +27 +34 +40 +46 +52 +55 +55 +65 +72 +121 | −68 −68 −64 −55 −53 −48 −48 +42 +54 +94 |

- **O viés sumiu:** o grande evento da 2ª passada desaparece (−12° → −0,1°;
  +13° → −1,1°), o lado deixa de depender do pé de partida (p = 0,16) e a
  média geral fica em −10 ± 60° (p = 0,52 contra zero).
- **Mas a curva não sumiu, só passou a ter lado sorteado.** A distribuição é
  bimodal: quase toda corrida termina a ~50° para um lado ou para o outro, e o
  |giro| típico não diminuiu. A rampa **não** é uma correção do desvio, e o
  default fica 0.
- **Não há marcador do lado** em regime: roll médio, torção por apoio e offset
  lateral do CoM não se correlacionam com a taxa (|r| ≤ 0,22). Por corrida, a
  taxa em regime se espalha continuamente entre −2,4 e +2,5 °/s.

**Leitura:** depois de removidos os dois vieses sistemáticos (o contato de
malha e a partida), sobra uma deriva de rumo **própria de cada corrida**, sem
sinal previsível, de ordem ±1,5 °/s, mais eventos grandes nos primeiros
passos. Isso não se corrige com feedforward. **É aqui que a hipótese original
do usuário vale:** sem realimentação em yaw, qualquer perturbação pequena vira
curva. O próximo passo natural é uma **malha de rumo** (yaw integrado do giro →
`rotationZ`).

## 6. Malha de rumo (2026-10-07)

### Três peças, e um bug achado no caminho

1. **Estimativa de yaw** (`TorsoAttitudeEstimator`): integração do giroscópio
   no frame do mundo, ψ̇ = (sen φ·ω_y + cos φ·ω_z)/cos θ. O
   `/marta/torso_attitude` passa a ter 6 campos (`yaw` e `yaw_rate` no fim).
   Testes offline travam o sinal (esquerda = +) através da montagem invertida
   e garantem que o balanço em roll não vira yaw. Em simulação, a estimativa
   acompanha a verdade de solo com ~4° de diferença em 30 s. Sem
   magnetômetro, ela deriva com o bias do giro, e no robô real o bias precisa
   ser calibrado.
2. **Gerador:** o teste `velocity.z < DBL_EPSILON` virou
   `std::fabs(velocity.z) < DBL_EPSILON`. Antes, toda rotação **negativa** caía
   na aproximação de linha reta. `test_walk_symmetry.cpp` §6 trava que curvar
   para a esquerda é o espelho exato de curvar para a direita; o código antigo
   falha por 2,4 mm.
3. **Malha** (`controller_node`, `heading_enabled`, `kp_heading` = 1,0 rad/s
   por rad, `max_heading_rate` = 0,3 rad/s): a referência é o yaw no início da
   marcha (avançado por `walk_velocity_yaw`, se houver), e a correção
   −kp·erro, saturada, é somada ao comando de rotação. O erro e a correção
   saem em `/marta/stabilizer_debug` (campos 8 e 9).

**Bug: o sinal do yaw na IK estava invertido para a simulação.** A primeira
corrida com a malha girou −171°: a malha empurrava para o lado errado,
saturada. Em malha aberta, pedir +0,2 rad/s (esquerda) fazia o robô girar
−8,5 °/s (direita). As duas `hip_yaw` têm eixo −z no URDF, então o yaw do pé é
−q, mas a IK fazia `hipYaw = theta`. Ninguém tinha visto porque a marcha nunca
tinha sido comandada a girar. Correção por parâmetro, para não mexer no robô
real sem verificar os servos: `hip_yaw_sign` (default +1, histórico), que o
`stabilizer_test.launch` passa como −1. Com −1, ±0,2 rad/s dá +7,7 / −7,5 °/s
(simétrico, ~65% do comandado).

### Resultado: τ = 6, pé em caixa, estabilizador ligado, 10 + 10 intercaladas

| | sem malha | **com malha** | p |
|---|---|---|---|
| giro final (30 s) | −74 −58 −48 −38 −36 −34 −29 −20 −11 +74 | −22 −17 −3 +1 +17 +19 +21 +21 +31 +32 | |
| \|giro final\| médio | 42° | **19°** | 0,009 |
| desvio lateral | −0,69 ± 0,62 m | **+0,03 ± 0,15 m** | 2e−4 |
| avanço no rumo inicial | 1,15 m | **1,63 m** | 2e−4 |
| pitch RMS real | 2,41° | 3,00° (**+25%**) | 0,002 |
| quedas | 0/10 | 0/10 | |

**A malha resolve o desvio de trajetória:** o desvio lateral cai de ~70 cm
para ~3 cm (dispersão de 15 cm), e o robô avança 40% mais no rumo pedido. O
yaw ainda oscila ±20–30° em torno da referência, com um viés de +10°. O
ganho atual (kp = 1) é conservador para uma planta com ganho de ~0,65 e
atraso de até um passo duplo (a rotação só entra no passo do pé certo).
**Custo:** o pitch RMS piora 25%, porque os passos de correção perturbam o
plano sagital.

## 7. O agachamento e os passos de ajuste não causam o giro inicial (2026-10-08)

Hipótese do usuário: o `Agacha()`, seguido dos ~3 passos no lugar que
`requestStop()` dá antes de parar (os pés arrastam), deixa o robô num estado
que a partida brusca da marcha "corrige" girando. Parâmetro novo
`skip_initial_stop_steps` (`stopNow()` no gerador; default `false`), que para
direto depois do agachamento. Verificado: 6 trocas de apoio depois do
`Agacha()` no padrão, 0 com o parâmetro. τ = 6, pé em caixa, estabilizador
ligado, sem malha de rumo, 10 + 10 intercaladas, 0 quedas.

| | com passos de ajuste (padrão) | sem passos de ajuste | p |
|---|---|---|---|
| pés esq − dir antes do 1º passo | +0,3 ± 1,6° | **−5,4 ± 0,2°** | 2e−4 |
| yaw do corpo ao partir | −1,9 ± 2,2° | −0,5 ± 0,2° | |
| giro nos primeiros 4 s | −20,2 ± 11,7° | −21,0 ± 14,2° | 0,62 |
| deriva em regime | −0,57 °/s | −0,59 °/s | |
| giro final (30 s) | −45 ± 8° | −41 ± 54° | 0,43 |

**Refutada.** Os passos de ajuste realinham os pés (sem eles, os pés partem
com 5,4° de desalinhamento sistemático), mas o giro dos primeiros 4 s é o
mesmo nos dois casos, inclusive com esse desalinhamento grande e fixo. Junto
com a correlação fraca (r = −0,27) entre desalinhamento e giro inicial, isso
põe a causa **nos primeiros passos da marcha em si**, não no que vem antes.

## 8. Sintonia da malha de rumo (2026-10-08)

τ = 6, pé em caixa, estabilizador ligado, malha ligada, 8 corridas por nível,
intercaladas.

| kp / saturação | quedas | giro 0–4 s (\|·\| médio) | \|giro final\| | lateral (30 s) | pitch RMS |
|---|---|---|---|---|---|
| **1,0 / 0,3 rad/s** | **0/8** | **8,0°** | **10,2°** | **0,00 ± 0,11 m** | 2,48° |
| 2,0 / 0,6 rad/s | 3/8 (11, 20, 31 s) | 8,7° | 9,0° | −0,01 ± 0,09 m | 3,62° |
| 3,0 / 0,6 rad/s | **8/8** (7–20 s) | — | — | — | — |

**O ganho atual é o melhor dos três.** Aumentar o ganho não segura mais o giro
dos primeiros passos (8,7° contra 8,0°) e derruba o robô: as quedas são para
frente, em pitch. Com τ = 6, passos de rotação maiores desestabilizam o plano
sagital. O teste não separa se o culpado é o kp ou a saturação maior; kp = 2
com saturação 0,3 rad/s ficou sem testar.

Com kp = 1, o giro nos primeiros 4 s ficou em ~8°, contra ~20° sem a malha na
campanha do §7. A malha já segura boa parte do evento inicial, e o desvio
lateral em 30 s fica em 0 ± 11 cm.

**Desde 2026-10-08 a malha vem ligada por default no `stabilizer_test.launch`**
(kp = 1, sat = 0,3). No código do controller o default continua desligado, para
não afetar outros launches nem o robô real. `rodar_yaw.sh` passa
`heading_enabled` explicitamente, então as campanhas antigas continuam
reproduzíveis.

## 9. Em aberto

- **O evento dos primeiros passos** (~−20° em ~2 s, sempre presente; o maior
  pedaço no apoio duplo após o 2º apoio esquerdo, com os dois pés pivotando
  juntos). A malha de rumo atual não o alcança: ela age a cada passo e satura
  em 0,3 rad/s.
- Sintonia da malha de rumo (kp, saturação) e o custo em pitch.
- `hip_yaw_sign` no robô real: verificar o sentido dos servos antes de usar
  rotação comandada lá.
- Por que alguns passos iniciais dão eventos de ±20–40° (as caudas, como o
  −130° visto na visualização).

- O **transiente de partida** com τ = 6 (~−25° nos primeiros 5 s) e a deriva
  marginal com τ = 1000 (−0,9 °/s, p = 0,049). Fontes candidatas: H5 (balanço de braços em malha aberta, Fujimoto & Kawamura 1997) e
  H6 (pé arrastando com μ = 1e10).
- Repetir a linha de base das campanhas antigas com a caixa.
- Malha de yaw (integração do giro → `rotationZ`). Antes dela: o teste
  `velocity.z < DBL_EPSILON` em `OmnidirectionalWalkZMP.cpp` vale para qualquer
  rotação **negativa**, então curvas à direita usam aproximação linear e à
  esquerda o arco exato.
