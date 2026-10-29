# Fase 3: compensação de gravidade (implementada) e estabilizador de roll (plano, não implementado)

> Parte de [README.md](README.md) — leia o mapa central primeiro.
> Documento irmão: [estabilizadores.md](estabilizadores.md) (Fases 1 e 2).

**Estado (2026-10-22): a compensação de gravidade (§4.1, §4.2, parte da §4.4) está implementada e testada; resultados e ressalvas em [ajustes_marcha.md](ajustes_marcha.md) §Compensação de gravidade. O estabilizador de roll (§4.3) não foi implementado.** Este documento guarda:
- os dados que motivam o projeto;
- as referências conferidas, com página;
- o desenho completo dos dois blocos;
- o protocolo de teste.

Quem for implementar começa pela §6.

---

## 1. O problema medido

O tronco **tomba em roll para o lado da perna no ar** a cada apoio simples, e
volta. A média por corrida é ~0, então não é inclinação fixa; é cedência que
alterna com a perna de apoio.

Os números vêm do pé em caixa, da verdade de solo do Gazebo e de 8 corridas de
`experiments/yaw/dados/` por condição:

| | τ = 6 (pitch ligado) | τ = 1000 |
|---|---|---|
| roll verdadeiro, apoio esquerdo / direito | **+5,9° / −5,9°** | +3,3° / −3,0° |
| cedência de `hip_roll` + `ank_roll` da perna de apoio | 1,9° + 2,0° = **3,9°** | 1,2° + 1,2° = 2,4° |
| torque dessas duas juntas | 2,4 e 1,8 N·m (**abaixo** do limite de 6) | 1,9 e 1,4 N·m |
| cedência do joelho de apoio | **−7,5°** (6,5 N·m, saturado) | −6,1° (5,4 N·m) |

**Conclusão de projeto:** quadril e tornozelo cedem **sem saturar**. Eles cedem
porque um servo de posição só produz torque com erro de posição (`effort =
p·(set − q)`, `p = 50` N·m/rad). Adiantar esse erro por feedforward
(compensação de gravidade) tem espaço para funcionar. No joelho com τ = 6,
não, porque ele já está no limite de torque.

**A estimativa de roll tem viés** (pitch ligado e τ = 6, contra a verdade):

| | τ = 6 | τ = 1000 |
|---|---|---|
| viés médio (estimado − verdadeiro) | **+2,3°** | **+2,5°** |
| viés com o robô parado (primeiro 1 s) | +1,3° | +2,2° |
| correlação estimado × verdadeiro | 0,96 | 0,96 |
| atraso | ~11 ms | ~8 ms |
| roll estimado no apoio E / D | +8,1° / −3,5° | +5,9° / −0,7° |

A amplitude alternada é bem estimada, mas o nível tem viés de ~2,4°. **Um termo
P com `roll_ref = 0` comandaria uma inclinação permanente de ~2,4°.** Solução de
projeto: o P age sobre `φ − φ̄`, com φ̄ um passa-baixa lento (*washout*) (§4.3).

**Massas** (URDF, total 4,781 kg):

| parte | massa |
|---|---|
| tronco (`base_link`) | 2,966 kg (62%) |
| cada perna | 0,542 kg |
| cada braço | 0,280 kg |
| cabeça + pescoço | 0,170 kg |

Dentro da perna, `waist_1` e `ankle_1` têm 165 g cada (provavelmente os servos),
a coxa 105 g e a canela só 21 g. Vale conferir contra o robô real, porque a
compensação de gravidade depende dessa distribuição.

---

## 2. Referências conferidas (com página)

As duas teses do Maximo foram lidas no PDF. O número impresso coincide com a
página do PDF. **Os PDFs não estão no repositório**; o usuário os tem.

**Maximo, M. R. O. A., *Automatic Walking Step Duration through Model Predictive
Control*, tese de doutorado, ITA, 2017:**

| assunto | página |
|---|---|
| Fig. 4.1: arquitetura (IK + compensador de gravidade + compensador de orientação do tronco, tudo somado depois da IK) | 73–74 |
| §4.3 Compensação de gravidade: hip-roll e knee-pitch são as juntas mais afetadas em apoio simples | 78 |
| Eq. 4.18–4.22: massa e CoM acima de cada junta (pé "parafusado"), τ_k = (r × m·g)·ê_k, Δθ = τ/K_p | 79 |
| "we only compensate the hip-roll and knee-pitch joints"; interpolação linear no apoio duplo (4.23–4.24) | 80 |
| Eq. 4.25 | 81 |
| §4.4.1: o yaw não é observável pelo IMU e deriva; a navegação fica com uma camada acima | 84 |
| §4.4.2: controladores P+V desacoplados em roll e pitch | 84 |
| **Eq. 4.34–4.35: roll no quadril e no tornozelo, com ganhos separados**; eq. 4.36–4.38: pitch no quadril, joelho e tornozelo. Ganhos por modelo linearizado de 2 links (frontal) e 3 links (sagital) + `fminsearch` (os valores não são publicados) | 85 |
| §4.5.1: resultado da compensação de gravidade | 86 |
| Fig. 4.7 (sem compensação: roll do tronco até ~7,5° em apoio simples) e Fig. 4.8 (com compensação: < 1°) | 88 e 89 |
| Fig. 3.4: colisão do modelo feita de caixas (apoia o achado I) | — |
| §3.3.3: servo no Gazebo como amortecimento equivalente + saturação no torque de stall, com a inércia refletida do motor (sem ela a simulação ficava instável) | — |

**Maximo, M. R. O. A., *Omnidirectional ZMP-Based Walking for a Humanoid Robot*,
dissertação de mestrado, ITA, 2015.** É a origem do nosso
`OmnidirectionalWalkZMP`; o cabeçalho do `.cpp` traz `Author: mmaximo`.

| assunto | página |
|---|---|
| §3.9 Stabilization Controller: realimentação **só pela velocidade angular** (θ += K·ω) | 72 |
| Eq. 3.73–3.77: correções em hip-pitch, joelho, foot-pitch, hip-roll e foot-roll | 73 |
| Tabela 5.2: ganhos (K_hr = 0,05; K_fr = 0,02; demais 0,01 s) | 85 |
| Fig. 5.9: efeito do estabilizador no roll sob perturbação | 94 |

**Atenção ao citar o mestrado.** Nas eq. 3.74, 3.76 e 3.77 os índices parecem
trocados: o joelho é corrigido com ω_r, e hip-roll e foot-roll com ω_p. O
esperado é o inverso. Provável erro de digitação; conferir no PDF.

**Raibert, *Legged Robots That Balance*** (`~/Documents/ic/study/`): atitude do
corpo pela perna de apoio, com pitch e roll como malhas independentes. É a ideia
original do usuário: estabilizador de roll "na cintura", espelhando o de pitch.
**Nenhuma das teses propõe roll só na cintura**; essa ideia é do Raibert. Não
há número de página dele conferido.

---

## 3. A "armadilha" do `ankle_roll = −hip_roll`, refinada

A IK impõe `ankle_roll = −hip_roll` para manter o pé paralelo ao chão. A versão
antiga desta nota dizia que corrigir roll só no quadril inclina o pé. **Isso só
vale em dois casos:**
- **na perna em balanço**, porque o pé pousaria inclinado;
- **no apoio duplo, com as duas pernas**, porque a cadeia fechada fica
  sobrevinculada: girar o torso em x desloca os quadris na vertical.

No apoio simples, mexer só no `hip_roll` da perna de apoio não tira o pé do
chão: o chão segura o pé e quem gira é o torso.

Por isso o roll **não** usa a política do pitch (peso 1 nas duas pernas no apoio
duplo). Ele usa pesos de carga que somam 1 (§4.1), e o "pé plano" vira métrica
medida (§5).

**Sinais** (a travar em teste offline, como no pitch):
- `hip_roll` e `ank_roll` têm eixo +x nas duas pernas (`marta_joints.xacro`);
- roll > 0 é tombar para a direita (rotação +x);
- na perna de apoio, +Δ no `hip_roll` gira o torso −Δ (para a esquerda);
- +Δ no `ank_roll` gira o corpo inteiro −Δ;
- portanto **Δ = +K·φ restaura nas duas juntas, com ganhos positivos**, a mesma
  estrutura do pitch.

---

## 4. Desenho

```
walker.update → IK (θ_IK) → +Δθ_g (gravidade, FF) → +Δθ_pitch (Fase 2, intacto)
                          → +Δθ_roll (Fase 3, FB) → publishJoints
```

### 4.1 Pesos de carga por perna (compartilhados pelos dois blocos)

É a interpolação de Maximo (eq. 4.23–4.25), contínua na troca de passo. Com t a
fase do passo e t1 = r_ds/2, t2 = 1 − t1, o peso da perna de apoio do passo
atual é:

| fase | w_apoio |
|---|---|
| t < t1 | 0,5 + 0,5·t/t1 |
| t1 ≤ t ≤ t2 | 1 |
| t > t2 | 1 − 0,5·(t − t2)/t1 |

- A outra perna recebe 1 − w.
- No estado STOPPED, cada perna recebe 0,5.
- **Onde:** um getter que só lê o estado,
  `OmnidirectionalWalkZMP::getLoadWeights(double &wl, double &wr)`, como o
  `getSupportState()`.
- **Teste:** os pesos somam 1, são contínuos na troca de passo e dão w = 1 no
  apoio simples (nova seção 7 de `test/test_walk_symmetry.cpp`).

### 4.2 `GravityCompensator` (Maximo §4.3)

**Arquivos:** `include/GravityCompensator.h` e `src/GravityCompensator.cpp`, em
C++/Eigen puro (testável offline).

**Modelo:** recebe um modelo de corpos rígidos (por link: junta pai, origem,
eixo, massa, CoM local), montado do `urdf::Model` do parâmetro
`robot_description` em `src/RobotModelLoader.cpp`. O `urdf` já é dependência, e
o padrão de leitura está em `walk_telemetry_node.cpp::loadMasses`.

**Cálculo:**
- Cinemática direta com os ângulos **comandados pela IK**, braços incluídos, no
  frame do torso, com g = (0,0,−g) (tronco suposto vertical, como em Maximo).
- Para cada junta k da perna de apoio L ("pé parafusado"):
  - B_k = todos os links menos os que ficam entre k e o pé;
  - τ_k = [Σ_{i∈B_k} (r_i − p_k) × m_i·g] · ê_k;
  - Δθ_k = clamp(τ_k, ±τ_lim) / K_p.
- Calcula-se a solução L e a solução R e mistura-se com os pesos da §4.1.

**Parâmetros:**

| parâmetro | default | observação |
|---|---|---|
| `gravity_comp_enabled` | false | |
| `gravity_comp_joints` | `hip_roll,knee` | máscara de juntas; Maximo usa estas duas |
| `servo_kp` | 50 | o `p` do `controller.yaml` |
| `gravity_comp_torque_limit` | `joint_effort` | o feedforward não pede torque que o servo não entrega |
| `max_gravity_comp` | 0,2 rad | |
| `gravity_comp_swing` | false | exploratório: compensa o peso da perna em balanço. Hipótese: o pé só sobe 2 dos 6 cm comandados por cedência do joelho em balanço |

### 4.3 `RollStabilizer` (Raibert / Maximo §4.4.2)

**Arquivos:** `include/RollStabilizer.h` e `src/RollStabilizer.cpp`. É uma classe
nova; o `TorsoStabilizer` (pitch) não é tocado.

**Lei de controle:**

    φ̄ ← φ̄ + (dt/τ_w)(φ − φ̄)          washout do viés do estimador
    u_h = kp_h·(φ − φ̄) + kd_h·φ̇,   u_a = kp_a·(φ − φ̄) + kd_a·φ̇
    hipRoll_L += w_L·sat(u_h),   ankleRoll_L += w_L·sat(u_a)     (idem R)

- **Variantes** pelos ganhos:
  - Raibert (cintura): `kp_a = kd_a = 0`;
  - Maximo: ganhos nas duas juntas;
  - mestrado (só D): `kp = 0`.
- A perna em balanço recebe peso ~0 pela §4.1, então a colocação do pé não
  muda.

**Parâmetros:**

| parâmetro | default |
|---|---|
| `roll_stabilizer_enabled` | false |
| `kp_roll_hip` | 0 |
| `kd_roll_hip` | 0 |
| `kp_roll_ankle` | 0 |
| `kd_roll_ankle` | 0 |
| `roll_washout_tau` | 2,0 s (≈ 5 passadas) |
| `max_roll_correction` | 0,14 rad |

Todos relidos com `getParamCached`, como o pitch.

### 4.4 Integração

**`controller.cpp`:**
- carregar o modelo uma vez e aplicar os blocos na ordem acima;
- em `isReal`, os dois blocos ficam forçados a off, com aviso (o K_p do servo
  real é desconhecido);
- publicar `/marta/leg_compensation`: θ_IK das 12 juntas de perna, Δθ_g,
  Δθ_roll, w_L, w_R, φ − φ̄ e as flags de saturação.

**Telemetria e scripts:**
- `WalkTelemetry.msg`: `l/r_foot_roll`, para a métrica de pé plano;
- `walk_test.py`: colunas novas (§5) e o tópico novo no bag;
- `push_test.py`: `_axis:=y` e métricas de roll;
- `stabilizer_test.launch`: args novos, todos com default off.

**Testes offline:**
- `test/test_gravity_compensator.cpp`:
  - pêndulo de 2 links com torque analítico;
  - massa total do URDF = 4,78 kg;
  - sinal do `hip_roll` em apoio simples igual ao da cedência medida;
- `test/test_roll_stabilizer.cpp`:
  - sinal;
  - a perna em balanço não é tocada;
  - saturação;
  - o washout remove um viés constante e preserva a alternância de 1,25 Hz
    (ganho > 0,95).

---

## 5. Métrica e protocolo (pré-registrar antes da primeira corrida)

**Métrica primária, A_roll:**
- roll **verdadeiro** no meio de cada apoio simples (fase em
  [t1 + 0,25·(t2 − t1), t2 − 0,25·(t2 − t1)]), com sinal tal que positivo =
  tombando para o lado da perna em balanço;
- valor da corrida: média sobre os apoios com t > 5 s.

**Métricas secundárias e de segurança:**
- quedas;
- `rms_roll_v`;
- `rms_pitch_v` e média de pitch (piora ≤ 10%);
- `avanco`, `|lateral|`, `|psi_final|`;
- roll do pé de apoio (< 2°, pé plano);
- cedência por junta, `q − θ_IK` (mecanismo);
- fração do tempo com torque ≥ 0,95·τ;
- fração com correção saturada;
- amplitude lateral do CoM no frame do corpo (deve **permanecer**: é o balanço
  do LIPM).

**Regras:**
- RTF < 0,85: a corrida é descartada e repetida;
- condições intercaladas, em ordem embaralhada com semente registrada;
- Mann–Whitney com Holm nos contrastes primários, Hodges–Lehmann com IC
  bootstrap, Fisher para quedas;
- critério de sucesso: A_roll −30% com p_Holm < 0,05, sem mais quedas, pitch
  ≤ +10% e pé plano.

**Estágios, com portões:**
- **E0.** Código + testes offline + `catkin_make`.
- **E1. Validação sem simulação.** Uma ferramenta de replay roda o
  `GravityCompensator` sobre os set-points gravados das corridas
  `tau1000_*_caixa` (pitch desligado, então set-point = θ_IK). Em apoio
  simples, compara τ_g previsto com o torque medido (`command` do PID).
  - **Portão:** inclinação ∈ [0,7; 1,3], R² > 0,6 e mesmo sinal em `hip_roll`
    e `ank_roll`.
  - Sinal trocado: corrigir a convenção, que é a mesma armadilha do
    `hip_yaw` (achado L).
- **E2. Regressão:** tudo desligado reproduz a linha de base.
- **E3. Piloto de G, n = 3, τ = 6.** Máscaras {hip_roll, knee},
  {hip_roll, ank_roll, knee} e todas. A cedência de `hip_roll` tem que cair.
- **E4. G confirmatório:** {off, on} × τ ∈ {6, 1000}, n = 10.
- **E5. Piloto do roll, sobre G on**, comparado **com o mesmo ganho total**
  K = k_h + k_a, para isolar a alocação:
  - cintura: (K, 0);
  - Maximo: (K/2, K/2);
  - K_p ∈ {0,25; 0,5; 1,0}, k_d ≈ 0,15·K_p;
  - mais a variante só-D (k_d = 0,05).
  - Sobe de nível só sem quedas.
- **E6. Fatorial confirmatório, τ = 6, n = 10:** G {off, on} × R {off,
  cintura, quadril + tornozelo}.
- **E7.** τ = 5 (sobrevivência) e empurrão lateral parado.

São ~170 corridas, ~90 s cada (~4,5 h). Infraestrutura: um `rodar_roll.sh` no
molde do `rodar_yaw.sh`, **com retomada** (pula tags já presentes no CSV),
checagem de disco e parada após 3 falhas seguidas.

---

## 6. Por onde começar

1. Ler as §1 e §3 e conferir os sinais de eixo no `marta_joints.xacro`.
2. Implementar a §4.1 (pesos) com o teste. É pequena e destrava as duas outras.
3. Implementar o `GravityCompensator` e rodar a validação E1 **antes** de
   qualquer simulação. Ela decide se o modelo de gravidade está certo usando
   dados que já existem.
4. Só então o `RollStabilizer` e as campanhas.
