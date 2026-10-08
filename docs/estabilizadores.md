# Desenvolvimento dos estabilizadores

> Parte de [README.md](README.md) — leia o mapa central primeiro.
> Documento irmão: [telemetria_marcha.md](telemetria_marcha.md).

Cobre o que foi construído em controle: a teoria aplicada, as decisões de
projeto e por quê, o que foi medido e o que ficou de fora.

---

## 1. A ideia central

O gerador de marcha (`OmnidirectionalWalkZMP`) é **malha aberta**: calcula a
trajetória do CoM e dos pés e manda para a cinemática inversa, sem nunca olhar
o resultado. Seguindo **Kajita §4.5.1**, o estabilizador não entra dentro dele:
é uma camada separada que corrige o desvio *em cima* da referência nominal,
entre a IK e o despacho aos motores.

Isso tem consequência prática: dá para depurar e desligar cada um sozinho — e é
o que torna a comparação "com e sem" possível.

```
walker.update()  →  InverseKinematics  →  [ESTABILIZADOR: +Δθ]  →  motores
    malha aberta                              malha fechada
```

## 2. Raibert adaptado a servo de posição

Raibert (cap. 2 e 3) injeta **torque** no tronco pela perna de apoio:

    τ = −k_p(φ − φ_d) − k_v·φ̇

Nossos servos recebem **posição**. O torque vira um desvio angular somado à
saída da IK:

    θ_junta = θ_IK + Δθ,    Δθ = k_p(φ − φ_ref) + k_d·φ̇

### O sinal — a parte que mais custa errar

No URDF, `l_hip_pitch_link` e `r_hip_pitch_link` têm eixo **+y**, e o comando
enviado é o próprio ângulo da junta. Ângulo positivo gira a **coxa** de +Δθ em
torno de +y em relação ao torso.

Na perna de apoio a coxa está presa ao solo pelo pé e pelo joelho, então quem
gira é o **torso**, de −Δθ. Como girar em torno de +y inclina o tronco para
frente (convenção fixada na Fase 1), somar Δθ > 0 ao hip pitch gira o tronco
**para trás**.

Logo, com `φ_P > 0` significando "tombando para frente", a correção restauradora
é `Δθ > 0`, e os ganhos entram **positivos**. Isto está travado por teste
offline (`marta_control/test/test_torso_stabilizer.cpp`).

**Só o quadril é tocado.** Joelho e tornozelo ficam como a IK calculou, então a
postura da perna e o contato do pé não mudam — o tronco gira em torno do
quadril, deslocando o CoM. É estratégia de **quadril**, não de tornozelo.

---

## 3. Fase 1 — estimação de atitude

`marta_control/{include/TorsoAttitudeEstimator.h, src/TorsoAttitudeEstimator.cpp}`
e o nó `torso_attitude_node`.

Substituiu um pseudo-EKF de 9 estados que só tinha a etapa de predição — sem
correção, e cujo estado nunca era lido por ninguém.

**Pipeline:** rotação de montagem → passa-baixa no giro → filtro complementar →
portão de confiança no acelerômetro.

| peça | escolha | justificativa medida |
|---|---|---|
| rotação de montagem | `R_y(π)`, parâmetro `imu_mount_rpy` | o sensor está declarado com rpy=(π,0,π) no URDF |
| filtro complementar | `α = 0,998` (τ ≈ 5 s) | com α=0,98 (τ=0,49 s), **64%** da aceleração do corpo vaza como inclinação falsa; com 0,998, 9,5% |
| passa-baixa do giro | Butterworth **2ª ordem**, corte 15 Hz | em 30 Hz passa 13% contra 38% de um polo só, com só 10 ms de atraso |
| portão de confiança | suspende o acelerômetro quando \|‖a‖−g\| > 2 m/s² | no impacto do pé o acelerômetro deixa de apontar para cima |

**Convenção de sinais** (travada por teste): `pitch > 0` tombando para **frente**;
`roll > 0` tombando para a **direita**. `φ̇` vem **direto do giroscópio filtrado**,
nunca de derivada numérica do ângulo.

**Defeito conhecido (achado C do README):** o portão testa só a *norma* e é cego
à *direção* da aceleração. No regime de torque saturado o viés chega a **+21°**.
Correção pendente: peso adaptativo em vez de portão binário.

Correções estruturais feitas junto: `isReal` lido do namespace certo, estado
compartilhado em `std::atomic`, `dt` real medido com `ros::Time` em vez de
constante com `usleep`, e desacoplamento da realimentação antiga por `Xscale_`.

---

## 4. Fase 2 — estabilizador de pitch

`marta_control/{include/TorsoStabilizer.h, src/TorsoStabilizer.cpp}`, integrado
ao `controller_node` entre a IK e o `publishJoints`.

| parâmetro | default | o que faz |
|---|---|---|
| `kp_pitch` | 0,2 | rad de correção por rad de inclinação |
| `kd_pitch` | 0,03 | rad de correção por rad/s |
| `max_pitch_correction` | 0,1745 (10°) | saturação rígida de segurança |
| `support_blend_time` | 0,05 s | transição de peso entre as pernas |
| `stabilizer_enabled` | true | liga/desliga |

Todos relidos a cada 0,5 s por `getParamCached` — dá para sintonizar com o robô
andando.

**Distribuição entre pernas.** A correção vai para a perna de **apoio**; aplicar
na oscilante mudaria a colocação do passo que o gerador planejou. Em apoio duplo
vai para as duas. Cada perna tem um peso próprio que caminha suavemente entre 0
e 1 — a suavização age no **peso**, não no valor, então em apoio simples
estabelecido a correção não sofre atraso nenhum.

Para isso foi preciso **expor o estado de apoio** no gerador, que até então era
implícito no par `(leftFootIsSwing, walkTime/period)`:
`SupportState.h`, `getSupportState()`, `getPhase()`, `isDoubleSupport()`.
A geração de trajetória não mudou.

### Resultados

Campanha de 78 corridas, verdade de solo, detalhes em
[resultados_campanha.md](resultados_campanha.md):

| τ [N·m] | quedas sem → com | pitch RMS real sem → com | veredito |
|---|---|---|---|
| ≥ 8 | 0 → 0 | ~2,5 → ~2,4 (p ≥ 0,16) | controlador desnecessário |
| **6** | 0 → 0 | **5,54 → 3,18 (−43%, p = 2,6e−7)** | ganho de qualidade |
| **5** | **8/8 → 6/8** | mediana até a queda **4,6 → 10,6 s** (log-rank p=0,024) | ganho de sobrevivência |
| 4 | 5/5 → 5/5 | — | nada salva |

Custo: Δθ de pico ~3°, **0% do tempo em saturação**, distância preservada.

**O mecanismo é postura, não amortecimento.** Decompondo RMS² = média² +
variância a 6 N·m: a postura (|média|) cai 78% (4,23° → 0,92°) e a oscilação
(desvio) cai só 16%. **82% do ganho vem de postura.** A saturação de torque faz
o robô ceder para trás ~4°, e o estabilizador devolve o tronco à vertical com
uma correção quase constante. Isso casa com o modo de falha medido: quedas
sagitais para trás.

### Teste de perturbação (robô parado)

`push_test.py`, pulso de 5 N × 0,1 s: pico de pitch 6,08° → 3,09°, acomodação de
"não acomodou" para 0,82 s. Impulsos acima de ~1 N·s derrubam com ou sem
estabilizador — jogam o CoM para fora da região de captura.

---

## 5. Fase 3 — roll: não iniciada, e com uma armadilha conhecida

O roll é o **maior resíduo** em regime (6,4° contra 3,2° de pitch a 6 N·m) mas
**não é o modo de falha** — as quedas reais são sagitais.

A armadilha: a IK mantém `ankle_roll = −hip_roll`, ou seja, os pés são mantidos
paralelos ao solo por construção. Corrigir roll **só no quadril inclina o pé
lateralmente**, em vez de manter a planta no chão. Isso precisa ser resolvido no
projeto antes de implementar, senão a Fase 3 contamina também a Fase 2.

---

## 6. Fase 4 — ajuste fino: parcial

Feito: a correção migra entre as pernas conforme o estado de apoio, com
transição suave, e 0% de saturação durante a marcha.

Não feito: varredura de ganhos em marcha contínua. Os ganhos atuais (0,2 / 0,03)
são os iniciais conservadores. A razão `k_d/k_p ≈ 1/ω ≈ 0,17 s` colocaria o zero
do PD sobre a dinâmica do pêndulo (ω = √(g/z) ≈ 5,7 rad/s) — é o teto natural
para procurar. Recomendação: fazer isso **depois** do roll, porque com o plano
frontal solto é difícil separar ressonância de pitch de acoplamento lateral.

---

## 7. Testes offline (sem ROS)

Documentos executáveis das convenções, rodam em segundos:

```bash
cd src/marta_control
g++ -std=c++14 -Iinclude -I/usr/include/eigen3 \
    test/test_torso_attitude.cpp src/TorsoAttitudeEstimator.cpp -o /tmp/t1 && /tmp/t1
g++ -std=c++14 -Iinclude -I/usr/include/eigen3 \
    test/test_torso_stabilizer.cpp src/TorsoStabilizer.cpp src/Joints.cpp -o /tmp/t2 && /tmp/t2
```

O primeiro trava a convenção de sinais e imprime as tabelas usadas para escolher
`α` e o corte do giroscópio. O segundo trava o sinal da correção, a saturação, a
garantia de que a perna oscilante não é tocada e a convergência em malha fechada.

---

## 8. Limitações a declarar

- Ganhos sintonizados no Gazebo, onde as juntas são controladas por esforço com
  PID. No robô real são servos de posição, com dinâmica diferente — **não
  transferem direto**.
- `imu_mount_rpy` vale para a simulação; no robô real a montagem precisa ser
  medida (robô parado e vertical → os quatro campos de atitude em ~0;
  inclinando para frente à mão → pitch sobe positivo).
- O plano frontal segue em malha aberta.
- **Não há realimentação nenhuma em yaw** — nem estimativa (o filtro
  complementar só faz pitch e roll; sem magnetômetro não há referência absoluta)
  nem controle. O giro sistemático que se via na simulação era artefato do pé
  em malha (achado I do README); resta um resíduo de ~1 °/s.
