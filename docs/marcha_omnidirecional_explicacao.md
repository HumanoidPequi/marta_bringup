# Marcha omnidirecional da Marta: explicação para a reunião

> Narrativa do estudo, na ordem em que o raciocínio se constrói. Os números,
> as tabelas e o método completos estão em
> [marcha_omnidirecional.md](marcha_omnidirecional.md).
>
> **Fato** = medido nas campanhas. **Inferência** = interpretação, marcada
> como tal.

## 1. O problema

Até aqui, a Marta só andava para a frente. O objetivo agora é uma marcha
**omnidirecional**: andar em qualquer direção e girar.
- **Mínimo:** girar no próprio eixo e andar na direção comandada.
- **Depois:** curva, passo lateral, diagonal e movimentos combinados.

Junto vêm duas perguntas de controle:
1. Um **estabilizador de roll** (o tombo lateral do tronco) ajuda, em especial
   no giro?
2. Os ganhos de **pitch** e de **rumo**, sintonizados num regime antigo,
   continuam bons para toda a faixa de massas e movimentos?

## 2. O ponto de partida: o gerador já é omnidirecional

O gerador de passos da Marta (`OmnidirectionalWalkZMP`) vem do mestrado do
Maximo (ITA, 2015; `~/Documents/ic/study/Mestrado Marcos Maximo-1.pdf`):
- ele recebe três velocidades: frente (vx), lado (vy) e giro (vψ);
- giro e passo lateral avançam em passos alternados, abrindo e fechando as
  pernas (§3, p. 56);
- um limitador impede que as pernas se cruzem.

**O Maximo testou** lateral, giro no lugar (π/18 rad/s), diagonal e curva
(Tabela 5.4, p. 86). No giro, chegou a 80° em 8 s (p. 90).

**Mas os testes dele foram**:
- num Bioloid simulado no V-REP;
- **com o estabilizador desligado** (p. 86);
- sem medir quedas.

**Na Marta, com servo MX-28 realista e 3,3–3,8 kg, isso nunca tinha sido
testado.** Este estudo acrescenta:
- os movimentos na Marta;
- os estabilizadores;
- um mapa de onde a marcha sobrevive.

## 3. Como foi testado, e por quê assim

**Uma corrida = um `roslaunch` de 30 s numa condição fixa** (massa, movimento,
velocidade), porque as velocidades só são lidas na partida. As duas massas
são as melhores receitas do panorama anterior:
- **M33:** 3,3 kg;
- **M38:** 3,8 kg.

**Por que não repetir a mesma condição várias vezes?** A simulação é quase
determinística. Repetir costuma dar o mesmo resultado, e contar repetições
como amostras independentes infla a estatística.
- **A replicação vem de fatores reais:** o pé que dá o primeiro passo e o
  sentido do movimento (esquerda/direita).
- **A comparação é pareada por condição.** Um braço com roll e outro sem, na
  mesma condição.
- **Holm** corrige os testes múltiplos.

**Uma descoberta de método no caminho (fato).** "Quase determinística" não é
"determinística". Em condições na borda da queda, corridas idênticas divergem
pelo ruído de temporização do ROS. O giro do M33, com a mesma configuração,
durou 6,8; 6,9; 12,6; 12,8 e 17,6 s em cinco corridas.
- **O que se repete é o desfecho binário:** "anda os 30 s" ou "cai logo na
  partida".
- **O instante de uma queda na borda não se repete.**
- **Por isso, as conclusões se apoiam em desfechos e em padrões que se
  repetem em várias condições, nunca no tempo de queda de uma corrida.** Uma
  conclusão da primeira fase foi retirada por esse motivo (§6).

**Outra descoberta de método (fato, verificada por diagnóstico).** O campo
simulado (`world.world`) tem dois objetos com colisão no caminho da reta:
- o **gol** em x = 3 m: a 0,1 m/s, o robô chega lá exatamente aos 30 s;
- uma **bola de 24 cm e 26 g** em (2,0; −0,2), deslocada da posição
  declarada por um `<pose>` no modelo.

Sem gol e sem bola (mesma física), a reta 0,15 e as duas diagonais do M38
que "caíram" andam os 30 s. **A "janela de velocidade" da reta, vista na
primeira fase, era o gol, e foi retirada.** Giro, curva e lateral não chegam
perto de nenhum dos dois, e suas quedas sem roll se repetem sem eles.

**Métricas novas**, todas a partir da verdade de solo do Gazebo:
- velocidade realizada no frame do corpo e razão realizada/comandada;
- erro de direção;
- escorregamento do pé;
- distância entre os pés;
- roll no meio do apoio simples (A_roll).

**Correção de método no LIPM.** A telemetria antiga usava os eixos do mundo
para as grandezas do pêndulo invertido linear. Isso vale em linha reta, mas
não no giro. Elas passaram a ser calculadas no frame do pé de apoio, girado
pelo yaw dele (Kajita et al., IROS 2001, `kajita_.pdf`, eq. 28–29 e 40).

## 4. O que cada controlador faz

Todos ficam **depois** da cinemática inversa: o gerador é malha aberta, e os
controladores somam uma correção em cima dele. É a arquitetura do Maximo
(doutorado, Fig. 4.1, p. 73–74). Isso permite ligar e desligar cada um e
comparar com e sem.

| controlador | o que mede | o que corrige | referência |
|---|---|---|---|
| **pitch** | inclinação frente/trás do tronco | hip pitch da perna de apoio, P+D | Raibert, cap. 2–3; [estabilizadores.md](estabilizadores.md) §2 |
| **rumo** | yaw integrado do giroscópio contra a referência (que avança com o giro comandado) | soma −kp·erro, saturado, ao comando de giro | [desvio_de_direcao.md](desvio_de_direcao.md) §6; Maximo (doutorado, §4.4.1, p. 84): yaw não observável, fica numa camada acima |
| **compensação de gravidade** | nada: é feedforward | adianta, no hip roll e no joelho, o erro de posição que o servo precisa para fazer torque | Maximo, doutorado, §4.3, eq. 4.18–4.25, p. 78–81 |
| **roll (novo)** | roll do tronco menos a sua média lenta (*washout*) | hip roll e ankle roll da perna de apoio, P+D, ponderados pela carga de cada perna | Maximo, doutorado, §4.4.2, eq. 4.34–4.35, p. 85; Raibert, cap. 3 |

**Detalhes do roll que importam na reunião:**
- **Por que *washout*?** A estimativa de roll tinha viés (+2,3° no regime
  antigo). Um P puro com referência zero comandaria uma inclinação
  permanente. O P age sobre φ − φ̄, com φ̄ um passa-baixa de 2 s, que tira o
  viés e preserva a alternância passo a passo (99% a 0,83 Hz, no teste
  offline).
- **Por que ponderar pela carga?** A correção entra na perna de apoio, que
  sustenta o tronco. A perna em balanço não é tocada, então a colocação do
  pé não muda.
- **Diferença em relação ao mestrado do Maximo:** o estabilizador do mestrado
  é **só derivativo** (θ += K·ω; §3.9, p. 72–73). Aqui, o só derivativo foi
  testado e **não bastou** (§6). O que segura a Marta é o termo
  proporcional.
- **Segurança:** desligado por padrão e forçado a desligado no robô real. Os
  ganhos são de simulação.
- **Testes offline:** 14 verificações (sinal, perna em balanço, saturação,
  washout).

## 5. A linha de base: o que acontece sem roll (E1, 128 corridas)

**Fatos:**
1. **O mínimo não é atingido sem roll.** O giro no lugar completou 1 vez em
   32. Ele sobrevive ~10 s no M33 e ~5 s no M38.
2. **Mecanismo do giro (M33).** A oscilação de roll cresce a cada passo: ±3°,
   depois ±8°, 14° e a queda. Em todas as 20 quedas no giro, o roll passa de
   15° antes do pitch.
3. **No M38, quase tudo cai na partida.** 43 de 47 quedas acontecem antes de
   8 s, e o roll chega a ~12° nos primeiros passos. Nos dois pés iniciais, só
   andam a reta 0,10, a curva (0,1; −0,10) e o combinado +0,175 (e a diagonal
   +0,03, descontado o gol).
4. **Quando anda, segue o comando.** Na curva e no combinado, ω realizado
   fica entre 1,00 e 1,01 do comandado. O lateral é a exceção: ~30° de erro
   de direção, por um avanço espúrio de ~1 cm/s.
5. **Na curva, o pé inicial importa** (análise exploratória). Começar com o
   pé do lado para onde se gira completa muito mais: 8 a 1 nos pares que
   diferem, p = 0,039.

## 6. Ganhos de pitch e de rumo (E2, 132 corridas)

- **Rumo:** os ganhos atuais (1,0 / 0,3 rad/s) estão bons em toda a faixa.
  - Nenhuma das 5 alternativas foi melhor.
  - O erro de rumo nas corridas completas fica em 1–9°.
  - O termo integral (previsto se o erro passasse de ~10°) **não foi
    necessário**.
- **Pitch:** pelo critério pré-registrado, **nenhuma combinação supera a
  atual**.
  - **Padrão exploratório:** kd = 0,06 completa mais fora da reta (6/18
    contra 2/36, p = 0,012), no lateral e na curva do M33.
  - **Nenhum ganho de pitch salva o giro**: 0/18. Coerente com o giro cair
    por roll.
  - **Fechado na E7 (com roll):** kd = 0,06 reduz o pitch RMS em 10%,
    melhor em 41 de 44 pares, e foi **adotado**.
- **Uma conclusão retirada.** Na E1, desligar a malha de rumo parecia
  alongar o giro (p = 0,016). As réplicas exatas da E2 mostraram que a
  diferença está dentro do ruído entre corridas idênticas. Saiu do
  relatório.

**Exploração de partida (E1b, fora do plano):**
- **A "partida suave" que já existia no código derruba o robô neste regime.**
  Ela reduz o balanço lateral do CoM, mas continua levantando o pé, e o robô
  tomba (inferência pelo código).
- **Mover o quadril 5 mm à frente (`hip_offset_x`) ajuda** algumas condições
  do M38. Fica como candidato.

## 7. O estabilizador de roll (E4: piloto; E5: confirmação)

**E4, piloto (52 corridas):** 10 variantes, em giro +0,175, lateral ±0,05 e
reta, nas duas massas:
- só quadril (a "cintura" do Raibert);
- quadril + tornozelo (o desenho do Maximo);
- só tornozelo;
- só derivativo.

**Fatos:**
1. **Com termo proporcional, o roll resolve o giro e o lateral nas duas
   massas.**
   - M33: 35 de 36 corridas completaram, contra 1 de 4 sem roll.
   - M38: 12 de 12, contra 1 de 4.
2. **O giro passa a seguir o comando:** ω realizado entre 0,997 e 1,004 do
   comandado. O pé de apoio quase não escorrega (< 1 °/s).
3. **O roll RMS no giro cai de 8,3° para 1,4–2,5°.** A correção não satura.
   Na reta, o pitch da escolhida ficou dentro do portão de +10% (+9% no M33),
   mas a E5 mostrou +10–19% (abaixo).
4. **Só derivativo não basta:** 2 de 4. O ingrediente é o proporcional.
5. **Escolhida para a confirmação: quadril + tornozelo, K = 1** (kp 0,5 e kd
   0,075 em cada junta). É a de menor roll RMS médio nas duas massas.

**E5, confirmação (88 corridas, intercaladas e pareadas).** São 11
movimentos × 2 pés × 2 massas, com e sem roll, em ordem embaralhada na mesma
campanha.

**Fatos:**
1. **Completaram 42 de 44 com roll, contra 13 de 44 sem.**
   - Giro (±0,175 e ±0,35): **16 de 16 com roll, 0 de 16 sem** (8 por
     massa). Lateral ±0,05: **8 de 8 com roll, 0 de 8 sem**.
   - As 2 falhas com roll são diagonais do M38: uma no gol e outra, por
     inferência, na bola. Sem os obstáculos, as duas andam.
2. **Os três testes pré-registrados passam** (Holm):
   - sobrevivência: p = 4·10⁻⁷;
   - oscilação de roll no apoio: p = 0,04 (o roll RMS cai de 1,7–18° para
     1,0–2,3°);
   - seguimento do comando: p = 0,008.
3. **O mínimo pedido é atingido nas duas massas:**
   - o giro no lugar segue o comando com ω entre 0,99 e 1,01;
   - o pé de apoio quase não escorrega (< 1 °/s);
   - a reta e a diagonal saem na direção comandada (±4°).
4. **O roll não corrige o lateral.** O robô anda de lado com um avanço
   espúrio de ~1 cm/s, que vem do gerador. O erro de direção é de 12–19° a
   0,05 m/s e de 26–34° a 0,025 m/s.
5. **Custo:** na reta do M33, o pitch RMS sobe 10–19% (2,0–2,1° → 2,3–2,4°).
   No M38, sobe 2–5%.

**E6env, o resto da faixa com roll (56 corridas):**
- completou 51 de 52 fora do gol;
- giro ±0,10 e ±0,25, curva até ±0,25 e lateral ±0,025: tudo nas duas
  massas;
- a única queda foi o combinado (0,1; 0,03; −0,175) do M38 com o pé
  esquerdo, na partida.

**Leitura (inferência):** o termo proporcional sobre φ − φ̄ corta o
crescimento da oscilação lateral passo a passo, que era o mecanismo de queda
do giro (§5). Com isso, o giro deixa de ser o limite da marcha.

## 8. Como ler a demo no Foxglove

O layout é `src/marta_bringup/foxglove/marta_omni.json`; os dados vêm do nó
`viz_marcha.py`.

| aba | o que mostrar | o que se espera ver |
|---|---|---|
| Torque / limite | torque pedido / máximo, nas 12 juntas das pernas | linha 1,0 = saturação. O joelho de apoio chega perto; os de roll ficam longe |
| Erro das juntas | alvo − real [°] | a cedência dos servos. Cresce no joelho de apoio |
| Controladores | pitch, roll (φ − φ̄, correções de quadril e tornozelo), rumo, compensação | no giro com roll, o roll verdadeiro fica entre −2° e +4°; sem roll, cresce de ±5° para ±13° e o robô cai |
| LIPM (Kajita) | CoM × referência, energia orbital, DCM | o CoM segue a referência a ~5 mm na reta e a ~10 mm no giro com roll; sem roll, a 25–42 mm antes de cair |
| Marcha | estado de apoio, taxa de giro | o giro em ~0,175 rad/s, constante |

**Roteiro** (bags em `experiments/omni/demo/`; detalhes e comandos ao vivo
em [marcha_omnidirecional.md](marcha_omnidirecional.md), seção Roteiro):
1. **Giro 3,3 kg sem roll** (`D_M33_off_giro…`): a oscilação de roll cresce,
   a taxa de giro fica abaixo do comando, e o robô cai aos 11,6 s.
2. **O mesmo giro com roll** (`D_M33_roll_giro…`): 30 s girando a 0,175
   rad/s. As correções de quadril e tornozelo alternam com a perna de apoio.
3. **3,8 kg sem e com roll** (`D_M38_off_giro…`, `D_M38_roll_giro…`): a
   queda na partida some.
4. **Lateral** (`D_M33_roll_lat…`): anda de lado e um pouco à frente, que é
   o erro que fica para o gerador.
5. **LIPM** (reta e giro): o modelo do Kajita continua descrevendo o CoM nos
   movimentos novos. O erro é de ~5 mm na reta e de ~10 mm no giro com roll.
   Figuras estáticas em `docs/figuras_marcha/lipm_omni/`.

## 9. Ressalvas

- **Tudo é simulação** (Gazebo/ODE). Os ganhos de roll não valem no robô
  real sem nova sintonia, e o código força o roll a desligado no robô real.
- **Desfechos na borda variam entre corridas idênticas.** Os números de
  sobrevida isolados não devem ser citados como precisos.
- **O pé plano não foi medido diretamente.** O ank_roll não satura, o que é
  indício, não prova.
- **A análise do pé inicial na curva é exploratória.** É hipótese a
  confirmar, não resultado. O kd 0,06 foi confirmado com roll na E7.
- **Obstáculos no campo:** a reta e a diagonal passam pela bola (~19 s) e
  chegam ao gol (~30 s). As campanhas automáticas passam a usar o campo
  livre; as demonstrações seguem no campo completo.
- **Uma corrida por condição:** o desfecho de uma condição na borda pode
  mudar entre corridas idênticas. Os números por célula valem como padrão,
  não como taxa.

## 10. Decisões

Tomadas em 2026-11-12:
1. **Roll:** quadril + tornozelo K1 (o confirmado na E5), **adotado**.
2. **Pitch e `hip_offset_x` +5 mm:** testados **com** roll na E7, um
   fatorial pré-registrado de 176 corridas (relatório, seção E7).
   - **kd 0,06 adotado:** −10% de pitch RMS, melhor em 41 de 44 pares; o
     ganho é maior no giro e no lateral.
   - **x +5 mm não adotado:** inclina o tronco +1,6° para frente, sem ganho
     de sobrevivência, que já estava em 44/44.
3. **Mundo:** campanhas automáticas no campo livre; demonstrações e
   comandos à mão no campo completo.

**Próximo passo técnico:** o avanço espúrio do gerador no lateral e no
giro.
