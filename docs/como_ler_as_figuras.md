# Como ler cada figura

> Parte de [README.md](README.md). Gerador das figuras: [telemetria_marcha.md](telemetria_marcha.md).

Guia de interpretação das figuras geradas por
`experiments/figuras_marcha.py`. Cada seção diz **o que está plotado**, **o que
significa uma figura saudável**, **o que significa uma doente**, e **qual
pergunta ela responde**.

Base teórica: Kajita et al., *The 3D Linear Inverted Pendulum Mode*, IROS 2001.
As referências de equação abaixo são desse artigo.

Gerar tudo:

```bash
docker run --rm -v "$PWD":/ws -w /ws marta_bringup:1.0 bash -lc '
  source /opt/ros/noetic/setup.bash && source devel/setup.bash
  python3 src/marta_bringup/experiments/bag_para_csv.py \
      src/marta_bringup/experiments/bags/SEU.bag \
      src/marta_bringup/experiments/dados
  LIMITE_TORQUE=6 JANELA_S=4 python3 src/marta_bringup/experiments/figuras_marcha.py \
      src/marta_bringup/experiments/dados/SEU docs/figuras_marcha'
```

`LIMITE_TORQUE` marca o limite de torque nas figuras de junta; `JANELA_S` é a
largura da janela temporal (padrão 4 s, como o artigo — com 20 s de marcha
seriam ~50 passos e nada seria legível).

---

## 1. `fig_paper5_com_tempo` — reprodução da figura 5

**O que está plotado.** Em cima, posição do CoM; embaixo, velocidade. Ambos
**com a origem no pé de apoio corrente**, não no mundo. Cinza é o planejado pelo
gerador, colorido é o medido no Gazebo. As linhas verticais pontilhadas marcam
as trocas de modo de apoio.

**Por que as curvas saltam.** Porque a origem muda de pé a cada troca. É o mesmo
efeito descrito na legenda da figura 5 do artigo — *"the position graphs jump
the distance of the step length at each support foot exchange"*. Não é defeito.

**Saudável.** O medido acompanha o cinza em fase e amplitude; os dentes de serra
de `x` são regulares; `y` oscila em torno de zero com o período do passo.

**Doente.** Medido defasado do planejado (o robô não consegue seguir o
cronograma); amplitude de `x` crescendo passo a passo (energia acumulando);
`y` com viés para um lado (o robô pende).

**Responde:** *o robô segue a trajetória que o gerador planejou?*

---

## 2. `fig_paper4_plano_xy` — reprodução da figura 4

**O que está plotado.** Vista de cima do traço do CoM. Linha grossa contínua =
apoio simples; pontilhada fina = apoio duplo; círculos = pegadas.

No artigo os trechos de apoio simples são **arcos de hipérbole** e os de apoio
duplo são retas. Aqui a distinção é por cor e estilo, para que a mesma leitura
valha.

**Saudável.** Zigue-zague regular avançando numa direção, com as pegadas
alternando esquerda/direita em espaçamento constante.

**Doente.** Pegadas amontoadas (o robô não sai do lugar); traço derivando para
um lado (a marcha não segue reto); arcos de tamanhos muito diferentes entre
passos.

**Responde:** *a marcha tem a geometria que o modelo prevê?*

---

## 3. `fig_energia_orbital` — o teste mais direto do modelo

**O que está plotado.** A energia orbital das eq. (28)(29):

    E_x = −(g/2z_c)·x² + ½·ẋ²        E_y = −(g/2z_c)·y² + ½·ẏ²

Sob o 3D-LIPM ela é **invariante no tempo**. As faixas claras são as fases de
apoio simples.

- **Painel de cima:** o valor absoluto. Salta nas trocas de apoio, pelo mesmo
  motivo da figura 5 — a origem do frame mudou de pé.
- **Painel de baixo:** a mesma energia **menos o seu valor no início de cada
  apoio simples**. Cada fase começa em zero, e o que sobra é só a deriva de
  dentro dela.

**Saudável.** O painel de baixo próximo de uma reta em zero durante as faixas
claras. Entre as faixas (apoio duplo) a energia muda bastante — isso é
**intencional**: é ali que o gerador rampa o ZMP para injetar ou remover
energia.

**Doente.** Deriva grande dentro das faixas claras. Significa que o robô real
não está se comportando como o pêndulo invertido que o gerador assumiu.

**Responde:** *quanto a realidade viola o modelo?* É a métrica mais valiosa do
conjunto, porque não depende de escolha arbitrária — a invariância é uma
propriedade matemática do modelo, não uma convenção nossa.

> O número no subtítulo, *amplitude média de E_x por fase*, é essa deriva
> resumida em um escalar. Ele também vai para o CSV da campanha
> (coluna `deriva_energia`), o que permite compará-lo entre condições.

---

## 4. `fig_plano_fase` — a órbita prevista contra a medida

**O que está plotado.** O plano de fase sagital: posição `x` do CoM no frame do
apoio contra sua velocidade `ẋ`. Cada fase de apoio simples vira uma curva
(azul = apoio esquerdo, âmbar = direito). As curvas cinza finas são as
**hipérboles previstas** pela eq. (40) para a energia média daquela fase.

**Saudável.** As curvas medidas seguindo o desenho das cinzas.

**Doente.** Medido cruzando as hipérboles em vez de segui-las — a energia está
mudando quando não deveria; é a mesma informação da figura 3, em forma
geométrica.

**Responde:** *a dinâmica medida tem a forma que o modelo prevê?*

O subtítulo traz o **`z_c` medido**, que é a altura real do CoM acima do
contato. Compare com o `z_c` que o gerador assume (0,2975 m): a diferença é um
resultado por si só.

---

## 5. `fig_torque_juntas` — o mapa de calor do esforço

**O que está plotado.** Uma linha por junta de perna (quadril em cima, pé
embaixo, esquerda e direita separadas), tempo na horizontal, cor = torque
**pedido pelo controlador**. Azul e vermelho são sentidos opostos; a intensidade
é o módulo.

**Atenção à distinção que mais confunde:** este é o torque que o PID *pediu*,
antes da saturação. O que o motor de fato *aplicou* está em
`/marta/joint_states.effort`, limitado por `joint_effort`. O par define
saturação: se o pedido é 22 N·m e o aplicado 6, a junta está saturada em 3,8×.

**Saudável.** Cores pálidas, padrão periódico acompanhando o passo.

**Doente.** Faixas saturadas de cor numa junta específica — ela é o gargalo.

**Responde:** *qual junta está no limite, e quando?*

O subtítulo diz em que fração das amostras o pedido encostou no limite.

---

## 6. `fig_desvio_juntas` — quem deixa de seguir o comando

**O que está plotado.** Mesma disposição, mas a cor é o **erro de rastreamento**
em graus: `set_point − process_value`, direto do controlador.

**Saudável.** Erros de poucos graus, periódicos.

**Doente.** Uma junta com erro persistente e grande — ela recebeu um comando que
não consegue executar.

**Responde:** *qual junta não está fazendo o que foi mandado?*

Leia junto com a figura 5: **a junta que satura é a que deixa de rastrear**, e
essa é a ponte entre "faltou torque" e "o CoM saiu do plano".

---

## 7. `fig_ranking_juntas` — o resumo causal

**O que está plotado.** Dois painéis lado a lado, **na mesma ordem de juntas**:
à esquerda o torque de pico pedido (com o limite marcado); à direita o desvio de
pico. A ordem é por torque.

**Como ler.** Se as duas metades tiverem o mesmo perfil, a cadeia está
confirmada: *quem pede mais torque é quem menos rastreia*. Uma junta alta à
esquerda e baixa à direita significaria que ela pede muito mas ainda assim
consegue seguir — outra história.

**Responde:** *qual é o gargalo mecânico da marcha?*

---

## A cadeia causal que as figuras contam juntas

```
   fig 5/7          fig 6/7           fig 1              fig 3/4
saturação de  →  desvio de     →   CoM sai da    →   deriva de      →  queda
   torque          junta           trajetória      energia orbital
```

Cada seta é uma figura diferente, e é essa encadeamento que a telemetria
anterior não conseguia mostrar: ela media o fim da cadeia (RMS de pitch, queda)
sem nenhum dos elos.

---

## Inspeção ao vivo, sem gerar figura

Para olhar qualquer um desses sinais durante a marcha, ou para explorar um bag
antes de decidir o que plotar:

```bash
rosrun plotjuggler plotjuggler
```

As mensagens são tipadas, então os campos aparecem **nomeados**
(`/marta/walk/telemetry/orbital_energy_x`) em vez de `data[17]`. Para um plot XY,
segure Ctrl, selecione duas séries e arraste com o botão direito — é assim que
se reproduz a figura 4 interativamente, usando `com_x` e `com_y`.
