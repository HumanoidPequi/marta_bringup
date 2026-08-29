# Amanda Turtlesim Tutorial

Pacote desenvolvido para a Fase 3 do estudo direcionado de ROS, utilizando o `turtlesim` para praticar conceitos fundamentais de ROS 1 com `rospy`.

## Estrutura do pacote

```text
amanda_turtlesim_tutorial/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── turtlesimControle.launch
├── msg/
│   └── status.msg
├── scripts/
│   ├── decisao.py
│   ├── leitura.py
│   ├── quadrado.py
│   ├── diferenca_cliente.py
│   └── diferenca_servidor.py
└── srv/
    └── diferenca.srv
```

## Pré-requisitos

Este pacote foi desenvolvido utilizando:

* Ubuntu 20.04
* ROS 1 Noetic
* Python 3
* `rospy`
* `turtlesim`

Antes de executar os exemplos, carregue o ambiente do ROS:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

Também é necessário iniciar o ROS Master:

## 1. Executar o turtlesim

Para executar os nós individualmente, primeiro abra um terminal e execute:

```bash
roscore
```

Em outro terminal:

```bash
rosrun turtlesim turtlesim_node
```


---

## 2. Mover a tartaruga pela linha de comando

A velocidade da tartaruga pode ser publicada diretamente no tópico `/turtle1/cmd_vel`.

Exemplo:

```bash
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist \
'{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

Para publicar continuamente:

```bash
rostopic pub -r 10 /turtle1/cmd_vel geometry_msgs/Twist \
'{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

---



## 3. Publisher: quadrado

O arquivo `quadrado.py` implementa um publisher que publica velocidades no tópico:

```text
/turtle1/cmd_vel
```

Para executar:

```bash
rosrun amanda_turtlesim_tutorial quadrado.py
```

O nó utiliza `rospy` e `geometry_msgs/Twist` para controlar o movimento da tartaruga.

---

## 4. Subscriber: leitura da pose

O arquivo `leitura.py` funciona como subscriber do tópico:

```text
/turtle1/pose
```

Ele recebe e imprime informações sobre a posição e orientação da tartaruga.

Para executar:

```bash
rosrun amanda_turtlesim_tutorial leitura.py
```

---

## 5. Malha fechada

O arquivo `decisao.py` implementa uma malha fechada.


O nó recebe a pose da tartaruga através de `/turtle1/pose`, toma uma decisão com base nessa informação e publica a velocidade em `/turtle1/cmd_vel`.

Para executar:

```bash
rosrun amanda_turtlesim_tutorial decisao.py
```

---

## 7. Serviço próprio

O pacote possui o serviço customizado:

```text
srv/diferenca.srv
```

Definição:

```text
int64 a
int64 b
---
int64 sum
```

O serviço recebe dois valores e retorna a diferença entre eles.

### Servidor

O servidor está no arquivo:

```text
scripts/diferenca_servidor.py
```

Para executar:

```bash
rosrun amanda_turtlesim_tutorial diferenca_servidor.py
```

Após iniciar o servidor, o serviço `/diferenca` ficará disponível.

É possível verificar com:

```bash
rosservice list
```

### Cliente

O cliente está no arquivo:

```text
scripts/diferenca_cliente.py
```

O cliente recebe os valores pelos argumentos da linha de comando.

Exemplo:

```bash
rosrun amanda_turtlesim_tutorial diferenca_cliente.py 10 4
```

O resultado esperado é:

```text
6
```

Também é possível chamar o serviço diretamente pela CLI:

```bash
rosservice call /diferenca "a: 10
b: 4"
```

Resultado:

```text
sum: 6
```

---

## 7. Mensagem customizada

O pacote possui a mensagem:

```text
msg/status.msg
```

Definição:

```text
string nome
float64 bateria
bool em_movimento
```

---

## 8. Launch

O arquivo:

```text
launch/turtlesimControle.launch
```

inicia conjuntamente:

* o nó `turtlesim_node`
* o nó de controle `decisao.py`

Para executar:

```bash
roslaunch amanda_turtlesim_tutorial turtlesimControle.launch
```

Assim, não é necessário iniciar o `turtlesim` e o nó de controle separadamente.

O `roscore` deve estar executando antes do `roslaunch`, caso ele não seja iniciado automaticamente.

---

