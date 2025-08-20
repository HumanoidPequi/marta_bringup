## marta_bringup

Este repositório contém os arquivos necessários para iniciar e gerenciar a execução da Marta, o robô humanoide, dentro e fora do ambiente de simulação com ROS Noetic e Gazebo. Ele inclui configurações de lançamento, controladores, parâmetros e scripts essenciais para o controle da Marta.

## Índice

- [marta_bringup](#marta_bringup)
  - [Índice](#índice)
  - [Introdução](#introdução)
  - [Estrutura do Repositório](#estrutura-do-repositório)
  - [Requisitos](#requisitos)
  - [Getting Started](#getting-started)
    - [Clonando o Repositório e Configurando o Ambiente](#clonando-o-repositório-e-configurando-o-ambiente)
    - [Executando o Sistema com Docker](#executando-o-sistema-com-docker)
  - [Executando o Sistema](#executando-o-sistema)
  - [Como o repositorio funciona](#estruturação-dos-arquivos-de-lançamento)

## Introdução

O repositório `marta_bringup` é projetado para fornecer os arquivos necessários para inicializar e operar o robô Marta em um ambiente de simulação com Gazebo e ROS Noetic ou operar o hardware real. Ele inclui pacotes e arquivos de lançamento para configurar o robô, definir os parâmetros de controle e garantir a interação correta entre os diversos componentes da simulação.

## Estrutura do Repositório

- **src/**: Contém os pacotes necessarios para a execução da Marta.
- **src/marta_bringup**: Pacote do ros usado para iniciar a simulação ou a Marta real.
- **docker/**: Contem o arquivo Dockerfile para iniciar a simulação em Docker.

## Requisitos

- Docker (opcional)
- Ubuntu 20.04
- ROS Noetic
- Gazebo-Classic
- Pacote `ros_control` e dependências específicas de controladores

## Getting Started

### Clonando o Repositório e Configurando o Ambiente

1. Clone o repositório:

```
git clone git@github.com:HumanoidPequi/marta_bringup.git --recursive
```

2. Instale as dependências necessárias, como o `ros_control` e outros pacotes ROS:

```
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
```


3. A partir do repositorio compile o workspace e configure o ambiente:

```bash
catkin_make
source devel/setup.bash
```


### Executando o Sistema com Docker
Para usar o Docker, crie a imagem Docker com o arquivo Dockerfile incluído:

```
docker build -t marta_bringup:1.0 -f docker/Dockerfile .
```

Depois, inicie a imagem:

```
docker run -it --net=host -e DISPLAY=$DISPLAY --gpus all marta_bringup:1.0
```

#### Abrindo em outros terminais
Caso um container já esteja aberto e você quer criar outros terminais para acessarem esse container faça:

```
docker exec marta_bringup -it
```


## Executando o Sistema

Após a configuração do ambiente (seja nativo ou via Docker), você pode executar o sistema com o seguinte comando:

```
roslaunch marta_bringup bringup.launch sim:=true rviz:=false
```

Os parametros passados para o bringup.launch indicam as seguintes instruções de inicialização:

`sim:=true` true indica a inicialização da Marta no gazebo, use `false` para iniciar o bringup no hardware real

`rviz:=true` true indica que o rviz será iniciado na execucao do bringup, caso nao queira isso, use `false`


## Como o repositorio funciona

O arquivo de inicio é o `bringup.launch`. 
Se o parametro for `sim:=true` esse launch chama o description da Marta no pacote `marta_description` em modo de simulação, o que isso faz é definir todas as juntas e links no nó /tf, em seguida ele executa a simulação no pacote `marta_gazebo`, por fim o launch chama o no `real_to_sim_topics` do pacote `marta_gazebo` para criar os topicos `/marta/arm_l_head`, `/marta/arm_r`, `/marta/left_leg`, `/marta/right_leg` e converter esses topicos para as juntas da marta simulada.
Se o parametro for `sim:=false` esse launch inicia o description da Marta no modo real, em seguida ele inicia o nó `joint_to_tf_publisher` do pacote `marta_bringup`, o que ele faz é converter as mensagens `/marta/arm_l_head/state`, `/marta/arm_r/state`, `/marta/left_leg/state`, `/marta/right_leg/state` para as joints da Marta nas Tfs, como a Marta controla as juntas a partir das Tfs, isso fecha a malha.  
