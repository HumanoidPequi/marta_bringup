# amanda_marta_vision_hello

Pacote ROS 1 Noetic para processamento de imagens da câmera da robô Marta.

O nó recebe imagens ROS, converte as imagens coloridas para tons de cinza utilizando OpenCV e publica o resultado em outro tópico ROS.


## Tópicos

### Entrada

`/marta/front_camera/image_raw`

Tipo: `sensor_msgs/Image`

Recebe as imagens coloridas da câmera.

### Saída

`/marta/front_camera/image_gray`

Tipo: `sensor_msgs/Image`

Publica as imagens convertidas para tons de cinza.

## Instalação

Clone ou copie o pacote para a pasta `src` do workspace catkin.

Entre no workspace:

```bash
cd ~/catkin_ws
```

Construa o workspace:

```bash
catkin_make
```

Carregue o ambiente:

```bash
source ~/catkin_ws/devel/setup.bash
```

## Execução com a simulação

Em um terminal, carregue o ambiente ROS e inicie a simulação da Marta.
Em outro terminal, carregue o ambiente e execute o pacote:

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch amanda_marta_vision_hello vision.launch
```

## Execução com rosbag

O rosbag precisa conter imagens no tópico de entrada configurado.

Em um terminal:

```bash
source ~/catkin_ws/devel/setup.bash
rosbag play --clock --loop ~/bagfiles/camera_marta.bag
```

Em outro terminal:

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch amanda_marta_vision_hello vision.launch
```

## Visualização da imagem

Com o rosbag ou a simulação e o nó de processamento em execução:

```bash
rqt_image_view
```

Selecione o tópico:

```text
/marta/front_camera/image_gray
```

## Funcionamento

1. O nó se inscreve no tópico de entrada.
2. Recebe mensagens do tipo `sensor_msgs/Image`.
3. Converte as mensagens para imagens OpenCV usando `cv_bridge`.
4. Converte as imagens coloridas para tons de cinza usando OpenCV.
5. Converte o resultado para uma mensagem ROS com codificação `mono8`.
6. Publica a imagem processada no tópico de saída.

