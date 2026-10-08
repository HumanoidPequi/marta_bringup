#!/bin/bash
set -e

# Nome da imagem Docker
IMAGE_NAME="marta_bringup:1.0"
CONTAINER_NAME="marta_bringup"
ROS_IP=$(hostname -I | awk '{print $1}')

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
PARENT_DIR="$(dirname -- "$SCRIPT_DIR")"
echo "Diretório pai do script: $PARENT_DIR"

# Comando padrão: sobe a simulação. Pode ser sobrescrito passando argumentos,
# ex: ./run.sh bash   ou   ./run.sh roslaunch marta_bringup bringup.launch sim:=false
if [ $# -eq 0 ]; then
  CMD=(roslaunch marta_bringup bringup.launch sim:=true rviz:=false)
else
  CMD=("$@")
fi

# Se o container já estiver de pé, apenas abre outro terminal nele
if [ -n "$(docker ps -q -f name="^${CONTAINER_NAME}$")" ]; then
  echo "Container $CONTAINER_NAME já está rodando; abrindo novo terminal nele."
  exec docker exec -it "$CONTAINER_NAME" bash
fi

# Libera o X do host para o root (usuário do container)
xhost +local:root >/dev/null

# Sem rocker: a imagem já traz libGL/GLVND e as variáveis NVIDIA, então não há
# build a cada execução — só sobe o container.
docker run --rm -it \
  --name "$CONTAINER_NAME" \
  --network host \
  --gpus all \
  --device /dev/dri \
  -e ROS_IP="$ROS_IP" \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
  -e __NV_PRIME_RENDER_OFFLOAD=1 \
  -v "$PARENT_DIR/src:/root/marta_bringup/src" \
  -v /tmp:/tmp \
  -v /var/log:/var/log \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /etc/localtime:/etc/localtime:ro \
  "$IMAGE_NAME" \
  bash -c 'source /root/marta_bringup/devel/setup.bash && exec "$@"' -- "${CMD[@]}"
