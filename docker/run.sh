#!/bin/bash

# Nome da imagem Docker
IMAGE_NAME="marta_bringup:1.0"
CONTAINER_NAME="marta_bringup"
ROS_IP=$(hostname -I | awk '{print $1}')

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
PARENT_DIR="$(dirname -- "$SCRIPT_DIR")"
echo "Diretório pai do script: $PARENT_DIR"

# Comando para inicializar o contêiner Docker com bash
# Força o GLX/OpenGL a usar o driver NVIDIA (evita llvmpipe/Mesa software no gzclient)
rocker --devices /dev/dri --nvidia --x11 \
  --name $CONTAINER_NAME \
  --network host \
  --env ROS_IP=$ROS_IP \
  --env DISPLAY=$DISPLAY \
  --env XAUTHORITY=$XAUTHORITY \
  --env __GLX_VENDOR_LIBRARY_NAME=nvidia \
  --env __NV_PRIME_RENDER_OFFLOAD=1 \
  --volume "$PARENT_DIR/src:/root/marta_bringup/src" \
  --volume /tmp:/tmp \
  --volume /var/log:/var/log \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  -- \
  $IMAGE_NAME \
  ${@:-"bash"}