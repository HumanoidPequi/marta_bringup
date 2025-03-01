#!/bin/bash

# Nome da imagem Docker
IMAGE_NAME="marta_bringup:1.0"
CONTAINER_NAME="marta_bringup"
ROS_IP=$(hostname -I | awk '{print $1}')

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
PARENT_DIR="$(dirname -- "$SCRIPT_DIR")"
echo "Diretório pai do script: $PARENT_DIR"

# Comando para inicializar o contêiner Docker com bash
rocker --device /dev/dri --x11 \
  --name $CONTAINER_NAME \
  --network host \
  --env ROS_IP=$ROS_IP \
  --env DISPLAY=$DISPLAY \
  --env XAUTHORITY=$XAUTHORITY \
  --oyr-run-arg " -v $PARENT_DIR/src:/root/marta_bringup/src -v /tmp:/tmp -v /var/log:/var/log -v /tmp/.X11-unix:/tmp/.X11-unix" \
  $IMAGE_NAME \
  ${@:-"bash"}