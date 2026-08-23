#!/bin/bash

# Habilitar permisos de interfaz gráfica para el contenedor
xhost +SI:localuser:root

echo "Iniciando Agente micro-ROS para STM32B..."
docker run --rm -d --name microrosB --net=host \
  --device=/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_33A2327A3237-if00 \
  microros/micro-ros-agent:humble \
  serial --dev /dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_33A2327A3237-if00 -v6

sleep 2

echo "Iniciando Agente micro-ROS para STM32A..."
docker run --rm -d --name microrosA --net=host \
  --device=/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_3579374C3437-if00 \
  microros/micro-ros-agent:humble \
  serial --dev /dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_3579374C3437-if00 -v6

sleep 2

echo "Levantando contenedor principal de ROS2 con soporte NVIDIA..."
./gui-docker \
    -v $HOME/Documentos/ROS2/NXT:/home/developer/ros_ws \
    --rm -it \
    --net=host \
    --privileged \
    --runtime=nvidia --gpus all \
    --device=/dev/nvidia-uvm \
    --device=/dev/nvidia-uvm-tools \
    --device=/dev/nvidia-modeset \
    --device=/dev/nvidiactl \
    --device=/dev/nvidia0 \
    ros2-humble-env:latest /bin/bash