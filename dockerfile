# Usamos la imagen oficial de ROS 2 Humble Desktop Full (incluye Gazebo y RViz)
FROM osrf/ros:humble-desktop-full

# Evitamos interacciones durante la instalación con apt
ENV DEBIAN_FRONTEND=noninteractive

# Actualizamos e instalamos herramientas esenciales, pip, git, Pinocchio y controladores de ROS 2
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    wget \
    curl \
    # MoveIt 2 para Humble
    ros-humble-moveit \
    ros-humble-moveit-visual-tools \
    # Controladores y ros2_control (¡Esto faltaba!)
    ros-humble-controller-manager \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-broadcaster \
    # Pinocchio (versión empaquetada para ROS Humble)
    ros-humble-pinocchio \
    # Herramientas de red y depuración para Foxglove / ROS
    ros-humble-rosbridge-suite \
    && rm -rf /var/lib/apt/lists/*

# Actualizamos pip globalmente por seguridad e instalamos paquetes comunes de python
RUN python3 -m pip install --no-cache-dir --upgrade pip setuptools wheel

# Creamos un usuario de trabajo no-root para evitar problemas de permisos con volúmenes
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
WORKDIR /home/$USERNAME

# Configuramos el entorno de ROS 2 automáticamente al abrir la terminal
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]