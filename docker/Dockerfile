# Use a imagem base do ROS Noetic com suporte a GPU
FROM ros:noetic-ros-core

# Comente temporariamente a fonte do ROS
RUN sed -i 's|^deb .*ros.*|#&|' /etc/apt/sources.list.d/*.list || true

# Atualize e instale curl e gnupg sem acessar o ROS
RUN apt-get update && apt-get install -y curl gnupg

# Etapa 3: Baixe e instale a nova chave GPG do ROS
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Etapa 4: Reative o repositório ROS usando o formato moderno com signed-by
RUN echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list

# Instale dependências necessárias
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-catkin-tools \
    build-essential \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Instale o pacote ROS desejado
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-swri-console \
    ros-$ROS_DISTRO-swri-roscpp \
    ros-$ROS_DISTRO-ros-control \
    ros-$ROS_DISTRO-ros-controllers \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-gazebo-ros-control \
    ros-$ROS_DISTRO-gazebo-plugins \
    ros-$ROS_DISTRO-gazebo-ros \
    ros-$ROS_DISTRO-rviz \
    && rm -rf /var/lib/apt/lists/*


RUN pip3 install \
    pandas \
    onnxruntime


# Copiar o diretório pacotes da Marta
RUN mkdir -p /root/marta_bringup/src
COPY /src /root/marta_bringup/src

COPY /src/marta_gazebo/src/worlds/robocup_3Dsim_ball /root/.gazebo/models/robocup_3Dsim_ball
COPY /src/marta_gazebo/src/worlds/robocup09_spl_field /root/.gazebo/models/robocup09_spl_field

# Construir o workspace
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && cd /root/marta_bringup && catkin_make"

# Fazer o source do setup.bash do workspace
RUN echo "source /root/marta_bringup/devel/setup.bash" >> ~/.bashrc

RUN echo "export LIBGL_ALWAYS_INDIRECT=0" >> ~/.bashrc

WORKDIR /root/marta_bringup

# Definir o comando padrão
CMD ["bash"]