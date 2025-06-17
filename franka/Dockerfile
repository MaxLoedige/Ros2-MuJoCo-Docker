ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO

ARG MUJOCO_VERSION=3.2.5
ARG BRANCH=main

WORKDIR /root/workdir

# TODO make the first part of the path an ARG
COPY ./demos/Ros2-MuJoCo-Docker/src/mujoco_ros2_control /root/workdir/src/mujoco_ros2_control
COPY ./demos/Ros2-MuJoCo-Docker/franka/src/mujoco_ros2_control_examples /root/workdir/src/mujoco_ros2_control_examples
COPY ./demos/Ros2-MuJoCo-Docker/franka/src/franka_ros2 /root/workdir/src/franka_ros2

RUN apt-get update && apt-get upgrade -y
RUN apt-get update && apt-get install -y --no-install-recommends \
    libosmesa6-dev \
    libgl1-mesa-glx \
    libglfw3-dev \
    libglew-dev \
    libabsl-dev libtinyxml2-dev libgtest-dev libbenchmark-dev\
    patchelf \
    unzip -y \
    libxrandr2 \
    libxrandr-dev\
    libxinerama1 \
    libxcursor1 \
    wget \
    libxinerama-dev \ 
    libxcursor-dev \
    libxi-dev \
    protobuf-compiler libprotobuf-dev \
	python3-pip \
    && rm -rf /var/lib/apt/lists/*

ENV SHELL /bin/bash

RUN pip3 install xacro

# install mujoco
ENV MUJOCO_VERSION $MUJOCO_VERSION
# RUN apt-get install -y libglfw3-dev
# RUN wget https://github.com/google-deepmind/mujoco/releases/download/3.2.5/mujoco-3.2.5-linux-x86_64.tar.gz
RUN wget https://github.com/google-deepmind/mujoco/releases/download/$MUJOCO_VERSION/mujoco-$MUJOCO_VERSION-linux-x86_64.tar.gz
RUN mkdir -p /root/workdir/mujoco
RUN tar -xzf mujoco-$MUJOCO_VERSION-linux-x86_64.tar.gz -C "/root/workdir/mujoco"


RUN echo 'export MUJOCO_DIR=/root/workdir/mujoco/mujoco-$MUJOCO_VERSION' >> ~/.bashrc
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

## === Installing Franka === ##

# Install essential packages and ROS development tools
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        bash-completion \
        curl \
        gdb \
        git \
        nano \
        openssh-client \
        python3-colcon-argcomplete \
        python3-colcon-common-extensions \
        sudo \
        vim \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends \
        ros-dev-tools \
        ros-humble-franka-description \
        ros-humble-libfranka \
        ros-humble-ros-gz \
        ros-humble-sdformat-urdf \
        ros-humble-joint-state-publisher-gui \
        ros-humble-ros2controlcli \
        ros-humble-controller-interface \
        ros-humble-hardware-interface-testing \
        ros-humble-ament-cmake-clang-format \
        ros-humble-ament-cmake-clang-tidy \
        ros-humble-controller-manager \
        ros-humble-ros2-control-test-assets \
        libignition-gazebo6-dev \
        libignition-plugin-dev \
        ros-humble-hardware-interface \
        ros-humble-control-msgs \
        ros-humble-backward-ros \
        ros-humble-generate-parameter-library \
        ros-humble-realtime-tools \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-broadcaster \
        ros-humble-moveit-ros-move-group \
        ros-humble-moveit-kinematics \
        ros-humble-moveit-planners-ompl \
        ros-humble-moveit-ros-visualization \
        ros-humble-joint-trajectory-controller \
        ros-humble-moveit-simple-controller-manager \
        ros-humble-rviz2 \
        ros-humble-xacro \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*

RUN cd /root/workdir/ \
    && sudo apt-get update \
    && rosdep fix-permissions \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -y \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*




RUN bash -c "source ~/.bashrc"