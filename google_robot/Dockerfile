ARG ROS_DISTRO=humble
FROM ros:humble

ARG MUJOCO_VERSION=3.3.2
ARG BRANCH=main

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /root/workdir
COPY ./src /root/workdir/src

# TODO currently the ros docker have unsecured repositories, rosdep needs to be called manualy 
# RUN apt-get update --allow-insecure-repositories && apt-get upgrade -y --allow-unauthenticated
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip wget
ENV SHELL /bin/bash

RUN pip3 install xacro

# install mujoco
ENV MUJOCO_VERSION $MUJOCO_VERSION
RUN apt-get install -y libglfw3-dev
# RUN wget https://github.com/google-deepmind/mujoco/releases/download/3.2.5/mujoco-3.2.5-linux-x86_64.tar.gz
RUN wget https://github.com/google-deepmind/mujoco/releases/download/$MUJOCO_VERSION/mujoco-$MUJOCO_VERSION-linux-x86_64.tar.gz
# RUN wget https://github.com/google-deepmind/mujoco/releases/download/$MUJOCO_VERSION/mujoco-$MUJOCO_VERSION-linux-x86_64.tar.gz
RUN mkdir -p /root/workdir/mujoco
RUN tar -xzf mujoco-$MUJOCO_VERSION-linux-x86_64.tar.gz -C "/root/workdir/mujoco"

# && git clone -b $BRANCH https://github.com/moveit/mujoco_ros2_control.git \
# RUN cd /root/workdir/ \
#     && rosdep fix-permissions
# RUN rosdep update
# RUN rosdep install --from-paths src --ignore-src -y

RUN echo 'export MUJOCO_DIR=/root/workdir/mujoco/mujoco-$MUJOCO_VERSION' >> ~/.bashrc
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    apt-get install -y --no-install-recommends\
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
        zlib1g-dev\
        libgl1-mesa-dev \
        # libgl1-mesa-glx \
        libglew-dev \
        libosmesa6-dev \
        software-properties-common \
        net-tools \
        virtualenv \
        wget \
        xpra \
        xserver-xorg-dev \
        libglfw3 \
        libglfw3-dev \
        patchelf \
        unzip \
        libxrandr2 \
        libxrandr-dev\
        libxinerama1 \
        libxcursor1 \
        vim \
        libxinerama-dev \ 
        libxcursor-dev \
        libxi-dev \
        libtinyxml2-dev \
        libspdlog-dev\
        libabsl-dev \
        libbenchmark-dev libgtest-dev\
        protobuf-compiler libprotobuf-dev \
        python3-pip \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends\
        ros-dev-tools \
        ros-humble-sdformat-urdf \
        ros-humble-joint-state-publisher-gui \
        ros-humble-ros2controlcli \
        ros-humble-controller-interface \
        ros-humble-hardware-interface-testing \
        ros-humble-ament-cmake-clang-format \
        ros-humble-ament-cmake-clang-tidy \
        ros-humble-controller-manager \
        ros-humble-ros2-control-test-assets \
        ros-humble-hardware-interface \
        ros-humble-control-msgs \
        ros-humble-backward-ros \
        ros-humble-generate-parameter-library \
        ros-humble-realtime-tools \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-broadcaster \
        ros-humble-moveit \
        ros-humble-moveit-ros-move-group \
        ros-humble-moveit-kinematics \
        ros-humble-moveit-planners-ompl \
        ros-humble-moveit-ros-visualization \
        ros-humble-joint-trajectory-controller \
        ros-humble-moveit-simple-controller-manager \
        ros-humble-rviz2 \
        ros-humble-xacro \
        ros-humble-position-controllers \
        ros-humble-gripper-controllers \
        ros-humble-diff-drive-controller \
        ros-humble-warehouse-ros\
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*

RUN cd /root/workdir/ \
    && apt-get update \
    && rosdep fix-permissions \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN bash -c "source ~/.bashrc"