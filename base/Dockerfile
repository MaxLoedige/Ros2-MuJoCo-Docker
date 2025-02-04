ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO

ARG MUJOCO_VERSION=3.2.5
ARG BRANCH=main

WORKDIR /root/workdir
COPY ./src /root/workdir/src

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
RUN cd /root/workdir/ \
    && rosdep fix-permissions && rosdep update \
    && rosdep install --from-paths src --ignore-src -y

RUN echo 'export MUJOCO_DIR=/root/workdir/mujoco/mujoco-$MUJOCO_VERSION' >> ~/.bashrc
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

RUN bash -c "source ~/.bashrc"