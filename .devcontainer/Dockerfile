FROM osrf/ros:noetic-desktop-full

# Add vscode user with same UID and GID as your host system
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
# ARG USERNAME=vscode
# ARG USER_UID=1000
# ARG USER_GID=$USER_UID
# RUN groupadd --gid $USER_GID $USERNAME \
#     && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
#     && apt-get update \
#     && apt-get install -y sudo \
#     && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
#     && chmod 0440 /etc/sudoers.d/$USERNAME
# # Switch from root to user
# USER $USERNAME

# Add user to video group to allow access to webcam
# RUN sudo usermod --append --groups video $USERNAME

RUN set -o errexit
RUN set -o verbose

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install 3rd party packages
RUN sudo apt install -y git wget cmake python3-catkin-tools libeigen3-dev libsuitesparse-dev libceres-dev libomp-dev libpcl-dev

# Rosdep update
RUN sudo mkdir -p /etc/ros/rosdep/sources.list.d
RUN sudo wget https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/sources.list.d/20-default.list -O /etc/ros/rosdep/sources.list.d/20-default.list
ENV ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml 
RUN rosdep update

# install libnabo and libpointmatcher
WORKDIR /root
RUN git clone https://github.com/norlab-ulaval/libnabo.git
RUN mkdir -p /root/libnabo/build
WORKDIR /root/libnabo/build
RUN cmake ..
RUN make -j 4
RUN make install

WORKDIR /root
RUN git clone https://github.com/norlab-ulaval/libpointmatcher.git
RUN mkdir -p /root/libpointmatcher/build
WORKDIR /root/libpointmatcher/build
RUN cmake ..
RUN make -j 4
RUN make install

# install g2o
RUN sudo apt install -y libspdlog-dev libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5
WORKDIR /root
RUN git clone https://github.com/RainerKuemmerle/g2o.git
RUN cd /root/g2o && git checkout 20200410_git
RUN mkdir -p /root/g2o/build
WORKDIR /root/g2o/build
RUN cmake ..
RUN make -j 4
RUN make install

# Source the ROS setup file
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# display
ENV XDG_RUNTIME_DIR=/tmp/runtime-root/
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV QT_X11_NO_MITSHM=1

RUN mkdir -p /tmp/runtime-root/ && \
    chmod 0700 /tmp/runtime-root/
