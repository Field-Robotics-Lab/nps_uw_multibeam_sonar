FROM osrf/ros:noetic-desktop-full

# Required utilities
RUN apt update \
 && apt install -y --no-install-recommends \
        build-essential \
        cmake \
        cppcheck \
        curl \
        git \
        gnupg \
        libeigen3-dev \
        libgles2-mesa-dev \
        libglu1-mesa-dev \
        libgl1-mesa-glx \
        libgl1-mesa-dri \
        xutils-dev \
        bison \
        zlib1g-dev \
        flex \
        lsb-release \
        pkg-config \
        protobuf-compiler \
        python3-dbg \
        python3-pip \
        python3-venv \
        python3-catkin-tools \
        qtbase5-dev \
        ruby \
        software-properties-common \
        sudo \
        wget \
        gdb \
        psmisc \
        vim \
        nano \
        x11-apps \
 && apt clean

# --- Install GCUDA Libs --- #
# Download Cuda local installer
WORKDIR /tmp
RUN wget https://developer.download.nvidia.com/compute/cuda/11.1.1/local_installers/cuda_11.1.1_455.32.00_linux.run

# Install CUDA Libs (without opengl libraries  to keep GUI libraries)
RUN sh cuda_11.1.1_455.32.00_linux.run --silent --toolkit --no-opengl-libs

# --- Install gcc-7 g++-7 --- #
RUN apt update \
 && apt install -y --no-install-recommends \
        gcc-7 g++-7 \
 && apt clean

RUN ln -sf /usr/bin/gcc-7 /usr/bin/gcc \
 && ln -sf /usr/bin/g++-7 /usr/bin/g++

# --- Install GPGPU-SIM --- #
# Clone source code
WORKDIR /tmp/gpgpu-sim
ENV VERSION_GPGPU_SIM=4.0.3
RUN curl -fsSL https://github.com/woensug-choi/gpgpu-sim_distribution/archive/v$VERSION_GPGPU_SIM.tar.gz | tar xz
RUN mv gpgpu-sim_distribution-$VERSION_GPGPU_SIM gpgpu-sim_distribution

# Define environment variables
ENV CUDA_INSTALL_PATH="/usr/local/cuda"
ENV PATH="$CUDA_INSTALL_PATH/bin:${PATH}"

# Compile
WORKDIR /tmp/gpgpu-sim/gpgpu-sim_distribution
RUN /bin/bash -c "source setup_environment && make -j$(nproc)"

# Define runtime env. variables
ENV PTX_SIM_MODE_FUNC=1
ENV PTX_SIM_MODE_DETAIL=0

# ---- Install dave dependencies ---- #
# Set ROS distribution
ARG DIST=noetic
# Set Gazebo verison
ARG GAZ=gazebo11

RUN /bin/sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros1-latest.list' \
 && apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
 && /bin/sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
 && apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743 \
 && /bin/sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
 && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
 && apt update \
 && apt install -y --no-install-recommends \
    libignition-common3-graphics-dev \
    ${GAZ} \
    lib${GAZ}-dev \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-vcstool \
    ros-${DIST}-gazebo-plugins \
    ros-${DIST}-gazebo-ros \
    ros-${DIST}-gazebo-ros-control \
    ros-${DIST}-gazebo-ros-pkgs \
    ros-${DIST}-effort-controllers \
    ros-${DIST}-geographic-info \
    ros-${DIST}-hector-gazebo-plugins \
    ros-${DIST}-joint-state-controller \
    ros-${DIST}-joint-state-publisher \
    ros-${DIST}-joy \
    ros-${DIST}-joy-teleop \
    ros-${DIST}-kdl-parser-py \
    ros-${DIST}-key-teleop \
    ros-${DIST}-move-base \
    ros-${DIST}-robot-localization \
    ros-${DIST}-robot-state-publisher \
    ros-${DIST}-ros-base \
    ros-${DIST}-rqt \
    ros-${DIST}-rqt-common-plugins \
    ros-${DIST}-rqt-robot-plugins \
    ros-${DIST}-rviz \
    ros-${DIST}-teleop-tools \
    ros-${DIST}-teleop-twist-joy \
    ros-${DIST}-teleop-twist-keyboard \
    ros-${DIST}-tf2-geometry-msgs \
    ros-${DIST}-tf2-tools \
    ros-${DIST}-urdfdom-py \
    ros-${DIST}-velodyne-gazebo-plugins \
    ros-${DIST}-velodyne-simulator \
    ros-${DIST}-xacro \
 && apt clean

# Add user
ARG USER_ID=1000
ARG GROUP_ID=1000
RUN groupadd -g ${GROUP_ID} gpgpu-sim \
 && useradd -ms /bin/bash gpgpu-sim -g gpgpu-sim

# RUN COMMAND
# docker run -it --net=host \
#     --env="DISPLAY" \
#     --env="QT_X11_NO_MITSHM=1" \
#     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
#     -v /home/${USER}:/home/gpgpu-sim \
#     -u $(id -u ${USER}):$(id -g ${USER}) \
#     -w /home/gpgpu-sim \
#     gpgpu-sim-noetic \
#     bash

# First things first
# source /tmp/gpgpu-sim/gpgpu-sim_distribution/setup_environment