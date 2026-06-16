FROM nvidia/cuda:12.8.1-devel-ubuntu22.04

SHELL ["/bin/bash", "-c"]

ENV RCUTILS_COLORIZED_OUTPUT=1
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

RUN apt update && apt install -y \
    build-essential \
    cmake \
    git \
    curl \
    lsb-release \
    gnupg \
    wget \
    libbullet-dev \
    libboost-program-options-dev \
    libusb-1.0-0-dev \
    python3-flake8 \
    python3-pip \
    python3-pytest-cov \
    python3-setuptools \
    clang-format

# Max version of pip supported by PX4-Autopilot v1.15.0 is 24.0
# See: Tools/setup/requirements.txt (matplotlib>=3.0.*)
RUN python3 -m pip install -U pip==24.0

# Python dependencies
RUN python3 -m pip install -U \
    argcomplete \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures \
    pytest \
    numpy \
    kconfiglib \
    jsonschema \
    jinja2 \
    pyros-genmsg \
    future \
    quaternionic \
    cflib \
    cfclient

# Install additional Python packages for MPC (+AI potentially)
RUN python3 -m pip install -U --ignore-installed \
    matplotlib \
    pandas \
    nbformat \
    notebook \
    ipywidgets \
    plotly \
    torch \
    torchvision \
    torchaudio \
    tensorboard \
    casadi

# Install additional Python packages l4casadi and Gazebo Harmonic
# setuptools>=68.1 required by l4casadi, <81 preferable by VCS tools
# protobuf 3.20.3 is required by Gazebo messages due to .pb2.py files
RUN python3 -m pip install -U \
    "setuptools>=68.1,<81" \
    "scikit-build>=0.17" \
    "cmake==3.27" \
    "ninja>=1.11" \
    "protobuf==3.20.3"

# Install ROS2 Humble
RUN apt update && \
    apt install -y --no-install-recommends tzdata && \
    ln -fs /usr/share/zoneinfo/Europe/Warsaw /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    apt update && apt install -y ros-${ROS_DISTRO}-desktop

RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-colcon-clean \
    python3-vcstool \
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-ament-cmake-auto \
    ros-${ROS_DISTRO}-ament-lint-auto \
    ros-${ROS_DISTRO}-ament-lint-cmake \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-plotjuggler-ros \
    ros-dev-tools

# Install Gazebo Harmonic - Default Gazebo version in ROS2 Humble is Fortress
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Some ROS2 and Gazebo dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ros-gzharmonic

RUN apt install -y \
    ros-${ROS_DISTRO}-turtle-tf2-py \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-dev-tools \
    libgz-msgs10-dev \
    libgz-transport13-dev

RUN apt-get update
RUN apt-get upgrade -y

# Create developer user with root-like privileges
RUN useradd -m -d /home/developer -s /bin/bash -u 1001 -G sudo developer && \
    echo 'developer:developer' | chpasswd && \
    echo "developer ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/developer && \
    chmod 0440 /etc/sudoers.d/developer
USER developer

# Install acados
RUN git clone https://github.com/acados/acados.git /home/developer/acados -b v0.5.1 && \
    cd /home/developer/acados && \
    git submodule update --recursive --init && \
    mkdir -p build && \
    cd build && \
    cmake -DACADOS_WITH_QPOASES=ON .. && \
    make install -j8

# Acados Python interface
RUN pip install "setuptools-scm<10" && \
    pip install /home/developer/acados/interfaces/acados_template && \
    echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/developer/acados/lib" >> /home/developer/.bashrc && \
    echo "export ACADOS_SOURCE_DIR=/home/developer/acados" >> /home/developer/.bashrc

# Install l4casadi
RUN git clone https://github.com/Tim-Salzmann/l4casadi.git /home/developer/l4casadi -b v2.0.0-release && \
    cd /home/developer/l4casadi && \
    CUDACXX=/usr/local/cuda-12.8/bin/nvcc pip install . --no-build-isolation

# Setup ROS2 workspace
RUN mkdir -p /home/developer/ros2_ws/src

# Copy helper scripts
COPY setup.sh /home/developer/ros2_ws/setup.sh
COPY build.sh /home/developer/ros2_ws/build.sh
WORKDIR /home/developer/ros2_ws

# Set locale to UTF-8
USER root
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8

# Additional configs for developer user
USER developer
RUN echo "export LANG=en_US.UTF-8" >> /home/developer/.bashrc \
    && echo "export LANGUAGE=en_US:en" >> /home/developer/.bashrc \
    && echo "export LC_ALL=en_US.UTF-8" >> /home/developer/.bashrc \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/developer/.bashrc \
    && echo "source /home/developer/ros2_ws/install/setup.bash" >> /home/developer/.bashrc \
    && echo 'export PATH="$HOME/.local/bin:$PATH"' >> /home/developer/.bashrc

CMD [ "sleep", "infinity" ]
