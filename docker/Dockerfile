FROM nvidia/cuda:11.3.1-base-ubuntu20.04

# disable terminal interaction for apt
ENV DEBIAN_FRONTEND=noninteractive

# Fundamentals
RUN apt-get update && apt-get install -y \
        bash-completion \
        build-essential \
        clang-format \
        cmake \
        curl \
        git \
        gnupg2 \
        locales \
        lsb-release \
        rsync \
        software-properties-common \
        wget \
        vim \
        unzip \
        mlocate \
	    libgoogle-glog-dev \
        && rm -rf /var/lib/apt/lists/*

# Python basics
RUN apt-get update && apt-get install -y \
        python3-flake8 \
        python3-opencv \
        python3-pip \
        python3-pytest-cov \
        python3-setuptools \
        && rm -rf /var/lib/apt/lists/*

# Python3 (PIP)
RUN python3 -m pip install -U \
        argcomplete \
        autopep8 \
        flake8 \
        flake8-blind-except \
        flake8-builtins \
        flake8-class-newline \
        flake8-comprehensions \
        flake8-deprecated \
        flake8-docstrings \
        flake8-import-order \
        flake8-quotes \
        onnx \
        pytest-repeat \
        pytest-rerunfailures \
        pytest \
        pydocstyle \
        scikit-learn \
        torch \
        torchvision
# Setup ROS2 Foxy
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'      

RUN apt-get update && apt-get install -y \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        ros-foxy-camera-calibration-parsers \
        ros-foxy-camera-info-manager \
        ros-foxy-desktop \
        ros-foxy-launch-testing-ament-cmake \
        ros-foxy-rqt* \
        ros-foxy-turtlesim \
        ros-foxy-v4l2-camera \
        ros-foxy-realsense2-camera \
        ros-foxy-vision-msgs \
        ros-foxy-pcl-conversions \
        ros-foxy-sensor-msgs-py \
        ros-foxy-stereo-image-proc \
        ros-foxy-pcl-ros \
        && rm -rf /var/lib/apt/lists/*
        
RUN apt-get update && apt-get install -y \
	&& apt install ros-foxy-rmw-cyclonedds-cpp -y \
        && rm -rf /var/lib/apt/lists/*

RUN rosdep init

RUN rosdep update

COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
