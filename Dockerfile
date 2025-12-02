ARG PLATFORM=linux/amd64
ARG ROS_DISTRO=humble
ARG UBUNTU_CODENAME=jammy
FROM --platform=${PLATFORM} ros:${ROS_DISTRO}-ros-base-${UBUNTU_CODENAME}

ARG ROS_DISTRO
RUN apt-get update && \
    apt-get --yes dist-upgrade && \
    apt-get --yes install \
        black \
        clang \
        clang-format \
        clang-tidy \
        isort \
        nano \
        pylint \
        python3-pip \
        ros-${ROS_DISTRO}-ackermann-msgs \
        ros-${ROS_DISTRO}-ament-index-cpp \
        ros-${ROS_DISTRO}-geometry-msgs \
        ros-${ROS_DISTRO}-nav-msgs \
        ros-${ROS_DISTRO}-nav2-lifecycle-manager \
        ros-${ROS_DISTRO}-nav2-map-server \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-sensor-msgs \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-visualization-msgs \
        ros-${ROS_DISTRO}-xacro \
        sudo \
        tmux \
        vim

ARG WITH_GPU=""
ARG CUDA_VERSION_MAJOR=12
ARG CUDA_VERSION_MINOR=9
RUN if [ -n "${WITH_GPU}" ]; then \
        . /etc/os-release && \
        case "${UBUNTU_CODENAME}" in \
            jammy) CUDA_REPO=ubuntu2204 ;; \
            focal) CUDA_REPO=ubuntu2004 ;; \
            *) echo "Unsupported UBUNTU_CODENAME: ${UBUNTU_CODENAME}"; exit 1 ;; \
        esac && \
        curl \
            --location https://developer.download.nvidia.com/compute/cuda/repos/${CUDA_REPO}/x86_64/cuda-keyring_1.1-1_all.deb \
            --output cuda-keyring.deb && \
        dpkg --install cuda-keyring.deb && \
        rm cuda-keyring.deb && \
        apt-get update && \
        apt-get --yes install cuda-toolkit-${CUDA_VERSION_MAJOR}-${CUDA_VERSION_MINOR}; \
    fi

ENV CUDA_HOME="/usr/local/cuda-${CUDA_VERSION_MAJOR}.${CUDA_VERSION_MINOR}"
# https://docs.docker.com/reference/dockerfile#environment-replacement
ENV PATH="${WITH_GPU:+${CUDA_HOME}/bin${PATH:+:}}${PATH}"
ENV LD_LIBRARY_PATH="${WITH_GPU:+${CUDA_HOME}/lib64${LD_LIBRARY_PATH:+:}}${LD_LIBRARY_PATH}"

RUN groupadd --gid 1000 developer && \
    useradd --uid 1000 --gid 1000 --shell /bin/bash --create-home developer && \
    echo "developer ALL=(root) NOPASSWD: ALL" >> /etc/sudoers.d/developer && \
    chmod 0440 /etc/sudoers.d/developer && \
    mkdir --parent /home/developer/sim_ws/src && \
    chown --recursive developer:developer /home/developer/sim_ws

USER developer
WORKDIR /home/developer/sim_ws
RUN echo >> /home/developer/.bashrc && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/developer/.bashrc

ARG WITH_F1TENTH_GYM=""
RUN if [ -n "${WITH_F1TENTH_GYM}" ]; then \
        git clone \
            --branch dev-dynamics \
            --single-branch \
            https://github.com/f1tenth/f1tenth_gym; \
        python3 -m pip install ./f1tenth_gym transforms3d; \
    fi

# Keep the container running.
CMD ["tail", "-f", "/dev/null"]
