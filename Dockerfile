FROM ros:humble-ros-base-jammy
ARG WITH_GPU

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
        ros-humble-ackermann-msgs \
        ros-humble-ament-index-cpp \
        ros-humble-geometry-msgs \
        ros-humble-nav-msgs \
        ros-humble-rviz2 \
        ros-humble-sensor-msgs \
        ros-humble-tf2-ros \
        ros-humble-visualization-msgs \
        sudo \
        tmux \
        vim

RUN if [ -n "${WITH_GPU}" ]; then \
        curl \
            --location https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb \
            --output cuda-keyring.deb && \
        dpkg --install cuda-keyring.deb && \
        rm cuda-keyring.deb && \
        apt-get update && \
        apt-get --yes install cuda-toolkit-12-9; \
    fi

ENV CUDA_HOME="/usr/local/cuda-12.9"
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
RUN echo -e "\n source /opt/ros/humble/setup.bash" >> /home/developer/.bashrc

RUN git clone \
        --branch dev-dynamics \
        --single-branch \
        https://github.com/f1tenth/f1tenth_gym && \
    python3 -m pip install ./f1tenth_gym

# Keep the container running.
CMD ["tail", "-f", "/dev/null"]
