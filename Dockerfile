FROM ros:humble-ros-base-jammy
ARG WITH_GPU=false

RUN apt-get update && \
    apt-get --yes dist-upgrade && \
    apt-get --yes install \
        black \
        isort \
        nano \
        pylint \
        python3-pip \
        ros-humble-rviz2 \
        tmux \
        vim

RUN if [ "${WITH_GPU}" = "true" ]; then \
        curl \
            --location https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb \
            --output cuda-keyring.deb && \
        dpkg --install cuda-keyring.deb && \
        rm cuda-keyring.deb && \
        apt-get update && \
        apt-get --yes install cuda-toolkit-12-9; \
    fi

RUN groupadd --gid 1000 developer && \
    useradd --uid 1000 --gid 1000 --shell /bin/bash --create-home developer && \
    apt-get install --yes sudo && \
    echo "developer ALL=(root) NOPASSWD: ALL" >> /etc/sudoers.d/developer && \
    chmod 0440 /etc/sudoers.d/developer && \
    mkdir --parent /home/developer/sim_ws/src && \
    chown --recursive developer:developer /home/developer/sim_ws

USER developer
WORKDIR /home/developer/sim_ws

RUN git clone \
        --branch dev-dynamics \
        --single-branch \
        https://github.com/f1tenth/f1tenth_gym && \
    python3 -m pip install ./f1tenth_gym

# Keep the container running.
CMD ["tail", "-f", "/dev/null"]
