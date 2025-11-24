FROM ros:humble-ros-base-jammy

RUN apt-get update && \
    apt-get -y dist-upgrade && \
    apt-get -y install \
        black \
        isort \
        nano \
        pylint \
        python3-pip \
        ros-humble-rviz2 \
        tmux \
        vim

RUN curl \
        -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb \
        -o cuda-keyring.deb && \
    dpkg -i cuda-keyring.deb && \
    rm cuda-keyring.deb && \
    apt-get update && \
    apt-get -y install cuda-toolkit

RUN git clone \
        --branch dev-dynamics \
        --single-branch \
        https://github.com/f1tenth/f1tenth_gym.git && \
    pip install ./f1tenth_gym

WORKDIR /sim_ws
SHELL ["/bin/bash", "-c"]

# Add CUDA to PATH
ENV PATH="/usr/local/cuda/bin:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda/lib64:${LD_LIBRARY_PATH}"

COPY src src
RUN source /opt/ros/humble/setup.bash && \
    rosdep install -i --from-path src --rosdistro humble -y
# RUN colcon build

# Keep the container running.
CMD ["tail", "-f", "/dev/null"]
