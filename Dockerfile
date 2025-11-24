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

RUN git clone \
        --branch dev-dynamics \
        --single-branch \
        https://github.com/f1tenth/f1tenth_gym.git && \
    pip install ./f1tenth_gym

WORKDIR /sim_ws
SHELL ["/bin/bash", "-c"]

COPY src src
RUN source /opt/ros/humble/setup.bash && \
    rosdep install -i --from-path src --rosdistro humble -y && \
    colcon build

# Keep the container running.
CMD ["tail", "-f", "/dev/null"]
