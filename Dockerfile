FROM ros:humble-ros-base-jammy

RUN apt-get update && \
    apt-get -y dist-upgrade && \
    apt-get -y install \
        black \
        isort \
        nano \
        pylint \
        python3-ipykernel \
        python3-ipywidgets \
        python3-matplotlib \
        python3-matplotlib-inline \
        python3-numba \
        python3-numpy \
        python3-opencv \
        python3-pandas \
        python3-pil \
        python3-pip \
        python3-pygame \
        python3-pyqt5 \
        python3-pyqtgraph \
        python3-pytest \
        python3-requests \
        python3-scipy \
        python3-shapely \
        python3-transforms3d \
        python3-yaml \
        ros-humble-rviz2 \
        tmux \
        vim && \
    rm -rf /var/lib/apt/lists/*
RUN pip3 install gymnasium

WORKDIR /sim_ws
# Keep the container running.
CMD ["tail", "-f", "/dev/null"]
