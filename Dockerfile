# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

FROM --platform=linux/amd64 ros:humble

SHELL ["/bin/bash", "-c"]

# dependencies
RUN apt-get update --fix-missing && \
    apt-get install -y git \
                       nano \
                       vim \
                       python3-pip \
                       libeigen3-dev \
                       tmux \
                       ros-humble-rviz2
RUN apt-get -y dist-upgrade
RUN pip3 install gymnasium transforms3d

# pyqt6 can break the editable install for f1tenth_gym in some cached environments.
# Installing a compatible pyqt6 first avoids .toml/pyqt install errors (see upstream README).
RUN pip3 install pyqt6==6.7.1 || true

# f1tenth gym (use dev-dynamics branch)
RUN git clone --branch dev-dynamics --single-branch https://github.com/f1tenth/f1tenth_gym
RUN cd f1tenth_gym && \
    pip3 install -e .

# Quick runtime import check during image build to fail-fast if f1tenth_gym isn't importable
RUN python3 -c "import importlib, sys; importlib.import_module('f1tenth_gym'); print('f1tenth_gym import ok')" || (python3 -m pip show f1tenth_gym && python3 -c "import sys; print('sys.path =', sys.path)" && false)

# ros2 gym bridge
RUN mkdir -p sim_ws/src/f1tenth_gym_ros
COPY ./f1tenth_gym_ros /sim_ws/src/f1tenth_gym_ros
COPY ./lab7_pkg /sim_ws/src/lab7_pkg
RUN source /opt/ros/humble/setup.bash && \
    cd sim_ws/ && \
    apt-get update --fix-missing && \
    rosdep install -i --from-path src --rosdistro humble -y && \
    colcon build

WORKDIR '/sim_ws'
ENTRYPOINT ["/bin/bash"]
