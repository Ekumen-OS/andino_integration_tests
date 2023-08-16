# Copyright 2023 Ekumen, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#####################
# Stage1-Base image #
#####################
FROM osrf/ros:humble-desktop-full as dev

# Arguments for building
ARG USERID
ARG USER

# Setup environment
ENV TERM linux
ENV DEBIAN_FRONTEND noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Create a user with passwordless sudo
RUN adduser --uid $USERID --gecos "ekumen developer" --disabled-password $USER
RUN adduser $USER sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN echo "export QT_X11_NO_MITSHM=1" >> /home/$USER/.bashrc
USER $USER

# Adds USER to dialout and plugdev group.
# This is needed to access the serial ports, for further references check
# the libserial documentation.
RUN sudo usermod -a -G dialout $USER
RUN sudo usermod -a -G plugdev $USER

# Creates the src folder of the workspace.
RUN mkdir -p /home/$USER/ws/src

# Adds to bashrc the ros humble overlay sourcing.
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USER/.bashrc

# Adds colcon autocomplete
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USER/.bashrc

# Copy dependencies
COPY docker/requirements.txt .

# Updates
RUN sudo apt update && sudo apt upgrade -y && rosdep update

# Install dependencies
RUN sudo apt install --no-install-recommends -y $(cat requirements.txt)

# Defines a workspace folder.
WORKDIR /home/$USER/ws

CMD ["/bin/bash"]

##################
#Stage2-CI image #
##################
FROM dev as ci

# Arguments for building
ARG SHORT_SHA
RUN echo "short_sha: ${SHORT_SHA}"

RUN pip install vcstool

# install virtual display driver
RUN set -x \
    && sudo apt-get update \
    && sudo apt-get install libgl1-mesa-glx xvfb -y \
    && export DISPLAY=:99.0 \
    && which Xvfb && Xvfb :99 -screen 0 1024x768x24 > /dev/null 2>&1 & sleep 3 \
    && set +x

# Copies the repository with the right version.
COPY docker/build/andino_integration_tests /home/$USER/ws/src/andino_integration_tests

# Setups the workspace.
# Clone repositories with vcstool
RUN vcs validate < /home/$USER/ws/src/andino_integration_tests/vcs_config.yaml
RUN vcs import --input /home/$USER/ws/src/andino_integration_tests/vcs_config.yaml src

# Install dependencies from the workspace
RUN sudo apt update \
    && sudo apt upgrade -y \
    && rosdep update \
    && rosdep install -i -y --rosdistro humble --from-paths src

# Compile the workspace, switch terminals (from default /bin/sh) to properly source the environment.
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && \
    source /usr/share/gazebo/setup.bash && \
    colcon build --event-handlers console_direct+

# Entrypoint
CMD ["/bin/bash"]
