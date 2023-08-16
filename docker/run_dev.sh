#!/bin/bash

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

set +e

# Prints information about usage.
function show_help() {
  echo $'\nUsage:\t run.sh [OPTIONS] \n
  Options:\n
  \t-i --image_name\t\t Name of the image to be run (default cloud_robotic_devops_base).\n
  \t-c --container_name\t Name of the container(default cloud_robotic_devops_container_base).\n
  \t--use_nvidia\t\t Use nvidia runtime.\n
  Examples:\n
  \trun.sh\n
  \t./src/andino_integration_tests/docker/run_dev.sh   [-i <imageName>] [-c <containerName>] \n'
}

echo "Running the container..."

# Location of the workspace
WORKSPACE_FOLDER_PATH="$(cd "$(dirname "$0")"; cd ..; cd ..; cd ..; pwd)"

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -i|--image_name) IMAGE_NAME="${2}"; shift ;;
        -c|--container_name) CONTAINER_NAME="${2}"; shift ;;
        -h|--help) show_help ; exit 1 ;;
        --use_nvidia) NVIDIA_FLAGS="--gpus=all -e NVIDIA_DRIVER_CAPABILITIES=all" ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

# Update the arguments to default values if needed.

IMAGE_NAME=${IMAGE_NAME:-cloud_robotic_devops_base}
CONTAINER_NAME=${CONTAINER_NAME:-cloud_robotic_devops_container_base}

SSH_PATH=/home/$USER/.ssh
WORKSPACE_ROOT_CONTAINER=/home/$(whoami)/ws
SSH_AUTH_SOCK_USER=$SSH_AUTH_SOCK

# Check if name container is already taken.
if docker container ls -a | grep "${CONTAINER_NAME}$" -c &> /dev/null; then
   printf "Error: Docker container called $CONTAINER_NAME is already opened.     \
   \n\nTry removing the old container by doing: \n\t docker rm $CONTAINER_NAME   \
   \nor just initialize it with a different name.\n"
   exit 1
fi

xhost +
docker run --privileged --net=host -it $NVIDIA_FLAGS \
       -e DISPLAY=$DISPLAY \
       -e SSH_AUTH_SOCK=$SSH_AUTH_SOCK_USER \
       -v $(dirname $SSH_AUTH_SOCK_USER):$(dirname $SSH_AUTH_SOCK_USER) \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       -v ${WORKSPACE_FOLDER_PATH}:$WORKSPACE_ROOT_CONTAINER \
       -v $SSH_PATH:$SSH_PATH \
       --name $CONTAINER_NAME $IMAGE_NAME
xhost -

# Trap workspace exits and give the user the choice to save changes.
function onexit() {
  while true; do
    read -p "Do you want to overwrite the image called '$IMAGE_NAME' with the current changes? [y/n]: " answer
    if [[ "${answer:0:1}" =~ y|Y ]]; then
      echo "Overwriting docker image..."
       docker commit $CONTAINER_NAME $IMAGE_NAME
      break
    elif [[ "${answer:0:1}" =~ n|N ]]; then
      break
    fi
  done
  docker stop $CONTAINER_NAME > /dev/null
  docker rm $CONTAINER_NAME > /dev/null
}

trap onexit EXIT
