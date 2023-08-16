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

set -e

# Prints information about usage.
function show_help() {
  echo $'\nUsage:\t build.sh [OPTIONS] \n
  Options:\n
  \t-i --image_name\t\t Name of the image to be built (default cloud_robotic_devops_ci).\n
  \t-short_sha --short_sha\t\t Short SHA version to pull the repository to this specific version.\n
  Example:\n
  \t./src/andino_integration_tests/docker/build_ci.sh -i <imageName> --short_sha <theSha>\n
  \t use -i <imageName> --short_sha <theSha> to override the default values.\n'
}

echo "Building the docker image for CI..."

SCRIPT_FOLDER_PATH="$(cd "$(dirname "$0")"; pwd)"
CONTEXT_FOLDER_PATH="$(cd "$(dirname "$0")"; cd .. ; pwd)"
echo "context folder path: ${CONTEXT_FOLDER_PATH}"

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -i|--image_name) IMAGE_NAME="${2}"; shift ;;
        -h|--help) show_help ; exit 1 ;;
        -short_sha| --short_sha) SHORT_SHA="${2}"; shift ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

CURRENT_HEAD_SHORT_SHA="$(cd $CONTEXT_FOLDER_PATH; git rev-parse --short HEAD)"

# Update the arguments to default values if needed.
IMAGE_NAME=${IMAGE_NAME:-cloud_robotic_devops_ci}
DOCKERFILE_PATH=$SCRIPT_FOLDER_PATH/Dockerfile
SHORT_SHA=${SHORT_SHA:-$CURRENT_HEAD_SHORT_SHA}

USERID=$(id -u)
USER=$(whoami)

rm -rf $SCRIPT_FOLDER_PATH/build
mkdir $SCRIPT_FOLDER_PATH/build
rsync -r $CONTEXT_FOLDER_PATH $SCRIPT_FOLDER_PATH/build/

CURRENT_DIR="$(pwd)"

cd $SCRIPT_FOLDER_PATH/build/andino_integration_tests
git checkout $SHORT_SHA

cd $CURRENT_DIR

# Build the docker image.
docker build -t $IMAGE_NAME \
  --file $DOCKERFILE_PATH \
  --build-arg SHORT_SHA=$SHORT_SHA \
  --build-arg USERID=$USERID \
  --build-arg USER=$USER \
  --target ci \
  $CONTEXT_FOLDER_PATH

# Removes the copy of the branch used to build the image.
rm -rf $SCRIPT_FOLDER_PATH/build
