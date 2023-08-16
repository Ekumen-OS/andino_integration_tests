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

# Function to show help message
show_help() {
    echo "Usage: ./docker/remove_ci.sh [options]"
    echo "Options:"
    echo "  -i, --image_name                Specify the image name"
    echo "  -r, --repository (optional)     Specify the repository name"
    echo "  -h, --help                      Show this help message"
}

# Initializing IMAGE_NAME to an empty string
IMAGE_NAME=""
REPOSITORY=${REPOSITORY:-cloud_robotic_devops_demo}

while [[ "$#" -gt 0 ]]; do
    case $1 in
    -i | --image_name)
        IMAGE_NAME="${2}"
        shift
        ;;
    -r | --respository)
        REPOSITORY="${2}"
        shift
        ;;
    -h | --help)
        show_help
        exit 1
        ;;
    *)
        echo "Unknown parameter passed: $1"
        show_help
        exit 1
        ;;
    esac
    shift
done

# Check if the IMAGE_NAME is empty (not provided)
if [ -z "$IMAGE_NAME" ]; then
    echo "Error: Image name not provided. Please use -i or --image_name to specify the image name."
    show_help
    exit 1
fi

echo "Deleting the docker image for cloud robotic devops."

# Save the output of the list-images command to a variable
image_digests=$(aws ecr list-images --repository-name $REPOSITORY --query 'imageIds[?imageTag!=`null` && contains(imageTag, `'$IMAGE_NAME'`) == `true`].imageDigest' --output text)

# Check if there are images to delete
if [[ -z "$image_digests" ]]; then
    echo "No images found with the specified tag pattern."
else
    # Loop through the image digests and print each digest before deleting the image
    for digest in $image_digests; do
        echo "Deleting image with digest: $digest"
        aws ecr batch-delete-image --repository-name $REPOSITORY --image-ids imageDigest=$digest
    done
fi
