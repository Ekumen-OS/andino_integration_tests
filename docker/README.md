# Docker


## Prerequisites for running the docker images locally.

It is a requirement to have `docker engine` and `aws cli` already installed in the host machine.

* See [Docker Installation Guide](https://docs.docker.com/engine/install/ubuntu/)

For NVIDIA GPU support, `nvidia-container-toolkit` should be installed. *Skip this step if you don't have an NVIDIA graphics card*

* Make sure you have the drivers installed:
  ```sh
  nvidia-smi
  ```
* See [NVIDIA Container Toolkit Installation Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

For AWS CLI, should be intalled following the next guide [Install or update the latest version of the AWS CLI](https://docs.aws.amazon.com/cli/latest/userguide/getting-started-install.html)

* To confirm you have AWS CLI installed , run the following command:
  ```sh
  aws --version
  ```

#### Building image and running container

- There are two sets of files to run the docker container in development or CI mode.

- The `build_dev.sh` and `run_dev.sh` scripts are used to build and run a container empty of packages and a bash console as an entry point.
It uses the following names by default: "cloud_robotic_devops_base" and  "cloud_robotic_devops_container_base".
```sh
./src/andino_integration_tests/docker/build_dev.sh [-i <imageName>]
./src/andino_integration_tests/docker/run_dev.sh   [-i <imageName>] [-c <containerName>] [--use_nvidia]
```
- **IMPORTANT**: If you are using nvidia drivers add the `--use_nvidia` flag.

- The `build_ci.sh` and  `run_ci.sh` scripts build and run a container with a specific code version to be tested using the --short-sha <"the sha"> flag and the required packages.
The entry point runs the colcon test command.
If no short sha flag is provided "-short_sha", then it takes by default the current head of the context folder in the local computer.
If no image name flag is provided "-i", then the default name will be "cloud_robotic_devops_ci"
```sh
./src/andino_integration_tests/docker/build_ci.sh -i <imageName> --short_sha <theSha>
./src/andino_integration_tests/docker/run_ci.sh -i <imageName>:<short_sha> -c <containerName>
```
- **IMPORTANT**: we are not using gpu virtual machines in this pipeline, avoid using `--use_nvidia` flag.

## Removing docker images from AWS ECR

After building the docker images as part of the CI pipeline, they are stored in AWS ECR, where AWS RoboMaker pulls them to run the requested tests. Then, after the pull request that generated the images is closed, this script is used in the CI pipeline to delete the old docker images from the registry to save space/costs.

- To manually remove the docker image on Amazon Elastic Container Registry use the next command:
```sh
./src/andino_integration_tests/docker/remove_image_ci.sh -i <imageTag> -r <repositoryName>
```
- **IMPORTANT**: we are using AWS CLI to remove the docker images hosted on [Amazon Elastic Container Registry](https://aws.amazon.com/ecr/) it's importat that you have it installed and config in your local host before execute this script.
