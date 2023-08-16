# andino_tests
## Description.

Hosts integration tests for the [Andino](https://github.com/Ekumen-OS/andino) robot.

## Prepare your workspace

1. Install in your Ubuntu 22.04 box, ROS 2 Humble full desktop.
Alternatively, you can use our [docker](https://github.com/gethenlabs/cloud_robotic_devops_demo/tree/main/docker) images and instructions.

2. Install vcs-tool

```sh
sudo pip install vcstool
```

3. Create a workspace folder, and the source folder.

```sh
mkdir -p ws/src
cd ws/src
```

4. Clone this repository into the `src` folder.

```sh
git clone git@github.com:gethenlabs/cloud_robotic_devops_demo.git
```
using the SSH authentication method
or
```sh
git clone https://github.com/gethenlabs/cloud_robotic_devops_demo.git
```
using HTTPS athentication method.

5. Pull all the other dependencies.

```sh
vcs import --input cloud_robotic_devops_demo/vcs_config.yaml .
```

6. An optional [docker](https://github.com/gethenlabs/cloud_robotic_devops_demo/tree/main/docker/README.md) environment for this package is provided. If you choose to use it, please follow the instructions, And continue to point 7 of the setup to launch the demo in the containerized workspace "your@computer:~/ws$"

7. Install the dependencies.
```sh
cd ..
rosdep install -i -y --rosdistro humble --from-paths src
```

## Build your workspace

1. Source your ROS 2 Humble distribution.

```sh
source /opt/ros/humble/setup.bash
```

2. Build the workspace

```sh
colcon build --packages-up-to andino_tests
# Alternatively, add more flags to see the full build output
# colcon build --packages-up-to andino_tests --event-handlers console_direct+
```

There is a known warning in the packages with python described in [ticket](https://github.com/colcon/colcon-core/issues/454) so the recommended workaround can be to ignore that issue.

```sh
PYTHONWARNINGS="ignore:setup.py install is deprecated::setuptools.command.install"; export PYTHONWARNINGS
```

## Test the workspace

Once it has been built, you can test it with:

```sh
source install/setup.bash
source /usr/share/gazebo/setup.bash
colcon test --packages-select andino_tests --event-handlers console_direct+
```
