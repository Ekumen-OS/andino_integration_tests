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
  echo $'\nUsage:\t test_entrypoint.sh [OPTIONS] \n
  Options:\n
  \t-t --test_name\t\t Name of the test to run.\n
  Example:\n
  \t./src/cloud_robotic_devops_demo/andino_tests/scripts/test_entrypoint.sh [-t <testName>]\n'
}

# Parse arguments
while [[ "$#" -gt 0 ]]; do
  case $1 in
    -t|--test_name) TEST_NAME="${2}"; shift ;;
    -h|--help) show_help ; exit 1 ;;
    *) echo "Unknown parameter passed: $1"; exit 1 ;;
  esac
  shift
done

TEST_NAME=${TEST_NAME:-}

# Test the name of the test to run.
if [ -z "$TEST_NAME" ]
then
  show_help
  exit 1
fi

# Source the environment and run the test.
source /opt/ros/humble/setup.bash && \
  source /usr/share/gazebo/setup.bash && \
  source install/setup.bash && \
  colcon test --packages-select andino_tests --event-handlers console_direct+ --pytest-args -k ${TEST_NAME}

# End the script with the final result
colcon test-result --verbose
