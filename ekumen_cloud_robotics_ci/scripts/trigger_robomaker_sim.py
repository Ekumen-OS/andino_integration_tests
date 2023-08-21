#!/usr/bin/env python3

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

import time
import os
import boto3
import json
import argparse
from typing import List
from process_simulation_results import main as process_simulation_results


def create_robot_application(
    name, docker_image_url, simulation_client: boto3.Session
) -> str:
    robot_application_response = simulation_client.create_robot_application(
        name=name,
        robotSoftwareSuite={"name": "General"},
        environment={"uri": docker_image_url},
    )
    return robot_application_response["arn"]


def create_simulation_application(
    name, docker_image_url, simulation_client: boto3.Session
) -> str:
    simulation_application_response = simulation_client.create_simulation_application(
        name=name,
        simulationSoftwareSuite={"name": "SimulationRuntime"},
        robotSoftwareSuite={"name": "General"},
        environment={"uri": docker_image_url},
    )
    return simulation_application_response["arn"]


def update_test_descriptions(
    iam_role: str,
    robot_application_arn: str,
    simulation_application_arn: str,
    vpn_config_subnets: list[str],
    vpn_security_groups: list[str],
    test_descriptions: list[dict],
) -> list[dict]:
    for test_description in test_descriptions:
        test_description["iamRole"] = iam_role
        test_description["robotApplications"][0]["application"] = robot_application_arn
        test_description["simulationApplications"][0][
            "application"
        ] = simulation_application_arn
        test_description["vpcConfig"]["subnets"] = vpn_config_subnets
        test_description["vpcConfig"]["securityGroups"] = vpn_security_groups
    return test_descriptions


def read_test_descriptions(test_descriptions_file: str) -> list[dict]:
    if os.path.exists(test_descriptions_file):
        with open(test_descriptions_file, "r") as f:
            json_tests = json.load(f)
    else:
        return None

    json_config_test = json_tests.get("json_config_test", [])
    if len(json_config_test) == 0:
        raise Exception("Test suite file need to have at list one test suite case")

    return json_config_test


def trigger_simulation_job(test_descriptions: dict, simulation_client: boto3.Session):
    response = simulation_client.start_simulation_job_batch(
        batchPolicy={"timeoutInSeconds": 120 * 60, "maxConcurrency": 2},
        createSimulationJobRequests=test_descriptions,
    )
    return response["arn"]


def wait_until_simulation_job_is_finished(
    simulation_job_arn: str, simulation_client: boto3.Session, time_step_sec: int = 10
):
    def query_simulation_job():
        response = simulation_client.describe_simulation_job_batch(
            batch=simulation_job_arn
        )
        return response["status"]

    simulation_job_status = "Pending"
    while simulation_job_status in ["Pending", "InProgress"]:
        time.sleep(time_step_sec)
        simulation_job_status = query_simulation_job()
        print(f"Simulation job batch status {simulation_job_status}")


def main(args: argparse.Namespace, simulation_client: boto3.Session):
    print(f"Creating robot application with {args.robot_image_url}")
    robot_application_arn = create_robot_application(
        name=args.robot_application_name,
        docker_image_url=args.robot_image_url,
        simulation_client=simulation_client,
    )

    print(f"Creating simulation application with {args.simulation_image_url}")
    simulation_application_arn = create_simulation_application(
        name=args.simulation_application_name,
        docker_image_url=args.simulation_image_url,
        simulation_client=simulation_client,
    )

    test_descriptions = read_test_descriptions(args.test_description_path)
    if test_descriptions is None:
        print(f"The file '{args.test_description_path}' does not exist.")
        exit(1)

    print(f"Completing fields of test config")
    final_test_description = update_test_descriptions(
        iam_role=args.iam_role,
        robot_application_arn=robot_application_arn,
        simulation_application_arn=simulation_application_arn,
        vpn_config_subnets=args.vpn_config_subnets,
        vpn_security_groups=args.vpn_security_groups,
        test_descriptions=test_descriptions,
    )

    with open("/tmp/json_config_file.json", "a") as json_config_file:
        json_config_file.write(json.dumps(final_test_description))

    print(f"Triggering simulation job batch")
    simulation_job_arn = trigger_simulation_job(
        test_descriptions=test_descriptions, simulation_client=simulation_client
    )

    print(f"Querying the status of a simulation jobs {simulation_job_arn}")
    wait_until_simulation_job_is_finished(simulation_job_arn, simulation_client)

    process_simulation_results(
        simulation_job_arn=simulation_job_arn,
        region=simulation_client.meta.region_name,
        simulation_summary_path=args.simulation_summary_path,
        simulation_client=simulation_client,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog="TriggerRobomakerSimulationJob")

    # Mandatory arguments
    parser.add_argument("--iam_role", required=True, type=str, help="")
    parser.add_argument("--simulation_image_url", required=True, type=str, help="")
    parser.add_argument(
        "--simulation_application_name", required=True, type=str, help=""
    )
    parser.add_argument("--robot_image_url", required=True, type=str, help="")
    parser.add_argument("--robot_application_name", required=True, type=str, help="")
    parser.add_argument(
        "--vpn_config_subnets",
        required=True,
        nargs="+",
        help="AWS RoboMaker VPN subnets to connect to. You may pass more than one.",
    )
    parser.add_argument(
        "--vpn_security_groups",
        required=True,
        nargs="+",
        help="AWS RoboMaker VPN security groups. You may pass more than one.",
    )
    parser.add_argument("--test_description_path", required=True, type=str, help="")

    # Optinal arguments
    parser.add_argument(
        "--simulation_summary_path",
        required=False,
        type=str,
        default="/tmp/simulation_summary.txt",
    )

    args = parser.parse_args()

    simulation_client = boto3.client("robomaker")
    main(args, simulation_client)
