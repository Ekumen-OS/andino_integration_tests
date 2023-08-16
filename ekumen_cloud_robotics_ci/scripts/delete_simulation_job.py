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

import boto3
import argparse


def get_robot_application(robot_application_name: str, simulation_client: boto3.Session):
    response = simulation_client.list_robot_applications()
    robot_applications = response["robotApplicationSummaries"]
    matching_apps = [
        app for app in robot_applications if robot_application_name in app["name"]
    ]

    if not matching_apps:
        print(
            f"No robot applications found with a name containing '{robot_application_name}'."
        )
        return []

    return matching_apps


def get_simulation_application(
    simulation_application_name: str, simulation_client: boto3.Session
):
    response = simulation_client.list_simulation_applications()
    simulation_applications = response["simulationApplicationSummaries"]
    matching_apps = [
        app
        for app in simulation_applications
        if simulation_application_name in app["name"]
    ]

    if not matching_apps:
        print(
            f"No simulation applications found with a name containing '{simulation_application_name}'."
        )
        return []

    return matching_apps


def delete_robot_application(matching_apps, simulation_client: boto3.Session):
    try:
        for app in matching_apps:
            robot_application_arn = app["arn"]
            simulation_client.delete_robot_application(application=robot_application_arn)
            print(
                f"Robot application '{robot_application_arn}' has been deleted successfully."
            )
    except simulation_client.exceptions.ResourceNotFoundException:
        print(f"Robot application not found.")
    except Exception as e:
        print(f"Error deleting robot application: {e}")


def delete_simulation_application(
    matching_apps: list[dict], simulation_client: boto3.Session
):
    try:
        for app in matching_apps:
            simulation_application_arn = app["arn"]
            simulation_client.delete_simulation_application(
                application=simulation_application_arn
            )
            print(
                f"Simulation application '{simulation_application_arn}' has been deleted successfully."
            )
    except simulation_client.exceptions.ResourceNotFoundException:
        print(f"Simulation application not found.")
    except Exception as e:
        print(f"Error deleting simulation application: {e}")


def list_simulation_jobs(
    robo_application_name: str,
    simulation_application_name: str,
    simulation_client: boto3.Session,
):
    response = simulation_client.list_simulation_jobs()

    matching_jobs = []

    while "simulationJobSummaries" in response:
        for job_summary in response["simulationJobSummaries"]:
            robot_app_names = job_summary["robotApplicationNames"]
            simulation_app_names = job_summary["simulationApplicationNames"]

            if any(
                robo_application_name in app_name for app_name in robot_app_names
            ) and any(
                simulation_application_name in app_name
                for app_name in simulation_app_names
            ):
                matching_jobs.append(job_summary["arn"])

        if "nextToken" in response:
            response = simulation_client.list_simulation_jobs(
                nextToken=response["nextToken"]
            )
        else:
            break

    return matching_jobs


def delete_simulation_job(
    simulation_job_arn: str,
    simulation_client: boto3.Session,
):
    try:
        simulation_client.cancel_simulation_job(job=simulation_job_arn)
        print(f"Simulation job '{simulation_job_arn}' has been deleted successfully.")
    except simulation_client.exceptions.ResourceNotFoundException:
        print(f"Simulation job '{simulation_job_arn}' not found.")
    except Exception as e:
        print(f"Error deleting simulation job '{simulation_job_arn}': {e}")


def main():
    parser = argparse.ArgumentParser(prog="DeleteRoboMakerSimulatioJob")

    # Mandatory arguments
    parser.add_argument(
        "robot_application", type=str, help="Robot application name"
    )
    parser.add_argument(
        "simulation_application", type=str, help="Simulation application name"
    )

    args = parser.parse_args()
    simulation_client = boto3.client("robomaker")

    matching_simulation_jobs = list_simulation_jobs(
        robo_application_name=args.robot_application,
        simulation_application_name=args.simulation_application,
        simulation_client=simulation_client,
    )

    print("Removing simulation jobs")
    for job_arn in matching_simulation_jobs:
        delete_simulation_job(
            simulation_job_arn=job_arn,
            simulation_client=simulation_client,
        )

    print(f"Searching robot applications by {args.robot_application}")
    robot_matching_apps = get_robot_application(
        robot_application_name=args.robot_application,
        simulation_client=simulation_client,
    )

    print("Removing robot application")
    delete_robot_application(
        matching_apps=robot_matching_apps,
        simulation_client=simulation_client,
    )

    print(f"Searching simulation applications by {args.simulation_application}")
    simulation_matching_apps = get_simulation_application(
        simulation_application_name=args.simulation_application,
        simulation_client=simulation_client,
    )

    print("Removing simulation application")
    delete_simulation_application(
        matching_apps=simulation_matching_apps,
        simulation_client=simulation_client,
    )


if __name__ == "__main__":
    main()
