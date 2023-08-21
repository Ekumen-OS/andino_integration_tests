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

from typing import List
import boto3
import argparse
import json
import pandas as pd

# setting the width to display
pd.set_option("display.max_colwidth", 150)


def add_job_result(data, sim_job, sim_job_detail, log_link):
    def add_hyperlink(link):
        # Displays only the sim job id
        return f"[Cloudwatch]({link})"

    data["SimulationTest"].append(
        sim_job_detail.get("simulation_application_command")
    )

    data["Sim-job"].append(sim_job.split("/")[-1])

    data["Status"].append(sim_job_detail.get("status"))
    data["Logs"].append(add_hyperlink(log_link))


def check_simulation_jobs(simulation_json_arn):
    """
    This function will check the status of a batch
    job and return the status along with an array of ARNs
    of the successful simulation jobs once complete.

    Structure:
    {
        arns: Array | Completed individual simulation job ARNs.
        isDone: Boolean | if the batch simulation describe call returns complete.
        batchSimJobArn: String | The ARN of the simulation batch.
        status: String | InProgress, Success or Failed for downstream processing.
    }

    """

    output = {
        "arns": [],
        "isDone": False,
        "batchSimJobArn": simulation_json_arn,
        "status": "InProgress",
        "failed_arns": [],
    }

    client = boto3.client("robomaker")
    response = client.describe_simulation_job_batch(batch=simulation_json_arn)

    status = response["status"]
    if status == "Completed":
        output["isDone"] = True
        # In this sample, we fail the code pipeline on any test failure.
        if len(response["failedRequests"]) == 0:
            output["status"] = "Success"
        else:
            output["status"] = "Failed"
            output["failed_arns"] = [
                sim_job["arn"] for sim_job in response["failedRequests"]
            ]

    elif status == "Failed" or status == "Canceled":
        output["isDone"] = True
        output["status"] = "Failed"

    elif status == "InProgress":
        output["isDone"] = False

    # Collect all arns
    output["arns"] = [sim_job["arn"] for sim_job in response["createdRequests"]]

    return output


def get_simulation_job_log_url(arn, region):
    # Get the log group name and stream name from the ARN
    log_stream_name = arn.split(":")[-1].split("/")[-1]
    # Build the Cloudwatch log page
    SLASH_ENCODED = "%252F"  # This is '/' enconded twice --> what happens on AWS
    logs_url = f"https://{region}.console.aws.amazon.com/cloudwatch/home?region={region}#logsV2:"
    logs_url += f"log-groups/log-group/{SLASH_ENCODED}aws{SLASH_ENCODED}robomaker{SLASH_ENCODED}"
    logs_url += f"SimulationJobs$3FlogStreamNameFilter$3D{log_stream_name}"

    return logs_url


def get_simulation_job_detail(simulation_jobs: List[dict]) -> List[dict]:
    simulation_job_detail  = list()

    for simulation_job in simulation_jobs:
        result = dict()
        result["status"] = (
            "SUCCESS" if simulation_job.get("status") == "Completed" else "FAILED"
        )

        result["simulation_application_command"] = (
            simulation_job.get("simulationApplications", [])[0]
            .get("launchConfig")
            .get("command")[-1]
        )

        simulation_job_detail.append(result)

    return simulation_job_detail


def main(
    simulation_job_arn: str,
    region: str,
    simulation_summary_path: str,
    simulation_client: boto3.Session,
):
    # Get the output of the StepFunction
    output = check_simulation_jobs(simulation_job_arn)

    # This contains the simulation jobs that didn't run successfully despite the tests results
    total_sim_jobs = output["arns"]

    # To verify if a simulation job was successful from a test POV we query the tag "Success"
    # If the tag Success == str(True) --> it means it passed the test
    simulation_jobs = simulation_client.batch_describe_simulation_job(
        jobs=output.get("arns")
    ).get("jobs", [])
    simulation_jobs = get_simulation_job_detail(simulation_jobs)

    data = {
        "SimulationTest": [],
        "Sim-job": [],
        "Status": [],
        "Logs": [],
    }
    for idx, sim_job in enumerate(total_sim_jobs):
        cloudwatch_logs_url = get_simulation_job_log_url(sim_job, region)
        add_job_result(data, sim_job, simulation_jobs[idx], cloudwatch_logs_url)

    # TODO: Refactor later and use pure python data structure
    df = pd.DataFrame(data)
    # Add a column with emoji for Success or Failed
    df.insert(
        0,
        "",
        [
            ":heavy_check_mark:" if status == "SUCCESS" else ":x:"
            for status in df["Status"]
        ],
    )
    # Sort columns by Status
    df.sort_values(by=["Status"], ascending=False, inplace=True)

    amount_success_jobs = data.get("Status").count("SUCCESS")
    header = f"Test results: {amount_success_jobs/len(total_sim_jobs)*100} %  |  ({amount_success_jobs}/{len(total_sim_jobs)}) \n"

    # Open a file with access mode 'a'
    with open(simulation_summary_path, "a") as simulation_summary_file:
        simulation_summary_file.write(header)
        simulation_summary_file.write(df.to_markdown(index=False))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        "Process the simulation job batch results",
        description="Post-process the StepFunction output of the sim job batch",
    )
    parser.add_argument(
        "--simulation-job-arn",
        required=True,
        type=str,
        help="",
    )

    parser.add_argument("--region", required=True, type=str)

    # Optional arguments
    parser.add_argument(
        "--simulation_summary_path",
        required=False,
        type=str,
        default="/tmp/simulation_summary.txt",
    )

    args = parser.parse_args()

    simulation_client = boto3.client("robomaker")
    main(
        args.simulation_job_arn,
        args.region,
        args.simulation_summary_path,
        simulation_client,
    )
