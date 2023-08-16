# ekumen_cloud_robotics_ci

## Description.
Contains the scripts used to communicate between the repository and the tests powered by AWS RoboMaker,
the dependency requirements and the template test suite configuration to describe AWS RoboMaker simulation jobs.

The aim of this package is to provide the necessary automation capabilities to simplify the process of running  tests powered by AWS RoboMaker in the context of a CI environment.

### Generate the test description
Refer to `scripts/process_json_test_description.py`.

This script acts as an intermediary between the user of this repository and the Step Function responsible for launching simulations. Its purpose is to simplify the process of providing the necessary input for the Step Function, ensuring that all request fields are fulfilled. Additionally, it has the potential to streamline the creation of new tests. For instance it could consume a default JSON or YAML template and make the requested changes only.

### Process simulation results
Refer to `scripts/process_simulation_results.py`.

It should be updated because we are checking directly the simulation job results. It could say `This script retrieves the test results given a simulation job ARN.

### Delete simulation jobs
Refer to `scripts/delete_simulation_job.py`.

This script deletes from AWS RoboMaker both the robot and the simulation applications.
Also, it frees the resources when the pull request that generated the applications are closed.

## Install dependencies

Install the python dependencies to execute the scripts within this package.
It is assumed that the system runs on Ubuntu 22.04, with Python 3.10. To install the dependencies, run:

```sh
pip install -r requirements.txt
```
