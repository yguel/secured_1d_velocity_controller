# Copyright (c) 2024, ICube
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# author: Manuel YGUEL (yguel at unistra.fr)

import click
import os


@click.command()
@click.argument("test_results", type=click.Path(exists=True), required=True)
def main(test_results: str):
    """
    Check if the test results contain any test failures. If there are any test failures, the script will print a message and exit with code 1. If there are no test failures, the script will print a message and exit with code 0.

    Parameters
    ----------
    test_results: str
        The path to the test results as a directory containing all the test-results xml files or a single xml file.

    Returns
    -------
    None

    """
    # Test if test_results is a file
    if os.path.isfile(test_results):
        # Open the file and read the content
        with open(test_results) as file:
            content = file.read()
        # Check if the content contains the string "<failure"
        if "<failure" in content:
            # If it does, print a message and exit with code 1
            print(f"Test failure in {test_results} .")
            exit(1)
        else:
            # If it does not, print a message and exit with code 0
            print(f"No test failures in {test_results} .")
            exit(0)
    elif os.path.isdir(test_results):
        # If test_results is a directory, iterate over the files in the directory
        for file in os.listdir(test_results):
            # Check if the file is a file
            if os.path.isfile(os.path.join(test_results, file)):
                # Test if the file is an xml file
                if file.endswith(".xml"):
                    # Open the file and read the content
                    with open(os.path.join(test_results, file)) as f:
                        content = f.read()
                    # Check if the content contains the string "<failure"
                    if "<failure" in content:
                        # If it does, print a message and exit with code 1
                        print(f"Test failure in {os.path.join(test_results, file)} .")
                        exit(1)

        # If it does not, print a message and exit with code 0
        print(f"No test failures in {os.path.join(test_results)} .")
        exit(0)


if __name__ == "__main__":
    main()
