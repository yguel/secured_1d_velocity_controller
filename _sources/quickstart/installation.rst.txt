Installation
============

Install and build the package
-----------------------------

If you have no ros2 workspace yet, create one:
    .. code-block:: console

        $ mkdir -p ~/ros2_ws/src

If you have a ros2 workspace, navigate to the source directory and clone the repository:
        $ cd ~/ros2_ws/src
        $ git clone https://github.com/yguel/secured_1d_velocity_controller

.. _How to build the package:

Navigate to the workspace root and build the package:
    .. code-block:: console

        $ cd ~/ros2_ws
        $ source /opt/ros/humble/setup.bash
        $ rosdep install --ignore-src --from-paths . -y -r
        $ colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install --packages-select secured_1d_velocity_controller
        $ source install/setup.bash

To build and run the tests, :ref:`see the instructions about building unit tests<Build and launch the tests>`.

Build this documentation
------------------------

1. Install dependencies using
    .. code-block:: console

        $ pipenv install

2. Create the parameter description file from the definition of the controller parameters using
    .. code-block:: console

        $ cd user_guide
        $ generate_parameter_library_markdown --input_yaml configuration_file_for_secured_1d_velocity_controller.yaml --output_markdown_file secured_1d_velocity_controller_parameters.md
        $ cd ..

3. Generate the documentation using ``make html`` using the pip environment:
    .. code-block:: console

        $ pipenv run make html

The result is in the ``_build/html`` directory. Open the ``index.html`` file in a browser to view the documentation.
