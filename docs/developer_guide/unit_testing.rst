Description of unit tests of the secured_1d_velocity_controller package
=======================================================================

Launch the tests
----------------
After installing and building the package (:ref:`see the build section of the user guide<How to build the package>`), you can launch the tests with the following command:

.. _Build and launch the tests:

    .. code-block:: console

        $ colcon test --ctest-args tests --packages-select  secured_1d_velocity_controller

You can see the results of the tests with the following command:

    .. code-block:: console

        $ colcon test-result --verbose --test-result-base build/secured_1d_velocity_controller

........................................................................

Tests for behavioral correctness
--------------------------------

Test if limits are enforced when the secure mode is set to ``SECURE``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The test is named ``update_logic_secure_mode``.

The test loads the controller with the configuration file ``test/secured_1d_velocity_controller_params.yaml`` and set the secure mode to ``SECURE``.

Then the test performs 8 tests:

1. Tests positive velocity and no limit activated
   1. The test sends a positive velocity command to the controller.
   2. It **tests** that the same velocity command is sent to the joint.
2. Tests negative velocity and no limit activated
   1. The test sends a negative velocity command to the controller.
   2. It **tests** that the same velocity command is sent to the joint.
3. Tests positive velocity and start limit activated:
   1. The test mocks the start limit being activated
   2. It sends a positive velocity command to the controller.
   3. It **tests** that the same velocity command is sent to the controller.
4. Tests positive velocity and end limit activated
   1. The test mocks the end limit being activated.
   2. A positive velocity command is sent to the controller.
   3. It **tests** that a zero velocity command is sent to the controller.
5. Tests negative velocity and start limit activated
   1. The test mocks the start limit being activated.
   2. A negative velocity command is sent to the controller.
   3. It **tests** that a zero velocity command is sent to the controller.
6. Tests negative velocity and end limit activated
   1. The test mocks the end limit being activated.
   2. A negative velocity command is sent to the controller.
   3. It **tests** that the same velocity command is sent to the controller.
7. Tests positive velocity and both limits activated
   1. The test mocks the start limit and the end limit being activated.
   2. A positive velocity command is sent to the controller.
   3. It **tests** that a zero velocity command is sent to the controller.
8. Tests negative velocity and both limits activated.
   1. The test mocks the start limit and the end limit being activated.
   2. A negative velocity command is sent to the controller.
   3. It **tests** that a zero velocity command is sent to the controller.


Test if limits are ignored when the secure mode is set to ``INSECURE``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The test is named ``update_logic_insecure_mode``.

The test loads the controller with the configuration file ``test/secured_1d_velocity_controller_params.yaml`` and set the secure mode to ``INSECURE``.

Then the test performs 8 tests:

1. Tests positive velocity and no limit activated
   1. The test sends a positive velocity command to the controller.
   2. It **tests** that the same velocity command is sent to the joint.
2. Tests negative velocity and no limit activated
   1. The test sends a negative velocity command to the controller.
   2. It **tests** that the same velocity command is sent to the joint.
3. Tests positive velocity and start limit activated:
   1. The test mocks the start limit being activated
   2. It sends a positive velocity command to the controller.
   3. It **tests** that the same velocity command is sent to the controller.
4. Tests positive velocity and end limit activated
   1. The test mocks the end limit being activated.
   2. A positive velocity command is sent to the controller.
   3.  It **tests** that the same velocity command is sent to the controller.
5. Tests negative velocity and start limit activated
   1.  The test mocks the start limit being activated.
   2. A negative velocity command is sent to the controller.
   3. It **tests** that the same velocity command is sent to the controller.
6. Tests negative velocity and end limit activated
   1. The test mocks the end limit being activated.
   2. A negative velocity command is sent to the controller.
   3. It **tests** that the same velocity command is sent to the controller.
7. Tests positive velocity and both limits activated
   1. The test mocks the start limit and the end limit being activated.
   2. A positive velocity command is sent to the controller.
   3. It **tests** that the same velocity command is sent to the controller.
8. Tests negative velocity and both limits activated.
   1. The test mocks the start limit and the end limit being activated.
   2. A negative velocity command is sent to the controller.
   3. It **tests** that the same velocity command is sent to the controller.

........................................................................

Standard ROS2 control tests
---------------------------

Test if the controller can be loaded
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Test if all controller parameters can be set
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
This test checks the ``on_configure`` stage.

It initializes the controller by defining its interfaces (commands and states).
\
The test action that are performed are the following:

1. it **tests** that before the controller is configured all its parameters are set to default values (e.g. empty strings, zeros, etc.);
2. it configures the controller with the example of configuration file (`test/secured_1d_velocity_controller_params.yaml`) with the ``on_configure`` method;
3. it **tests** that the controller can correctly pass the configure stage;
4. it **tests** that after the controller is configured all its parameters are set to the values defined in the configuration file.

Test if the controller interfaces are correctly exported
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
It initializes the controller by defining its interfaces (commands and states)

1. ``SetUpController`` method
2. call the ``on_configure`` method

The test action that are performed are the following:

1. it **tests** that the controller can correctly pass the configure stage;
2. it gets the command interfaces after configuration;
3. it **tests** that the controller has the correct number of command interfaces;
4. it **tests** that the controller has the correct order and name for the vector of command interfaces;
5. it gets the state interfaces after configuration;
6. it **tests** that the controller has the correct number of state interfaces;
7. it **tests** that the controller has the correct order and name for the vector of state interfaces.

Test if the controller can be activated
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
It initializes the controller by defining its interfaces (commands and states)

1. ``SetUpController`` method
2. call the ``on_configure`` method
3. call the ``on_activate`` method

The test action that are performed are the following:

1. it **tests** that the controller can correctly pass the configure stage;
2. it **tests** that the controller can correctly pass the activate stage;
3. it **tests** that the velocity value in the controller reference is set to zero.

Test if the controller update method does not crash
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
This test call the ``update`` method of the controller. The most important method that convert the reference command to the control command.
This test does not check the correctness of the control command, but only if the method does not crash and returns the correct return code.

It initializes the controller by defining its interfaces (commands and states)

1. ``SetUpController`` method
2. call the ``on_configure`` method
3. call the ``on_activate`` method
4. call the ``update`` method

The test action that are performed are the following:

1. it **tests** that the controller can correctly pass the configure stage;
2. it **tests** that the controller can correctly pass the activate stage;
3. it **tests** that the return code of the update method is correct;

Test if the controller can be deactivated
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
It initializes the controller by defining its interfaces (commands and states)

1. ``SetUpController`` method
2. call the ``on_configure`` method
3. call the ``on_activate`` method
4. call the ``on_deactivate`` method

The test action that are performed are the following:

1. it **tests** that the controller can correctly pass the configure stage;
2. it **tests** that the controller can correctly pass the activate stage;
3. it **tests** that the controller can correctly pass the deactivate stage;

Test if the controller can be reactivated
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
It initializes the controller by defining its interfaces (commands and states)

1. ``SetUpController`` method
2. call the ``on_configure`` method
3. call the ``on_activate`` method
4. Get the first command interface value after activation
5. call the ``on_deactivate`` method
6. Get the first command interface value after deactivation
7. call the ``on_activate`` method
8. Get the first command interface value after reactivation
9. call the ``update`` method

The test action that are performed are the following:

1. it **tests** that the controller can correctly pass the configure stage;
2. it **tests** that the controller can correctly pass the activate stage;
3. it **tests** that the velocity command interface value is set to the reference command;
4. it **tests** that the controller can correctly pass the deactivate stage;
5. it **tests** that the velocity command interface value is set to nan;
6. it **tests** that the controller can correctly pass the activate stage;
7. it **tests** that the velocity command interface value is set to the zero;
8. it **tests** that the return code of the update method is correct;

Test if the controller can publish its status correctly
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Test if the controller can receive message and update its status correctly
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
