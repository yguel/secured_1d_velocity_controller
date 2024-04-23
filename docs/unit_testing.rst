Description of unit tests of the secured_1d_velocity_controller package
=======================================================================

Tests for behavioral correctness
================================


Standard ROS2 control tests
===========================

Test if the controller can be loaded
-------------------------------------

Test if all controller parameters can be set
---------------------------------------------
It initializes the controller by defining its interfaces (commands and states)
1. It **tests** that before the controller is configured all its parameters are set to default values (e.g. empty strings, zeros, etc.)
2. It configures the controller with the example of configuration file (`test/secured_1d_velocity_controller_params.yaml`)
3. It **tests** that after the controller is configured all its parameters are set to the values defined in the configuration file

Test if the controller interfaces are correctly exported
--------------------------------------------------------

Test if the controller can be activated
----------------------------------------

Test if the controller can be deactivated
------------------------------------------

Test if the controller can be reactivated
------------------------------------------

Test if the controller can publish its status correctly
-------------------------------------------------------

Test if the controller can receive message and update its status correctly
--------------------------------------------------------------------------
