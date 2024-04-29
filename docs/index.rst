..
  secured_1d_velocity_controller documentation master file

Secure 1d controllers stack for ROS2 control
============================================

In order to build reliable and secure robotic systems, it is necessary to integrate security mechanisms in the control loops such as limit checking, velocity and acceleration saturation, etc.
In order to facilitate the integration of these mechanisms, we propose to develop a stack for secure 1d controllers for ROS2 control that would allow to easily :
1. discover/calibrate limit elements for 1d actioners
2. discover/calibrate span of the actioner for position control
3. setup the security parameters for the control loops like maximum velocity, maximum acceleration, maximum tracking error, etc.
4. provides tools to integrate these mechanisms with any other control requirements (other higher level controllers) in the control loops.

This project is at its early stage and we propose to start with the development of a secure 1d velocity controller.

**Project GitHub repository**: secured_1d_velocity_controller (<https://github.com/yguel/secured_1d_velocity_controller>)

.. toctree::
   :maxdepth: 2
   :caption: Quickstart
   :glob:

   quickstart/installation
   quickstart/simple_example

.. toctree::
   :maxdepth: 2
   :caption: User Guide
   :glob:

   user_guide/secured_1d_velocity_controller

.. toctree::
   :maxdepth: 2
   :caption: Developer Guide
   :glob:

   developer_guide/unit_testing
