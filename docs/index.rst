.. secured_1d_velocity_controller documentation master file, created by
   sphinx-quickstart on Thu Feb 15 16:40:09 2024.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to secured_1d_velocity_controller's documentation!
==========================================================

.. toctree::
   :maxdepth: 2
   :caption: Contents:



Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

Secure 1D Velocity Controller
=============================

We consider that the 1 degree of freedom (1D) system has at least one limit.
Our model of 1D actioned system is that the position is increasing:

1. from the start limit to the end limit in case of 2 limits
2. from the start limit in case of 1 limit (the end limit not being defined)
3. to the end limit in case of 1 limit (the start limit not being defined)

The velocity being the derivative of the position with respect to time, the
sign of the velocity is positive when the motion direction is towards the end
limit and negative when the motion direction is towards the start limit.

Then the following safety rules are applied in conjunction except if specified otherwise:
1. The velocity is limited by the maximum velocity
2. The acceleration is limited by the maximum acceleration
3. The tracking error is limited by the maximum tracking error (the tracking error is the difference between the reference velocity and the actual velocity)
4. If one limit is active, the only admissible velocities should 
have a direction/sign that direct the motion away from the active limit.
In that case the tracking error check is not applied.
4. If an effort sensor is available, the effort is limited by the maximum effort.

It is possible to disable any of those checks, but it is necessary to do it explicitly.
For all those checks there is a default value (very conservative).
These values can (must) be changed by the user.
This default behavior is here to ensure that the user is aware of the security problem and to force the user to take a decision about it.

Configuration of the security parameters
========================================
The security parameters are configured in the yaml configuration file.

start_limit_joint: {
   type: string,
   default_value: "",
   description: "Joint name holding the start limit sensor",
   read_only: true,
   validation: {
      unique<>: null,
}
end_limit_joint: {
   type: string,
   default_value: "",
   description: "Joint name holding the end limit sensor",
   read_only: true,
   validation: {
      unique<>: null,
}
start_limit_state_interface: {
   type: string,
   default_value: "limit_switch",
   description: "State interface provided by the hardware responsible for the start limit sensor",
   read_only: true,
   validation: {
      unique<>: null,
}
end_limit_state_interface: {
   type: string,
   default_value: "limit_switch",
   description: "State interface provided by the hardware responsible for the end limit sensor",
   read_only: true,
   validation: {
      unique<>: null,
}
