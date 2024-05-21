A Simple Example
================



.. literalinclude:: secured_1d_velocity_controller_config_minimal_example.yaml
   :language: yaml
   :caption: Example of a yaml configuration file for the secured 1d velocity controller
   :linenos:

In this example, you can suppose having a switch at each limit of the joint range of motion.
Each of those switches emit a signal as a voltage of 0V when not activated (switch open) and 1V when activated (switch closed).
When the joint does not reach a limit, none of the switches are activated and each of their voltage is 0V.
When the joint reaches a limit, the corresponding switch is activated and its voltage switches to 1V.
