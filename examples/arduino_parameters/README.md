
# Example: Parameters #

This example builds on the [One Link - One Component](/examples/arduino_one_link_one_component) example and adds parameters to the component, using the `lib/fastmavlink_parameters.h` tool box. 

The example implements a component, which sends out a `HEARTBEAT` message at intervals of 1 Hz, and listens to incoming `HEARTBEAT` messages from a flight controller or GCS (you need to select) and sends out a "Hello World" `STATUSTEXT` message as response. In addition it reacts to the `PARAM_REQUEST_READ`, `PARAM_REQUEST_LIST`, and `PARAM_SET` messages.

In order to check its functioning, you can connect to a GCS like MissionPlanner.

See the Arduino sketch [arduino_parameters.ino](arduino_parameters.ino) for an implementation (I have tested it on a STM32F103 bluepill).

Please inspect the sketch, and especially the comments. The usage of the `lib/fastmavlink_parameters.h` functions and tools should be self-explaining.

