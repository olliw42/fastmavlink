
# The fastMavlink Library: Examples #

There are usually so many different ways of how to implement and realize one and the same goal, that code examples hardly can cover them. The following examples, which are written for Arduino, hence may not match exactly what you need. They however hopefully provide insight into how the fastMavlink library can be used, and they may serve as starting templates for your own projects.

## Assumed Interface ##

The examples make some common assumptions about your system and the serial (UART or USB/VCP) handling:
 
It is assumed that the serial write and read functions are interrupt-driven and not of polling nature. The code might be simpler in the latter case, but in my experience except for maybe only the most simplest applications polling is not an option. That means, there are Tx and Rx buffers, which are filled and emptied by the write and read functions.

The following interface of serial functions is specifically assumed:

- `uint16_t serial_available(void)`: Returns the number of bytes in the Rx buffer (0 for none)

- `void serial_read_char(uint8_t* c)`: Returns the next available byte in the Rx buffer. Should not be called without having checked availabe before.

- `uint8_t serial_has_space(uint16_t count)`: Returns 1 if the Tx buffer has enough space to accomodate count bytes, else return 0.

- `void serial_write_buf(uint8_t* buf, uint16_t len)`: Copies len bytes from the buf into the Tx buffer. Should not be called without having checked has_space before.

This serial interface may seem overly complicated, and the available and has_space functions indeed often can be combined into the read and write functions, but this interface makes the discrete steps more explicit. It should be easy enough to convert the examples to your interface.

If an application uses more than one serial port, such as in a MAVLink router, the same interface is assumed, just with the `serial` appended by a number, e.g., `serial1_has_space()` or `serial2_write_buf()`, and so on.

In addition to the above, a timer is also assumed:

- `uint32_t get_time_ms()`: Returns the current system time in units of ms.


For some examples, both a very details code version is presented which uses primitive functions and a more compact (and often more performant) code version which uses higher-level functions. This hopefully helps enlightening the inner working of the fastMavlink library.

In order to test them out easily, the Arduino sketches are included.

## One Link - One Component ##

This is the simplest and probably most typical type of application. It consists of an electronic device which should behave as a MAVLink component and which is connected to the MAVLink network via a serial connection.

Go to [One Link - One Component](arduino_one_link_one_component/).


## Parameters ##

This example builds on the [One Link - One Component](/examples/arduino_one_link_one_component) example and adds parameters to the component, using the tool box provided by `lib/fastmavlink_parameters.h`. 

Go to [Parameters](arduino_parameters/).


## Several Links - No Component: MAVLink Router ##

A MAVLink router is a device with several serial ports, which connect to the MAVLink network. It essentially plays the same role as routers do for Ethernet networks, i.e., forwards incoming messages on one link to the other links. It is not a MAVLink component itself. An example would be a device with a serial port connected to a telemetry unit such as 3DR radios or DragonLink, a bluetooth or wifi port to connect wirelessly, and a USB port to connect to a PC. This needs a 3-link router in order to establish communication between the ports. 

This is not the most typical use case for the fastMavlink library, even though fastMavlink is very well suited for it. The example's major intention is to prepare the stage for the next example.

Explore the next example, [Several Links - One Component](#several-links---one-component-component-with-routing-capabilities), it provides sufficient information and code examples to easily realize this.

## Several Links - One Component: Component with Routing Capabilities ##

This is a component with several serial ports, which all sould be connectable to the MAVLink network. An example would be an autopilot like ArduPilot or PX4 flight controllers. The autopilot is a MAVLink component, which allows us to configure several serial ports for MAVLink. Another example would be an OpenTx transmitter loaded with firmware of the [MAVLink for OpenTx](http://www.olliw.eu/2020/olliwtelem/) project.

Go to [Several Links - One Component](arduino_several_links_one_component/).


## One Link - Several Components ##

In this application a physical device with only one serial port implements two or more distinct MAVLink components. That is the communication with the device's components all goes via one and the same serial port. An example would be the [STorM32 gimbal controller](http://www.olliw.eu/storm32bgc-wiki/MAVLink_Communication), which implements both a Gimbal component and a Camera component.

TBD
