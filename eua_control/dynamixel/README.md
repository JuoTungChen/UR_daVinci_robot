# Dynamixel

Dynamixel is a small Python library for interfacing with ROBOTIS Dynamixel
servos through a serial interface.

A couple of notes:

* Tested with the U2D2 and USB2AX adapters.
* Currently only supports Dynamixel Protocol 1.0
* Currently only supports AX-12A and MX-28 servos, but can easily be extended to
  support more device types.
* Currently only position/joint (and not velocity/free-wheel) mode control has
  been tested.
* For improved communication speed, configure the latency timer of FTDI devices
  (like the U2D2) to 1ms with `setserial /dev/ttyUSB0 low_latency`
  (assuming the device is on /dev/ttyUSB0)
