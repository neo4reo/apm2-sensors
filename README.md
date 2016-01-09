apm2-sensors - replacement firmware for an inexpensive APM2 board.
This firmware converts the APM2 into a standalone sensors module
and adds a full suite of inertial and position sensors to any host
computer or application.  This firmware maintains APM2 support for
reading RC receiver values in, and driving output servos so for some
applications it could serve as a robot controller.


v2.30 release notes -
========================================

Tighter main loop timing, synced with IMU samples.  (Removes a free-running
loop which leads to timing jitter and indeterminant latencies.)

Reduced code size by 10%.

Quite a few changes to remove prog_char from the libraries and remove
FastSerial in favor of HardwareSerial.

Support for compiling with newer version of the arduino IDE.

Make PWM center and range match Futaba 'standard'.

Add commands to setup SAS mode and adjust gains from host computer.

Updated several packet formats and ID numbers.

Report bytes transfer rate to host in the config/status message.

Better support for saving configuration parameters in the EEPROM.


v2.20 release notes - April 21, 2015
========================================

Add a simple 3-axis stability augmentation system.  Essentially it is:

  aileron_cmd += roll_gyro * gain
  elevator_cmd += elevator_gyro * gain
  rudder_cmd += yaw_gyro * gain

Each axis can be enabled/disabled independently and each has it's own
unique gain value.

The stability augmentation is performed by manipulating the normalized
input 'commands' and thus will work with any downstream mixing modes
that may be active for any particular airframe.

The SAS will also work identically in manual or autopilot mode because it 
is downstream of the flight commands (reciever or autopilot), but upstream
of the actuator mixing.

Add support for reading/writing the setup/configuration/gain values to
eeprom and loading them automatically at power up.  This eliminates
the need to send the configuration values to the device every boot.
Presumably this would enable the device to survive an inflight reboot
without losing it's config, or even run 'headless' without a host
computer once it has been setup (i.e. as in a smart RC receiver mode.)


v2.10 release notes - February 16, 2015
=======================================

The major functionality change in this release is a move towards doing
all mixing modes onboard the APM2 rather than on the RC transmitter
(and attempting to match in the upstream autopilot code.)  This
simplifies and standardizes the mixing mode operation (elevons,
flaperons, vtail, etc.) so that actuator behavior in response to
control commands is identical in autonomous flight and manual flight.

Internally, flight commands (either from the RC transmitter or the
autopilot) are normalized to a range of [-1.0, 1.0] for symmetrical
inputs and [0.0, 1.0] for other inputs (like throttle, flaps)

The mixing operations are done using floating point math in normalized
space.  Then the final result is converted back to PWM pulses and sent
to the actuators.

The convention used (at the APM2 input side) is:

Ch1: aileron
Ch2: elevator
Ch3: throttle
Ch4: rudder
Ch5: gear
Ch6: flaps
Ch7: Aux1
Ch8: Manual/Auto selection switch.

Elevon mixing drives output channels 1 & 2.

Flaperon mixing drives output channels 1 & 6.  (Each aileron servo is
plugged into it's own actuator output channel.)

Vtail mixing drives output channels 2 & 4.

This set of mixing changes also incorporate extensions to the two-way
communication protocol between the host and the APM2.  There is a new
message type to enable/disable specific mixing modes and set the
gains.  These configuration commands can be sent at anytime during the
flight if the specific use case warrents such a thing.

In addition, all successful configuration commands result in an ACK
packet being sent to the host.  This ACK packet has been extended to
two bytes so mixing mode configuration commands can respond with the
specific mode ACK that was received.  The host can know which specific
mix mode command was received, not just that "a" mixing mode config
command was received.

There was some small nuance changes to the main loop processing order.
Hopefully the servo outputs will be written a small fraction sooner.
Probably not enough to make any noticable difference, but the code is
now setup to minimize latency as much as possible.

All code is licensed under the GPLv3 except where otherwise noticed in a 
source file header.
