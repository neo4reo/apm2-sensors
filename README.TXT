apm2-sensorhead v2.10 release notes:

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
