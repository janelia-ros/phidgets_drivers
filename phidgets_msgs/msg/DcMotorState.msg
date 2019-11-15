# This is a message that holds data to describe the state of a set of dc motors.
#
# The state of each dc motor is defined by:
#  * the back emf of the motor (V),
#
# Each dc motor is uniquely identified by its name
# The header specifies the time at which the dc motor states were recorded. All the dc motor states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the dc motor state.
# The goal is to make each of the fields optional.
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the dc motor name with the correct
# states.


std_msgs/Header header

string[] name
float64[] back_emf
