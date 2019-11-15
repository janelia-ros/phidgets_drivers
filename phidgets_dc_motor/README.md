Phidgets DC Motor ROS 2 driver
==============================

This is the ROS 2 driver for Phidgets DC motor.  The various topics, services, and parameters that the node operates with are listed below.

Published Topics
----------------
* `/joint_state` (`sensor_msgs/JointState`) - A joint state message containing the current state of all dc motors.
* `/dc_motor_state` (`phidgets_msgs/DcMotorState`) - A dc motor state message containing the current state of all dc motors.

Subscribed Topics
-----------------
* `/set_motor_duty_cycleXX` (`std_msgs/Float64`) - Set the motor duty cycle.  One topic is created for each motor attached.

Parameters
----------
* `serial` (int) - The serial number of the phidgets motor to connect to.  If -1 (the default), connects to any motor phidget that can be found.
* `hub_port` (int) - The phidgets VINT hub port to connect to.  Only used if the motor phidget is connected to a VINT hub.  Defaults to 0.
* `braking_strength` (double) - The braking strength to apply when the duty cycle is 0.  Defaults to 0.0.
* `data_interval_ms` (int) - The number of milliseconds between acquisitions of data on the device (allowed values are dependent on the device).  Defaults to 250 ms.
* `publish_rate` (double) - How often the driver will publish data on the ROS topic.  If 0 (the default), it will publish every time there is an update from the device (so at the `data_interval_ms`).  If positive, it will publish the data at that rate regardless of the acquisition interval.

Command Line Examples
---------------------

```bash
ros2 launch phidgets_dc_motors dc-motor-launch.py
ros2 topic echo /joint_state
ros2 topic echo /dc_motor_state
```
