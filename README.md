# `roboclaw_driver`

[![Build Status](https://travis-ci.org/sheaffej/roboclaw_driver.svg?branch=master)](https://travis-ci.org/sheaffej/roboclaw_driver)

A ROS node providing a driver interface to the Roboclaw motor controller.

## Why another ROS Roboclaw driver?
There are several existing ROS nodes for the roboclaw, including:

* [https://github.com/sonyccd/roboclaw_ros](https://github.com/sonyccd/roboclaw_ros)
* [https://github.com/doisyg/roboclaw_ros](https://github.com/doisyg/roboclaw_ros)
* [https://github.com/SV-ROS/roboclaw_driver](https://github.com/SV-ROS/roboclaw_driver)

All three follow a similar approach, where the Roboclaw node is essentially the 2-wheel differential drive base (aka base_link). Therefore, the Roboclaw node in these repositories compute and publish the Odometry and tf frame data.

However I am buidling a 4-wheel differential drive base that drives two Roboclaw controllers. I have a separate base node that sends commands to two separate Roboclaw driver nodes. Therefore this roboclaw driver node needed to be more of a ROS wrapper to the Roboclaw controller. 

In my robot, the Odometry and tf frames will be computed and published by the separate base node, using encoder readings published by the Roboclaw nodes as well as fused with IMU sensor data to improve the Odometry accuracy.

## Parameters

* `~speed_cmd_topic` - Topic on which to listen for SpeedCommand messages
* `~dev_name` - Serial (aka USB) device name
* `~baud` - Default 115200
* `~address` - Serial address, default 0x80
* `~loop_hz` - Number of publisher loops per second (Hertz)
* `~deadman_secs` - Seconds until motors stop without additional commands
* `~test_mode` - True = run the Roboclaw simulator stub for testing

## Topics

### Publishes:

Stats topic: `<node_name>/stats`

* Motor 1 & 2 encoder values
* Motor 1 & 2 speed values in QPPS (+/-)

Diagnostic updater topic: `/diagnostics`

* Temperature 1 (C)
* Temperature 2 (C)
* Main Battery (V)
* Logic Battery (V)
* Motor 1 current (Amps)
* Motor 2 current (Amps)

#### Subscribes to:
SpeedCommand topic: `base_node/speed_command`

* M1 and M2 motor speed in QPPS
* Max seconds before automatically stopping

## Launching
Clone the repository

```
cd $ROS_WORKSPACE/src
git clone https://github.com/sheaffej/roboclaw_driver.git
```

Build the package to create the message bindings

```
cd $ROS_WORKSPACE
catkin_make
```

Launch the node

```
roslaunch roboclaw_driver roboclaw_node
```

## Tests

### Unit tests
The only logic that is non-trivial and therefore likely to break during refactoring is the RoboclawStub object that simulates the hardware controller for use in testing. The rest of the logic is more of a wrapper, and therefore will be tested during node-level integration testing.

```
pytest src/test_roboclaw_stub_unit.py
```

### Node-level integration tests
Launches a test node to control the roboclaw_driver node using the simulated roboclaw controller (RoboclawStub)

```
rostest roboclaw_driver stub.launch
```

## Attributions
The `roboclaw.py` library is every so slightly modified (basic formatting and comments) from the version downloadable from the Ion Motion control site :
[http://downloads.ionmc.com/code/roboclaw_python.zip]()