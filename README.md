# Summary: #

This package simulates an inverted pendulum connected to a motor with a fixed arm and controls it using torque into a stable upright position.

![inverted pendulum demo](demo/demo.gif)

## Structure: ##

* Nodes:
  - Simulator:
    Simulates inverted pendulum motion. Sends out control inputs. Takes in control outputs. Visualizes in rviz.
  - Controller:
    Uses the simulated state information to make a control decision about what torque to apply to the pendulum base.

* Massages:
  - control_input.msg
  - Visualization_msgs

* Note:
  No libraries is needed to install. The standard libraries is enough.

## Launch: ##

`roslaunch pendulum_control control.launch`
