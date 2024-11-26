#!/bin/bash

source /opt/ros/humble/setup.bash
source /autoware/install/setup.bash

rviz2 -d /autoware/simulation/scenario_simulator.rviz
